#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry

try:
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState
    _GAZEBO_MSGS_AVAILABLE = True
except ImportError:
    _GAZEBO_MSGS_AVAILABLE = False


class LeaderSMC:
    def __init__(self):
        rospy.init_node('leader_smc')

        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/turtlebot3_leader/cmd_vel')
        odom_topic = rospy.get_param('~odom_topic', '/turtlebot3_leader/odom')
        self.pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('/sPath', Path, self.path_callback)

        self.path_pub = rospy.Publisher('/leader/path', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"  # Fix 1: odom-space consistency

        # Robot states
        self.x, self.y = 0.0, 0.0
        self.theta = 0.0
        self.v, self.omega = 0.0, 0.0

        # Velocity from odom (world frame); updated in odom_callback (Fix 2)
        self.xdot, self.ydot = 0.0, 0.0
        self.F, self.tau = 0.0, 0.0

        # Constants
        self.L = 0.1
        self.m, self.I = 1.0, 1.0
        self.T = 0.05

        # SMC parameters
        self.lambda_gain = 2.0
        self.K_dis = 3.5
        self.k_con = 1.5

        # Path handling   
        self.path_points = []
        self.wp_index = 0
        self.goal_tol = 0.15
        self.lookahead_dist = 0.4
        # Desired speed (m/s); turtle-like motion = small value (e.g. 0.12–0.2)
        self.desired_speed = rospy.get_param('~desired_speed', 0.15)
        self.max_linear = rospy.get_param('~max_linear', 0.22)
        self.max_angular = rospy.get_param('~max_angular', 1.2)
        # Logging: step counter and last logged wp_index for tracing logs
        self._trace_step = 0
        self._last_log_wp = -1
        self._log_interval = 10  # log every N control steps
        self._odom_received = False
        self._stuck_warn_step = -1

        # v1.2: initialize bot at path start (via Gazebo set_model_state)
        self.init_at_path_start = rospy.get_param('~init_at_path_start', True)
        self.gazebo_model_name = rospy.get_param('~gazebo_model_name', 'turtlebot3_burger')
        self._set_state_srv = None
        if _GAZEBO_MSGS_AVAILABLE and self.init_at_path_start:
            rospy.loginfo("Leader v1.2: init_at_path_start enabled; bot will be placed at path start when a new path is received.")

        self.timer = rospy.Timer(rospy.Duration(self.T), self.control_loop)

    # ----------------------------------------------------
    def _set_robot_to_path_start(self, first_pose_msg):
        """Place the robot in Gazebo at the path start (first pose). Uses /gazebo/set_model_state if available."""
        if not _GAZEBO_MSGS_AVAILABLE or not self.init_at_path_start:
            return
        try:
            if self._set_state_srv is None:
                rospy.wait_for_service('/gazebo/set_model_state', timeout=2.0)
                self._set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        except (rospy.ROSException, rospy.ROSInterruptException) as e:
            rospy.logdebug_throttle(5.0, "Gazebo set_model_state not available: %s", str(e))
            return

        state = ModelState()
        state.model_name = self.gazebo_model_name
        state.reference_frame = "world"
        state.pose.position.x = first_pose_msg.pose.position.x
        state.pose.position.y = first_pose_msg.pose.position.y
        state.pose.position.z = getattr(first_pose_msg.pose.position, 'z', 0.0) or 0.0
        if state.pose.position.z < 1e-6:
            state.pose.position.z = 0.01  # avoid sinking into ground
        q = first_pose_msg.pose.orientation
        if abs(q.w) < 1e-9 and abs(q.x) < 1e-9 and abs(q.y) < 1e-9 and abs(q.z) < 1e-9:
            state.pose.orientation.w = 1.0
            state.pose.orientation.x = state.pose.orientation.y = state.pose.orientation.z = 0.0
        else:
            state.pose.orientation = first_pose_msg.pose.orientation
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0
        try:
            resp = self._set_state_srv(state)
            if resp.success:
                rospy.loginfo("Leader v1.2: robot placed at path start (%.2f, %.2f).", state.pose.position.x, state.pose.position.y)
            else:
                rospy.logwarn("Leader v1.2: set_model_state failed: %s", getattr(resp, 'status_message', 'unknown'))
        except rospy.ServiceException as e:
            rospy.logwarn("Leader v1.2: set_model_state call failed: %s", str(e))

    # ----------------------------------------------------
    def path_callback(self, msg):
        # Fix 7: warn if path is not in odom
        frame = msg.header.frame_id.strip() or "unknown"
        if frame != "odom":
            rospy.logwarn_throttle(2.0, f"/sPath frame is '{frame}'; odom expected for odom-space control.")

        # Fix 8: only reset when path actually changes
        new_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not new_points:
            self.path_points = []
            self.wp_index = 0
            rospy.loginfo("Received empty path, cleared.")
            print("[trace] path cleared", flush=True)
            return

        same_path = (
            len(self.path_points) == len(new_points)
            and len(self.path_points) > 0
            and np.hypot(self.path_points[0][0] - new_points[0][0], self.path_points[0][1] - new_points[0][1]) < 1e-6
            and np.hypot(self.path_points[-1][0] - new_points[-1][0], self.path_points[-1][1] - new_points[-1][1]) < 1e-6
        )
        if same_path:
            return

        self.path_points = new_points
        self.wp_index = 0
        self._trace_step = 0
        self._last_log_wp = -1
        start_pt = new_points[0]
        goal_pt = new_points[-1]
        rospy.loginfo("[trace] New path: %d points, start (%.2f, %.2f) -> goal (%.2f, %.2f)",
                      len(self.path_points), start_pt[0], start_pt[1], goal_pt[0], goal_pt[1])
        print("[trace] New path: {} points, start ({:.2f}, {:.2f}) -> goal ({:.2f}, {:.2f})".format(
            len(self.path_points), start_pt[0], start_pt[1], goal_pt[0], goal_pt[1]), flush=True)

        # v1.2: initialize bot at path start (source point)
        self._set_robot_to_path_start(msg.poses[0])

    # ----------------------------------------------------
    def odom_callback(self, msg):
        self._odom_received = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.theta = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])[2]

        # Fix 2: velocity from odom twist (child frame -> world frame)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        c, s = np.cos(self.theta), np.sin(self.theta)
        self.xdot = c * vx - s * vy
        self.ydot = s * vx + c * vy

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "odom"  # Fix 1
        pose.pose = msg.pose.pose
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

    # ----------------------------------------------------
    def find_lookahead_point(self):
        # Advance past any waypoints we're already within lookahead_dist of (so we don't stop when reaching them)
        for i in range(self.wp_index, len(self.path_points)):
            px, py = self.path_points[i]
            d = np.hypot(px - self.x, py - self.y)
            if d > self.lookahead_dist:
                self.wp_index = i
                return px, py
            # Within lookahead_dist of this point; advance past it and keep looking for next target
            self.wp_index = i + 1
        return self.path_points[-1]

    # ----------------------------------------------------
    def control_loop(self, event):
        if not self.path_points:
            return

        goal_x, goal_y = self.path_points[-1]
        x_d, y_d = self.find_lookahead_point()
        dx = x_d - self.x
        dy = y_d - self.y

        if self.wp_index != self._last_log_wp:
            self._last_log_wp = self.wp_index
            dg = np.hypot(goal_x - self.x, goal_y - self.y)
            print("[trace] waypoint {}/{} target=({:.2f}, {:.2f}) pos=({:.2f}, {:.2f}) dist_to_goal={:.2f}m".format(
                self.wp_index, len(self.path_points) - 1, x_d, y_d, self.x, self.y, dg), flush=True)
        self._trace_step += 1
        if self._trace_step % self._log_interval == 0:
            dg = np.hypot(goal_x - self.x, goal_y - self.y)
            print("[trace] step {} pos=({:.2f}, {:.2f}) target=({:.2f}, {:.2f}) dist_to_goal={:.2f}m v={:.2f}".format(
                self._trace_step, self.x, self.y, x_d, y_d, dg, self.v), flush=True)
        if self._stuck_warn_step < 0 and self._trace_step >= 50 and abs(self.x) < 1e-3 and abs(self.y) < 1e-3 and abs(self.v) > 0.05:
            self._stuck_warn_step = self._trace_step
            print("[trace] WARNING: position stuck at (0,0) — odometry may not be updating. Ensure /turtlebot3_leader/odom is published. If Gazebo uses /odom, run with _odom_topic:=/odom", flush=True)

        if np.hypot(goal_x - self.x, goal_y - self.y) <= self.goal_tol:
            self.v = 0.0
            self.omega = 0.0
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            print("[trace] goal reached at pos=({:.2f}, {:.2f}) after {} steps".format(self.x, self.y, self._trace_step), flush=True)
            return

        # Fix 5: desired velocity = constant speed toward lookahead (not dx/T)
        dist = np.hypot(dx, dy)
        if dist > 1e-6:
            scale = self.desired_speed / dist
            x_d_dot = scale * dx
            y_d_dot = scale * dy
        else:
            # At or past lookahead point; drive slowly forward to avoid getting stuck
            x_d_dot = self.desired_speed * 0.5 * np.cos(self.theta)
            y_d_dot = self.desired_speed * 0.5 * np.sin(self.theta)

        theta_d = np.arctan2(dy, dx)

        # Velocity state (xdot, ydot) comes from odom_callback; no integration here.

        # Errors
        ex = self.x - x_d
        ey = self.y - y_d
        ex_dot = self.xdot - x_d_dot
        ey_dot = self.ydot - y_d_dot

        Sx = ex_dot + self.lambda_gain * ex
        Sy = ey_dot + self.lambda_gain * ey

        reach_x = -self.K_dis * np.tanh(Sx) - self.k_con * Sx
        reach_y = -self.K_dis * np.tanh(Sy) - self.k_con * Sy

        u1 = self.m * (reach_x * np.cos(theta_d) + reach_y * np.sin(theta_d))
        # Fix 4: wrap angle error to [-pi, pi]
        theta_err = np.arctan2(np.sin(self.theta - theta_d), np.cos(self.theta - theta_d))
        u2 = -2.0 * theta_err

        self.F = u1
        self.tau = u2

        self.v += (self.F / self.m) * self.T
        self.omega = self.tau

        self.v = np.clip(self.v, 0.0, self.max_linear)
        self.omega = np.clip(self.omega, -self.max_angular, self.max_angular)

        # Fix 6: xdot/ydot come from odom only (no integration drift); clipping handled by next odom update.

        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = self.omega
        self.pub.publish(cmd)


# ----------------------------------------------------
if __name__ == '__main__':
    try:
        LeaderSMC()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
