#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry


class LeaderSMC:
    def __init__(self):
        rospy.init_node('leader_smc')

        self.pub = rospy.Publisher('/turtlebot3_leader/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/turtlebot3_leader/odom', Odometry, self.odom_callback)
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
        self.desired_speed = 0.3  # Fix 5: constant desired speed toward lookahead

        self.timer = rospy.Timer(rospy.Duration(self.T), self.control_loop)

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
        rospy.loginfo(f"Received new path with {len(self.path_points)} points (frame: {frame})")

    # ----------------------------------------------------
    def odom_callback(self, msg):
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
        for i in range(self.wp_index, len(self.path_points)):
            px, py = self.path_points[i]
            if np.hypot(px - self.x, py - self.y) > self.lookahead_dist:
                self.wp_index = i
                return px, py
        return self.path_points[-1]

    # ----------------------------------------------------
    def control_loop(self, event):
        if not self.path_points:
            return

        x_d, y_d = self.find_lookahead_point()
        dx = x_d - self.x
        dy = y_d - self.y

        # Fix 3: stop when within goal_tol of final waypoint
        goal_x, goal_y = self.path_points[-1]
        if np.hypot(goal_x - self.x, goal_y - self.y) <= self.goal_tol:
            self.v = 0.0
            self.omega = 0.0
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            return

        # Fix 5: desired velocity = constant speed toward lookahead (not dx/T)
        dist = np.hypot(dx, dy)
        if dist > 1e-6:
            scale = self.desired_speed / dist
            x_d_dot = scale * dx
            y_d_dot = scale * dy
        else:
            x_d_dot = 0.0
            y_d_dot = 0.0

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

        self.v = np.clip(self.v, 0.0, 0.5)
        self.omega = np.clip(self.omega, -1.5, 1.5)

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
