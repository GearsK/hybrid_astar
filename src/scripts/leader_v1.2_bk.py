#!/usr/bin/env python3
import rospy
import numpy as np
import tf

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class LeaderSMC:

    def __init__(self):

        rospy.init_node('leader_smc')

        # Publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('/sPath', Path, self.path_callback)
        self.start_sub = rospy.Subscriber('/initialpose',
                                          PoseWithCovarianceStamped,
                                          self.start_callback)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.omega = 0.0

        # Path
        self.path_points = []
        self.wp_index = 0
        self.goal_tol = 0.15
        self.lookahead_dist = 0.4

        self.controller_enabled = False

        # Control frequency
        self.dt = 0.05
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.control_loop)

        rospy.loginfo("Hybrid A* Leader Controller Ready")

    # ----------------------------------------------------------
    def start_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation

        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        state = ModelState()
        state.model_name = "turtlebot3_waffle_pi"
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.orientation = orientation
        state.twist.linear.x = 0
        state.twist.angular.z = 0

        set_state(state)

        self.path_points = []
        self.wp_index = 0
        self.controller_enabled = False

        rospy.loginfo("Robot pose reset")

    # ----------------------------------------------------------
    def path_callback(self, msg):

        if len(msg.poses) == 0:
            return

        self.path_points = []
        self.wp_index = 0

        # If planner already publishes world coordinates
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            self.path_points.append((x, y))

        self.controller_enabled = True
        rospy.loginfo("Path received with %d points", len(self.path_points))

    # ----------------------------------------------------------
    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.theta = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])[2]

        self.omega = msg.twist.twist.angular.z

    # ----------------------------------------------------------
    def find_lookahead_point(self):

        for i in range(self.wp_index, len(self.path_points)):
            x_d, y_d = self.path_points[i]
            dist = np.hypot(self.x - x_d, self.y - y_d)

            if dist > self.lookahead_dist:
                self.wp_index = i
                return x_d, y_d

        return self.path_points[-1]

    # ----------------------------------------------------------
    def control_loop(self, event):

        if not self.controller_enabled:
            return

        if not self.path_points:
            return

        x_d, y_d = self.find_lookahead_point()

        dx = x_d - self.x
        dy = y_d - self.y
        dist = np.hypot(dx, dy)

        if dist < self.goal_tol:
            rospy.loginfo("Goal reached")
            self.controller_enabled = False
            return

        # Desired heading
        theta_d = np.arctan2(dy, dx)

        # Heading error (wrapped)
        e_theta = theta_d - self.theta
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

        # Sliding surface (SMC heading control)
        S = e_theta + 0.4 * self.omega

        # Angular velocity command
        omega_cmd = -2.5 * np.tanh(S)

        # Linear velocity command
        v_cmd = 0.20 * np.cos(e_theta)

        # Slow down in sharp turns
        turn_ratio = min(abs(omega_cmd) / 1.5, 1.0)
        speed_scale = 1.0 - 0.7 * turn_ratio
        speed_scale = max(speed_scale, 0.2)

        v_cmd *= speed_scale

        # Safety limits (TurtleBot3)
        v_cmd = np.clip(v_cmd, 0.0, 0.22)
        omega_cmd = np.clip(omega_cmd, -1.5, 1.5)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = omega_cmd

        self.pub.publish(cmd)


if __name__ == '__main__':
    try:
        LeaderSMC()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
