#!/usr/bin/env python3
import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry


class LeaderSMC:
    def __init__(self):
        rospy.init_node('leader_smc')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('/sPath', Path, self.path_callback)

        self.path_pub = rospy.Publisher('/leader/path', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        # TF
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0)  # IMPORTANT: allow TF buffer to fill

        # Robot states
        self.x, self.y = 0.0, 0.0
        self.theta = 0.0
        self.v, self.omega = 0.0, 0.0

        # Internal dynamics (UNCHANGED)
        self.xdot, self.ydot = 0.0, 0.0
        self.F, self.tau = 0.0, 0.0

        # Constants (UNCHANGED)
        self.L = 0.1
        self.m, self.I = 1.0, 1.0
        self.T = 0.05

        # SMC parameters (UNCHANGED)
        self.lambda_gain = 2
        self.K_dis = 3.5
        self.k_con = 1.5

        # Path
        self.path_points = []
        self.wp_index = 0
        self.goal_tol = 0.25  # slightly larger for real motion

        self.timer = rospy.Timer(rospy.Duration(self.T), self.control_loop)

    # ----------------------------------------------------
    def path_callback(self, msg):
        self.path_points = []
        self.wp_index = 0

        source_frame = msg.header.frame_id if msg.header.frame_id else "map"

        for pose in msg.poses:
            try:
                pose.header.frame_id = source_frame
                pose_odom = self.tf_listener.transformPose("odom", pose)

                x = pose_odom.pose.position.x
                y = pose_odom.pose.position.y
                self.path_points.append((x, y))

            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                pass

        rospy.loginfo(f"Received path with {len(self.path_points)} points")

    # ----------------------------------------------------
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.theta = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])[2]

    # ----------------------------------------------------
    def control_loop(self, event):
        if not self.path_points or self.wp_index >= len(self.path_points):
            return

        # Skip already-reached waypoints
        while self.wp_index < len(self.path_points):
            dx = self.path_points[self.wp_index][0] - self.x
            dy = self.path_points[self.wp_index][1] - self.y
            if np.hypot(dx, dy) > self.goal_tol:
                break
            self.wp_index += 1

        if self.wp_index >= len(self.path_points):
            return

        x_d, y_d = self.path_points[self.wp_index]

        # Desired derivatives
        if self.wp_index > 0:
            x_prev, y_prev = self.path_points[self.wp_index - 1]
        else:
            x_prev, y_prev = self.x, self.y

        x_d_dot = (x_d - x_prev) / self.T
        y_d_dot = (y_d - y_prev) / self.T

        x_d_ddot = 0.0
        y_d_ddot = 0.0

        # Orientation (UNCHANGED)
        self.theta = np.arctan2(y_d_dot, x_d_dot)

        # Dynamics (UNCHANGED)
        xdd = (-self.v * self.omega * np.sin(self.theta)
               - self.L * self.omega**2 * np.cos(self.theta)
               + (self.F/self.m)*np.cos(self.theta)
               - ((self.tau*self.L)/self.I)*np.sin(self.theta))

        ydd = (self.v * self.omega * np.cos(self.theta)
               - self.L * self.omega**2 * np.sin(self.theta)
               + (self.F/self.m)*np.sin(self.theta)
               + ((self.tau*self.L)/self.I)*np.cos(self.theta))

        self.xdot += xdd * self.T
        self.ydot += ydd * self.T

        # Errors (UNCHANGED)
        ex = self.x - x_d
        ey = self.y - y_d
        ex_dot = self.xdot - x_d_dot
        ey_dot = self.ydot - y_d_dot

        Sx = ex_dot + self.lambda_gain * ex
        Sy = ey_dot + self.lambda_gain * ey

        reach_x = -self.K_dis * np.tanh(Sx) - self.k_con * Sx
        reach_y = -self.K_dis * np.tanh(Sy) - self.k_con * Sy

        # Control laws (UNCHANGED)
        u1 = (self.m * np.cos(self.theta) *
              (self.v * self.omega * np.sin(self.theta)
               + self.L * self.omega**2 * np.cos(self.theta))
              + self.m * (np.cos(self.theta)*reach_x
                          + np.sin(self.theta)*reach_y))

        u2 = (-(self.I/self.L) *
              (np.sin(self.theta)*reach_x
               - np.cos(self.theta)*reach_y))

        self.F = u1
        self.tau = u2

        # Force → velocity
        self.v += (self.F/self.m) * self.T
        self.omega += (self.tau/self.I) * self.T

        self.v = np.clip(self.v, 0.0, 0.5)
        self.omega = np.clip(self.omega, -1.5, 1.5)

        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = self.omega
        self.pub.publish(cmd)


if __name__ == '__main__':
    try:
        LeaderSMC()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

