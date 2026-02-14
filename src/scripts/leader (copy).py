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
        self.path_msg.header.frame_id = "map"

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

        # Path storage
        self.path_points = []
        self.wp_index = 0
        self.goal_tol = 0.15

        self.timer = rospy.Timer(rospy.Duration(self.T), self.control_loop)

    # ----------------------------------------------------
    def path_callback(self, msg):
        self.path_points = [(p.pose.position.x, p.pose.position.y)
                            for p in msg.poses]
        self.wp_index = 0

    # ----------------------------------------------------
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.theta = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])[2]

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose = msg.pose.pose
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

    # ----------------------------------------------------
    def control_loop(self, event):
        if not self.path_points or self.wp_index >= len(self.path_points):
            return

        # Current waypoint
        x_d, y_d = self.path_points[self.wp_index]

        # Waypoint switching
        if np.hypot(self.x - x_d, self.y - y_d) < self.goal_tol:
            self.wp_index += 1
            return

        # Finite-difference derivatives (NO equation change)
        if self.wp_index > 0:
            x_prev, y_prev = self.path_points[self.wp_index - 1]
        else:
            x_prev, y_prev = self.x, self.y

        x_d_dot = (x_d - x_prev) / self.T
        y_d_dot = (y_d - y_prev) / self.T

        x_d_ddot = 0.0
        y_d_ddot = 0.0

        # Orientation update (UNCHANGED)
        self.theta = np.arctan2(y_d_dot, x_d_dot)

        # Dynamics update (UNCHANGED)
        xdd = -self.v * self.omega * np.sin(self.theta) - self.L * self.omega**2 * np.cos(self.theta) + \
              (self.F/self.m)*np.cos(self.theta) - ((self.tau*self.L)/self.I)*np.sin(self.theta)

        ydd = self.v * self.omega * np.cos(self.theta) - self.L * self.omega**2 * np.sin(self.theta) + \
              (self.F/self.m)*np.sin(self.theta) + ((self.tau*self.L)/self.I)*np.cos(self.theta)

        self.xdot += xdd * self.T
        self.ydot += ydd * self.T

        # SMC errors (UNCHANGED)
        ex = self.x - x_d
        ey = self.y - y_d
        ex_dot = self.xdot - x_d_dot
        ey_dot = self.ydot - y_d_dot

        Sx = ex_dot + self.lambda_gain * ex
        Sy = ey_dot + self.lambda_gain * ey

        reach_x = -self.K_dis * np.tanh(Sx) - self.k_con * Sx
        reach_y = -self.K_dis * np.tanh(Sy) - self.k_con * Sy

        # Control laws (UNCHANGED)
        u1 = self.m * np.cos(self.theta) * (self.v * self.omega * np.sin(self.theta) + self.L * self.omega**2 * np.cos(self.theta)) \
           - self.m * np.sin(self.theta) * (self.v * self.omega - self.L * self.omega**2 * np.sin(self.theta)) \
           + self.m * (np.cos(self.theta) * x_d_ddot + np.sin(self.theta) * y_d_ddot) \
           - self.m * self.lambda_gain * (np.cos(self.theta)*ex_dot + np.sin(self.theta)*ey_dot) \
           + self.m * (np.cos(self.theta)*reach_x + np.sin(self.theta)*reach_y)

        u2 = -(self.I/self.L) * (np.sin(self.theta)*(self.v * self.omega * np.sin(self.theta) + self.L * self.omega**2 * np.cos(self.theta)) \
           + np.cos(self.theta)*(self.v * self.omega - self.L * self.omega**2 * np.sin(self.theta))) \
           - (self.I/self.L) * (np.sin(self.theta)*x_d_ddot - np.cos(self.theta)*y_d_ddot) \
           + (self.I/self.L) * self.lambda_gain * (np.sin(self.theta)*ex_dot - np.cos(self.theta)*ey_dot) \
           - (self.I/self.L) * (np.sin(self.theta)*reach_x - np.cos(self.theta)*reach_y)

        self.F = u1
        self.tau = u2

        # Force/Torque → Velocity (ONLY ADDITION)
        self.v     += (self.F/self.m) * self.T
        self.omega += (self.tau/self.I) * self.T

        self.v     = np.clip(self.v, 0.0, 0.5)
        self.omega = np.clip(self.omega, -1.5, 1.5)

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

