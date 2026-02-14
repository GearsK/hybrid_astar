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

        # Internal dynamics
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
        self.lookahead_dist = 0.4   # 🔑 KEY FIX

        self.timer = rospy.Timer(rospy.Duration(self.T), self.control_loop)

    # ----------------------------------------------------
    def path_callback(self, msg):
        self.path_points = []
        self.wp_index = 0

        for p in msg.poses:
            self.path_points.append(
                (p.pose.position.x, p.pose.position.y)
            )

        rospy.loginfo(f"Received path with {len(self.path_points)} points")

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

        x_d_dot = dx / self.T
        y_d_dot = dy / self.T

        x_d_ddot = 0.0
        y_d_ddot = 0.0

        theta_d = np.arctan2(dy, dx)

        # Dynamics
        xdd = (self.F/self.m) * np.cos(self.theta)
        ydd = (self.F/self.m) * np.sin(self.theta)

        self.xdot += xdd * self.T
        self.ydot += ydd * self.T

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
        u2 = -2.0 * (self.theta - theta_d)

        self.F = u1
        self.tau = u2

        self.v     += (self.F/self.m) * self.T
        self.omega  = self.tau

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

