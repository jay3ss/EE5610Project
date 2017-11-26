from __future__ import division

from threading import Lock

import math
import numpy as np
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from kalman_filter_state_estimation.msg import State

from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

from kalman_filter_state_estimation.kalman import Kalman

class ROSKalman(Kalman):
    """ROS wrapper for the Kalman and DeadReckoning classes"""
    def __init__(self):
        """Constructor"""
        rospy.init_node('kalman')
        prediction_rate = rospy.get_param('~prediction_rate', 50)
        # correction_rate = rospy.get_param('~correction_rate', 13)
        self.rate = rospy.Rate(prediction_rate)
        self.header = Header()

        # name = '/joint_states'
        # joint_state_topic = rospy.get_param('~joint_state_topic', name)
        # joint_state_cb = rospy.Subscriber(
        #     joint_state_topic, JointState, self.joint_state_cb)
        #
        # name = '/dead_reckoning/odom'
        # odom_sub_topic = rospy.get_param('~odom_sub_topic', name)
        # self.odom_sub = rospy.Subscriber(odom_sub_topic, Odometry, self.odom_cb)

        name = '/state'
        state_sub_topic = rospy.get_param('~state_sub_topic', name)
        self.state_sub = rospy.Subscriber(state_sub_topic, State, self.state_cb)

        odom_pub_topic = '/kalman/odom'
        self.odom_pub = rospy.Publisher(odom_pub_topic, Odometry, queue_size=100)
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.odom_child_frame_id = rospy.get_param(
            '~odom_child_frame_id', 'base_link')

        super(ROSKalman, self).__init__(1/prediction_rate)

        self.wheel_radius = rospy.get_param('~wheel_radius', 0.035)
        self.wheel_distance = rospy.get_param('~wheel_distance', 0.230)

        self.data_lock = Lock()

        # self.throttle = 20
        # self.num_js_calls = 0

    def get_x(self):
        return self.x.item(0)

    def get_xdot(self):
        return self.x.item(3)

    def get_y(self):
        return self.x.item(1)

    def get_ydot(self):
        return self.x.item(4)

    def get_theta(self):
        return self.x.item(2)

    def get_thetadot(self):
        return self.x.item(5)

    # def joint_state_cb(self, data):
    #     self.num_js_calls += 1
    #     x = self.get_x()
    #     y = self.get_y()
    #     theta = self.get_theta()
    #
    #     v_l = data.velocity[0]
    #     v_r = data.velocity[1]
    #     (lin_vel, ang_vel) = self.update_velocities(v_r, v_l)
    #     x_dot = lin_vel * math.cos(theta)
    #     y_dot = lin_vel * math.sin(theta)
    #
    #     # Measurement vector
    #     measurement = np.array([
    #         # [x], [y], [theta],
    #         [0.0], [0.0], [0.0],
    #         [x_dot], [y_dot], [ang_vel]
    #     ])
    #
    #     print('Measurement')
    #     print('x_dot: %f\ty_dot: %f\ttheta_dot: %f'%(x_dot, y_dot, ang_vel))
        # if self.num_js_calls % self.throttle == 0:
        #     k = self.calculateKalmanGain(self.sigma, self.H, self.R)
        #
        #     self.x = self.correctState(measurement, self.x, k, self.H)
        #     self.sigma = self.correctCovariance(self.sigma, k, self.H)

        # self.publish_state()

    # def odom_cb(self, data):
    #     # print('In odom_cb')
    #     x = data.pose.pose.position.x
    #     y = data.pose.pose.position.y
    #     qx = data.pose.pose.orientation.x
    #     qy = data.pose.pose.orientation.y
    #     qz = data.pose.pose.orientation.z
    #     qw = data.pose.pose.orientation.w
    #     (r, p, yaw) = euler_from_quaternion([qx, qy, qz, qw])
    #
    #     xdot = self.get_xdot()
    #     ydot = self.get_ydot()
    #     thetadot = self.get_thetadot()
    #     # Measurement vector
    #     measurement = np.array([[x], [y], [yaw], [0], [0], [0]])
    #
    #     k = self.calculateKalmanGain(self.sigma, self.H, self.R)
    #
    #     self.x = self.correctState(measurement, self.x, k, self.H)
    #     self.sigma = self.correctCovariance(self.sigma, k, self.H)

        # self.publish_state()

    def state_cb(self, data):
        with self.data_lock:
            self.header.stamp = rospy.Time.now()
        xn = data.pose.position.x
        yn = data.pose.position.y

        qx = data.pose.orientation.x
        qy = data.pose.orientation.y
        qz = data.pose.orientation.z
        qw = data.pose.orientation.w

        x_dot = data.velocity.linear.x
        y_dot = data.velocity.linear.y
        theta_dot = data.velocity.angular.z

        (r, p, yaw) = euler_from_quaternion([qx, qy, qz, qw])

        measurement = np.array([
            [xn], [yn], [yaw],
            [x_dot], [y_dot], [theta_dot]
        ])

        k = self.calculateKalmanGain(self.sigma, self.H, self.R)

        self.x = self.correctState(measurement, self.x, k, self.H)
        self.sigma = self.correctCovariance(self.sigma, k, self.H)

        print('State')
        print('x: %f\ty: %f\t theta: %f'%(xn, yn, yaw))
        print('x_dot: %f\ty_dot: %f\t theta_dot: %f'%(x_dot, y_dot, theta_dot))


    def publish_state(self):
        odom = Odometry()
        # odom.header = self.header
        with self.data_lock:
            odom.header.stamp = rospy.Time.now()
        # odom.header.stamp = rospy.get_rostime()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.odom_child_frame_id
        odom.pose.pose.position.x = self.get_x()
        odom.pose.pose.position.y = self.get_y()
        quat = quaternion_from_euler(0, 0, self.get_theta())
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = self.get_xdot()
        odom.twist.twist.linear.y = self.get_ydot()
        odom.twist.twist.angular.z = self.get_thetadot()
        self.odom_pub.publish(odom)
        # print('x: %f\ty: %f\t yaw: %f'%(self.get_x(), self.get_y(), self.get_theta()))

    def run(self):
        while not rospy.is_shutdown():
            self.state_callback()
            self.publish_state()
            self.rate.sleep()
        # A = self.A
        # x = self.x
        # self.predictState(A, x)

    # def update_velocities(self, v_r, v_l):
    #     """Updates the velocities."""
    #     # v_r *= math.pi/180
    #     # v_l *= math.pi/180
    #     v_r *= self.wheel_radius
    #     v_l *= self.wheel_radius
    #     return (self._calc_lin_vel(v_r, v_l), self._calc_ang_vel(v_r, v_l))
    #
    # def _calc_ang_vel(self, v_r, v_l):
    #     """Private method to calculate the angular velocity
    #
    #     w = (v_r - v_l) / l # angular velocity (l is separation of wheels)
    #     """
    #     return (v_r - v_l) / self.wheel_distance
    #     # print self.angular_velocity
    #
    # def _calc_lin_vel(self, v_r, v_l):
    #     """Private method to calculate the linear velocity
    #
    #     v = (v_r + v_l) / 2 # linear velocity
    #     """
    #     return (v_r + v_l) / 2
