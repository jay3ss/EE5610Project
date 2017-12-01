from __future__ import division

from threading import Lock

import rospy
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from kalman_filter_state_estimation.dead_reckoning import DeadReckoning


class ROSDeadReckoning(DeadReckoning):
    """ROS wrapper for the DeadReckoning class"""
    # def __init__(self, wheel_radius=0.1, wheel_distance=0.1, dt=10):
    def __init__(self):
        rospy.init_node('dead_reckoning', anonymous=True)
        rospy.loginfo('dead_reckoning node started')

        self.data_lock = Lock()

        update_rate = rospy.get_param('~update_rate', 5)
        self.rate = rospy.Rate(update_rate)

        # IMU information
        # Not using IMU information anymore
        name = '/noisy/imu'
        imu_topic = rospy.get_param('~imu_topic', name)
        imu_sub = rospy.Subscriber(imu_topic, Imu, self.imu_cb)

        # JointState information
        name = '/noisy/joint_states'
        imu_topic = rospy.get_param('~joint_state_topic', name)
        joint_state_sub = rospy.Subscriber(
            imu_topic, JointState, self.joint_state_cb)

        # Odometry information
        name = '/dead_reckoning/odom'
        odom_topic = rospy.get_param('~odom_topic', name)
        self.odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=5)
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.odom_child_frame_id = rospy.get_param(
            '~odom_child_frame_id', 'base_link')

        init_x = rospy.get_param('~initial_x', 0.0)
        init_y = rospy.get_param('~initial_y', 0.0)
        init_theta = rospy.get_param('~initial_theta', -0.08946428280993846)
        init_state = (init_x, init_y, init_theta)

        # Differential steering model information
        # Defaults used are from the turtlebot.urdf file found in the (modified)
        # turtlebot_description package
        wheel_radius = rospy.get_param('~wheel_radius', 0.035)
        wheel_distance = rospy.get_param('~wheel_distance', 0.230)
        dt = 1 / update_rate

        self.odom = Odometry()
        self.odom.header.frame_id = self.odom_frame_id
        self.odom.child_frame_id = self.odom_child_frame_id

        super(ROSDeadReckoning, self).__init__(
            wheel_radius, wheel_distance, dt, init_state)

    def imu_cb(self, data):
        with self.data_lock:
            # self.odom.header.stamp = data.header.stamp
            self.odom.header.stamp = rospy.Time.now()
        self.angular_velocity = data.angular_velocity.z
        qx = data.orientation.x
        qy = data.orientation.y
        qz = data.orientation.z
        qw = data.orientation.w
        (r, p, theta) = euler_from_quaternion([qx, qy, qz, qw])
        self.state.set_theta(theta)

    def joint_state_cb(self, data):
        # v_l = self.state.wheel_radius * data.velocity[0]
        # v_r = self.state.wheel_radius * data.velocity[1]
        with self.data_lock:
            self.odom.header.stamp = data.header.stamp
            # self.odom.header.stamp = rospy.Time.now()
        v_l = data.velocity[0]
        v_r = data.velocity[1]
        self.update_velocities(v_r, v_l)
        # self.update()

    def update(self):
        lin_vel = self.linear_velocity
        ang_vel = self.angular_velocity
        self.update_state(lin_vel, ang_vel)
        # odom = Odometry()
        # self.odom.header.stamp = rospy.Time.now()
        # self.odom.header.frame_id = self.odom_frame_id
        # self.odom.child_frame_id = self.odom_child_frame_id
        self.odom.pose.pose.position.x = self.state.get_x()
        self.odom.pose.pose.position.y = self.state.get_y()
        yaw = self.state.get_theta()
        quat = quaternion_from_euler(0, 0, yaw)
        self.odom.pose.pose.orientation.z = quat[2]
        self.odom.pose.pose.orientation.w = quat[3]
        self.odom_pub.publish(self.odom)

    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()
        # rospy.spin()
