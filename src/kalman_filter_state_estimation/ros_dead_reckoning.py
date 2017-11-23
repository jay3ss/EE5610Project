from __future__ import division

import rospy
from tf.transformations import quaternion_from_euler

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

        update_rate = rospy.get_param('~update_rate', 5)
        self.rate = rospy.Rate(update_rate)

        # IMU information
        # Not using IMU information anymore
        name = '/mobile_base/sensors/imu_data'
        imu_topic = rospy.get_param('~imu_topic', name)
        imu_sub = rospy.Subscriber(imu_topic, Imu, self.imu_cb)

        # JointState information
        name = '/joint_states'
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

        # Differential steering model information
        # Defaults used are from the turtlebot.urdf file found in the (modified)
        # turtlebot_description package
        wheel_radius = rospy.get_param('~wheel_radius', 0.035)
        wheel_distance = rospy.get_param('~wheel_distance', 0.230)
        dt = 1 / update_rate

        super(ROSDeadReckoning, self).__init__(wheel_radius, wheel_distance, dt)

    def imu_cb(self, data):
        self.angular_velocity = data.angular_velocity.z

    def joint_state_cb(self, data):
        # v_l = self.state.wheel_radius * data.velocity[0]
        # v_r = self.state.wheel_radius * data.velocity[1]
        v_l = data.velocity[0]
        v_r = data.velocity[1]
        self.update_velocities(v_r, v_l)
        # self.update()

    def update(self):
        lin_vel = self.linear_velocity
        ang_vel = self.angular_velocity
        self.update_state(lin_vel, ang_vel)
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.odom_child_frame_id
        odom.pose.pose.position.x = self.state.get_x()
        odom.pose.pose.position.y = self.state.get_y()
        yaw = self.state.get_theta()
        quat = quaternion_from_euler(0, 0, yaw)
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        self.odom_pub.publish(odom)

    def run(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()
        # rospy.spin()
