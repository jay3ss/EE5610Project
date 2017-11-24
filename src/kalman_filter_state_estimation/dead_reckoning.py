"""Contains the code for dead reckoning"""
from __future__ import division
# from threading import Lock
import math

from tf.transformations import euler_from_quaternion
# from tf.transformations import euler_matrix

from kalman_filter_state_estimation.utils import integrate
from kalman_filter_state_estimation.diff_steering import State
import kalman_filter_state_estimation.utils as utils

class DeadReckoning(object):
    """Dead reckoning class"""
    def __init__(self, wheel_radius, wheel_distance, dt, init_state):
        """Constructor"""
        self.right_velocity = 0.0
        self.left_velocity = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.x_velocity = 0.0
        self.y_velocity = 0.0
        self.yaw = 0.0
        self.dt = dt

        self.state = State(wheel_radius, wheel_distance, init_state)

    def _calc_ang_vel(self, v_r, v_l):
        """Private method to calculate the angular velocity

        w = (v_r - v_l) / l # angular velocity (l is separation of wheels)
        """
        self.angular_velocity = (v_r - v_l) / self.state.wheel_distance
        # print self.angular_velocity

    def _calc_lin_vel(self, v_r, v_l):
        """Private method to calculate the linear velocity

        v = (v_r + v_l) / 2 # linear velocity
        """
        self.linear_velocity = (v_r + v_l) / 2
        # print('vr: %f\tvl: %f\tlin_vel: %f'%(v_r, v_l, self.linear_velocity))


    def compute_dynamics(self):
        """Computes dynamics (deprecated). Use update_velocities() instead."""
        pass

    def compute_state(self, x_n_1, y_n_1, theta_n_1):
        pass

    def update_state(self, linear_velocity, angular_velocity):
        xn_1 = self.state.get_x()
        yn_1 = self.state.get_y()
        theta_n_1 = self.state.get_theta() #* math.pi/180
        dt = self.dt

        (x, y, theta) = utils.integrate(xn_1, yn_1, theta_n_1,
            linear_velocity, angular_velocity, dt)
        self.state.update(x, y, theta)

    def update_velocities(self, v_r, v_l):
        """Updates the velocities."""
        # v_r *= math.pi/180
        # v_l *= math.pi/180
        v_r *= self.state.wheel_radius
        v_l *= self.state.wheel_radius
        self._calc_lin_vel(v_r, v_l)
        self._calc_ang_vel(v_r, v_l)
