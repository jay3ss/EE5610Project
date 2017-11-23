"""Module for the differential steering model"""
import numpy as np

class State(object):
    """Class for the model parameters of the differential steering robot"""
    def __init__(self, wheel_radius, wheel_distance):
        self.state = np.zeros((3, 1), dtype=float)
        self.angular_velocity = 0.0
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance

    def get_x(self):
        """Returns x state variable"""
        return self.state.item(0)

    def get_y(self):
        """Returns y state variable"""
        return self.state.item(1)

    def get_state(self):
        """Returns the state as a numpy array"""
        return self.state

    def get_theta(self):
        """Returns theta state variable"""
        return self.state.item(2)

    def set_x(self, x):
        self.state[0] = x

    def set_y(self, y):
        self.state[0] = y

    def set_theta(self, theta):
        self.state[0] = theta

    def update(self, x, y, theta):
        self.state = np.array([[x], [y], [theta]], dtype=float)
