"""Utility module"""
from __future__ import division
import math

# def integrate(prev, integrand, dt):
#     return prev + integrand*dt

def differntiate(prev, curr, dt):
    """Simple differentiation"""
    return (curr - prev) / dt

def integrate(xn_1, yn_1, theta_n_1, lin_vel, ang_vel, dt):
    """Fourth-order Runga-Kutta numerical integration. Implementation taken from
    https://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/s07/labs/NXTLabs/Lab%203.html

    Args:
        prev_state: previous state (numpy array)
            prev_state[0]: x
            prev_state[1]: y
            prev_state[2]: theta
        lin_vel: current linear velocity
        ang_vel: current angular velocity
        dt: time step

    Returns:
        Current state (numpy array)

    k00 = lin_vel*cos(theta_n_1)
    k01 = lin_vel*sin(theta_n_1)
    k02 = ang_vel

    k10 = lin_vel*cos(theta_n_1 + dt/2*k02)
    k11 = lin_vel*sin(theta_n_1 + dt/2*k02)
    k12 = ang_vel

    k20 = lin_vel*cos(theta_n_1 + dt/2*k12)
    k21 = lin_vel*sin(theta_n_1 + dt/2*k12)
    k22 = ang_vel

    k30 = lin_vel*cos(theta_n_1 + dt*k22)
    k31 = lin_vel*sin(theta_n_1 + dt*k22)
    k32 = ang_vel

    And the solution to the differential equation is:

    xn      = xn_1      + dt/6*(k00 + 2*(k10 + k20) + k30)
    yn      = yn_1      + dt/6*(k01 + 2*(k11 + k21) + k31)
    theta_n = theta_n_1 + dt/6*(k02 + 2*(k12 + k22) + k32)
    """
    # print('Previous state:')
    # print(xn_1, yn_1, theta_n_1, lin_vel, ang_vel)

    k00 = lin_vel*math.cos(theta_n_1)
    k01 = lin_vel*math.sin(theta_n_1)
    k02 = ang_vel

    k10 = lin_vel*math.cos(theta_n_1 + dt/2*k02)
    k11 = lin_vel*math.sin(theta_n_1 + dt/2*k02)
    k12 = ang_vel

    k20 = lin_vel*math.cos(theta_n_1 + dt/2*k12)
    k21 = lin_vel*math.sin(theta_n_1 + dt/2*k12)
    k22 = ang_vel

    k30 = lin_vel*math.cos(theta_n_1 + dt*k22)
    k31 = lin_vel*math.sin(theta_n_1 + dt*k22)
    k32 = ang_vel

    xn      = xn_1      + dt/6*(k00 + 2*(k10 + k20) + k30)
    yn      = yn_1      + dt/6*(k01 + 2*(k11 + k21) + k31)
    theta_n = theta_n_1 + dt/6*(k02 + 2*(k12 + k22) + k32)

    # print('Current state:')
    # print("x: %r\ty: %r\ttheta: %r"%(xn, yn, theta_n))

    return (xn, yn, theta_n)
