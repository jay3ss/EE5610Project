import numpy as np

class Kalman(object):

    def __init__(self, dt):
        # dt = 0.005

        #State-transition model
        self.A = np.array([
            [1,0,dt,0,0,0],
            [0,1,0,dt,0,0],
            [0,0,1,0,dt,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]
        ])
        #Observation model
        self.H = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]
        ])
        # self.H = np.identity(6)

        #Process/State noise
        vel_noise_std = 2
        pos_noise_std = 5
        self.Q = np.array([
            [pos_noise_std*pos_noise_std,0,0,0,0,0],
            [0,pos_noise_std*pos_noise_std,0,0,0,0],
            [0,0,pos_noise_std*pos_noise_std,0,0,0],
            [0,0,0,vel_noise_std*vel_noise_std,0,0],
            [0,0,0,0,vel_noise_std*vel_noise_std,0],
            [0,0,0,0,0,vel_noise_std*vel_noise_std]
        ])

        #Sensor/Measurement noise
        measurement_noise_std = 15
        self.R = measurement_noise_std * measurement_noise_std * np.identity(6)

        self.x = np.zeros((6,1)) #Initial state vector [x,y,th,vx,vy,vth]
        self.sigma = np.identity(6) #Initial covariance matrix

    def predictState(self, A, x):
        '''
        :param A: State-transition model matrix
        :param x: Current state vector
        :return x_p: Predicted state vector as 4x1 numpy array
        '''
        x_p = np.dot(A, x)

        return x_p

    def predictCovariance(self, A, sigma, Q):
        sigma_p = np.dot(np.dot(A, sigma), np.transpose(A))+Q
        return sigma_p

    def calculateKalmanGain(self, sigma_p, H, R):
        k = np.dot(
            np.dot(sigma_p, np.transpose(H)),
            np.linalg.inv(
                np.dot(H, np.dot(sigma_p, np.transpose(H)))+R))
        return k

    def correctState(self, z, x_p, k, H):
        '''
        :param z: Measurement vector
        :param x_p: Predicted state vector
        :param k: Kalman gain
        :param H: Observation model
        :return x: Corrected state vector as 6x1 numpy array
        '''
        x = x_p + np.dot(k, (z - np.dot(H, x_p)))

        return x

    def correctCovariance(self, sigma_p, k, H):
        sigma = np.dot((np.identity(6)-np.dot(k, H)), sigma_p)
        # print(sigma)
        return sigma

    def state_callback(self):
        self.x = self.predictState(self.A, self.x)
        self.sigma = self.predictCovariance(self.A, self.sigma, self.Q)
