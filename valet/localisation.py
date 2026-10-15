"""Pose estimation: GPS + wheel odometry + IMU yaw, fused by a Kalman filter.

Process model (dead reckoning with the measured speed v and IMU yaw psi):
    x_k = x_{k-1} + v dt cos(psi),   y_k = y_{k-1} + v dt sin(psi)
    P_k = P_{k-1} + Q dt             (odometry drift, same for x and y)
Measurement model:   z = [x, y] + n,  n ~ N(0, R I)   (GPS)
Update:              K = P / (P + R);  x <- x + K (z - x);  P <- (1 - K) P

x and y are decoupled because the yaw comes from the IMU, so two scalar
filters with a shared variance P are exact and very cheap. A gate rejects
GPS fixes whose innovation is above 5 sigma (multipath near buildings).
"""
import math


class PoseFilter:
    def __init__(self, gps_sigma=0.15, drift_sigma=0.03, gate=5.0):
        self.R = gps_sigma ** 2
        self.Q = drift_sigma ** 2
        self.gate = gate
        self.x = self.y = None
        self.P = self.R

    def update(self, gps_x, gps_y, yaw, v, dt):
        if self.x is None:
            self.x, self.y = gps_x, gps_y
            return self.x, self.y
        # predict
        self.x += v * dt * math.cos(yaw)
        self.y += v * dt * math.sin(yaw)
        self.P += self.Q * dt
        # correct (with innovation gate)
        ix, iy = gps_x - self.x, gps_y - self.y
        S = self.P + self.R
        if ix * ix + iy * iy <= (self.gate ** 2) * S:
            K = self.P / S
            self.x += K * ix
            self.y += K * iy
            self.P *= (1 - K)
        return self.x, self.y

    @property
    def sigma(self):
        return math.sqrt(self.P)
