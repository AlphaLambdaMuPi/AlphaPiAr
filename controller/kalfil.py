import numpy as np
from time import time as curtime

class KalmanFilter:
    def __init__(self, dimx, dimz, dimu):
        self.dimx = dimx
        self.dimz = dimz
        self.dimu = dimu
        self.x = np.zeros(dimx)
        self.u = np.zeros(dimu)
        self.S = np.eye(dimx) * 1E10
        self.time = 0

    def set_u(self, u):
        self.u = u

    def predict(self, time):
        dt = time - self.time
        if dt < 1E-3:
            return self.x
        self.time = time
        A = self.A(dt)
        Bf = self.Bf(dt, self.u)
        Q = self.Q(dt)
        self.x = np.dot(A, self.x) + Bf
        self.S = np.dot(np.dot(A, self.S), A.T) + Q
        return self.x

    def update(self, time, z):
        self.predict(time)
        C = self.C()
        R = self.R()
        t1 = z - np.dot(C, self.x)
        t2 = np.dot(self.S, C.T)
        t3 = np.linalg.pinv(np.dot(C, t2) + R)
        K = np.dot(t2, t3)
        self.x += np.dot(K, t1)
        self.S -= np.dot(np.dot(K, C), self.S)

    def setA(self, f):
        self.A = f
    def setBf(self, f):
        self.Bf = f
    def setC(self, f):
        self.C = f
    def setQ(self, f):
        self.Q = f
    def setR(self, f):
        self.R = f

class ThetaOmegaKalmanFilter(KalmanFilter):
    def __init__(self, theta_std, omega_std, time_const):
        super().__init__(2, 2, 1)
        def a(dt):
            return np.array([
                [1, dt],
                [0, 1]
            ])
        def bf(dt, u):
            return 0.
        def c():
            return np.eye(2)
        def q(dt):
            return np.array([
                [theta_std**2 * dt / time_const, 0.],
                [0., omega_std**2 * dt / time_const]
            ])
        def r():
            return np.array([
                [theta_std**2, 0.],
                [0., omega_std**2]
            ])
        self.setA(a)
        self.setBf(bf)
        self.setC(c)
        self.setQ(q)
        self.setR(r)

if __name__ == '__main__':
    kf = ThetaOmegaKalmanFilter(0.1, 0.1, 0.04)

