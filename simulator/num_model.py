import numpy as np
import scipy.linalg

class Drone:
    def __init__(self):
        self.dt = 2E-4
        self.time = 0.0
        self.g = 9.80
        self.gvec = np.array([0., 0., -self.g])
        self.M = 1.250
        self.R = 0.23
        self.Iz = 0.25 * self.M * self.R**2
        self.Ixy = self.Iz * 0.5
        self.I = np.diag([self.Ixy, self.Ixy, self.Iz])
        self.LIFT_K = 0.01
        self.TDRAG_K = 0.0
        self.DRAG_B = 0.5

        self.noise_acc = 0.07
        self.noise_omega = 0.02

        self.pos = np.zeros(3)
        self.rot = np.eye(3)
        self.vel = np.zeros(3)
        self.omega = np.zeros(3)
        self.acc_sensor = np.zeros(3)
        self.motor = np.zeros(4)

        rz = self.R * (2. ** -0.5)
        self.ppos = [
            np.array([rz, rz, 0.]),
            np.array([rz, -rz, 0.]),
            np.array([-rz, -rz, 0.]),
            np.array([-rz, rz, 0.]),
        ]
        self.pdir = [-1., 1., -1., 1.]

    def invrot(self):
        return self.rot.T

    def diff_matrix(self, omega, dt):
        olen = np.linalg.norm(omega)
        wx, wy, wz = omega / olen
        th = olen * dt
        K = np.array([
            [0., -wz, wy], 
            [wz, 0., -wx], 
            [-wy, wx, 0.],
        ])
        return np.eye(3) + np.sin(th) * K + (1. - np.cos(th)) * np.dot(K, K)
            # Rodrigue's formula; equivalent to exponential map exp(th*K)

    def lift(self, pomega):
        return self.LIFT_K * pomega

    def force(self, lifts): # internal frame
        f = np.array([0., 0., sum(lifts)])
        f -= self.DRAG_B * np.dot(self.invrot(), self.vel)
        return f

    def torque(self, lifts, pomega): # internal frame
        tau = np.zeros(3)
        for i in range(4):
            lf = np.array([0., 0., lifts[i]])
            tau += np.cross(self.ppos[i], lf)
        return tau

    def set_motors(self, motor):
        self.motor = motor

    def step(self):
        pomega = self.motor
        rot = self.rot
        lifts = [self.lift(x) for x in pomega]
        force_int = self.force(lifts)
        torque_int = self.torque(lifts, pomega)
        force_ref = np.dot(rot, force_int) + self.M * self.gvec
        torque_ref = np.dot(rot, torque_int)
        I_ref = np.dot(rot, self.I)
        omega_ref = self.omega
        
        self.acc_sensor = force_int / self.M
        acc_ref = force_ref / self.M
        rotacc_ref = np.dot(
            np.linalg.inv(I_ref),
            torque_ref - np.cross(omega_ref, np.dot(I_ref, omega_ref))
        )

        dmx = self.diff_matrix(self.omega + rotacc_ref * self.dt / 2., self.dt)
        self.rot = np.dot(dmx, self.rot)
        self.pos += self.vel * self.dt + acc_ref * self.dt**2 / 2.
        self.vel += acc_ref * self.dt
        self.omega += rotacc_ref * self.dt
        self.time += self.dt

    def get_time(self):
        return self.time

    def get_sensors(self):
        acc = self.acc_sensor + np.random.normal(scale=self.noise_acc)
        omega = np.dot(self.invrot(), self.omega) + np.random.normal(scale=self.noise_omega)
        return acc, omega

    def get_position(self):
        return self.pos

    def get_orientation(self):
        return np.dot(self.rot, np.array([0., 0., 1.]))
    
    def set_init(self, vel, omega):
        self.vel = np.array(vel, dtype=np.float64)
        self.omega = np.array(omega, dtype=np.float64)

if __name__ == '__main__':
    drone = Drone()
    drone.step()

