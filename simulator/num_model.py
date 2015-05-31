import numpy as np

class Drone:
    def __init__(self):
        self.dt = 1E-5
        self.time = 0.0
        self.g = 9.80
        self.gvec = np.array([0, 0, -self.g])
        self.M = 1.250
        self.R = 0.23
        self.Iz = 0.25 * self.M * self.R**2
        self.Ixy = self.Iz * 0.5
        self.I = np.diag([self.Ixy, self.Ixy, self.Iz])
        self.LIFT_K = 0.01
        self.TDRAG_K = 0

        self.pos = np.eye(4)
        self.vel = np.zeros(3)
        self.omega = np.zeros(3)
        self.acc_sensor = np.zeros(3)
        self.motor = np.zeros(4)

        rz = self.R * (2 ** -0.5)
        self.ppos = [
            np.array([rz, rz, 0]),
            np.array([rz, -rz, 0]),
            np.array([-rz, -rz, 0]),
            np.array([-rz, rz, 0]),
        ]
        self.pdir = [-1, 1, -1, 1]

    def rot(self):
        return self.pos[:3, :3]

    def diff_matrix(self, dt):
        vx, vy, vz = self.vel * dt
        wx, wy, wz = self.omega * dt
        ret = np.array([
            [1, -wz, wy, vx], 
            [wz, 1, -wx, vy], 
            [-wy, wx, 1, vz],
            [0, 0, 0, 1.],
        ])
        return ret

    def lift(self, pomega):
        return self.LIFT_K * pomega

    def force(self, lifts):
        f = np.array([0, 0, sum(lifts)])
        return f

    def torque(self, lifts, pomega):
        tau = np.zeros(3)
        for i in range(4):
            lf = np.array([0, 0, lifts[i]])
            tau += np.cross(self.ppos[i], lf)
        return tau

    def set_motors(self, motor):
        self.motor = motor

    def step(self):
        pomega = self.motor
        rot = self.rot()
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

        dmx = self.diff_matrix(self.dt)
        self.pos = np.dot(dmx, self.pos)
        self.vel += acc_ref * self.dt
        self.omega += rotacc_ref * self.dt
        self.time += self.dt

    def get_time(self):
        return self.time

    def get_sensors(self):
        return self.acc_sensor, np.dot(np.linalg.inv(self.rot()), self.omega)

    def get_position(self):
        return self.pos[:3, 3]

    def get_orientation(self):
        return np.dot(self.rot(), np.array([0, 0, 1.0]))
    
    def set_init(self, vel, omega):
        self.vel = np.array(vel, dtype=np.float64)
        self.omega = np.array(omega, dtype=np.float64)

if __name__ == '__main__':
    drone = Drone()
    drone.step()

