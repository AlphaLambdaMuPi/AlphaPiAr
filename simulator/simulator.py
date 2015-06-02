#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

from pid import PIDController
from num_model import Drone

np.set_printoptions(precision=4, suppress=True)

    KPxy = 0.2
    KPz = 0.8
    KPt = 1.
    kp = np.vstack([np.diag([-KPxy, -KPxy, KPz]), np.zeros((3, 3))])
    kd = np.array([1.2]*3 + [0.05]*3)[np.newaxis].T * kp
    ki = 0.1 * kp
    ke = 0.9
    controller = PIDController(kp, kd, ki, ke)
    return controller

def get_pid2():
    KPxy = 5.
    KPz = 10.
    KPw = 10.
    # KPxy = 0.
    # KPz = 0.
    # KPw = 0.
    kpw1 = [KPxy, KPxy, KPz, KPw, -KPw, KPw]
    kpw2 = [KPxy, -KPxy, KPz, -KPw, -KPw, -KPw]
    kpw3 = [-KPxy, -KPxy, KPz, -KPw, KPw, KPw]
    kpw4 = [-KPxy, KPxy, KPz, KPw, KPw, -KPw]
    kp = np.array([kpw1, kpw2, kpw3, kpw4])
    kd = np.array([0.025]*3 + [0.02]*3) * kp
    ki = np.array([1/4]*3 + [1/8]*3) * kp
    ke = 0.9
    controller = PIDController(kp, kd, ki, ke)
    return controller

class Simulator:
    def __init__(self):
        self.drone = Drone()
        self.drone.set_init([0., 0., 0.], [2., 2., 0.])
        self.drone.dt = 5e-4
        # self.drone.noise_z = 1e-10

    def run(self):
        AAAO = []
        last_time = 0.
        G = self.drone.g
        ctl1 = get_pid1()
        ctl2 = get_pid2()
        DES = np.array([2., 0., 2.])
        action = np.array([0.] * 4)
        DTIME = 20e-3

        zmm = 0
        alpha = 0.9
        while True and self.drone.get_time() < 1000:
            self.drone.step()
            if self.drone.get_time() - last_time > DTIME:
                dt = self.drone.get_time() - last_time
                last_time = self.drone.get_time()

                acc, omega, z = self.drone.get_sensors()
                zmm = zmm * alpha + (1-alpha) * z
                pos = self.drone.get_position()
                pos = np.array([pos[0], pos[1], zmm])
                uacc = ctl1.get_control(last_time, dt, pos, DES)
                # uacc[0] += 1
                # uacc[2] += np.sqrt(G**2 - 1)
                uacc[2] += G
                meas = np.array((acc, omega)).flatten()
                u = ctl2.get_control(last_time, dt, meas, uacc)
                action += u
                action = np.maximum.reduce([action, np.zeros(4)])
                self.drone.set_motors(action)

                ori = self.drone.get_orientation()
                print('update motors: {}'.format(u))
                print('update target: {}'.format(uacc))
                print('action: {}'.format(action))
                print('time: {}'.format(last_time))
                print('pos : {}'.format(pos))
                print('vel : {}'.format(self.drone.vel))
                print('ori : {}'.format(ori))
                print('meas : {}'.format(meas))
                pos = self.drone.get_position()
                ori = self.drone.rot
                yield pos, ori
                # AAAO.append((uacc[2], self.drone.acc_sensor[2],
                             # uacc[2] + ctl1._kd[2,2] * self.drone.vel[2] + ctl1._kp[2,2] * pos[2]))
        # plt.plot(AAAO)
        # plt.show()



sim = Simulator()
