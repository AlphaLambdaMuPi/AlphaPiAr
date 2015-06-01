#! /usr/bin/env python3

import numpy as np

from pid import PIDController
from num_model import Drone

np.set_printoptions(precision=4, suppress=True)

def get_pid1():
    # KPxy = 0.1
    # KPz = 0.8
    # KPt = 1.
    KPxy = 0.
    KPz = 0.
    KPt = 1.
    kp = np.vstack([np.diag([-KPxy, -KPxy, KPz]), np.zeros((3, 3))])
    kd = np.array([1]*2 + [1.2] + [1.2]*3)[np.newaxis].T * kp
    kd[2] = 2 * np.sqrt(kp[2])
    ki = 0.1 * kp
    ki[2] = 0.
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
    kd = np.array([0.6]*3 + [1.2]*3) * kp
    ki = np.array([1/4]*3 + [1/8]*3) * kp
    ke = 0.9
    controller = PIDController(kp, kd, ki, ke)
    return controller

class Simulator:
    def __init__(self):
        self.drone = Drone()
        self.drone.set_init([0., 0., 0.], [2., 2., 0.])
        # drone.noise_acc = 1e-10
        # drone.noise_omega = 1e-10

    def run(self):
        last_time = 0.
        G = self.drone.g
        ctl1 = get_pid1()
        ctl2 = get_pid2()
        DES = np.array([3., 3., 3.])
        action = np.array([0.] * 4)
        DTIME = 20e-3
        while True:
            self.drone.step()
            if self.drone.get_time() - last_time > DTIME:
                dt = self.drone.get_time() - last_time
                last_time = self.drone.get_time()

                pos = self.drone.get_position()
                meas = np.array(self.drone.get_sensors()).flatten()
                uacc = ctl1.get_control(last_time, dt, pos, DES)
                # uacc[0] += 1
                # uacc[2] += np.sqrt(G**2 - 1)
                uacc[2] += G
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


sim = Simulator()
