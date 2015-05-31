#! /usr/bin/env python3

import numpy as np

from pid import PIDController
from num_model import Drone

np.set_printoptions(precision=4, suppress=True)

def get_pid1():
    KPxy = 0.1
    KPz = 0.1
    KPt = 0.1
    kp = np.vstack([np.diag([KPxy, KPxy, KPz]), np.zeros((3, 3))])
    kd = np.array([1.0]*3 + [1.2]*3)[np.newaxis].T * kp
    ki = -0.0 * kp
    ke = 0.9
    controller = PIDController(kp, kd, ki, ke)
    return controller

def get_pid2():
    KPxy = 15.
    KPz = 15.
    KPw = 5.
    kpw1 = [KPxy, KPxy, KPz, KPw, -KPw, KPw]
    kpw2 = [KPxy, -KPxy, KPz, -KPw, -KPw, -KPw]
    kpw3 = [-KPxy, -KPxy, KPz, -KPw, KPw, KPw]
    kpw4 = [-KPxy, KPxy, KPz, KPw, KPw, -KPw]
    kp = np.array([kpw1, kpw2, kpw3, kpw4])
    kd = np.array([1/3]*3 + [1.2]*3) * kp
    ki = -0.1 * kp
    ke = 0.9
    controller = PIDController(kp, kd, ki, ke)
    return controller

def simulate(drone):
    last_time = 0.
    G = drone.g
    ctl1 = get_pid1()
    ctl2 = get_pid2()
    DES = np.array([0., 0., 0.])
    action = np.array([0.] * 4)
    DTIME = 20e-3
    while True:
        drone.step()
        if drone.get_time() - last_time > DTIME:
            dt = drone.get_time() - last_time
            last_time = drone.get_time()

            pos = drone.get_position()
            meas = np.array(drone.get_sensors()).flatten()
            uacc = ctl1.get_control(last_time, dt, pos, DES)
            uacc[2] += G
            u = ctl2.get_control(last_time, dt, meas, uacc)
            action += u
            drone.set_motors(action)

            ori = drone.get_orientation()
            print('update motors: {}'.format(u))
            print('action: {}'.format(action))
            print('time: {}'.format(last_time))
            print('pos : {}'.format(pos))
            print('vel : {}'.format(drone.vel))
            print('ori : {}'.format(ori))
            print('meas : {}'.format(meas))
        # pos = drone.get_position()


def main():
    drone = Drone()
    drone.set_init([0., 0., 0.], [2., 2., 0.])
    # drone.noise_acc = 1e-10
    # drone.noise_omega = 1e-10
    try:
        simulate(drone)
    except KeyboardInterrupt:
        print()
        print('exit')

if __name__ == '__main__':
    main()
