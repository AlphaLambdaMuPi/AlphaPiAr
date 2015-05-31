#! /usr/bin/env python3

import numpy as np

from pid import PIDController
from num_model import Drone

np.set_printoptions(precision=4, suppress=True)

def simulate(drone):
    KPxy = 2
    KPz = 15
    KPw = 5
    kpw1 = [-KPxy, -KPxy, KPz, KPw, -KPw, KPw]
    kpw2 = [-KPxy, KPxy, KPz, -KPw, -KPw, -KPw]
    kpw3 = [KPxy, KPxy, KPz, -KPw, KPw, KPw]
    kpw4 = [KPxy, -KPxy, KPz, KPw, KPw, -KPw]
    kp = np.array([kpw1, kpw2, kpw3, kpw4])
    kd = np.array([1/3]*3 + [1.2]*3) * kp
    ki = -0.1 * kp
    ke = 0.9
    controller = PIDController(kp, kd, ki, ke)
    last_time = 0
    G = drone.g
    DES = np.array([0, 0, G, 0, 0, 0])
    action = np.array([0.0] * 4)
    while True:
        drone.step()
        if drone.get_time() - last_time > 5e-3:
            dt = drone.get_time() - last_time
            last_time = drone.get_time()
            meas = np.array(drone.get_sensors()).flatten()
            u = controller.get_control(last_time, dt, meas, DES)
            action += u
            drone.set_motors(action)

            pos = drone.get_position()
            ori = drone.get_orientation()
            print('update motors: {}'.format(u))
            print('action: {}'.format(action))
            print('pos : {}'.format(pos))
            print('ori : {}'.format(ori))
            print('meas : {}'.format(meas))
        # pos = drone.get_position()


def main():
    drone = Drone()
    drone.set_init([0, 0, 0], [1, 0, 0])
    try:
        simulate(drone)
    except KeyboardInterrupt:
        print()
        print('exit')

if __name__ == '__main__':
    main()
