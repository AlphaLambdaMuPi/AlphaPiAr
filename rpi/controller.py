#! /usr/bin/env python3

import asyncio
import numpy as np

from pid import PID

class Controller:
    def __init__(self, drone, *, loop=None):
        self.loop = loop if loop else asyncio.get_event_loop()
        self.drone = drone
        self.action = np.array([0., 0., 0., 0.])
        self.despos = np.array([0., 0., 0.])

        self.stop_signal = asyncio.Future(loop=self.loop)

        self.offset = 0. # testing purpose

        def get_pid1():
            KPxy = 0.
            KPz = 0.
            KPt = 1.
            kp = np.vstack([np.diag([-KPxy, -KPxy, KPz]), np.zeros((3, 3))])
            kd = np.array([1.2]*3 + [0.05]*3)[np.newaxis].T * kp
            ki = 0.1 * kp
            ke = 0.9
            ctl = PID(kp, kd, ki, ke)
            return ctl

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
            ctl = PID(kp, kd, ki, ke)
            return ctl

        self.ctl1 = get_pid1()
        self.ctl2 = get_pid2()

    @asyncio.coroutine
    def run(self):
        try:
            yield from self.drone.start_control()
            ready = yield from self.drone.get_ready()
            if not ready:
                return False

            yield from self._run()
        except asyncio.CancelledError:
            while sum(action) > 0.:
                self.action -= np.array([20, 20, 20, 20])
                self.drone.set_motors(self.action)
                yield from asyncio.sleep(0.02)

        return True

    @asyncio.coroutine
    def _run(self):
        DTIME = 20e-3
        self.last_time = self.loop.time()

        while self.drone.alive() and not self.stop_signal.done():
            yield from asyncio.sleep(DTIME)
            self.update()

    def udpate(self):
        now = self.loop.time()
        dt = now - self.last_time

        acc, omega, z = self.drone.get_sensors()

        pos = np.array([0., 0., 0.])
        uacc = ctl1.get_control(last_time, dt, pos, despos)
        uacc[2] += self.drone.g + self.offset # testing purpose
        meas = np.array((acc, omega)).flatten()
        u = ctl2.get_control(now, dt, meas, uacc)
        self.action += u
        self.action = np.maximum.reduce([self.action, np.zeros(4)])
        self.drone.set_motors(self.action)

        self.last_time = now

    def set_despos(self, pos):
        self.despos = pos

    def get_despos(self):
        return self.despos

    def stop(self):
        self.stop_signal.set_result(True)

