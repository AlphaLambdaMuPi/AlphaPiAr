#! /usr/bin/env python3

import asyncio
import logging
import json
import numpy as np

from .pid import PID

logger = logging.getLogger()

class Controller:
    def __init__(self, drone, *, loop=None, log=False):
        self.loop = loop if loop else asyncio.get_event_loop()
        self.drone = drone
        self.action = np.array([0., 0., 0., 0.])
        self.despos = np.array([0., 0., 0.])

        self.stop_signal = False
        self.stopped = asyncio.Future(loop=self.loop)

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
            kpw1 = [-KPxy, 0., KPz, 0., KPw, KPw]
            kpw2 = [0., -KPxy, KPz, KPw, 0., -KPw]
            kpw3 = [KPxy, 0., KPz, 0., -KPw, KPw]
            kpw4 = [0., KPxy, KPz, -KPw, 0., -KPw]
            kp = np.array([kpw1, kpw2, kpw3, kpw4])
            kd = np.array([0.]*3 + [0.0]*3) * kp
            ki = np.array([0.]*3 + [0.]*3) * kp
            ke = 0.9
            ctl = PID(kp, kd, ki, ke)
            return ctl

        self.ctl1 = get_pid1()
        self.ctl2 = get_pid2()

        # logging
        self.datalogger = None
        if log:
            self.logger_setup()

    def logger_setup(self):
        self.datalogger = logging.getLogger('data')
        self.datalogger.propagate = False
        fh = logging.FileHandler('action.log')
        self.datalogger.addHandler(fh)

    def set_despos(self, pos):
        self.despos = pos

    def get_despos(self):
        return self.despos

    @asyncio.coroutine
    def run(self):
        try:
            yield from self.drone.start_control()
            ready = yield from self.drone.get_ready()
            if not ready:
                return False

            yield from self._run()
        except (asyncio.CancelledError, KeyboardInterrupt):
            logger.debug('capture ctrl-C in controller.')
        finally:
            yield from self.landing()

        return True

    @asyncio.coroutine
    def _run(self):
        DTIME = 20e-3
        self.last_time = self.loop.time()

        while self.drone.alive() and not self.stop_signal:
            yield from asyncio.sleep(DTIME)
            yield from self.update()

    @asyncio.coroutine
    def update(self):
        now = self.loop.time()
        dt = now - self.last_time

        acc, omega, z = yield from self.drone.get_sensors()

        pos = np.array([0., 0., 0.])
        uacc = self.ctl1.get_control(now, dt, pos, self.despos)
        uacc[2] += self.drone.g + self.offset # testing purpose
        meas = np.array((acc, omega)).flatten()
        u = self.ctl2.get_control(now, dt, meas, uacc)
        self.action += u
        self.action = np.maximum.reduce([self.action, np.zeros(4)])
        self.action = np.minimum.reduce([self.action, np.full((4,), 800)])
        yield from self.drone.set_motors(self.action)

        self.last_time = now

        # logging
        if self.datalogger:
            self.datalogger.info(json.dumps({
                'action':self.action.tolist(),
                'meas': meas.tolist(),
            }))

    @asyncio.coroutine
    def takeoff(self):
        pass
    
    @asyncio.coroutine
    def landing(self):
        logger.debug('landing...')
        logger.debug('landed.')
        self.stopped.set_result(True)

    @asyncio.coroutine
    def stop(self):
        self.stop_signal = True
        yield from self.stopped

