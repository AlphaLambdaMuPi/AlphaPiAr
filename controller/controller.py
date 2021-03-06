#! /usr/bin/env python3

import asyncio
import logging
import json
import numpy as np

from .pid import PID

logger = logging.getLogger()

class Controller(object):
    def __init__(self, drone, *, loop=None, log=False):
        if not loop:
            loop = asyncio.get_event_loop()
        self._loop = loop
        self._drone = drone
        self._action = np.array([0., 0., 0., 0.])
        self._despos = np.array([0., 0., 0.])

        self._stablized = asyncio.Future(loop=self._loop)
        self.stop_signal = False
        self.stopped = asyncio.Future(loop=self._loop)

        self._restriction = 800
        # restriction for testing rotation
        self._restriction = 200

        self._offset = 0. # testing purpose

        self._zmm = 0

        # def get_pos_gain(*,, KPxy=0., KPz=0.):
            # KPxy = 0.
            # KPz = 0.
            # kp = np.vstack([np.diag([-KPxy, -KPxy, KPz])])
            # kd = np.array([1.2]*3)[:,np.newaxis] * kp
            # ki = 0.1 * kp
            # ke = 0.9
            # return (kp, kd, ki, ke)

        # def get_acc_gain(*, KPxy=5., KPz=10., KPxyw=10., KPzw=10.):
            # kpw0 = [KPxy, 0., KPz, 0., -KPxyw, -KPzw]
            # kpw1 = [0., KPxy, KPz, KPxyw, 0., KPzw]
            # kpw2 = [-KPxy, 0., KPz, 0., KPxyw, -KPzw]
            # kpw3 = [0., -KPxy, KPz, -KPxyw, 0., KPzw]
            # kp = np.array([kpw0, kpw1, kpw2, kpw3])
            # kd = np.array([0.]*3 + [0.02]*3) * kp
            # ki = np.array([0.]*3 + [0.]*3) * kp
            # ke = 0.9
            # return (kp, kd, ki, ke)

        # def get_th_gain(*, KPxyth=5., KPzth=0.):
            # kpw0 = [0., -KPxyth, -KPzth]
            # kpw1 = [KPxyth, 0., KPzth]
            # kpw2 = [0., KPxyth, -KPzth]
            # kpw3 = [-KPxyth, 0., KPzth]
            # kp = np.array([kpw0, kpw1, kpw2, kpw3])
            # kd = np.array([0.]*3 + [0.02]*3) * kp
            # ki = np.array([0.]*3 + [0.]*3) * kp
            # ke = 0.9
            # return (kp, kd, ki, ke)

        # testing rotation
        self._pids = {
            # 'pos': PID(*get_pos_gain(), gen_gain=get_pos_gain),
            # 'acc': PID(*get_acc_gain(), gen_gain=get_acc_gain),
            # 'th': PID(*get_th_gain(), gen_gain=get_th_gain),
            'th': PID(5., 0., 0., 0.9),
        }

        # logging
        self._datalogger = None
        if log:
            self._logger_setup()

    def _logger_setup(self):
        self._datalogger = logging.getLogger('data')
        self._datalogger.propagate = False
        fh = logging.FileHandler('action.log')
        self._datalogger.addHandler(fh)

    def set_despos(self, pos):
        self._despos = pos

    def get_despos(self):
        return self._despos

    @asyncio.coroutine
    def run(self):
        try:
            yield from self._drone.start_control()
            ready = yield from self._drone.get_ready()
            if not ready:
                return False

            logger.info('controller start.')
            yield from self._run()
        except (asyncio.CancelledError, KeyboardInterrupt):
            logger.info('capture ctrl-C in controller.')
        finally:
            yield from self.landing()

        return True

    @asyncio.coroutine
    def _run(self):
        DTIME = 20e-3
        self._last_time = self._loop.time()

        res = yield from self._stablized
        if not res:
            return

        while self._drone.alive() and not self.stop_signal:
            yield from self.update()

    @asyncio.coroutine
    def update(self):
        '''updates the motors according to the sensors' data
        '''
        now = self._loop.time()
        dt = now - self._last_time

        acc, theta, omega, z = yield from self._drone.get_sensors()

        # alpha = 0.9
        # self._zmm = self._zmm*alpha + z*(1-alpha)
        # pos = np.array([0., 0., 0.])
        # pos = np.array([0., 0., self._zmm])
        # uacc = self._pids['pos'].get_control(now, dt, self._despos-pos)
        # uacc[2] += self._drone.g + self._offset # testing purpose
        # meas = np.array((acc, omega)).flatten()
        # u = self._ctl2.get_control(now, dt, uacc-meas)


        # testing rotation
        roffset = self._pids['th'](now, dt, -theta[1], -omega[1])
        self._action[1] = self._action[3] = self._restriction/2
        self._action += roffset

        yield from self.send_control()

        self._last_time = now

        # logging
        if self._datalogger:
            self._datalogger.info(json.dumps({
                'action':self._action.tolist(),
                'meas': meas.tolist(),
            }))

    @asyncio.coroutine
    def send_control(self):
        self._action = np.maximum.reduce([self._action, np.zeros(4)])
        #testing rotation
        self._action = np.minimum.reduce([self._action,
                                          np.full((4,), self._restriction)])
        yield from self._drone.set_motors(self._action)

    @asyncio.coroutine
    def takeoff(self):
        # TODO:
        # implement take off process and check if drone is stable.
        if self.stopped.done():
            return False
        if self._stablized.done():
            raise RuntimeError('the drone has already took off.')
        self._stablized.set_result(True)
        return True

    @asyncio.coroutine
    def landing(self):
        logger.info('landing...')

        '''
        motor = min(self._action)
        self._action = np.full((4,), motor)

        while motor > 0:
        self._action -= 50
        yield from self.send_control()
        motor -= 50

        '''

        # for testing
        self._action = np.zeros(4)
        yield from self.send_control()

        logger.info('landed.')

        self.stopped.set_result(True)

    @asyncio.coroutine
    def stop(self):
        if not self._stablized.done():
            self._stablized.set_result(False)

        self.stop_signal = True
        yield from self.stopped

