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

        self._restriction = 700
        # restriction for testing rotation
        self._restriction = 200
        self._action[0] = self._action[2] = self._restriction/2

        # lowpass sensors
        self._zmm = 0
        self.lrf = 0
        self.thethe = np.zeros(3)
        self.omeome = np.zeros(3)

        # pid gain functions
        def get_pos_gain(*, kkp=0.8, kkd=0.8, kki=0.0):
            m = np.array([
                [-1., 0., 0.],
                [0., -1., 0.],
                [0., 0., 1.],
            ])
            m *= np.array([0., 0., 0.])
            kp = kkp * m
            kd = kkd * m
            ki = kki * m
            ke = 0.99
            return (kp, kd, ki, ke)

        def get_acc_gain(*, kkp=10., kkd=0.2, kki=0.):
            # x, y, z, xw, yw, zw
            m = np.array([
                [1., 0., 1., 0., -1., -1.],
                [0., 1., 1., 1., 0., 1.],
                [-1., 0., 1., 0., 1., -1.],
                [0., -1., 1., -1., 0., 1.],
            ])
            m *= np.array([0.5, 0.5, 0., 1., 1., 0.])
            kp = kkp * m
            kd = kkd * m
            ki = kki * m
            ke = 0.99
            return (kp, kd, ki, ke)

        def get_th_gain(*, kkp=5., kkd=0., kki=0.):
            # raw, pitch, yaw
            # xth, yth, zth
            m = np.array([
                [0., -1., -1.],
                [1., 0., 1.],
                [0., 1., -1.],
                [-1., 0., 1.],
            ])
            m *= np.array([1., 1., 0.])
            kp = kkp * m
            kd = kkd * m
            ki = kki * m
            ke = 0.999
            return (kp, kd, ki, ke)

        # testing rotation
        self._pids = {
            'pos': PID(*get_pos_gain()),
            'acc': PID(*get_acc_gain()),
            'th': PID(*get_th_gain()),
        }
        self._pids['pos'].gen_gain = get_pos_gain
        self._pids['acc'].gen_gain = get_acc_gain
        self._pids['th'].gen_gain = get_th_gain

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

        logger.info('wait for taking off.')
        res = yield from self._stablized
        if not res:
            return
        logger.info('took off and stablized.')

        while self._drone.alive() and not self.stop_signal:
            yield from asyncio.sleep(DTIME)
            yield from self.update()

    @asyncio.coroutine
    def update(self):
        '''updates the motors according to the sensors' data
        '''
        now = self._loop.time()
        dt = now - self._last_time

        acc, omega, z = yield from self._drone.get_sensors()

        alpha = 0.9
        self._zmm = self._zmm*alpha + z*(1-alpha)
        pos = np.array([0., 0., 0.])
        # pos = np.array([0., 0., self._zmm])
        uacc = self._pids['pos'].get_control(now, dt, self._despos-pos)
        uacc[2] += self._drone.g # testing purpose
        uacc = np.array((uacc, np.zeros(3))).flatten()
        meas = np.array((acc, omega)).flatten()
        print(uacc)
        print(meas)

        # testing rotation
        
        # alpha = 0.9
        # self.thethe = self.thethe * alpha + theta * (1-alpha)
        # self.omeome = self.omeome * alpha + omega * (1-alpha)

        # roffset = self._pids['th'].get_control(now, dt, -self.thethe, -self.omeome)
        # roffset = self._pids['th'].get_control(now, dt, -theta, -omega)
        # roffset = self._pids['th'].get_control(now, dt, -omega)
        # meas = np.array((acc, omega)).flatten()
        # meas[2] -= 9.8
        roffset = self._pids['acc'].get_control(now, dt, uacc-meas)
        # conzi = roffset - self.lrf
        # self.lrf = roffset
        # self._action[0] = self._action[2] = self._restriction/2
        self._action += roffset
        #testing mgr sin(theta) compensation
        # mgrpR = 60 * np.sin(theta[1]) / 2
        # mgoffset = np.array([mgrpR, 0., -mgrpR, 0.])
        # action = self._action + mgoffset

        logger.debug('{}'.format(self._action))

        yield from self.send_control()

        self._last_time = now

        # logging
        if self._datalogger:
            self._datalogger.info(json.dumps({
                'action':self._action.tolist(),
                'meas': theta.tolist(),
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
        self._action = np.full((4,), -100)
        yield from self._drone.set_motors(self._action)

        logger.info('landed.')

        self.stopped.set_result(True)

    @asyncio.coroutine
    def stop(self):
        if not self._stablized.done():
            self._stablized.set_result(False)

        self.stop_signal = True
        yield from self.stopped

