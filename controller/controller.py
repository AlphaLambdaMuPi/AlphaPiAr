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
            # roll, pitch, yaw
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
            'roll': PID(100, 0, 0, 0),
            'pitch': PID(0, 0, 0, 0),
            'omegaz': PID(0, 0, 0, 0),
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

    def set_action(self, action):
        self._action = action

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
            # yield from asyncio.sleep(DTIME)
            yield from self.update()

    @asyncio.coroutine
    def update(self):
        '''updates the motors according to the sensors' data
        '''
        now = self._loop.time()
        dt = now - self._last_time

        acc, omega, z = yield from self._drone.get_sensors()
        theta = self._drone.gettheta()
        target_theta = np.array([0, 0, 0])
        target_omegaz = np.array([0, 0, 0])
        err_theta = target_theta - theta
        err_omega = target_omega - omega

        action_tx = self._pids['roll'].get_control(now, 
                err_theta[0], err_omega[0])
        action_ty = self._pids['pitch'].get_control(now, 
                err_theta[1], err_omega[1])

        action_omegaz = self._pids['omegaz'].get_control(now, 
                err_omega[2])

        TEST_VALUE = 1300
        self._action = np.array([TEST_VALUE] * 4) + np.array([
            -action_ty - action_omegaz,
            action_tx  - action_omegaz,
            action_ty  + action_omegaz,
            -action_tx + action_omegaz,
        ])


        logger.debug('{}'.format(self._action))

        yield from self.send_control()

        self._last_time = now

        # logging
        if self._datalogger:
            self._datalogger.info(json.dumps({
                'action': self._action.tolist(),
                'accel': acc.tolist(),
                'theta': theta.tolist(),
                'omega': omega.tolist(),
                'time': now,
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

