#! /usr/bin/env python3

import asyncio
import logging
import json
import numpy as np

from .pid import PID
from .utils import Momentum

logger = logging.getLogger()

class Controller(object):
    def __init__(self, drone, *, loop=None, log=False):
        if not loop:
            loop = asyncio.get_event_loop()
        self._loop = loop
        self._drone = drone
        self._action = np.array([-100., -100., -100., -100.])
        self._despos = np.array([0., 0., 0.])

        self._stablized = asyncio.Future(loop=self._loop)
        self.stop_signal = False
        self.stopped = asyncio.Future(loop=self._loop)

        # self._restriction = 700
        # restriction for testing rotation
        self._restriction = 1000
        # self._action[0] = self._action[2] = self._restriction/2

        # lowpass sensors
        self._zmm = 0
        self.lrf = 0
        self.theta_mom = Momentum()
        self.omega_mom = Momentum()

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

        def get_th_gain(*, kkp=40., kkd=20., kki=8.):
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
        #self._pids = {
            #'pos': PID(*get_pos_gain()),
            #'acc': PID(*get_acc_gain()),
            #'th': PID(*get_th_gain()),
        #}
        #self._pids['pos'].gen_gain = get_pos_gain
        #self._pids['acc'].gen_gain = get_acc_gain
        #self._pids['th'].gen_gain = get_th_gain

        self._pids = {
                'theta_x': PID(80., 40., 10., 50.),
                'theta_y': PID(50., 20., 8., 50.),
                'omega_z': PID(0., 0., 0., 0.),
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
        # self._action = np.array([-100, action[0], -100, -100])

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

        # alpha = 0.9
        # self._zmm = self._zmm*alpha + z*(1-alpha)
        # pos = np.array([0., 0., 0.])
        # pos = np.array([0., 0., self._zmm])
        # uacc = self._pids['pos'].get_control(now, dt, self._despos-pos)
        # uacc[2] += self._drone.g # testing purpose
        # uacc = np.array((uacc, np.zeros(3))).flatten()
        # meas = np.array((acc, omega)).flatten()

        # testing rotation
        
        # final
        #alpha = 0.5
        #self.thethe = self.thethe * alpha + theta * (1-alpha)
        #self.omeome = self.omeome * alpha + omega * (1-alpha)

        theta_smooth = self.theta_mom.append_value(now, theta)
        omega_smooth = self.omega_mom.append_value(now, omega)
        theta_x_action, xxx = self._pids['theta_x'].get_control(now, 
                -theta_smooth[0], -omega_smooth[0])
        # theta_y_action = self._pids['theta_y'].get_control(now, 
                # -theta_smooth[1], -omega_smooth[1])
        # omega_z_action = self._pids['omega_z'].get_control(now, 
                # -omega_smooth[2])



        # roffset = self._pids['th'].get_control(now, dt, -theta, -omega)
        # roffset = self._pids['th'].get_control(now, dt, -omega)
        # meas = np.array((acc, omega)).flatten()
        # meas[2] -= 9.8
        # roffset = self._pids['acc'].get_control(now, dt, uacc-meas)
        # conzi = roffset - self.lrf
        # self.lrf = roffset


        # final
        # self._action[0] = self._action[2] = self._restriction/2
        # self._action[0] += -theta_y_action
        # self._action[2] += theta_y_action

        self._action[1] = self._action[3] = self._restriction / 2.
        self._action[1] +=  theta_x_action
        self._action[3] += -theta_x_action
        #self._action[0] += roffset[0]# - 20# * dt
        #self._action[2] += roffset[2]# + 20# * dt


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
                'action': self._action.tolist(),
                'action_pid': xxx,
                'accel': acc.tolist(),
                'theta': theta.tolist(),
                'omega': omega.tolist(),
                'theta_smooth': theta_smooth.tolist(),
                'omega_smooth': omega_smooth.tolist(),
                'time': now,
            }))

    @asyncio.coroutine
    def send_control(self):
        self._action = np.maximum.reduce([self._action, np.full((4,), -100)])
        #testing rotation
        self._action = np.minimum.reduce([self._action,
                                    np.full((4,), self._restriction)])
        # self._action[1] = self._action[3] = -100
        # self._action[2] = -100
        self._action[2] = self._action[0] = -100
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

