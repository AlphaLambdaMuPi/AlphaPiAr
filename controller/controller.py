#! /usr/bin/env python3

import asyncio
import logging
import json
import numpy as np

from .pid import PID
from .utils import Momentum
from .kalfil import ThetaOmegaKalmanFilter

logger = logging.getLogger()

class Controller(object):
    def __init__(self, drone, *, loop=None, log=False):
        if not loop:
            loop = asyncio.get_event_loop()
        self._loop = loop
        self._drone = drone
        self._kf = []
        for i in range(3):
            self._kf.append(ThetaOmegaKalmanFilter(0.1, 0.1, 0.04))
        self._action = np.array([0., 0., 0., 0.])
        self._thrust = 0
        self._target_angle = np.array([0., 0.])

        self._stablized = asyncio.Future(loop=self._loop)
        self.stop_signal = False
        self.stopped = asyncio.Future(loop=self._loop)

        self._restriction = 800

        # self.theta_mom = Momentum()
        # self.omega_mom = Momentum()

        self._pid_thetaxy = np.array(80., 40., 10., 50.)

        self._pids = {
                'theta_x': PID(*self._pid_thetaxy),
                'theta_y': PID(*self._pid_thetaxy),
                'omega_z': PID(20., 0., 5., 40.),
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

    def set_thrust(self, thrust):
        self._thrust = thrust

    def set_angle(self, angle):
        self._target_angle = np.array(angle).flatten()

    def tweak_pid(self, type_, per):
        gain = self._pid_thetaxy * per
        self._pids['theta_x'].set_gain(*gain)
        self._pids['theta_y'].set_gain(*gain)

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

        theta_smooth = []
        omega_smooth = []
        for i in range(3):
            theome = np.array([theta[i], omega[i]])
            self._kf[i].update(now, theome)
            res = self._kf[i].predict(now)
            theta_smooth.append(res[0])
            omega_smooth.append(res[1])
        theta_smooth = np.array(theta_smooth)
        omega_smooth = np.array(omega_smooth)
        # theta_smooth = self.theta_mom.append_value(now, theta)
        # omega_smooth = self.omega_mom.append_value(now, omega)
        thetaxy_error = self._target_angle - theta_smooth[0:2]
        omegaz_error  = 0                  - omega_smooth[2]

        theta_x_action = self._pids['theta_x'].get_control(
                now, thetaxy_error[0], 0 - omega_smooth[0]
            )
        theta_y_action = self._pids['theta_y'].get_control(
                now, thetaxy_error[1], 0 - omega_smooth[1]
            )
        omega_z_action = self._pids['omega_z'].get_control(
                now, omegaz_error
            )

        self._action[0] += -theta_y_action + -omega_z_action
        self._action[1] +=  theta_x_action +  omega_z_action
        self._action[2] +=  theta_y_action + -omega_z_action
        self._action[3] += -theta_x_action +  omega_z_action

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
                'theta_smooth': theta_smooth.tolist(),
                'omega_smooth': omega_smooth.tolist(),
                'time': now,
            }))

    @asyncio.coroutine
    def send_control(self):
        if self._thrust >= 10:
            final_action = self._action + self._thrust
            final_action = np.maximum.reduce([final_action, np.full((4,), -100)])
            final_action = np.minimum.reduce([final_action,
                                        np.full((4,), self._restriction)])
        else:
            final_action = np.full((4, ), -100)

        yield from self._drone.set_motors(final_action)

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

