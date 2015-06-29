#! /usr/bin/env python3

import asyncio
import logging
import json
import numpy as np

from .pid import PID
from .utils import Momentum
from .kalfil import ThetaOmegaKalmanFilter
from .constant import CONST

logger = logging.getLogger()

class Controller(object):
    def __init__(self, drone, *, loop=None, log=False):
        if not loop:
            loop = asyncio.get_event_loop()
        self._armed = False
        self._landing = True
        self._loop = loop
        self._drone = drone
        self._kf = []
        for i in range(3):
            self._kf.append(ThetaOmegaKalmanFilter(0.1, 0.1, 0.04))
        self._action = np.array([0., 0., 0., 0.])
        self._thrust = 0
        self._target_angle = np.array([0., 0., 0.])

        self.stop_signal = False
        self.stopped = asyncio.Future(loop=self._loop)

        self._restriction = 700

        # self.theta_mom = Momentum()
        # self.omega_mom = Momentum()

        self._pid_thetaxy = np.array([40., 15., 25.])
        self._pid_tweakper = np.array([1., 1., 1.])

        self._pids = {
                'theta_x': PID(*self._pid_thetaxy, imax=60.),
                'theta_y': PID(*self._pid_thetaxy, imax=60.),
                'theta_z': PID(40., 10., 20., imax=40.),
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

    def tweak_pid(self, type_, per):
        if type_ == 'P':
            self._pid_tweakper[0] = per
        elif type_ == 'I':
            self._pid_tweakper[1] = per
        elif type_ == 'D':
            self._pid_tweakper[2] = per
        gain = self._pid_thetaxy * self._pid_tweakper
        print(gain)
        self._pids['theta_x'].set_gain(*gain)
        self._pids['theta_y'].set_gain(*gain)

    def get_control(self, thrust, angle_x, angle_y, omega_z):
        if not self._armed or self.stop_signal:
            self._thrust = -100
            return

        if self._thrust > CONST['armed_thrust']:
            dx = CONST['max_thrust'] - CONST['armed_thrust']
        else:
            dx = CONST['armed_thrust'] - CONST['disarmed_thrust']
        self._thrust = thrust * dx + CONST['armed_thrust']

        mxy, mz = CONST['max_anglexy'], CONST['max_anglez']
        self._target_angle = np.array([angle_x*mxy, angle_y*mxy, omega_z*mz])
        
        

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

        while self._drone.alive() and not self.stop_signal:
            if self._armed:
                yield from self.update()
            yield from asyncio.sleep(0.)


    @asyncio.coroutine
    def update(self):
        '''updates the motors according to the sensors' data
        '''
        if self._thrust < 10:
            return
        now = self._loop.time()
        dt = now - self._last_time

        acc, omega, z = yield from self._drone.get_sensors()
        theta = self._drone.gettheta()

        theta_smooth = []
        omega_smooth = []
        for i in range(3):
            theome = np.array([theta[i], omega[i]])
            self._kf[i].update(now, theome)
            the, ome = self._kf[i].predict(now)
            theta_smooth.append(the)
            omega_smooth.append(ome)
        theta_smooth = np.array(theta_smooth)
        omega_smooth = np.array(omega_smooth)
        # theta_smooth = self.theta_mom.append_value(now, theta)
        # omega_smooth = self.omega_mom.append_value(now, omega)
        #thetaxy_error = self._target_angle - theta_smooth[0:2]
        #thetaz_error  = 0                  - theta_smooth[2]
        theta_error    = self._target_angle - theta_smooth

        theta_x_action = self._pids['theta_x'].get_control(
            now, theta_error[0], 0 - omega_smooth[0]
        )
        theta_y_action = self._pids['theta_y'].get_control(
            now, theta_error[1], 0 - omega_smooth[1]
        )
        theta_z_action = self._pids['theta_z'].get_control(
            now, theta_error[2], 0 - omega_smooth[2]
        )

        self._action[0] = -theta_y_action +  theta_z_action
        self._action[1] =  theta_x_action + -theta_z_action
        self._action[2] =  theta_y_action +  theta_z_action
        self._action[3] = -theta_x_action + -theta_z_action

        # logger.debug('{}'.format(self._action))

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
            final_action = np.maximum.reduce([final_action,
                                              np.full((4,), -100)])
            final_action = np.minimum.reduce(
                [final_action, np.full((4,), self._restriction)]
            )
        else:
            final_action = np.full((4, ), -100.)

        final_action[1] += 4
        final_action[2] += 10
        # final_action[1] = final_action[3] = -100.
        print(final_action)
        yield from self._drone.set_motors(final_action)

    @asyncio.coroutine
    def takeoff(self):
        # TODO:
        # implement take off process and check if drone is stable.
        return True

    @asyncio.coroutine
    def arm(self):
        self._armed = True
        self._thrust = CONST['armed_thrust']
        yield from self.send_control()
        return True

    @asyncio.coroutine
    def disarm(self):
        self._armed = False
        self._landing = True
        self._target_angle = np.array([0., 0., 0.])
        yield from self.landing()
        return True


    @asyncio.coroutine
    def landing(self):
        logger.info('landing...')

        # Land at speed 400 thrust per second
        while not self.stop_signal and self._thrust > 0:
            self._thrust -= 20
            yield from self.send_control()
            yield from asyncio.sleep(.05)

        self._thrust = -100
        self._landing = True
        logger.info('landed.')

    @asyncio.coroutine
    def stall(self):
        self._thrust = -100
        yield from self.send_control()


    @asyncio.coroutine
    def stop(self):
        self.stop_signal = True
        yield from self.stall()
        yield from self.stopped

    @asyncio.coroutine
    def preform_action(self, action, args):
        if action == 'stop':
            yield from self.stop()
        elif action == 'arm':
            print('Get Arm')
            yield from self.arm()
        elif action == 'disarm':
            yield from self.disarm()
        elif action == 'control':
            self.get_control(*args)
        elif action == 'tweak':
            self.tweak_pid(*args)
            


