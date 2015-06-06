#! /usr/bin/env python3

import asyncio
import logging
import numpy as np
import scipy.linalg

logger = logging.getLogger()

class Drone(object):
    def __init__(self):
        self.dt = 2E-4
        self.time = 0.0
        self.g = 9.80
        self.gvec = np.array([0., 0., -self.g])
        self.M = 1.250
        self.R = 0.23
        self.Iz = 0.25 * self.M * self.R**2
        self.Ixy = self.Iz * 0.5
        self.I = np.diag([self.Ixy, self.Ixy, self.Iz])
        self.LIFT_K = 7.5e-3
        self.TDRAG_K = 0.0
        self.DRAG_B = 0.5

        self.noise_acc = 0.04
        self.noise_omega = 0.002
        self.noise_z = 0.5

        self.pos = np.zeros(3)
        self.rot = np.eye(3)
        self.vel = np.zeros(3)
        self.omega = np.zeros(3)
        self.acc_sensor = np.zeros(3)
        self.motor = np.zeros(4)

        self.ppos = [
            np.array([self.R, 0., 0.]),
            np.array([0., self.R, 0.]),
            np.array([-self.R, 0., 0.]),
            np.array([0., -self.R, 0.]),
        ]
        self.pdir = [1., -1., 1., -1.]

    def invrot(self):
        return self.rot.T

    def diff_matrix(self, omega, dt):
        olen = np.linalg.norm(omega)
        if olen:
            wx, wy, wz = omega / olen
        else:
            wx, wy, wz = 0., 0., 0.
        th = olen * dt
        K = np.array([
            [0., -wz, wy], 
            [wz, 0., -wx], 
            [-wy, wx, 0.],
        ])
        return np.eye(3) + np.sin(th) * K + (1. - np.cos(th)) * np.dot(K, K)
            # Rodrigue's formula; equivalent to exponential map exp(th*K)

    def lift(self, pomega):
        return self.LIFT_K * pomega

    def force(self, lifts): # internal frame
        f = np.array([0., 0., sum(lifts)])
        f -= self.DRAG_B * np.dot(self.invrot(), self.vel)
        return f

    def torque(self, lifts, pomega): # internal frame
        tau = np.zeros(3)
        for i in range(4):
            lf = np.array([0., 0., lifts[i]])
            tau += np.cross(self.ppos[i], lf)
        return tau

    @asyncio.coroutine
    def set_motors(self, motor):
        self.motor = motor

    def step(self, dt=5e-4):
        pomega = self.motor
        rot = self.rot
        lifts = [self.lift(x) for x in pomega]
        force_int = self.force(lifts)
        torque_int = self.torque(lifts, pomega)
        force_ref = np.dot(rot, force_int) + self.M * self.gvec
        torque_ref = np.dot(rot, torque_int)
        I_ref = np.dot(rot, self.I)
        omega_ref = self.omega
        
        self.acc_sensor = force_int / self.M
        acc_ref = force_ref / self.M
        rotacc_ref = np.dot(
            np.linalg.inv(I_ref),
            torque_ref - np.cross(omega_ref, np.dot(I_ref, omega_ref))
        )

        dmx = self.diff_matrix(self.omega + rotacc_ref * dt / 2., dt)
        self.rot = np.dot(dmx, self.rot)
        self.pos += self.vel * dt + acc_ref * dt**2 / 2.
        self.vel += acc_ref * dt
        self.omega += rotacc_ref * dt

    def get_time(self):
        return asyncio.get_event_loop().time()

    @asyncio.coroutine
    def get_sensors(self):
        acc = self.acc_sensor + np.random.normal(scale=self.noise_acc)
        omega = np.dot(self.invrot(), self.omega) + np.random.normal(scale=self.noise_omega)
        z = self.pos[2] + np.random.normal(scale=self.noise_z)
        return acc, omega, z

    def get_position(self):
        return self.pos

    def get_orientation(self):
        return np.dot(self.rot, np.array([0., 0., 1.]))

    def set_init(self, vel, omega):
        self.vel = np.array(vel, dtype=np.float64)
        self.omega = np.array(omega, dtype=np.float64)

    @asyncio.coroutine
    def _run(self):
        last_time = self.get_time()
        while True:
            try:
                yield from asyncio.sleep(0.0002)
                now = self.get_time()
                dt = now - last_time
                self.step(dt)
                last_time = now
            except asyncio.CancelledError:
                logger.debug('stop num_model simulation.')
                break
            except KeyboardInterrupt:
                logger.debug('capture ctrl-C in num_model.')
                break

    @asyncio.coroutine
    def start_control(self):
        self._worker = asyncio.get_event_loop().create_task(self._run())

    @asyncio.coroutine
    def get_ready(self):
        return True

    @asyncio.coroutine
    def stop(self):
        self._worker.cancel()
        logger.debug('stopping num model...')
        yield from asyncio.wait_for(self._worker, None)

    def alive(self):
        return len(self._worker.get_stack()) > 0

