#! /usr/bin/env python3

import numpy as np

class PID(object):
    def __init__(self, kp, kd, ki, ke, *, gamma=0):
        self.set_gain(kp, kd, ki, ke)
        self._gamma = gamma
        self._last_err = None
        self._int_err = 0

    def set_gain(self, kp, kd, ki, ke):
        self._kp = kp
        self._kd = kd
        self._ki = ki
        self._ke = ke

    def get_control(self, t, dt, err, derr=None):
        if self._last_err is None:
            self._last_err = err
        if derr is None:
            derr = (err - self._last_err) / dt
        self._int_err = self._int_err * self._ke + err * dt
        u = (np.dot(self._kp, err) + np.dot(self._kd, derr)
             + np.dot(self._ki, self._int_err))

        self._last_err = err

        self._tweak_parameters(dt, err, derr, self._int_err)

        return u

    def _tweak_parameters(self, dt, err, derr, ierr):
        self._kp -= self._gamma * err * err * dt
        self._kd -= self._gamma * err * derr * dt
        self._ki -= self._gamma * err * ierr * dt
        # pass
