#! /usr/bin/env python3

import numpy as np

class PIDController:
    def __init__(self, kp, kd, ki, ke):
        self._kp = kp
        self._kd = kd
        self._ki = ki
        self._ke = ke
        self._last_err = None
        self._int_err = 0

    def get_control(self, t, dt, now, des):
        err = des - now
        if self._last_err is None:
            self._last_err = err
        derr = (err - self._last_err) / dt
        self._int_err = self._int_err * self._ke + err * dt
        u = (np.dot(self._kp, err) + np.dot(self._kd, derr)
             + np.dot(self._ki, self._int_err))

        self._last_err = err

        return u
