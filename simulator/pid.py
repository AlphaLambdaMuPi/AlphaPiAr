#! /usr/bin/env python3

import numpy as np

class PIDController:
    def __init__(self, kp, kd, ki, ke):
        self._kp = kp
        self._kd = kd
        self._ki = ki
        self._ke = ke
        self._last_err = 0
        self._int_err = 0

    def get_control(self, t, dt, now, des):
        err = des - now
        derr = err - self._last_err
        self._int_err = self._int_err * self._ke + err * dt
        u = (np.dot(self._kp, err) + np.dot(self._kd, derr)
             + np.dot(self._ki, self._int_err))

        self._last_err = err

        return u
