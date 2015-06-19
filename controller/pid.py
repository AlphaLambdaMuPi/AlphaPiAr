#! /usr/bin/env python3

import numpy as np

class PID(object):
    def __init__(self, kp, ki, kd, *, imax):
        self._last_time = None
        self._last_err = None
        self._int_err = 0
        self._imax = imax
        self.set_gain(kp, ki, kd)

    def set_gain(self, kp, ki, kd):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._int_restriction = self._imax / (self._ki + 1e-10)

    def get_control(self, t, err, derr=None):

        up = err * self._kp 

        if self._last_err is None:
            self._last_err = err

        if self._last_time is None:
            self._last_time = t
            return up

        dt = t - self._last_time + 1e-10
        if derr is None:
            derr = (err - self._last_err) / dt

        ud = derr * self._kd

        self._int_err += err * dt

        if self._int_err > self._int_restriction:
            self._int_err = self._int_restriction

        if self._int_err < -self._int_restriction:
            self._int_err = -self._int_restriction

        ui = self._int_err * self._ki

        self._last_err = err
        self._last_time = t

        return (up + ud + ui)

