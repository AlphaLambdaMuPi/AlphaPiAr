#! /usr/bin/env python3

import numpy as np

class PID(object):
    def __init__(self, kp, kd, ki, imax):
        self._last_time = None
        self._last_err = None
        self._int_err = 0
        self._imax = imax
        self.set_gain(kp, kd, ki)

    def set_gain(self, kp, kd, ki):
        self._kp = kp
        self._kd = kd
        self._ki = ki
        self._int_restriction = self._imax / (self._ki + 1e-10)

    def get_control(self, t, err, derr=None):

        up = self._kp * err

        if self._last_err is None:
            self._last_err = err

        if self._last_time is None:
            self._last_time = t
            return up

        dt = t - self._last_time + 1e-10
        if derr is None:
            derr = (err - self._last_err) / dt

        ud = derr * kd

        self._int_err += self._int_err * dt

        if self._int_err > self._int_restriction:
            self._int_err = self._int_restriction

        if self._int_err < -self._int_restriction:
            self._int_err = -self._int_restriction

        ui = self._int_err * ki

        self._last_err = err
        self._last_time = t

        return up + ud + ui

