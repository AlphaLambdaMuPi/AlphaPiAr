import numpy as np

class Momentum:
    def __init__(self, tau=0.03):
        self._tau = tau
        self._value = None
        self._last_time = None

    def set_value(self, value):
        '''
        Set the value, not to confuse with append_value
        '''
        self._value = value

    def append_value(self, time, value):
        if self._value is None or self._last_time is None:
            self._last_time = time
            self._value = value
            return value

        dt = time - self._last_time 
        alpha = np.exp(-dt/self._tau)
        self._value = self._value * alpha + value * (1. - alpha)

        return self._value
        
        
