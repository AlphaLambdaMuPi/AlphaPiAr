#! /usr/bin/env python

import asyncio
import logging
import numpy as np
import json

from .arduino import Arduino

logger = logging.getLogger()

class Drone:
    def __init__(self):
        self.loop = asyncio.get_event_loop()
        self.ready = asyncio.Future(loop=self.loop)
        self.g = 9.80

    @asyncio.coroutine
    def start_control(self):
        self.arduino = Arduino()
        yield from self.arduino.setup()

        logger.info('Self testing...')
        TEST_COUNT = 500
        t = 0
        accs = []
        omegas = []
        pressures = []
        while t < TEST_COUNT:
            s = yield from self.arduino.read_sensors()
            t += 1
            self.data = s
            acc = list(self.data['accel'])
            omega = list(self.data['gyro'])
            pressure = self.data['pressure']

            acc[2] -= self.g
            accs.append(np.array(acc))
            omegas.append(np.array(omega))
            pressures.append(np.array(pressure))

        accs = np.array(accs)
        omegas = np.array(omegas)
        pressures = np.array(pressures)

        self.acc0 = np.mean(accs, axis=0)
        self.omega0 = np.mean(omegas, axis=0)
        self.p0 = np.mean(pressures, axis=0)

        logger.info('Self test completed.')
        logger.info('acc0 : {} ± {}'.format(self.acc0, np.std(accs, axis=0)))
        logger.info('omega0 : {} ± {}'.format(self.omega0, np.std(omegas, axis=0)))
        logger.info('p0 : {} ± {}'.format(self.p0, np.std(pressures, axis=0)))

        self.ready.set_result(True)

    @asyncio.coroutine
    def get_ready(self):
        yield from self.ready
        return self.ready.result()

    @asyncio.coroutine
    def stop(self):
        self.arduino.close()

    def alive(self):
        return self.arduino.alive()

    @asyncio.coroutine
    def set_motors(self,motorcmd):
        motorcmd = list(map(int, np.minimum(motorcmd+1200, 1500)))
        motorcmd[1] = motorcmd[3] = 0
        yield from self.arduino.write_motors(motorcmd)

    def getacc(self):
        return self.data['accel'] - self.acc0

    def getomega(self):
        return self.data['gyro'] - self.omega0

    def getz(self):
        return (self.p0 - self.data['pressure']) * 0.083
    
    @asyncio.coroutine
    def get_sensors(self):
        self.data = yield from self.arduino.read_sensors()
        return self.getacc(), self.getomega(), self.getz()

rpi_drone = Drone()

