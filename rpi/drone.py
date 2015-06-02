#! /usr/bin/env python

import asyncio
import logging
import numpy as np
import json

logger = logging.getLogger()

class Drone:
    def __init__(self):
        self.loop = asyncio.get_event_loop()
        self.ready = asyncio.Future(loop=self.loop)
        self.g = 9.80

    def alive(self):
        return self.p.returncode is None

    @asyncio.coroutine
    def start_control(self):
        self.p = yield from asyncio.create_subprocess_shell(
            'python arduino.py',
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
        )
        self.reader = self.p.stdout
        self.writer = self.p.stdin

        logger.debug('Self testing...')
        TEST_COUNT = 250
        t = 0
        accs = []
        omegas = []
        pressures = []
        for i in range(5):
            s = yield from self.reader.readline()
        while t < TEST_COUNT:
            s = yield from self.reader.readline()
            t += 1
            self.data = json.loads(s.decode())
            acc = self.data['accel']
            omega = self.data['gyro']
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

        logger.debug('Self test completed.')
        logger.debug('acc0 : {} ± {}'.format(self.acc0, np.std(accs, axis=0)))
        logger.debug('omega0 : {} ± {}'.format(self.omega0, np.std(omegas, axis=0)))
        logger.debug('p0 : {} ± {}'.format(self.p0, np.std(pressures, axis=0)))

        self._worker = self.loop.create_task(self._run())
        self.ready.set_result(True)

    @asyncio.coroutine
    def get_ready(self):
        yield from self.ready
        return self.ready.result()

    def set_motors(self,motorcmd):
        motorcmd = map(int, np.minimum(motorcmd+1200, 1700))
        self.writer.write((' '.join(map(str, motorcmd)) + '\n').encode())

    def getacc(self):
        return self.data['accel'] - self.acc0

    def getomega(self):
        return self.data['gyro'] - self.omega0

    def getz(self):
        return (self.p0 - self.data['pressure']) * 0.083

    def get_sensors(self):
        return self.getacc(), self.getomega(), self.getz()

    @asyncio.coroutine
    def _run(self):
        while True:
            try:
                data = yield from self.reader.readline()
                tmp = json.loads(data.decode())
                if tmp['pressure'] < 90000:
                    continue
                self.data = tmp
            except Exception as epsilon:
                print('Epsilon', epsilon)
                print('Data : ', data)
                yield from asyncio.sleep(0.2)

rpi_drone = Drone()

