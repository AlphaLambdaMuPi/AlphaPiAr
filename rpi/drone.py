#! /usr/bin/env python

import asyncio
import logging

logger = logging.getLogger()

class Drone:
    def __init__(self):
        self.loop = asyncio.get_event_loop()
        self.ready = asyncio.Future(loop=self.loop)

    @asyncio.coroutine
    def start_control(self):
        self.p = yield from asyncio.create_subprocess_shell(
            'python arduino.py',
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
        )
        self.reader = self.p.stdout
        self.writer = self.p.stdin
        self._worker = self.loop.create_task(self._run())

        TEST_COUNT = 250
        t = 0
        accs = np.zeros(3)
        omegas = np.zeros(3)
        pressures = np.zeros(1)
        for i in range(5):
            s = yield from self.reader.readline()
        while t < TEST_COUNT:
            s = yield from self.reader.readline()
            t += 1
            dt = json.loads(s.decode())
            acc = dt['accel']
            omega = dt['gyro']
            pressure = dt['pressure']

            accs += np.array(acc)
            omegas += np.array(omega)
            pressures += np.array(pressure)

        self.acc0 = accs / TEST_COUNT
        self.acc0[2] -= 9.80
        self.omega0 = omegas / TEST_COUNT
        self.p0 = pressures / TEST_COUNT

        self.ready.set_result(True)

    @asyncio.coroutine
    def get_ready(self):
        yield from self.ready
        return self.ready.result()

    def set_motors(self,motorcmd):
        motorcmd = map(int, np.min(motorcmd+1200, 1700))
        self.writer.write((' '.join(map(str, motorcmd)) + '\n').encode())

    def getacc(self):
        return dt['accel'] - self.acc0

    def getomega(self):
        return dt['gyro'] - self.omega0

    def getz(self):
        return (self.p0 - self.data['pressure']) * 0.083

    def get_sensors(self):
        return self.getacc(), self.getomega(), self.getz()

    @asyncio.coroutine
    def _run(self):
        while True:
            try:
                data = yield from self.reader.readline()
                self.data = json.loads(data.decode())
            except:
                print('ConConCon')

rpi_drone = Drone()

