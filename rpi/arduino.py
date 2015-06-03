#! /usr/bin/env python3

import sys
import asyncio
import json
import random
import struct
import serial

class Arduino:
    def __init__(self, *, loop=None):
        if loop:
            self._loop = loop
        else:
            self._loop = asyncio.get_event_loop()

        self._waitings = asyncio.Queue(loop=self._loop)
        self._loop.add_reader(self._ser.fileno(), self._get_data)
    
    def _get_data(self):
        try:
            fut, s = self._waitings.get_nowait()
        except asyncio.QueueEmpty:
            logger.error('arduino yapsilon!!!!')
        else:
            res = self._ser.read(s)
            fut.set_result(res)

    def communicate(self, cmd, size=None):
        w = None
        if size:
            w = asyncio.Future(loop=self._loop)
            self._waitings.put_nowait((w, size))
        self._ser.write(cmd)
        self._ser.flush()
        return w

    @asyncio.coroutine
    def setup(self):
        self._ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=3)
        while True:
            ret = yield from self.communicate(b'S', 1)
            if ret == b's':
                logger.info('Connected with Arduino.')
                break
            elif ret == b'':
                logger.info('Waiting for Arduino...')
            else:
                logger.error('Communication with Arduino failed !!'
                             ' (received {})'.format(ret))

    def decode_sensors(self, b):
        res = []
        ret = {}
        for i in range(11):
            res.append(struct.unpack('f', b[4*i:4*(i+1)]))
        
        ret['accel'] = res[0:3]
        ret['gyro'] = res[3:6]
        ret['mag'] = res[6:9]
        ret['temperature'] = res[9]
        ret['pressure'] = res[10]
        ret['time'] = self._loop.time()

        return ret

    @asyncio.coroutine
    def read_sensors(self):
        # ax, ay, az, gx, gy, gz, mx, my, mz, temp, pres
        data = yield from self.communicate(b'R', 4*11)
        data = self.decode_sensor(data)
        return data

    @asyncio.coroutine
    def write_motors(motors):
        '''
        send control signals to drone's motors via arduino
        motors - a list containing four int numbers which indicate
                 the control signals for drone's motors.
        '''
        self.communicate(b'M')
        st = struct.pack('hhhh', motors)
        res = yield from self.communicate(st, 1)
        if res != b'm':
            raise IOError('Write motor to Arduino failed !!!')

if __name__ == "__main__":
    loop = asyncio.get_event_loop()

    arduino = Arduino()

    @asyncio.coroutine
    def read_stdin():
        reader = asyncio.StreamReader()
        reader_protocol = asyncio.StreamReaderProtocol(reader)
        yield from loop.connect_read_pipe(lambda: reader_protocol, sys.stdin)
        while True:
            data = (yield from reader.readline()).encode().strip()
            motors = [int(x) for x in data.split()]
            try:
                yield from arduino.write_motors(motors)
            except IOError as e:
                logger.error(e)
    
    try:
        loop.run_until_complete(read_stdin())
    except KeyboardInterrupt:
        print("stoping..., please press Ctrl-C again")
    finally:
        print('exit.')

