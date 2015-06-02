#! /usr/bin/env python

import asyncio
import logging
from collections import namedtuple
import time
import numpy as np

from logsetting import log_setup
from client import Client
from pid import PIDController
from drone import rpi_drone

Server = namedtuple('Server', 'ip port')

log_setup()
logger = logging.getLogger()

def get_pid1():
    KPxy = 0.
    KPz = 0.
    KPt = 1.
    kp = np.vstack([np.diag([-KPxy, -KPxy, KPz]), np.zeros((3, 3))])
    kd = np.array([1.2]*3 + [0.05]*3)[np.newaxis].T * kp
    ki = 0.1 * kp
    ke = 0.9
    controller = PIDController(kp, kd, ki, ke)
    return controller

def get_pid2():
    KPxy = 5.
    KPz = 10.
    KPw = 10.
    # KPxy = 0.
    # KPz = 0.
    # KPw = 0.
    kpw1 = [KPxy, KPxy, KPz, KPw, -KPw, KPw]
    kpw2 = [KPxy, -KPxy, KPz, -KPw, -KPw, -KPw]
    kpw3 = [-KPxy, -KPxy, KPz, -KPw, KPw, KPw]
    kpw4 = [-KPxy, KPxy, KPz, KPw, KPw, -KPw]
    kp = np.array([kpw1, kpw2, kpw3, kpw4])
    kd = np.array([0.025]*3 + [0.02]*3) * kp
    ki = np.array([1/4]*3 + [1/8]*3) * kp
    ke = 0.9
    controller = PIDController(kp, kd, ki, ke)
    return controller

despos = np.array([0., 0., 0.])

@asyncio.coroutine
def start_control():
    global despos
    ctl1 = get_pid1()
    ctl2 = get_pid2()

    yield from rpi_drone.start_control()
    ready = yield from rpi_drone.get_ready()
    if not ready:
        return

    G = rpi_drone.g
    logger.info('detect G: {}'.format(G))
    ctl1 = get_pid1()
    ctl2 = get_pid2()
    action = np.array([0.] * 4)
    last_time = time.time()
    DTIME = 20e-3

    while rpi_drone.alive():
        try:
            while time.time() - last_time < DTIME:
                pass
            dt = time.time() - last_time
            last_time = time.time()
            acc, omega, z = rpi_drone.get_sensors()

            uacc = ctl1.get_control(last_time, dt, pos, despos)
            uacc[2] += G
            meas = np.array((acc, omega)).flatten()
            u = ctl2.get_control(last_time, dt, meas, uacc)
            action += u
            action = np.maximum.reduce([action, np.zeros(4)])
            self.drone.set_motors(action)
        except KeyboardInterrupt:
            break
    while sum(action) > 0.:
        action -= np.array([20, 20, 20, 20])
        yield from asyncio.sleep(0.02)



@asyncio.coroutine
def get_command(client):
    global despos

    yield from client._connected
    if not client._connected.result():
        return
    data = {'role': 'DRONE', 'name': 'RPI_DRONE'}
    client.send(data)

    ready = yield from rpi_drone.get_ready()
    data = {'status': ready}
    client.send(data)

    if not ready:
        client.close()
        return

    while client._connected.result() and not client._closed:
        data = yield from client.recv()
        # parse four number for motors control
        if data.get('cmd', None):
            pass
            despos = np.array(data)


    logger.debug("control connection closed.")



if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    s = Server('140.112.18.210', 12345)
    cc = Client(s)
    loop.run_until_complete(cc.connect())
    loop.create_task(start_control())
    loop.create_task(get_command(cc))
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        loop.run_until_complete(cc.close())
        print('exit.')
    finally:
        loop.close()

