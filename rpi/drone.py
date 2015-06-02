#! /usr/bin/env python

import asyncio
import logging
from collections import namedtuple

from client import Client
from pid import PIDController

Server = namedtuple('Server', 'ip port')

def log_setup():
    async_logger = logging.getLogger("asyncio")
    # async_logger.setLevel(logging.WARNING)
    async_logger.setLevel(logging.DEBUG)

    FORMAT = "%(asctime)s [%(module)s] [%(levelname)-5.5s] - %(message)s"

    logFormatter = logging.Formatter(FORMAT)

    logging.basicConfig(format=FORMAT, level=logging.DEBUG)

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

ctl1 = get_pid1()
ctl2 = get_pid2()

@asyncio.coroutine
def start_control(client):
    yield from client._connected
    if not client._connected.result():
        return
    data = {'role': 'DRONE', 'name': 'TEST_RPI_DRONE'}
    client.send(data)
    p = yield from asyncio.create_subprocess_shell(
        'python arduino.py',
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.PIPE,
    )
    reader = p.stdout
    writer = p.stdin
    while client._connected.result() and not client._closed:
        data = yield from client.recv()
        # parse four number for motors control
        despos = np.array(data)
        writer.write(motorcmd.encode())

    print("connection closed.")

def main():
    loop = asyncio.get_event_loop()
    s = Server('140.112.18.210', 12345)
    cc = Client(s)
    loop.run_until_complete(cc.connect())
    loop.create_task(start_control(cc))
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        loop.run_until_complete(cc.close())
        print('exit.')
    finally:
        loop.close()

if __name__ == "__main__":
    main()

