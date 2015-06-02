#! /usr/bin/env python

import asyncio
import logging
from collections import namedtuple
import time
import numpy as np

from logsetting import log_setup
from client import Client
from controller import Controller
from drone import rpi_drone

Server = namedtuple('Server', 'ip port')

controller = Controller(rpi_drone)

log_setup()
logger = logging.getLogger()

@asyncio.coroutine
def start_control():
    rst = yield from controller.run()

    if not rst:
        logger.info("drone is not ready QQ")
    

@asyncio.coroutine
def get_command(client):

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
        if data == 1:
            controller.offset = 0.1
        elif data == 2:
            controller.offset = 0.
        elif data == 3:
            controller.offset = -0.1
        elif data == 0:
            controller.stop()

        # parse four number for motors control
        if data.get('cmd', None):
            pass
            # controller.set_despos(np.array(data['despos']))

    logger.debug("control connection closed.")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    s = Server('140.112.18.210', 12345)
    cc = Client(s)
    loop.create_task(start_control())
    loop.create_task(get_command(cc))
    loop.run_until_complete(cc.connect())
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        loop.run_until_complete(cc.close())
        print('exit.')
    finally:
        loop.close()


