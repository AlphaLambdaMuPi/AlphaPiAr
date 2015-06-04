#! /usr/bin/env python

import asyncio
import logging
from collections import namedtuple
import time
import numpy as np

from .client import Client
from .drone import rpi_drone

from controller import Controller

logger = logging.getLogger()

Server = namedtuple('Server', 'ip port')
controller = Controller(rpi_drone, log=True)

@asyncio.coroutine
def cleanup(client, controller, drone, tasks):
    logger.info('clean up...')
    yield from client.close()
    yield from controller.stop()
    yield from drone.stop()
    done, pending = yield from asyncio.wait(tasks)

@asyncio.coroutine
def start_control():
    try:
        rst = yield from controller.run()
        if not rst:
            logger.info("drone is not ready QQ")
    except asyncio.CancelledError:
        yield from controller.stop()
        logger.info("control stop.")
    

@asyncio.coroutine
def get_command(client):

    res = yield from client._connected
    if not res:
        return

    data = {'role': 'DRONE', 'name': 'RPI_DRONE'}
    client.send(data)

    ready = yield from rpi_drone.get_ready()
    logger.debug("drone ready")

    data = {'status': ready}
    client.send(data)

    if not ready:
        client.close()
        return

    while client._connected.result() and not client._closed:
        data = yield from client.recv()
        # parse four number for motors control
        if not controller.stopped.done():
            if data == 1:
                controller.offset = 0.1
            elif data == 2:
                controller.offset = 0.
            elif data == 3:
                controller.offset = -0.1
            elif data == 0:
                controller.stop()
            logger.debug("current target g: {}".format(controller.offset + controller.drone.g))

    logger.debug("control connection closed.")

def run_server():
    loop = asyncio.get_event_loop()
    s = Server('140.112.18.210', 12345)
    cc = Client(s)
    tasks = [
        loop.create_task(cc.connect()),
        loop.create_task(get_command(cc)),
        loop.create_task(start_control()),
    ]
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        logger.debug('capture ctrl-C in rpi main.')
        loop.run_until_complete(cleanup(cc, controller, rpi_drone, tasks))
    finally:
        loop.close()
        logger.info('exit.')


