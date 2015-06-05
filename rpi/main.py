#! /usr/bin/env python

import asyncio
import logging
from collections import namedtuple
import time
import numpy as np

from .client import SocketClient, ConsoleClient
from .drone import rpi_drone

from controller import Controller

logger = logging.getLogger()

Server = namedtuple('Server', 'ip port')
controller = Controller(rpi_drone, log=True)

@asyncio.coroutine
def cleanup(coros):
    logger.info('clean up...')
    for coro in coros:
        yield from coro

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

    res = yield from client.connected
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

    while not client.alive:
        data = yield from client.recv()
        # parse four number for motors control
        try:
            cmd = data['action']
            mag = data['mag']
            if cmd == 'T':
                ret = yield from controller.takeoff()
            elif cmd == 'M':
                if not controller.stop_signal:
                    if mag == 1:
                        controller.offset = 0.1
                    elif mag == 2:
                        controller.offset = 0.
                    elif mag == 3:
                        controller.offset = -0.1
                else:
                    yield from client.send(
                        {'Error': 'controller is stopped.'}
                    )

            elif cmd == 'S':
                controller.stop()
        except RuntimeError:
            pass
        except KeyError:
            yield from client.send({'Error': 'wrong parameters'})

    logger.debug("control connection closed.")

def run_server():
    loop = asyncio.get_event_loop()
    s = Server('140.112.18.210', 12345)
    cc = SocketClient(s)
    tasks = [
        loop.create_task(cc.connect()),
        loop.create_task(get_command(cc)),
        loop.create_task(start_control()),
    ]
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        logger.debug('capture ctrl-C in rpi main.')
        coros = [
            cc.close(),
            controller.stop(),
            rpi_drone.stop(),
            asyncio.wait(tasks),
        ]
        loop.run_until_complete(cleanup(coros))
    finally:
        loop.close()
        logger.info('exit.')


