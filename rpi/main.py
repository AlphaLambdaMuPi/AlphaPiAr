#! /usr/bin/env python

import asyncio
import logging
import argparse
from collections import namedtuple
import time
import numpy as np

from .client import SocketClient, ConsoleClient
from .drone import rpi_drone

from controller import Controller

logger = logging.getLogger()

Server = namedtuple('Server', 'ip port')

@asyncio.coroutine
def cleanup(coros):
    logger.info('clean up...')
    for coro in coros:
        yield from coro

@asyncio.coroutine
def start_control(controller):
    try:
        rst = yield from controller.run()
        if not rst:
            logger.info("drone is not ready QQ")
    except asyncio.CancelledError:
        yield from controller.stop()
        logger.info("control stop.")
    
@asyncio.coroutine
def do_action(action, args):
    if not controller.stop_signal:
        print(action)
        yield from controller.preform_action(action, args)
        #if action == 'arm':
            #ret = yield from controller.arm()
        #elif action == 'thrust':
            #controller.set_thrust(*args)
        #elif action == 'angle':
            #controller.set_angle(*args)
        #elif action == 'tweak':
            #controller.tweak_pid(*args)
        #elif action == 'disarm':
            #ret = yield from controller.disarm()
        #elif action == 'stop':
            #yield from controller.stop()
            #logger.info('drone stopped.')
    else:
        pass
        # client.send({'Error': 'controller is stopped.'})

@asyncio.coroutine
def get_command(client, controller):
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

    while client.alive:
        data = yield from client.recv()
        if not data:
            continue
        # parse four number for motors control
        print(data)
        try:
            action = data['action']
            args = data['args']
            yield from do_action(action, args)
        except RuntimeError as e:
            logger.error('RuntimeError: {}'.format(e))
        except ValueError:
            logger.warning('wrong values')
        except KeyError:
            client.send({'Error': 'wrong parameters'})
        except IndexError:
            client.send({'Error': 'index out of range'})
        except TypeError as e:
            logger.warning('wrong type, {}'.format(e))

    logger.debug("control connection closed.")

def run_server():
    global controller
    controller = Controller(rpi_drone, log=True)
    loop = asyncio.get_event_loop()
    s = Server('140.112.18.210', 12345)
    sclient = SocketClient(s)
    # cclient = ConsoleClient()
    tasks = [
        loop.create_task(sclient.connect()),
        # loop.create_task(cclient.connect()),
        loop.create_task(get_command(sclient, controller)),
        # loop.create_task(get_command(cclient, controller)),
        loop.create_task(start_control(controller)),
    ]
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        logger.debug('capture ctrl-C in rpi main.')
        coros = [
            sclient.close(),
            # cclient.close(),
            controller.stop(),
            rpi_drone.stop(),
            asyncio.wait(tasks),
        ]
        loop.run_until_complete(cleanup(coros))
    finally:
        loop.close()
        logger.info('exit.')


