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

    while client.alive:
        data = yield from client.recv()
        if not data:
            continue
        # parse four number for motors control
        try:
            data = data.split()
            cmd = data[0]
            args = data[1:]
            if not controller.stop_signal:
                if cmd == 'T':
                    ret = yield from controller.takeoff()
                elif cmd == 'M':
                    args = int(args[0])
                    if args == 1:
                        controller.offset = 0.1
                    elif args == 2:
                        controller.offset = 0.
                    elif args == 3:
                        controller.offset = -0.1
                elif cmd == 'P':
                    # testing rotation
                    kp, kd, ki = map(float, args)
                    kp, kd, ki, ke = controller._pids['th'].gen_gain(
                        kkp=kp, kkd=kd, kki=ki
                    )
                    controller._pids['th'].set_gain(kp, kd, ki, ke)
                    logger.info('set pid.')
                elif cmd == 'R':
                    # testing rotation
                    a = float(args[0])
                    controller._restriction = a
                    controller._action[0] = controller._action[2] = a
                    logger.info('set restriction.')
                elif cmd == 'E':
                    acc, th, omg, z = yield from rpi_drone.get_sensors()
                    logger.info('theta: {}, omega: {}'.format(th, omg))
                    logger.info('action: {}'.format(controller._action))
                elif cmd == 'S':
                    yield from controller.stop()
                    logger.info('drone stopped.')
            else:
                yield from client.send(
                    {'Error': 'controller is stopped.'}
                )
        except RuntimeError:
            pass
        except ValueError:
            logger.warning('wrong values')
        except KeyError:
            yield from client.send({'Error': 'wrong parameters'})
        except IndexError:
            yield from client.send({'Error': 'index out of range'})
        except TypeError as e:
            logger.warning('wrong type, {}'.format(e))

    logger.debug("control connection closed.")


def run_server():
    loop = asyncio.get_event_loop()
    s = Server('140.112.18.210', 12345)
    # cc = SocketClient(s)
    cc = ConsoleClient()
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


