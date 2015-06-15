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
        try:
            action = data['action']
            args = data['args']
            if not controller.stop_signal:
                if action == 'T':
                    ret = yield from controller.takeoff()
                elif action == 'M':
                    args = np.array([int(x) for x in args])
                    controller.set_action(args)
                    logger.info('set action to {}'.format(args))
                elif action == 'P':
                    # testing rotation
                    kp, kd, ki = map(float, args)
                    kp, kd, ki, ke = controller._pids['th'].gen_gain(
                        kkp=kp, kkd=kd, kki=ki
                    )
                    controller._pids['th'].set_gain(kp, kd, ki, ke)
                    logger.info('set pid.')
                elif action == 'R':
                    # testing rotation
                    controller._restriction = float(args[0])
                    client.send({
                    })
                    logger.info('set restriction.')
                elif action == 'E':
                    acc, omg, z = yield from rpi_drone.get_sensors()
                    client.send({
                        'accel': acc,
                        'omega': omg,
                        'z': z,
                        'time': rpi_drone.data['time']
                    })
                    logger.info('theta: {}, omega: {}'.format(th, omg))
                    logger.info('action: {}'.format(controller._action))
                elif action == 'S':
                    yield from controller.stop()
                    logger.info('drone stopped.')
            else:
                client.send({'Error': 'controller is stopped.'})
        except RuntimeError:
            pass
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
    cc = SocketClient(s)
    # cc = ConsoleClient()
    tasks = [
        loop.create_task(cc.connect()),
        loop.create_task(get_command(cc, controller)),
        loop.create_task(start_control(controller)),
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


