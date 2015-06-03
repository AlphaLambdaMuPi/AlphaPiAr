#!/usr/bin/env python

import asyncio
import websockets
import logging

from .server import SimServer

async_logger = logging.getLogger("asyncio")
async_logger.setLevel(logging.WARNING)

logger = logging.getLogger()

def clear_tasks(loop):
    logger.info('clear tasks...')
    for task in asyncio.Task.all_tasks():
        task.cancel()
    try:
        loop.run_until_complete(asyncio.gather(*asyncio.Task.all_tasks()))
    except asyncio.CancelledError:
        logger.info('some tasks failed.')

def run_server():
    server = SimServer()
    start_server = websockets.serve(server, 'localhost', 9007)

    loop = asyncio.get_event_loop()
    s = loop.run_until_complete(start_server)

    logger.info(
        'simulation is serving on {}'.format(s.sockets[0].getsockname())
    )
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        logger.debug('capture ctrl-C in sim main.')
        loop.run_until_complete(server.close())
    finally:
        loop.close()
        logger.info("exit.")

