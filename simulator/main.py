#!/usr/bin/env python

import asyncio
import websockets
import logging

from .server import SimServer

async_logger = logging.getLogger("asyncio")
async_logger.setLevel(logging.WARNING)

logger = logging.getLogger()

@asyncio.coroutine
def cleanup(coros):
    logger.info('clean up...')
    for coro in coros:
        yield from coro

def run_server():
    server = SimServer()
    start_server = websockets.serve(server, 'localhost', 9007)

    loop = asyncio.get_event_loop()
    try:
        s = loop.run_until_complete(start_server)
        logger.info(
            'simulation is serving on {}'.format(s.sockets[0].getsockname())
        )
        loop.run_forever()
    except KeyboardInterrupt:
        logger.debug('capture ctrl-C in sim main.')
        coros = [
            server.close()
        ]
        loop.run_until_complete(cleanup(coros))
    finally:
        loop.close()
        logger.info("exit.")

