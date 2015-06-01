#!/usr/bin/env python

import asyncio
import websockets
import logging

from server import server

async_logger = logging.getLogger("asyncio")
async_logger.setLevel(logging.WARNING)

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def clear_tasks(loop):
    logger.info('clear tasks...')
    for task in asyncio.Task.all_tasks():
        task.cancel()
    try:
        loop.run_until_complete(asyncio.gather(*asyncio.Task.all_tasks()))
    except asyncio.CancelledError:
        logger.info('some tasks failed.')

if __name__ == "__main__":
    start_server = websockets.serve(server, 'localhost', 9007)

    loop = asyncio.get_event_loop()
    s = loop.run_until_complete(start_server)

    print('serving on', s.sockets[0].getsockname())
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        print()
        loop.run_until_complete(server.close())
        clear_tasks(loop)
    finally:
        loop.close()
        print("exit.")
