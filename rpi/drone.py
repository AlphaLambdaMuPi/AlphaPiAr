#! /usr/bin/env python

import asyncio
import logging
from collections import namedtuple

from client import Client

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

@asyncio.coroutine
def start_control(client):
    yield from client._connected
    if not client._connected.result():
        return
    loop = asyncio.get_event_loop()
    p = yield from asyncio.create_subprocess_shell(
        'python test.py',
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.PIPE,
    )
    reader = p.stdout
    writer = p.stdin
    while client._connected.result() and not client._closed:
        data = yield from client.recv()
        # parse four number for motors control
        motorcmd = ' '.join(data)
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

