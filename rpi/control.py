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
def read_from_stdin(client):
    loop = asyncio.get_event_loop()
    reader = asyncio.StreamReader()
    reader_protocol = asyncio.StreamReaderProtocol(reader)
    yield from loop.connect_read_pipe(lambda: reader_protocol, sys.stdin)
    print("Enter control name:", end='')
    data = yield from reader.readline()
    data = {'role': 'CONTROL', 'name': data}
    client.send(data)
    print("Enter drone name:", end='')
    data = yield from reader.readline()
    data = {'target': data}
    client.send(data)
    print("Enter your motors command (four number each command):")
    while client._connected.result() and not client._closed:
        data = yield from reader.readline()
        data = data.decode()
        # parse four number for motors control
        motors = [int(x) for x in data.split()]
        client.send(motors)

    print("connection closed.")

def main():
    loop = asyncio.get_event_loop()
    s = Server('140.112.18.210', 12345)
    cc = Client(s)
    loop.run_until_complete(cc.connect())
    loop.create_task(read_from_stdin(cc))
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        loop.run_until_complete(cc.close())
        print('exit.')
    finally:
        loop.close()

if __name__ == "__main__":
    main()

