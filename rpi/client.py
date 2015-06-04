#! /usr/bin/env python3

import asyncio
from asyncio.queues import Queue, QueueEmpty
import json
import logging

from .connection import JsonConnection

logger = logging.getLogger()

class Client:
    def __init__(self, server, *, loop=None):
        if not loop:
            loop = asyncio.get_event_loop()
        self._loop = loop
        self._msgs = Queue(loop=loop)
        self._server = server
        self._connected = asyncio.Future(loop=loop)
        self._conn = None
        self._closed = False

    @asyncio.coroutine
    def connect(self):
        @asyncio.coroutine
        def _connect():
            while not self._connected.done():
                try:
                    sr, sw = yield from asyncio.open_connection(
                        self._server.ip,
                        self._server.port,
                        loop=self._loop
                    )
                    self._conn = JsonConnection(sr, sw, loop=self._loop)
                    self._connected.set_result(True)
                    logger.info('connection with drone server was made')
                except ConnectionError:
                    logger.warning('connection failed, retry in 10 seconds.')
                    yield from asyncio.sleep(10)
                except KeyboardInterrupt:
                    logger.info('capture ctrl-C, cancel the connection.')
                    break

        try:
            yield from _connect()
        except (asyncio.CancelledError, KeyboardInterrupt):
            self._connected.set_result(False)

        if not self._connected.result():
            logger.info('connection was cancelled')

    @asyncio.coroutine
    def recv(self):
        res = yield from self._connected
        if not res:
            return
        return (yield from self._conn.recv())

    def send(self, data):
        if self._connected.done():
            self._conn.send(data)

    @asyncio.coroutine
    def close(self):
        if not self._connected.done():
            self._connected.cancel()
        elif self._connected.result():
            yield from self._conn.close()
            self._closed = True

