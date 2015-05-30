#! /usr/bin/env python3

import asyncio
from asyncio.queues import Queue, QueueEmpty
import json
import logging

from connection import JsonConnection

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
        try:
            sr, sw = yield from asyncio.open_connection(
                self._server.ip,
                self._server.port,
                loop=self._loop
            )
            self._conn = JsonConnection(sr, sw, loop=self._loop)
        except ConnectionError:
            logger.debug('connection failed')
            self._connected.set_result(False)
            return
        logger.debug('connection made')
        self._connected.set_result(True)

    @asyncio.coroutine
    def recv(self):
        yield from self._connected
        if not self._connected.result():
            return
        yield from self._conn.recv()

    def send(self, data):
        if self._connected.done():
            self._conn.send(data)

    @asyncio.coroutine
    def close(self):
        yield from self._connected
        if not self._connected.result():
            return
        yield from self._conn.close()
        self._closed = True

