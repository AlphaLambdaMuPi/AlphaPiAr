#! /usr/bin/env python3

import sys
import asyncio
from asyncio.queues import Queue, QueueEmpty
import json
import logging

from .connection import JsonConnection, ConsoleConnection

logger = logging.getLogger()

class Client(object):
    def __init__(self, *, loop=None):
        if not loop:
            loop = asyncio.get_event_loop()
        self._loop = loop
        self._conn = None

        self.connected = asyncio.Future(loop=self._loop)
        self.alive = False

    @asyncio.coroutine
    def connect(self):
        try:
            yield from self._connect()
        except (asyncio.CancelledError, KeyboardInterrupt):
            self.connected.set_result(False)

        if not self.connected.result():
            logger.warning('connection was cancelled or failed.')
        self.alive = self.connected.result()

    def _connect(self):
        """classes inherits from this class should override this method to
        connect and set the result in self.connected.
        """
        self.connected.set_result(False)

    @asyncio.coroutine
    def recv(self):
        res = yield from self.connected
        if not res:
            return
        return (yield from self._conn.recv())

    def send(self, data):
        if self.connected.done():
            self._conn.send(data)
            return True
        else:
            return False

    @asyncio.coroutine
    def close(self):
        if not self.connected.done():
            self.connected.cancel()
        elif self.connected.result():
            yield from self._conn.close()
        self.alive = False

class SocketClient(Client):
    def __init__(self, server, *, loop=None):
        super().__init__(loop=loop)
        self._server = server

    @asyncio.coroutine
    def _connect(self):
        while not self.connected.done():
            try:
                sr, sw = yield from asyncio.open_connection(
                    self._server.ip,
                    self._server.port,
                    loop=self._loop
                )
                self._conn = JsonConnection(sr, sw, loop=self._loop)
                self.connected.set_result(True)
                logger.info('connection with drone server was made')
            except ConnectionError:
                logger.warning('connection failed, retry in 10 seconds.')
                yield from asyncio.sleep(10)
            except KeyboardInterrupt:
                logger.info('capture ctrl-C, cancel the connection.')
                break

class ConsoleClient(Client):
    def __init__(self, parser=None, *, loop=None):
        super().__init__(loop=loop)
        self._parser = parser

    @asyncio.coroutine
    def _connect(self):
        reader = asyncio.StreamReader()
        protocol = asyncio.StreamReaderProtocol(reader)
        yield from self._loop.connect_read_pipe(
            lambda: protocol,
            sys.stdin
        )
        tp, _ = yield from self._loop.connect_write_pipe(
            asyncio.BaseProtocol,
            sys.stdout
        )
        writer = asyncio.StreamWriter(tp, protocol, reader, self._loop)
        self._conn = ConsoleConnection(reader, writer, loop=self._loop)
        self.connected.set_result(True)

    @asyncio.coroutine
    def recv(self):
        res = yield from super().recv()
        if res is None or self._parser is None:
            return res
        return self._parser.parse_args(res.split())

