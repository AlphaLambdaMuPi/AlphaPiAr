#! /usr/bin/env python3

import asyncio
from asyncio.queues import Queue, QueueEmpty
import json
import logging

logger = logging.getLogger()

class StreamConnection(object):
    def __init__(self, sr, sw, *, loop=None):
        if not loop:
            loop = asyncio.get_event_loop()
        self._loop = loop
        self._sr = sr
        self._sw = sw
        self._msgs = Queue(loop=loop)
        self._worker = loop.create_task(self._run())

    @asyncio.coroutine
    def _run(self):
        while self.alive():
            try:
                data = yield from self._sr.readline()
                data.strip()
                if data:
                    self._msgs.put_nowait(self._convert(data))
            except asyncio.CancelledError:
                logger.debug("readline from stream reader was cancelled.")
            except ConnectionError:
                logger.debug("connection error")
                break
            except KeyboardInterrupt:
                logger.debug("capture ctrl-C in connection, ignored.")

        logger.debug("connection closed")

    def _convert(self, data):
        return data

    @asyncio.coroutine
    def recv(self):
        try:
            return self._msgs.get_nowait()
        except QueueEmpty:
            pass

        # Wait for a message until the connection is closed
        next_message = self._loop.create_task(self._msgs.get())
        done, pending = yield from asyncio.wait(
                [next_message, self._worker],
                loop=self._loop, return_when=asyncio.FIRST_COMPLETED)
        if next_message in done:
            return next_message.result()
        else:
            next_message.cancel()

    def send(self, data):
        if not self.alive():
            raise ConnectionError("connection was closed.")
        try:
            data = data + b'\n'
            self._sw.write(data)
        except OSError:
            raise ConnectionError("can't send data.")
        except Exception as e:
            logger.critical("unexpected exception: {}".format(e))

    def alive(self):
        return not self._sr.at_eof()

    @asyncio.coroutine
    def drain():
        yield from self._sw.drain()
    
    @asyncio.coroutine
    def close(self):
        if self.alive():
            try:
                yield from self._sw.drain()
                self._sw.write_eof()
            except ConnectionError:
                pass
            else:
                self._sr.feed_eof()
                self._sw.close()
        self._worker.cancel()

class JsonConnection(StreamConnection):
    def __init__(self, sr, sw, *, loop=None):
        super().__init__(sr, sw, loop=loop)

    def _convert(self, data):
        data = super()._convert(data)
        try:
            data = json.loads(data.decode())
        except UnicodeError:
            logger.warning("can't convert byte to string")
        except ValueError:
            logger.warning("get wrong json format")
        else:
            return data

    def send(self, data):
        try:
            logger.debug("send: {}".format(data))
            data = json.dumps(data).encode()
            super().send(data)
        except ValueError:
            raise ValueError("wrong json format")

class ConsoleConnection(StreamConnection):
    def __init__(self, sr, sw, *, loop=None):
        super().__init__(sr, sw, loop=loop)

    def _convert(self, data):
        data = super()._convert(data)
        try:
            data = data.decode().split()
            data = {'action': data[0], 'args': data[1:]}
        except UnicodeError:
            logger.warning("can't convert byte to string")
        else:
            return data

    def send(self, data):
        try:
            logger.debug("send: {}".format(data))
            data = json.dumps(data).encode()
            super().send(data)
        except ValueError:
            raise ValueError("wrong json format")

