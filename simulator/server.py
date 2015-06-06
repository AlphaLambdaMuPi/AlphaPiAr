#! /usr/bin/env python

import asyncio
import websockets
import json
import logging

from .simulator import Simulator

logger = logging.getLogger('websockets.protocol')
logger.setLevel(logging.WARNING)
logger = logging.getLogger()

class SimServer(object):
    def __init__(self):
        self._sim = Simulator()
        self._sim.run()
        self._conns = []

    @asyncio.coroutine
    def _send(self, ws, mes):
        try:
            yield from ws.send(mes.encode())
        except websockets.exceptions.InvalidState:
            logger.warning("EInvalidState")
            return "EInvalidState"
        except Exception:
            logger.warning("G__________G")
            yield from ws.close()

    @asyncio.coroutine
    def __call__(self, ws, uri):
        logger.info('concon connected')
        self._conns.append(ws)
        yield from self.run(ws)

    @asyncio.coroutine
    def run(self, ws):
        while ws.open:
            pos, ori = yield from self._sim.get_data()
            pos = list(pos)
            ori = list(ori.flatten())
            data = json.dumps({'pos':pos, 'ori':ori})
            yield from self._send(ws, data)
            yield from asyncio.sleep(0.02)

    @asyncio.coroutine
    def close(self):
        for ws in self._conns:
            yield from ws.close()
        yield from self._sim.stop()

