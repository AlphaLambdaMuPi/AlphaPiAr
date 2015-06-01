#! /usr/bin/env python

import asyncio
import websockets
import json
import logging

from simulator import sim

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SimServer:
    def __init__(self):
        self.simdata = sim.run()
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
            pos, ori = next(self.simdata)
            pos = list(pos)
            ori = list(ori.flatten())
            data = json.dumps({'pos':pos, 'ori':ori})
            yield from self._send(ws, data)

    @asyncio.coroutine
    def close(self):
        for ws in self._conns:
            yield from ws.close()

server = SimServer()
