#! /usr/bin/env python3

import asyncio
import logging
import matplotlib.pyplot as plt
import numpy as np

from .num_model import Drone

from controller import Controller

logger = logging.getLogger()

np.set_printoptions(precision=4, suppress=True)

class Simulator:
    def __init__(self):
        self.drone = Drone()
        self.controller = Controller(self.drone)
        self.loop = asyncio.get_event_loop()
        self.drone.set_init([0., 0., 0.], [2., 0., 0.])
        self.AOO = []
        # self.drone.dt = 5e-4
        # self.drone.noise_z = 1e-10

    def run(self):
        self.loop.call_soon_threadsafe(
            self.loop.create_task,
            self.controller.run()
        )

    @asyncio.coroutine
    def get_data(self):
        pos = self.drone.get_position()
        ori = self.drone.rot
        oori = ori[:, 2]
        # self.AOO.append(self.drone.acc_sensor[2])
        self.AOO.append(oori)
        return pos, ori

    @asyncio.coroutine
    def stop(self):
        yield from self.controller.stop()
        yield from self.drone.stop()
        logger.debug('plotting...')
        plt.plot(self.AOO)
        plt.show()

sim = Simulator()
