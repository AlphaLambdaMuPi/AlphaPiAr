#! /usr/bin/env python

import os.path
import logging

def log_setup():
    logpath = "./"
    logfile = "drone.log"

    async_logger = logging.getLogger("asyncio")
    async_logger.setLevel(logging.WARNING)

    FORMAT = "%(asctime)s [%(module)s] [%(levelname)-5.5s] - %(message)s"

    logFormatter = logging.Formatter(FORMAT)

    logging.basicConfig(filename=os.path.join(logpath, logfile),
                        format=FORMAT, level=logging.DEBUG)
    rootLogger = logging.getLogger()

    consoleHandler = logging.StreamHandler()
    consoleHandler.setFormatter(logFormatter)
    rootLogger.addHandler(consoleHandler)
