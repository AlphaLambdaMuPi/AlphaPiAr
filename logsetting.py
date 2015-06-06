#! /usr/bin/env python

import os.path
import logging

def log_setup(*, level=logging.DEBUG, filepath=None):
    async_logger = logging.getLogger("asyncio")
    async_logger.setLevel(logging.WARNING)

    FORMAT = "%(asctime)s [%(module)s] [%(levelname)-5.5s] - %(message)s"

    logging.basicConfig(format=FORMAT, level=level)
    rootLogger = logging.getLogger()
    
    logFormatter = logging.Formatter(FORMAT)

    if filepath:
        fileHandler = logging.FileHandler(filepath)
        fileHandler.setFormatter(logFormatter)
        rootLogger.addHandler(fileHandler)
