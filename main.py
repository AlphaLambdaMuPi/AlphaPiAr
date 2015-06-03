#! /usr/bin/env python3

import logging
import argparse

from rpi import run_server as rpi_server
from simulator import run_server as sim_server

from logsetting import log_setup

parser = argparse.ArgumentParser(description='drone control and simulation.')
parser.add_argument('-m', '--mode', help='execution mode', type=str,
                    choices=['sim', 'rpi'], default='sim')
parser.add_argument('-f', '--logfile', help='log file', default=None)
parser.add_argument('-l', '--loglevel', help='log level',
                    choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
                    default='DEBUG')

if __name__ == '__main__':
    args = parser.parse_args()
    log_setup(level=args.loglevel, filepath=args.logfile)
    if args.mode == 'sim':
        sim_server()
    elif args.mode == 'rpi':
        rpi_server()
