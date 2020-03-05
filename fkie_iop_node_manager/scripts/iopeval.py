#!/usr/bin/env python

from __future__ import division, absolute_import, print_function, unicode_literals
import argparse
import sys

from fkie_iop_node_manager.statistics.eval_bytes import EvalBytes
from fkie_iop_node_manager.statistics.eval_connections import EvalConnections


def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def init_arg_parser():
    parser = argparse.ArgumentParser(description='Analyse statistics of IOP Node Manager.')
    parser.add_argument("mode", type=str, nargs='?', choices=['bytes', 'connections'], help="Select evaluation mode.")
    parser.add_argument("statfile", type=str, nargs='?', help="Path to file with statistics of IOP node manaer.")
    parser.add_argument("-i", "--interval", type=float, nargs='?', default=1.0, metavar="Hz", help="Evaluation interval in Hz.")
    parser.add_argument("--print_only", type=float, nargs='+', default=[], metavar="INTERVAL", help="Prints only the sspecified intervals.")
    return parser


if __name__ == '__main__':
    parser = init_arg_parser()
    parsed_args = parser.parse_args(sys.argv[1:])
    print_help = True
    if parsed_args.statfile is not None:
        if parsed_args.mode == 'bytes':
            print_help = False
            eb = EvalBytes(parsed_args.statfile, interval=parsed_args.interval)
            eb.evaluate()
            eb.stop()
        elif parsed_args.mode == 'connections':
            print_help = False
            ec = EvalConnections(parsed_args.statfile, interval=parsed_args.interval, print_only=parsed_args.print_only)
            ec.evaluate()
            ec.stop()
    if print_help:
        parser.print_help()
