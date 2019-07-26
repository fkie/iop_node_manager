#!/usr/bin/env python

from __future__ import division, absolute_import, print_function, unicode_literals
import argparse
import socket
import sys


def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def init_arg_parser():
    parser = argparse.ArgumentParser(description='Change parameter on running IOP Node Manager.')
    parser.add_argument("--host", type=str, default='localhost',
                        help="Port for dynamic configuration.")
    parser.add_argument("-p", "--port", type=int, default=37940,
                        help="Port for dynamic configuration.")
    parser.add_argument("--loglevel", type=str, choices=['debug', 'info', 'warning', 'error', 'critical'],
                        help="Set new logging level.")
    parser.add_argument("--statistic", type=str2bool, choices=[True, False],
                        help="Enable / Disable statistic logs.")
    return parser


def send_parameter(params, host='localhost', port=37940):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        sock.sendall(params, socket.MSG_DONTWAIT)
        sock.shutdown(socket.SHUT_RD)
        sock.close()
        print("parameter sent successfully")
    except Exception as err:
        # import traceback
        # print(traceback.format_exc())
        print("Can not send params to '%s:%s': %s" % (host, port, err), file=sys.stderr)


if __name__ == '__main__':
    parser = init_arg_parser()
    parsed_args = parser.parse_args(sys.argv[1:])
    print_help = True
    params = ''
    if parsed_args.loglevel is not None:
        print_help = False
        params += 'global/loglevel: %s\n' % parsed_args.loglevel
    if parsed_args.statistic is not None:
        print_help = False
        params += 'global/statistics/enable: %s\n' % parsed_args.statistic
    if print_help:
        parser.print_help()
    else:
        print("Send to %s:%s: %s" % (parsed_args.host, parsed_args.port, params))
        send_parameter(params, parsed_args.host, parsed_args.port)
