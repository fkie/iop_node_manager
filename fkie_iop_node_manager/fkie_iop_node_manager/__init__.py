# ****************************************************************************
#
# fkie_iop_node_manager
# Copyright 2019 Fraunhofer FKIE
# Author: Alexander Tiderko
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ****************************************************************************

from __future__ import division, absolute_import, print_function, unicode_literals

import argparse
import sys
import logging

from .server import Server

__version__ = '0.9.0'

server = None


def set_terminal_name(name):
    '''
    Change the terminal name.

    :param str name: New name of the terminal
    '''
    sys.stdout.write("\x1b]2;%s\x07" % name)


def set_process_name(name):
    '''
    Change the process name.

    :param str name: name new process name
    '''
    try:
        from ctypes import cdll, byref, create_string_buffer
        libc = cdll.LoadLibrary('libc.so.6')
        buff = create_string_buffer(len(name) + 1)
        buff.value = name
        libc.prctl(15, byref(buff), 0, 0, 0)
    except Exception:
        pass


def init_arg_parser():
    global __version__
    parser = argparse.ArgumentParser()
    parser.add_argument("--version", action="version", version="%s" % (__version__))
    parser.add_argument("-c", "--config", nargs='?', type=str, default='', help="path to configuration. (default: ${HOME}/.config/iop.fkie/node_manager.yaml)")
    parser.add_argument("-s", "--save-config", action="store_true", help="Saves default configuration to ${HOME}/.config/iop.fkie/node_manager.yaml")
    return parser


def start(name, block=True, argv=None, params={}):
    '''
    Start the IOP NodeManager

    :param str name: the process name
    :param bool block: block until Ctrl+C or shutdown signal
    :param list argv: if argv is None uses sys.argv parameter (used in testunit)
    '''
    logging.basicConfig(level=logging.INFO)
    set_terminal_name(name)
    set_process_name(name)
    parser = init_arg_parser()
    parsed_args, remaining_args = parser.parse_known_args()
    global __version__
    global server
    try:
        cfg = ''
        if hasattr(parsed_args, "config"):
            cfg = parsed_args.config
        server = Server(cfg, __version__, params=params)
        save_config = False
        if hasattr(parsed_args, "save_config"):
            save_config = parsed_args.save_config
        if save_config:
            logging.info("save configuration to %s" % server.cfg.filename)
            server.cfg.save(reset=True, save_msg_ids=True)
        server.start(block)
    except Exception as err:
        import traceback
        print("Error while start node manager: %s" % (traceback.format_exc()), file=sys.stderr)
    if block:
        shutdown()


def shutdown():
    global server
    server.shutdown()
    print("Bye")


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
        class DummyNode(Node):
            def __init__(self):
                super().__init__("iop_node_manager")

        rclpy.init(args=args)
        node = DummyNode()
        #rospy.names.reload_mappings(sys.argv)
        #mappings = rospy.names.get_mappings()
        #name = "iop_node_manager"
        #if '__name' in mappings:
        #    name = mappings['__name']
        # params = rospy.get_param(rosgraph.names.resolve_name(name, rospy.core.get_caller_id()), {})
        # start(name, block=False, params=params)
        start(node.get_name(), block=False)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('stop node manager')
    except Exception as err:
        import traceback
        print(traceback.format_exc())
        print("Error while initialize ROS-Node: %s" % (err), file=sys.stderr)
    finally:
        try:
            shutdown()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
