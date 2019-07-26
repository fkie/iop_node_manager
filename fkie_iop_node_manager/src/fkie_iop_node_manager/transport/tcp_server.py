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

import logging
import socket
import threading
import time
import traceback

from .net import getaddrinfo, is_local_iface
from .tcp_client import TCPClient
from .tcp_input import TCPInput


class TCPServer(socket.socket):

    def __init__(self, port=0, router=None, interface='', logger_name='tcp', recv_buffer=5000, queue_length=0):
        '''
        :param int port: the port to bind the socket. If zero an empty one will be used.
        :param router: class which provides `route_tcp_msg(fkie_iop_node_manager.message.Message)` method. If `None` receive will be disabled.
        :param str interface: The interface to bind to. If empty, it binds to all interfaces
        '''
        self._closed = False
        self._lock = threading.RLock()
        self.logger = logging.getLogger('%s[%s:%d]' % (logger_name, interface, port))
        self.interface = interface
        self.port = port
        self._router = router
        self._recv_buffer = recv_buffer
        self._queue_length = queue_length
        self._socket_type = socket.AF_INET
        bind_ip = self.interface
        if self.interface:
            addrinfo = getaddrinfo(self.interface)
            self._socket_type = addrinfo[0]
            bind_ip = addrinfo[4][0]
        socket.socket.__init__(self, self._socket_type, socket.SOCK_STREAM)
        self._address = (bind_ip, self.port)
        self._message_parser = {}
        self._clients = {}
        self._thread_bind = threading.Thread(target=self._bind_with_retry)
        self._thread_bind.start()

    def _bind_with_retry(self):
        '''
        Try to bind to the socket until it is available or node manager is stopped.
        '''
        ok = False
        self.logger.info("+ Bind to @(%s:%s)" % (self._address[0], self._address[1]))
        while not ok and not self._closed:
            try:
                self.bind((self._address[0], self._address[1]))
                self.listen(5)
                # create a thread to handle the received unicast messages
                self._thread_loop = threading.Thread(target=self._loop)
                self._thread_loop.start()
                ok = True
                self.logger.info("server ready")
            except Exception as err:
                self.logger.error("TCP bind failed: %s, next try in 5 seconds..." % err)
                time.sleep(5)

    def close(self):
        self._closed = True
        self.logger.info("Close socket")
        self.logger.debug("Close %s clients: %s" % (len(self._clients), str(self._clients)))
        for _dst, conn in self._clients.items():
            conn.close()
        try:
            # Important: Close read direction
            self.shutdown(socket.SHUT_RDWR)
        except Exception:
            self.logger.debug(traceback.format_exc())
        socket.socket.close(self)

    def send_queued(self, msg):
        try:
            if msg.tinfo_dst is not None:
                dst = msg.tinfo_dst.address_tuple()
                try:
                    self._clients[dst].send_queued(msg)
                except KeyError:
                    if not is_local_iface(dst[0]):
                        tcp_client = TCPClient(dst[0], port=dst[1], queue_recv=self._router, interface=self.interface, recv_buffer=self._recv_buffer, queue_length=self._queue_length)
                        tcp_client.send_queued(msg)
                        self._clients[dst] = tcp_client
            else:
                # send to all destinations if no specified
                for _dst, conn in self._clients.items():
                    conn.send_queued(msg)
        except Exception as err:
            self.logger.debug(traceback.format_exc())
            self.logger.warning("Error while send message through TCP: %s" % err)

    def _loop(self):
        while not self._closed:
            try:
                connection, client_address = self.accept()
                self.logger.debug("Add new input connection from %s" % str(client_address))
                tcp_input = TCPInput(connection, queue_recv=self._router, recv_buffer=self._recv_buffer, queue_length=self._queue_length, close_callback=self._close_callback)
                with self._lock:
                    if client_address in self._clients:
                        self._clients[client_address].close()
                    self._clients[client_address] = tcp_input
            except OSError:
                pass
            except Exception as rerr:
                if rerr.errno != 22:
                    self.logger.debug("Error in receive loop: %s", traceback.format_exc())

    def _close_callback(self, connection):
        with self._lock:
            if connection.getpeername() in self._clients:
                self.logger.debug("Remove connection %s", str(connection))
                self._clients[connection.getpeername()].close()
                del self._clients[connection.getpeername()]
