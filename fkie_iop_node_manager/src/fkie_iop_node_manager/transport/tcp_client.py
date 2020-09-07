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

import socket
import threading
import traceback

import fkie_iop_node_manager.queue as queue
from fkie_iop_node_manager.addrbook import AddressBook
from fkie_iop_node_manager.message_parser import MessageParser
from .net import getaddrinfo
from fkie_iop_node_manager.logger import NMLogger


class TCPClient(socket.socket):

    def __init__(self, host='', port=0, router=None, interface='', logger_name='tcp_client', recv_buffer=5000, queue_length=0, loglevel='info'):
        '''
        :param str host: destination host.
        :param int port: destination port.
        :param router: class which provides `route_tcp_msg(fkie_iop_node_manager.message.Message)` method. If `None` receive will be disabled.
        '''
        self._closed = False
        self._connected = False
        self._connection_error_printed = False
        self.logger = NMLogger('%s[%s:%d]' % (logger_name, host, port), loglevel)
        self._router = router
        self._recv_buffer = recv_buffer
        self._queue_length = queue_length
        self._socket_type = socket.AF_INET
        self._queue_send = queue.PQueue(queue_length, 'queue_%s_send_%s:%d' % (logger_name, host, port), loglevel=loglevel)
        self._raddr = (host, port)
        self.interface = interface
        self._first_send_msg = True
        if self.interface:
            addrinfo = getaddrinfo(self.interface)
            self._socket_type = addrinfo[0]
        self._endpoint_client = AddressBook.Endpoint(AddressBook.Endpoint.TCP, host, port)
        self._message_parser = MessageParser(self._endpoint_client, stream=True, loglevel=loglevel)
        self._thread_connect = threading.Thread(target=self._connect, args=(self._raddr,))
        self._thread_connect.start()
        self._thread_send = threading.Thread(target=self._loop_send)
        self._thread_send.start()

    def __eq__(self, other):
        return self._raddr == other._raddr

    def close(self):
        self._closed = True
        self._connected = False
        self.logger.info("Close connection, own socket: %s" % str(self.getsockname()))
        self._queue_send.clear()
        try:
            # Important: Close read direction
            self.shutdown(socket.SHUT_RDWR)
        except Exception:
            self.logger.debug(traceback.format_exc())
        socket.socket.close(self)

    def send_queued(self, msg):
        try:
            self._queue_send.put(msg)
        except queue.Full as full:
            self.logger.debug(traceback.format_exc())
            self.logger.warning("Can't send message: %s" % full)
        except Exception as e:
            self.logger.warning("Error while put message into queue: %s" % e)

    def _loop_send(self):
        while not self._closed:
            try:
                # Waits for next available Message. get() method cancel waiting on clear() of PQueue and return None.
                msg = self._queue_send.get()
                if msg is not None:
                    try:
                        if self._connected:
                            self.logger.debug("Send message to %s:%d" % (self._raddr[0], self._raddr[1]))
                            self.send(msg.bytes(prepend_version=self._first_send_msg), socket.MSG_DONTWAIT)
                            self._first_send_msg = False
                        elif not self._thread_connect.is_alive():
                            self._thread_connect = threading.Thread(target=self._connect, args=(self._raddr,))
                            self._thread_connect.start()
                            self._queue_send.put(msg)
                    except Exception as err:
                        self.logger.debug("Error while send message through TCP: %s" % err)
                        self._connected = False
                        try:
                            # try to close on exception
                            socket.socket.shutdown(self, socket.SHUT_RDWR)
                        except Exception:
                            pass
                        try:
                            # reconnect on exception
                            self._thread_connect = threading.Thread(target=self._connect, args=(self._raddr,))
                            self._thread_connect.start()
                            self._queue_send.put(msg)
                        except Exception:
                            pass
            except Exception as serr:
                self.logger.debug("Error on send message to %s:%d: %s" % (self._raddr[0], self._raddr[1], serr))

    def _loop_recv(self):
        while self._connected:
            try:
                data = self.recv(self._recv_buffer)
                if data:
                    # parse the message and put it in recv queue
                    msgs = self._message_parser.unpack(data)
                    for msg in msgs:
                        self.logger.debug("Received from %s" % (msg.tinfo_src))
                        self._router.route_tcp_msg(msg)
                else:
                    self._connected = False
            except socket.timeout:
                pass
            except Exception:
                if not self._closed:
                    self.logger.warning(traceback.format_exc())
                else:
                    self.logger.debug(traceback.format_exc())

    def _connect(self, address):
        '''
        Tries to connect to a destination address. On success a thread with receive loop will be started.
        This method is called in a thread.
        '''
        try:
            self.logger.debug("Connecting to %s:%d" % (address[0], address[1]))
            socket.socket.__init__(self, self._socket_type, socket.SOCK_STREAM)
            self.connect(address)
            self._message_parser._version = None
            self.logger.debug("Connected to %s:%d" % (address[0], address[1]))
            self._connected = True
            self._connection_error_printed = False
            self._first_send_msg = True
            if self._router is not None:
                self._thread_recv = threading.Thread(target=self._loop_recv)
                self._thread_recv.start()
            if self._connection_error_printed:
                self.logger.info("Connected to %s:%d" % (address[0], address[1]))
        except Exception as err:
            if not self._connection_error_printed:
                self.logger.warning("Error while TCP connect: %s" % err)
                self._connection_error_printed = True
            else:
                self.logger.debug("Error while TCP connect: %s" % err)
