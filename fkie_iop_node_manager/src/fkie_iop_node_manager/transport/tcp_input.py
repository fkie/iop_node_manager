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
from fkie_iop_node_manager.logger import NMLogger


class TCPInput(object):

    def __init__(self, connection, router=None, logger_name='tcp_input', recv_buffer=5000, queue_length=0, close_callback=None, loglevel='info'):
        '''
        :param (str,int) connection: client address.
        :param router: class which provides `route_tcp_msg(fkie_iop_node_manager.message.Message)` method. If `None` receive will be disabled.
        '''
        self._closed = False
        self._send_error_printed = False
        self._connection = connection
        self._raddr = connection.getpeername()
        self.logger = NMLogger('%s[%s:%d]' % (logger_name, self._raddr[0], self._raddr[1]), loglevel)
        self._router = router
        self._recv_buffer = recv_buffer
        self._queue_length = queue_length
        self._close_callback = close_callback
        self._first_send_msg = True
        self._queue_send = queue.PQueue(queue_length, 'queue_%s_send_%s:%d' % (logger_name, self._raddr[0], self._raddr[1]), loglevel=loglevel)
        self._endpoint_client = AddressBook.Endpoint(AddressBook.Endpoint.TCP, self._raddr[0], self._raddr[1])
        self._message_parser = MessageParser(self._endpoint_client, stream=True, loglevel=loglevel)
        self._thread_send = threading.Thread(target=self._loop_send)
        self._thread_send.start()
        if self._router is not None:
            self._thread_recv = threading.Thread(target=self._loop_recv)
            self._thread_recv.start()

    def getpeername(self):
        return self._raddr

    def __eq__(self, other):
        return self._raddr == other._raddr

    def close(self):
        self._closed = True
        self.logger.info("Close input connection, own socket: %s" % str(self._connection.getsockname()))
        self._queue_send.clear()
        try:
            # Important: Close read and direction
            self._connection.shutdown(socket.SHUT_RDWR)
        except Exception:
            self.logger.debug(traceback.format_exc())
        self._connection.close()

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
                        self.logger.debug("Send message to %s:%d" % (self._raddr[0], self._raddr[1]))
                        self._connection.sendall(msg.bytes(prepend_version=self._first_send_msg), socket.MSG_DONTWAIT)
                        self._first_send_msg = False
                    except Exception as err:
                        self.logger.debug("Error while send message through TCP input connection: %s" % err)
                        self._connected = False
                        try:
                            self._connection.shutdown(socket.SHUT_RD)
                        except Exception as errshd:
                            print("ERR shutdown on send:", errshd)
            except Exception as serr:
                self.logger.debug("Error on send message to %s:%d: %s" % (self._raddr[0], self._raddr[1], serr))

    def _loop_recv(self):
        while not self._closed:
            try:
                data = self._connection.recv(self._recv_buffer)
                if data:
                    # parse the message and put it in recv queue
                    msgs = self._message_parser.unpack(data)
                    for msg in msgs:
                        self.logger.debug("Received from %s" % (msg.tinfo_src))
                        self._router.route_tcp_msg(msg)
                else:
                    if self._close_callback:
                        self._close_callback(self)
                    try:
                        # try to shutdown the connection
                        self._connection.shutdown(socket.SHUT_RD)
                    except Exception as errshd:
                        print("ERR shutdown on recv:", errshd)
            except socket.timeout:
                pass
            except Exception:
                if not self._closed:
                    self.logger.warning(traceback.format_exc())
