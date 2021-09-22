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

import errno
import os
import stat
import socket
import traceback
import time

from fkie_iop_node_manager.addrbook import AddressBook
from fkie_iop_node_manager.message_parser import MessageParser
from fkie_iop_node_manager.logger import NMLogger

class UDSSocket(socket.socket):
    '''
    Wrapper for Unix Domain Sockets.
    '''

    def __init__(self, name, remove_on_close=False, force_bind=False, root_path='/tmp', recv_buffer=5000, loglevel='info'):
        self._closed = False
        self.logger = NMLogger('uds[%s]' % name, loglevel)
        self._remove_on_close = remove_on_close
        self._recv_buffer = recv_buffer
        socket.socket.__init__(self, socket.AF_UNIX, socket.SOCK_DGRAM)
        self.setblocking(True)
        self._socket_path = os.path.join(root_path, name)
        self._parser = MessageParser(AddressBook.Endpoint(AddressBook.Endpoint.UDS, self._socket_path), loglevel=loglevel)
        if os.path.exists(self._socket_path) and not force_bind:
            self.logger.debug("Connect to local socket %s" % self._socket_path)
            self.connect(self._socket_path)
        else:
            self.logger.debug("Create local socket connection %s" % self._socket_path)
            if os.path.exists(self._socket_path):
                os.unlink(self._socket_path)
            self.bind(self._socket_path)
            os.chmod(self._socket_path, stat.S_IRWXU | stat.S_IRWXG | stat.S_IRWXO)

    @property
    def socket_path(self):
        return self._socket_path

    def reconnect(self):
        self.logger.debug("Reconnect to local socket %s" % self._socket_path)
        self.connect(self._socket_path)

    def close(self):
        '''
        Close the socket.
        '''
        self._closed = True
        self.logger.info("Close socket [%s]" % self._socket_path)
        self.shutdown(socket.SHUT_RDWR)
        socket.socket.close(self)
        if self._remove_on_close:
            try:
                os.unlink(self._socket_path)
            except OSError:
                if os.path.exists(self._socket_path):
                    raise

    def recv_msg(self):
        '''
        This method handles the received messages.
        '''
        running = True
        while not self._closed and running:
            try:
                data = self.recv(self._recv_buffer)
                if data:
                    return self._parser.unpack(data)
                else:
                    running = False
            except socket.error as err:
                err = err.args[0]
                if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                    #time.sleep(0.001)
                    continue
                else:
                    running = False
                    if not self._closed:
                        self.logger.warning("Reported socket error: %s" % traceback.format_exc())
        return []

    def send_msg(self, msg):
        '''
        This method sends the messages in the send queue.
        '''
        try:
            val = self.send(msg.bytes())
            return val == msg.raw_size
        except Exception as e:
            self.logger.warning("Error while send message [len: %d]: %s" % (len(msg.bytes()), e))
