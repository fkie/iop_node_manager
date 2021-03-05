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
import socket
import threading
import time
import traceback

import fkie_iop_node_manager.queue as queue
from fkie_iop_node_manager.addrbook import AddressBook
from fkie_iop_node_manager.message_parser import MessageParser
from fkie_iop_node_manager.message import Message
from .net import getaddrinfo, localifs
from fkie_iop_node_manager.logger import NMLogger


class UDPucSocket(socket.socket):

    def __init__(self, port=0, router=None, addrbook=None, interface='', logger_name='udp', default_dst=None, send_buffer=0, recv_buffer=0, queue_length=0, loglevel='info'):
        '''
        Creates a socket, bind it to a given interface+port for unicast send/receive.
        IPv4 and IPv6 are supported.

        :param int port: the port to bind the socket. If zero an empty one will be used.
        :param router: class which provides `route_udp_msg(fkie_iop_node_manager.message.Message)` method. If `None` receive will be disabled.
        :param str interface: The interface to bind to. If empty, it binds to all interfaces
        :param tuple(str,int) default_dst: used for loopback to send messages to predefined destination.
        '''
        self._closed = False
        self.logger = NMLogger('%s[%s:%d]' % (logger_name, interface, port), loglevel)
        self.interface = interface
        self.port = port
        self._router = router
        self._addrbook = addrbook
        self._default_dst = default_dst
        self._locals = [ip for _ifname, ip in localifs()]
        self._locals.append('localhost')
        self._recv_buffer = recv_buffer
        self._sender_endpoints = {}
        self.sock_5_error_printed = []
        # If interface isn't specified, try to find an non localhost interface to
        # get some info for binding. Otherwise use localhost
        # if not self.interface:
        #     ifaces = localifs()
        #     for iface in ifaces:
        #         if not (iface[1].startswith('127') or iface[1].startswith('::1')):
        #             self.interface = iface[1]
        #             break
        self.logger.info("+ Bind to unicast socket @(%s:%s)" % (self.interface, port))
        socket_type = socket.AF_INET
        bind_ip = self.interface
        if self.interface:
            addrinfo = getaddrinfo(self.interface)
            socket_type = addrinfo[0]
            bind_ip = addrinfo[4][0]
            # Configure socket type
        socket.socket.__init__(self, socket_type, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        # Bind to the port
        try:
            self.logger.debug("Ucast bind to: (%s:%s)" % (bind_ip, port))
            self.bind((bind_ip, port))
        except socket.error as errobj:
            msg = str(errobj)
            self.logger.critical("Unable to bind unicast to interface: %s, check that it exists: %s" % (bind_ip, msg))
            raise
        if self.port == 0:
            self.port = self.getsockname()[1]
        if send_buffer:
            # update buffer size
            old_bufsize = self.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
            if old_bufsize != send_buffer:
                self.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, send_buffer)
#                self.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, buffersize)
                bufsize = self.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
                self.logger.debug("Changed buffer size from %d to %d" % (old_bufsize, bufsize))
        self._parser_ucast = MessageParser(None, loglevel=loglevel)
        self._queue_send = queue.PQueue(queue_length, 'queue_%s_send' % logger_name, loglevel=loglevel)
        # create a thread to handle the received unicast messages
        if self._router is not None:
            self._thread_recv = threading.Thread(target=self._loop_recv)
            self._thread_recv.start()
        self._thread_send = threading.Thread(target=self._loop_send)
        self._thread_send.start()

    def close(self):
        """ Cleanup and close the socket"""
        self._closed = True
        self.logger.info("Close unicast socket")
        try:
            # shutdown to cancel recvfrom()
            socket.socket.shutdown(self, socket.SHUT_RD)
        except socket.error:
            pass
        socket.socket.close(self)
        self._queue_send.clear()

    def send_queued(self, msg):
        try:
            self._queue_send.put(msg)
        except queue.Full as full:
            print(traceback.format_exc())
            self.logger.warning("Can't send message: %s" % full)
        except Exception as e:
            self.logger.warning("Error while put message into queue: %s" % e)

    def _loop_send(self):
        while not self._closed:
            # Waits for next available Message. This method cancel waiting on clear() of PQueue and return None.
            msg = self._queue_send.get()
            if msg is not None:
                dst = msg.tinfo_dst
                if self._default_dst is not None:
                    # it is a loopback socket, send to fictive debug destination
                    dst = AddressBook.Endpoint(AddressBook.Endpoint.UDP, self._default_dst[0], self._default_dst[1])
                if dst is not None:
                    # send to given addresses
                    self._sendto(msg.bytes(), dst.address, dst.port)
                else:
                    # send to local clients through UDP connections
                    for local_dst in self._addrbook.get_local_udp_destinations():
                        self._sendto(msg, local_dst.address, local_dst.port)
            # TODO: add retry mechanism?

    def _sendto(self, msg, addr, port):
        '''
        Sends the given message to the joined multicast group. Some errors on send
        will be ignored (``ENETRESET``, ``ENETDOWN``, ``ENETUNREACH``)

        :param str msg: message to send
        :param str addr: IPv4 or IPv6 address
        :param int port: destination port
        '''
        try:
            self.logger.debug("Send to %s:%d" % (addr, port))
            self.sendto(msg, (addr, port))
        except socket.error as errobj:
            msg = str(errobj)
            if errobj.errno in [-5]:
                if addr not in self.sock_5_error_printed:
                    self.logger.warning("socket.error[%d]: %s, addr: %s" % (errobj.errno, msg, addr))
                    self.sock_5_error_printed.append(addr)
            elif errobj.errno in [errno.EINVAL, -2]:
                raise
            elif errobj.errno not in [errno.ENETDOWN, errno.ENETUNREACH, errno.ENETRESET]:
                raise

    def _loop_recv(self):
        '''
        This method handles the received unicast messages.
        '''
        while not self._closed:
            try:
                (data, address) = self.recvfrom(self._recv_buffer)
                if data and not self._closed:
                    msgs = self._parser_ucast.unpack(data)
                    for msg in msgs:
                        if msg.dst_id.zero or msg.cmd_code > 0:
                            # handle connection requests/closing
                            try:
                                if msg.cmd_code == Message.CODE_CONNECT:
                                    # Connection request from client.
                                    self.logger.debug("Connection request from %s" % msg.src_id)
                                    resp = Message()
                                    resp.version = Message.AS5669
                                    resp.dst_id = msg.src_id
                                    resp.cmd_code = Message.CODE_ACCEPT
                                    resp.ts_receive = time.time()
                                    resp.tinfo_src = AddressBook.Endpoint(AddressBook.Endpoint.UDP_LOCAL, self.mgroup, self.getsockname()[1])
                                    resp.tinfo_dst = AddressBook.Endpoint(AddressBook.Endpoint.UDP_LOCAL, address[0], address[1])
                                    self._addrbook.add_jaus_address(msg.src_id, address=address[0], port=address[1], ep_type=AddressBook.Endpoint.UDP_LOCAL)
                                    self.send_queued(resp)
                                elif msg.cmd_code == Message.CODE_CANCEL:
                                    # Disconnect client.
                                    self.logger.debug("Disconnect request from %s" % msg.src_id)
                                    self._addrbook.remove(msg.src_id)
                            except Exception as e:
                                import traceback
                                print(traceback.format_exc())
                                self.logger.warning("Error while handle connection management message: %s" % e)
                        else:
                            try:
                                msg.tinfo_src = self._sender_endpoints[address]
                            except KeyError:
                                etype = AddressBook.Endpoint.UDP
                                if address[0] in self._locals:
                                    etype = AddressBook.Endpoint.UDP_LOCAL
                                endpoint = AddressBook.Endpoint(etype, address[0], address[1])
                                msg.tinfo_src = endpoint
                                self._sender_endpoints[address] = endpoint
                            # self.logger.debug("Received from %s" % (msg.tinfo_src))
                            self._router.route_udp_msg(msg)
            except queue.Full as full_error:
                self.logger.warning("Error while process received unicast message: %s" % full_error)
            except socket.error:
                if not self._closed:
                    self.logger.warning("unicast socket error: %s" % traceback.format_exc())
