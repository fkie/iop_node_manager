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
import struct
import threading
import time
import traceback

import fkie_iop_node_manager.queue as queue
from fkie_iop_node_manager.addrbook import AddressBook
from fkie_iop_node_manager.message_parser import MessageParser
from fkie_iop_node_manager.message import Message
from .net import getaddrinfo, localifs
from fkie_iop_node_manager.logger import NMLogger

SEND_ERRORS = {}


class UDPmcSocket(socket.socket):
    '''
    The UdpSocket class enables the send and receive UDP messages to a
    multicast group and unicast address. The unicast socket is only created if
    'send_mcast' and 'listen_mcast' parameter are set to False or a specific interface is defined.
    '''

    def __init__(self, port, mgroup, router=None, addrbook=None, ttl=16, interface='', logger_name='udp_mc', send_buffer=0, recv_buffer=0, queue_length=0, loglevel='info'):
        '''
        Creates a socket, bind it to a given port and join to a given multicast
        group. IPv4 and IPv6 are supported.

        :param int port: the port to bind the socket
        :param str mgroup: the multicast group to join
        :param router: class which provides `route_udp_msg(fkie_iop_node_manager.message.Message)` method. If `None` receive will be disabled.
        :param type: fkie_iop_node_manager.queue
        :param int ttl: time to leave (Default: 20)
        :param str interface: IP of interface to bind (Default: '').
        '''
        self.logger = NMLogger('%s[%s:%d]' % (logger_name, mgroup.replace('.', '_'), port), loglevel)
        self.port = port
        self.mgroup = mgroup
        self._addrbook = addrbook
        self._lock = threading.RLock()
        self._closed = False
        self._recv_buffer = recv_buffer
        self._locals = [ip for _ifname, ip in localifs()]
        self._locals.append('localhost')
        self._sender_endpoints = {}
        self.sock_5_error_printed = []
        self.SOKET_ERRORS_NEEDS_RECONNECT = False
        self.interface = interface
        # get the AF_INET information for group to ensure that the address family
        # of group is the same as for interface
        addrinfo = getaddrinfo(self.mgroup)
        self.interface_ip = ''
        if self.interface:
            addrinfo = getaddrinfo(self.interface, addrinfo[0])
            if addrinfo is not None:
                self.interface_ip = addrinfo[4][0]
        self.logger.debug("destination: %s" % self.mgroup)
        self.logger.debug("interface : %s (detected ip: %s)" % (self.interface, self.interface_ip))
        self.logger.debug("inet: %s" % str(addrinfo))

        socket.socket.__init__(self, addrinfo[0], socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.logger.info("Create multicast socket @('%s', %d)" % (self.mgroup, port))
        # initialize multicast socket
        # Allow multiple copies of this program on one machine
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except Exception:
                self.logger.warning("SO_REUSEPORT failed: Protocol not available, some functions are not available.")
        # Set Time-to-live (optional) and loop count
        ttl_bin = struct.pack('@i', ttl)
        if addrinfo[0] == socket.AF_INET:  # IPv4
            self.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl_bin)
            self.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        else:  # IPv6
            self.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_HOPS, ttl_bin)
            self.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_LOOP, 1)

        try:
            if addrinfo[0] == socket.AF_INET:  # IPv4
                # Create group_bin for de-register later
                # Set socket options for multicast specific interface or general
                if not self.interface_ip:
                    self.group_bin = socket.inet_pton(socket.AF_INET, self.mgroup) + struct.pack('=I', socket.INADDR_ANY)
                    self.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                    self.group_bin)
                else:
                    self.group_bin = socket.inet_aton(self.mgroup) + socket.inet_aton(self.interface_ip)
                    self.setsockopt(socket.IPPROTO_IP,
                                    socket.IP_MULTICAST_IF,
                                    socket.inet_aton(self.interface_ip))
                    self.setsockopt(socket.IPPROTO_IP,
                                    socket.IP_ADD_MEMBERSHIP,
                                    self.group_bin)
            else:  # IPv6
                # Create group_bin for de-register later
                # Set socket options for multicast
                self.group_bin = socket.inet_pton(addrinfo[0], self.mgroup) + struct.pack('@I', socket.INADDR_ANY)
                self.setsockopt(socket.IPPROTO_IPV6,
                                socket.IPV6_JOIN_GROUP,
                                self.group_bin)
        except socket.error as errobj:
            msg = str(errobj)
            if errobj.errno in [errno.ENODEV]:
                msg = "socket.error[%d]: %s,\nis multicast route set? e.g. sudo route add -net 224.0.0.0 netmask 224.0.0.0 eth0" % (errobj.errno, msg)
            raise Exception(msg)

        # set buffer size if configured
        if send_buffer:
            old_bufsize = self.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
            if old_bufsize != send_buffer:
                self.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, send_buffer)
                bufsize = self.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
                self.logger.debug("Changed buffer size from %d to %d" % (old_bufsize, bufsize))

        # Bind to the port
        try:
            # bind to default interfaces if not unicast socket was created
            self.bind((self.interface_ip, port))
        except socket.error as errobj:
            msg = str(errobj)
            self.logger.critical("Unable to bind multicast to interface: %s, check that it exists: %s" % (self.mgroup, msg))
            raise
        self._router = router
        self._queue_send = queue.PQueue(queue_length, 'queue_udp_send', loglevel=loglevel)
        self._parser_mcast = MessageParser(None, loglevel=loglevel)
        self.addrinfo = addrinfo
        # create a thread to handle the received multicast messages
        if self._router is not None:
            self._thread_recv = threading.Thread(target=self._loop_recv)
            self._thread_recv.start()
        self._thread_send = threading.Thread(target=self._loop_send)
        self._thread_send.start()

    def close(self):
        '''
        Unregister from the multicast group and close the socket.
        '''
        self._closed = True
        self.logger.info("Close multicast socket")
        try:
            # shutdown to cancel recvfrom()
            socket.socket.shutdown(self, socket.SHUT_RD)
        except socket.error:
            pass
        # Use the stored group_bin to de-register
        if self.addrinfo[0] == socket.AF_INET:  # IPv4
            self.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, self.group_bin)
        else:  # IPv6
            self.setsockopt(socket.IPPROTO_IPV6,
                            socket.IPV6_LEAVE_GROUP,
                            self.group_bin)
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
            # Wait for next available Message. This method cancel waiting on clear() of PQueue and return None.
            msg = self._queue_send.get()
            if msg is not None:
                dst = msg.tinfo_dst
                if dst is None:
                    if msg.tinfo_src.etype == AddressBook.Endpoint.UDP_LOCAL:
                        self._sendto(msg, AddressBook.Endpoint(AddressBook.Endpoint.UDP, self.mgroup, self.getsockname()[1]))
                    # send to local clients through UDP connections
                    for local_dst in self._addrbook.get_local_udp_destinations():
                        if local_dst != msg.tinfo_src:
                            self._sendto(msg, local_dst)
                elif msg.tinfo_dst.etype in [AddressBook.Endpoint.UDP, AddressBook.Endpoint.UDP_LOCAL]:
                    self._sendto(msg, dst)
                #else:
                #    self.logger.warning("Can't send message to %s, destination not found!" % (dst))
                # TODO: add retry mechanism

    def _sendto(self, msg, endpoint):
        # send to given addresses
        try:
            self.logger.debug("Send to %s:%d" % (endpoint.address, endpoint.port))
            val = self.sendto(msg.bytes(), (endpoint.address, endpoint.port))
            if val != msg.raw_size:
                raise Exception("not complete send %d of %d" % (val, msg.raw_size))
            if endpoint.address in SEND_ERRORS:
                del SEND_ERRORS[endpoint.address]
        except socket.error as errobj:
            erro_msg = "Error while send to '%s': %s" % (endpoint.address, errobj)
            SEND_ERRORS[endpoint.address] = erro_msg
            # -2: Name or service not known
            if errobj.errno in [-5, -2]:
                if endpoint.address not in self.sock_5_error_printed:
                    self.logger.warning(erro_msg)
                    self.sock_5_error_printed.append(endpoint.address)
            else:
                self.logger.warning(erro_msg)
            if errobj.errno in [errno.ENETDOWN, errno.ENETUNREACH, errno.ENETRESET, errno]:
                self.SOKET_ERRORS_NEEDS_RECONNECT = True
        except Exception as e:
            erro_msg = "Send to host '%s' failed: %s" % (endpoint.address, e)
            self.logger.warning(erro_msg)
            SEND_ERRORS[endpoint.address] = erro_msg

    def _loop_recv(self):
        '''
        This method handles the received multicast messages.
        '''
        while not self._closed:
            try:
                (data, address) = self.recvfrom(self._recv_buffer)
                if data and not self._closed and not (address[0] in self._locals and self.port == address[1]):  # skip messages received from self
                    msgs = self._parser_mcast.unpack(data)
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
            except socket.timeout:
                pass
            except queue.Full as full_error:
                self.logger.warning("Error while process received multicast message: %s" % full_error)
            except socket.error:
                if not self._closed:
                    self.logger.warning("socket error: %s" % traceback.format_exc())
