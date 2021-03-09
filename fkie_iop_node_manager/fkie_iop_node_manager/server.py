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

import threading
import time

from .addrbook import AddressBook
from .config import Config
from .statistics.collector import Collector
from .transport.uds_server import UDSServer
from .transport.udp_mc import UDPmcSocket
from .transport.udp_uc import UDPucSocket
from .transport.tcp_server import TCPServer

from fkie_iop_node_manager.logger import NMLogger


class Server():

    def __init__(self, cfg_file, version='', params={}):
        self.cfg = Config(cfg_file, version, params)
        loglevel = self.cfg.param('global/loglevel', 'info')
        self.logger = NMLogger('server', loglevel)
        self.cfg.init_cfgif()
        self._stop = False
        default_port = self.cfg.param('transport/udp/port', 3794)
        addrbook_udp = self.cfg.param('addrbook/udp', {})
        addrbook_tcp = self.cfg.param('addrbook/tcp', {})
        self.addrbook = AddressBook(default_port=default_port, addrbook_udp=addrbook_udp, addrbook_tcp=addrbook_tcp, loglevel=loglevel)
        self.statistics = Collector(self.cfg)
        self._local_mngr = None
        self._udp = None
        self._tcp_server = None
        self._lock = threading.RLock()

    def start(self, block=True):
        # self._callback_change_loglevel('global/loglevel', self.cfg.param('global/loglevel', 'info'))
        self._local_mngr = UDSServer(self, self.cfg, self.addrbook, self.statistics)
        self._on_discover = {}
        port = self.cfg.param('transport/udp/port', 3794)
        mgroup = self.cfg.param('transport/udp/group', '239.255.0.1')
        ttl = self.cfg.param('transport/udp/ttl', 16)
        use_mcast = self.cfg.param('transport/udp/use_mcast', '')
        interface = self.cfg.param('transport/udp/interface', '')
        buffer_size = self.cfg.param('transport/udp/buffer_size', 0)
        queue_length = self.cfg.param('transport/udp/queue_length', 0)
        if use_mcast:
            self._udp = UDPmcSocket(port, mgroup, router=self, addrbook=self.addrbook, ttl=ttl, interface=interface, send_buffer=buffer_size, recv_buffer=self.cfg.RECV_BUFFER, queue_length=queue_length, loglevel=self.logger.level())
        else:
            self._udp = UDPucSocket(port, router=self, addrbook=self.addrbook, interface=interface, send_buffer=buffer_size, recv_buffer=self.cfg.RECV_BUFFER, queue_length=queue_length, loglevel=self.logger.level())
        # create TCP server
        tcp_enabled = self.cfg.param('transport/tcp/enable', False)
        self._tcp_server = None
        if tcp_enabled:
            tcp_port = self.cfg.param('transport/tcp/port', 3794)
            tcp_interface = self.cfg.param('transport/tcp/interface', '')
            tcp_queue_length = self.cfg.param('transport/tcp/queue_length', 0)
            self._tcp_server = TCPServer(port=tcp_port, router=self, interface=tcp_interface, logger_name='tcp', recv_buffer=self.cfg.RECV_BUFFER, queue_length=tcp_queue_length, loglevel=self.logger.level())
        try:
            while block:
                time.sleep(1)
        except KeyboardInterrupt:
            print("caught keyboard interrupt, exiting")

    def route_local_msg(self, msg):
        try:
            if msg.dst_id.has_wildcards():
                # it is a broadcast message, try send to all matched locals (except sender)
                self.logger.debug("send 0x%.4X broadcast from %s" % (msg.msg_id, msg.src_id))
                self._local_mngr.send_queued(msg)
                # it comes not from UDP socket, send to UDP and TCP
                self._udp.send_queued(msg)
                if self._tcp_server is not None:
                    self._tcp_server.send_queued(msg)
                msg.forward = True
            else:
                # it is an unique id, search in address book for receiver
                if self.addrbook.apply_destination(msg):
                    if (msg.tinfo_src == msg.tinfo_dst):
                        # skip messages with equal source and destination
                        return
                    self.logger.debug("send 0x%.4X unicast from %s to %s (%s)" % (msg.msg_id, msg.src_id, msg.dst_id, msg.tinfo_dst))
                    if msg.tinfo_dst.etype == AddressBook.Endpoint.UDS:
                        self._local_mngr.send_queued(msg)
                    elif msg.tinfo_dst.etype in [AddressBook.Endpoint.UDP, AddressBook.Endpoint.UDP_LOCAL]:
                        # send through UDP socket
                        self._udp.send_queued(msg)
                    elif msg.tinfo_dst.etype == AddressBook.Endpoint.TCP:
                        # send through TCP socket
                        if self._tcp_server is not None:
                            self._tcp_server.send_queued(msg)
                    msg.forward = True
                else:
                    # no receiver found
                    # do not send every message to not known receiver
                    ts = 0
                    if msg.dst_id in self._on_discover:
                        ts = self._on_discover[msg.dst_id]
                    ts_cur = time.time()
                    if ts_cur - ts > 1:
                        # try to find the receiver -> send as broadcast with ACK requested
                        self.logger.debug("%s not found, try to discover, send as broadcast with ACK requested" % msg.dst_id)
                        msg.acknak = 1
                        msg.tinfo_dst = None
                        self._udp.send_queued(msg)
                        if self._tcp_server is not None:
                            self._tcp_server.send_queued(msg)
                        self._on_discover[msg.dst_id] = ts_cur
                        msg.forward = True
            self.statistics.add(msg)
        except Exception as err:
            print("ERROR", err)

    def route_udp_msg(self, msg):
        try:
            add_to_statistics = True
            if msg.tinfo_src.etype != AddressBook.Endpoint.UDP_LOCAL:
                self.addrbook.add(msg)
            if msg.dst_id.has_wildcards():
                # it is a broadcast message, try send to all matched locals (except sender)
                self.logger.debug("send 0x%.4X broadcast from %s" % (msg.msg_id, msg.src_id))
                self._local_mngr.send_queued(msg)
                self.route_local_msg(msg)
                add_to_statistics = False
                msg.forward = True
            else:
                # it is an unique id, search in address book for receiver
                if self.addrbook.apply_destination(msg):
                    if (msg.tinfo_src == msg.tinfo_dst):
                        # skip messages with equal source and destination
                        return
                    self.logger.debug("send 0x%.4X unicast from %s to %s (%s)" % (msg.msg_id, msg.src_id, msg.dst_id, msg.tinfo_dst))
                    if msg.tinfo_dst.etype == AddressBook.Endpoint.UDS:
                        self._local_mngr.send_queued(msg)
                        msg.forward = True
                    else:
                        self.route_local_msg(msg)
                        add_to_statistics = False
                    # TODO: forward message to TCP?
                elif msg.tinfo_src.etype == AddressBook.Endpoint.UDP_LOCAL:
                    # no receiver found
                    # do not send every message to not known receiver
                    ts = 0
                    if msg.dst_id in self._on_discover:
                        ts = self._on_discover[msg.dst_id]
                    ts_cur = time.time()
                    if ts_cur - ts > 1:
                        # try to find the receiver -> send as broadcast with ACK requested
                        self.logger.debug("%s not found, try to discover, send as broadcast with ACK requested" % msg.dst_id)
                        msg.acknak = 1
                        msg.tinfo_dst = None
                        self._udp.send_queued(msg)
                        if self._tcp_server is not None:
                            self._tcp_server.send_queued(msg)
                        self._on_discover[msg.dst_id] = ts_cur
                        msg.forward = True
            if add_to_statistics:
                self.statistics.add(msg)
        except Exception as err:
            print("ERROR", err)

    def route_tcp_msg(self, msg):
        try:
            self.addrbook.add(msg)
            if msg.dst_id.has_wildcards():
                # it is a broadcast message, try send to all matched locals (except sender)
                self.logger.debug("send 0x%.4X broadcast from %s" % (msg.msg_id, msg.src_id))
                self._local_mngr.send_msg(msg)
                msg.forward = True
            else:
                # it is an unique id, search in address book for receiver
                if self.addrbook.apply_destination(msg):
                    self.logger.debug("send 0x%.4X unicast from %s to %s (%s)" % (msg.msg_id, msg.src_id, msg.dst_id, msg.tinfo_dst))
                    if msg.tinfo_dst.etype == AddressBook.Endpoint.UDS:
                        self._local_mngr.send_msg(msg)
                        msg.forward = True
                    # TODO: forward message to UDP?
            self.statistics.add(msg)
        except Exception as err:
            print("ERROR", err)

    def shutdown(self):
        self.cfg.close()
        self.statistics.stop()
        self._stop = True
        if self._udp is not None:
            self._udp.close()
        if self._local_mngr is not None:
            self._local_mngr.stop()
        if self._tcp_server is not None:
            self._tcp_server.close()
            self._tcp_server = None
