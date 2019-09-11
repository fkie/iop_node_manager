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
import os
import threading
import time

import fkie_iop_node_manager.queue as queue
from fkie_iop_node_manager.addrbook import AddressBook
from fkie_iop_node_manager.message import Message
from .net import is_local_iface
from .uds import UDSSocket
from .udp_mc import UDPmcSocket
from .udp_uc import UDPucSocket


class UDSServer(object):

    def __init__(self, router, cfg, addrbook, statistics):
        self._stop = False
        self._cfg = cfg
        self._addrbook = addrbook
        self._statistics = statistics
        self.logger = logging.getLogger('uds_server')
        override_priority = cfg.param('priority/override', True)
        ormap = cfg.param('priority/map', {})
        self._priority_map = {}
        if override_priority:
            # create overide map
            try:
                for msg_id, prio in ormap.items():
                    try:
                        msgid = int(msg_id, 16)
                        self.logger.info("Override priority for 0x%.4X to %d" % (msgid, prio))
                        if prio >= 0 and prio <= 3:
                            self._priority_map[msgid] = prio
                        else:
                            self.logger.warning("Ignored invalid priority for %s: %d" % (msg_id, prio))
                    except ValueError as ve:
                        self.logger.warning("Ignored invalid message id %s: %s" % (msg_id, ve))
            except Exception as err:
                import traceback
                print(traceback.format_exc())
                self.logger.warning("Can not read priority override map: %s" % err)
        self._local_sockets = {}
        self._recv_buffer = cfg.RECV_BUFFER
        self._root_path = cfg.param('transport/local/root', '/tmp')
        self._socket_path_server = os.path.join(self._root_path, cfg.param('transport/local/nm_path'))
        self.logger.info("Listen for local connections @%s" % (self._socket_path_server))
        if os.path.exists(self._socket_path_server):
            os.unlink(self._socket_path_server)
        self._local_socket = UDSSocket(cfg.param('transport/local/nm_path'), remove_on_close=True, force_bind=True, root_path=self._root_path)
        self._udp_looback = None
        self._udp_looback_dest = None
        if cfg.param('transport/loopback_debug/enable', False):
            self._init_loopback()
        # Listen for incoming connections
        self._router = router
        self._queue_send = queue.PQueue(cfg.param('transport/local/queue_length', 0), 'queue_uds_send')
        self._thread_send = threading.Thread(target=self._loop_handle_send_queue)
        self._thread_send.start()
        self._thread_recv = threading.Thread(target=self._loop_recv_local_socket)
        self._thread_recv.start()

    def _init_loopback(self):
        # initialize loopback socket for debug mode
        port = self._cfg.param('transport/loopback_debug/port', 55555)
        use_mcast = self._cfg.param('transport/loopback_debug/use_mcast', False)
        address = self._cfg.param('transport/loopback_debug/address', '169.255.0.100')
        buffer_size = self._cfg.param('transport/loopback_debug/buffer_size', 0)
        queue_length = self._cfg.param('transport/loopback_debug/queue_length', 0)
        if not use_mcast:
            if is_local_iface(address):
                # create a receive socket to avoid ICMP messages with 'port unreachable'
                self.logger.info("Loopback destination is local address, create receive socket ")
                self._udp_looback_dest = UDPucSocket(interface=address, port=port, logger_name='loopback_recv', send_buffer=buffer_size, recv_buffer=self._recv_buffer, queue_length=queue_length)
                self._udp_looback = UDPucSocket(interface=address, logger_name='loopback', default_dst=(address, port))
            else:
                self._udp_looback = UDPucSocket(port=port, logger_name='loopback', default_dst=(address, port), send_buffer=buffer_size, recv_buffer=self._recv_buffer, queue_length=queue_length)
        else:
            interface = self._cfg.param('transport/loopback_debug/interface', '')
            mgroup = self._cfg.param('transport/loopback_debug/group', '239.255.0.1')
            self._udp_looback = UDPmcSocket(port, mgroup, ttl=1, interface=interface, logger_name='loopback_mc', send_buffer=buffer_size, recv_buffer=self._recv_buffer, queue_length=queue_length)

    def _close_loopback(self):
        if self._udp_looback is not None:
            self._udp_looback.close()
            self._udp_looback = None
        if self._udp_looback_dest is not None:
            self._udp_looback_dest.close()
            self._udp_looback_dest = None

    def stop(self):
        self._stop = True
        self._local_socket.close()
        for _key, sock in self._local_sockets.items():
            sock.close()
        self._local_sockets.clear()
        # self._queue_recv_locals.clear()
        self._queue_send.clear()
        self._close_loopback()

    def send_msg(self, msg):
        failed = []
        not_found = []
        found = False
        if msg.tinfo_dst is not None:
            # found valid destination entry
            found = True
            self.logger.debug("Send to local socket %s" % msg.tinfo_dst.address)
            if msg.dst_id in self._local_sockets:
                sock = self._local_sockets[msg.dst_id]
                ok = sock.send_msg(msg)
                if not ok:
                    failed.append(msg.dst_id)
            else:
                self.logger.debug("No socket for %s found!" % msg.tinfo_dst.address)
        else:
            # the destination is None -> send as broadcast
            for key, sock in self._local_sockets.items():
                # do not send message to the socket received from
                if key != msg.src_id:
                    if key.match(msg.dst_id):
                        found = True
                        logging.debug("forward message to %s" % (key))
                        ok = sock.send_msg(msg)
                        if not ok:
                            failed.append(msg.dst_id)
        if not found and self._local_sockets:
            self.logger.debug("No UDS destination found for: %d, seqnr: %d" % (msg.dst_id, msg.seqnr))
            not_found.append(msg.dst_id)
        return failed, not_found

    def send_queued(self, msg):
        try:
            self._queue_send.put(msg)
        except queue.Full as full_error:
            self.logger.warning("Error while put message into send queue: %s" % full_error)

    def _loop_recv_local_socket(self):
        '''
        Receive messages from all local connections in one thread to reduce thread count.
        '''
        while not self._stop:
            try:
                # we listen only to 'JuniorRTE' socket, other local sockets are used for send direction
                msgs = self._local_socket.recv_msg()
                for msg in msgs:
                    self._handle_msg(msg)
                    if self._udp_looback is not None:
                        self._udp_looback.send_queued(msg)
            except Exception:
                import traceback
                self.logger.warning(traceback.format_exc())

    def _handle_msg(self, msg):
        try:
            if msg is None:
                return
            if msg.dst_id.zero or msg.cmd_code > 0:
                # handle connection requests/closing
                try:
                    self._statistics.add(msg)
                    if msg.cmd_code == Message.CODE_CONNECT:
                        # Connection request from client.
                        self.logger.debug("Connection request from %s" % msg.src_id)
                        resp = Message()
                        resp.version = Message.AS5669
                        resp.dst_id = msg.src_id
                        resp.cmd_code = Message.CODE_ACCEPT
                        dest_sock = self.create_local_socket(msg.src_id)
                        if dest_sock is not None:
                            dest_sock.send_msg(resp)
                        resp.ts_receive = time.time()
                        resp.tinfo_src = AddressBook.Endpoint(AddressBook.Endpoint.UDS, self._local_socket.socket_path)
                        resp.tinfo_dst = AddressBook.Endpoint(AddressBook.Endpoint.UDS, dest_sock.socket_path)
                        self._statistics.add(resp)
                    elif msg.cmd_code == Message.CODE_CANCEL:
                        # Disconnect client.
                        self.logger.debug("Disconnect request from %s" % msg.src_id)
                        self.remove_local_socket(msg.src_id)
                except Exception as e:
                    import traceback
                    print(traceback.format_exc())
                    self.logger.warning("Error while handle connection management message: %s" % e)
            else:
                # all other message put in priority queue
                try:
                    # override priority
                    if self._priority_map:
                        try:
                            msg_id = int(msg.msg_id)
                            if msg_id in self._priority_map:
                                prio = self._priority_map[msg_id]
                                # self.logger.debug("Override priority for msg ID: 0x%x, current: %d, new: %d" % (msg_id, msg.priority, prio))
                                msg.priority = prio
                        except Exception as err:
                            import traceback
                            print(traceback.format_exc())
                            self.logger.warning("can not changed priority: %s" % (err))
                    self._router.route_local_msg(msg)
                    if msg.src_id not in self._local_sockets:
                        self.create_local_socket(msg.src_id)
                except Exception as e:
                    import traceback
                    print(traceback.format_exc())
                    self.logger.warning("Error while put local message to global queue: %s" % e)
        except Exception as e:
            import traceback
            print(traceback.format_exc())
            self.logger.warning("Error while get send item from queue: %s" % e)

    # def send_loopback(self, msg):
    #     if self._udp_looback is not None:
    #         self._udp_looback.send_queued(msg)

    # def handle_msg(self, msg):
    #     try:
    #         if msg is None:
    #             return
    #         if msg.dst_id.zero or msg.cmd_code > 0:
    #             # handle connection requests/closing
    #             try:
    #                 self._statistics.add(msg)
    #                 if msg.cmd_code == Message.CODE_CONNECT:
    #                     # Connection request from client.
    #                     self.logger.debug("Connection request from %s" % msg.src_id)
    #                     resp = Message()
    #                     resp.version = Message.AS5669
    #                     resp.dst_id = msg.src_id
    #                     resp.cmd_code = Message.CODE_ACCEPT
    #                     dest_sock = self.create_local_socket(msg.src_id)
    #                     if dest_sock is not None:
    #                         dest_sock.send_msg(resp)
    #                     resp.ts_receive = time.time()
    #                     resp.tinfo_src = AddressBook.Endpoint(AddressBook.Endpoint.UDS, self._local_socket.socket_path)
    #                     resp.tinfo_dst = AddressBook.Endpoint(AddressBook.Endpoint.UDS, dest_sock.socket_path)
    #                     self._statistics.add(resp)
    #                 elif msg.cmd_code == Message.CODE_CANCEL:
    #                     # Disconnect client.
    #                     self.logger.debug("Disconnect request from %s" % msg.src_id)
    #                     self.remove_local_socket(msg.src_id)
    #             except Exception as e:
    #                 print(traceback.format_exc())
    #                 self.logger.warning("Error while handle connection management message: %s" % e)
    #         else:
    #             # all other message put in priority queue
    #             try:
    #                 # override priority
    #                 if self._priority_map:
    #                     msg_id = int(msg.msg_id)
    #                     try:
    #                         if msg_id in self._priority_map:
    #                             prio = self._priority_map[msg_id]
    #                             # self.logger.debug("Override priority for msg ID: 0x%x, current: %d, new: %d" % (msg_id, msg.priority, prio))
    #                             msg.priority = prio
    #                     except KeyError:
    #                         pass
    #                     except Exception as err:
    #                         import traceback
    #                         print(traceback.format_exc())
    #                         self.logger.warning("can not changed priority: %s" % (err))
    #                 if msg.dst_id not in self._local_sockets:
    #                     self.create_local_socket(msg.src_id)
    #             except Exception as e:
    #                 print(traceback.format_exc())
    #                 self.logger.warning("Error while put local message to global queue: %s" % e)
    #     except Exception as e:
    #         print(traceback.format_exc())
    #         self.logger.warning("Error while get send item from queue: %s" % e)

    def _loop_handle_send_queue(self):
        while not self._stop:
            # send message from outside
            try:
                msg = self._queue_send.get()
                if msg is None:
                    continue
                try:
                    failed, not_found = self.send_msg(msg)
                    if failed:
                        # this part is still for tests
                        print("failed send seqnr: %d, %s" % (msg.seqnr, failed))
                        failed, not_found = self.send_msg(msg)
                        if failed:
                            failed, not_found = self.send_msg(msg)
                            if failed:
                                print("  still failed, skip seqnr: %d, %s" % (msg.seqnr, failed))
                except Exception as e:
                    import traceback
                    print(traceback.format_exc())
                    self.logger.warning("Error while forward external message: %s" % e)
            except Exception as e:
                import traceback
                print(traceback.format_exc())
                self.logger.warning("Error while get send item from queue: %s" % e)

    def create_local_socket(self, dst_id):
        if self._stop:
            return None
        sock = None
        if dst_id not in self._local_sockets:
            try:
                print("Create local socket connection to %s" % dst_id)
                self.logger.debug("Create local socket connection to %s" % dst_id)
                sock = UDSSocket('%d' % dst_id.value, root_path=self._root_path, recv_buffer=self._recv_buffer)
                self._local_sockets[dst_id] = sock
                self._addrbook.add_jaus_address(dst_id, sock.socket_path, port=None, ep_type=AddressBook.Endpoint.UDS)
            except Exception as connerr:
                self.logger.error("Can't create local socket to %s: %s" % (dst_id, connerr))
        else:
            sock = self._local_sockets[dst_id]
            # reconnect to socket if new request was received
            sock.reconnect()
        return sock

    def remove_local_socket(self, dst_id):
        if self._stop:
            return
        if dst_id in self._local_sockets:
            try:
                self.logger.debug("Remove local socket connection to %s" % dst_id)
                sock = self._local_sockets[dst_id]
                sock.close()
                del self._local_sockets[dst_id]
                # remove from address book
                self._addrbook.remove(dst_id)
            except Exception as connerr:
                self.logger.error("Can't close local socket to %s: %s" % (dst_id, connerr))
