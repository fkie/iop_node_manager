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

from .jaus_address import JausAddress


class AddressBook():

    class Endpoint:
        UDS = 0
        UDP = 1
        TCP = 2

        def __init__(self, etype, address='', port=None):
            self.etype = etype
            self.address = address
            self.port = port

        def __repr__(self):
            return "Endpoint<%s; %s%s>" % (self.etype_str(), self.address, ':%d' % self.port if self.port is not None else '')

        def etype_str(self):
            result = ''
            if self.etype == self.UDS:
                result = 'UDS'
            elif self.etype == self.UDP:
                result = 'UDP'
            elif self.etype == self.TCP:
                result = 'TCP'
            return result

        def address_str(self):
            result = self.address
            if self.port is not None:
                result += ':%d' % self.port
            return result

        def address_tuple(self):
            return (self.address, self.port)

        def __eq__(self, other):
            return self.etype == other.etype and self.address == other.address and self.port == other.port

        def __ne__(self, other):
            return not self.__eq__(other)

    def __init__(self, default_port=3794, addrbook_udp={}, addrbook_tcp={}):
        '''
        :param fkie_iop_node_manager.config.Config cfg: configuration
        '''
        self.logger = logging.getLogger('addrbook')
        self._default_port = default_port
        self._map = {}
        self._static_tcp_port_map = {}
        self._static_udp = self._read_static_addr(addrbook_udp, AddressBook.Endpoint.UDP)
        self._static_tcp = self._read_static_addr(addrbook_tcp, AddressBook.Endpoint.TCP)

    def __str__(self):
        return "<AddressBook discovered[%d]=%s, configured_udp[%d]=%s, configured_tcp[%d]=%s/>" % (len(self._map), self._map, len(self._static_udp), self._static_udp, len(self._static_tcp), self._static_tcp)

    def _read_static_addr(self, from_dict, etype):
        etype_str = 'None'
        if etype == AddressBook.Endpoint.UDP:
            etype_str = 'UDP'
        elif etype == AddressBook.Endpoint.TCP:
            etype_str = 'TCP'
        result = []
        for addr, items in from_dict.items():
            host, port = self._parse_hostport(addr, self._default_port)
            endpoint = AddressBook.Endpoint(etype, host, port)
            if isinstance(items, list):
                for item in items:
                    jid = JausAddress.from_string(item)
                    self.logger.info("Add from config: %s:%s [%s]" % (jid, endpoint, etype_str))
                    result.append((jid, endpoint))
            else:
                jid = JausAddress.from_string(items)
                self.logger.info("Add from config: %s:%s [%s]" % (jid, endpoint, etype_str))
                result.append((jid, endpoint))
            if etype == AddressBook.Endpoint.TCP:
                self._static_tcp_port_map[host] = port
        return result

    def _parse_hostport(self, param, default_port):
        if param[-1] == ']':
            # ipv6 literal (with no port)
            return (param, default_port)
        out = param.rsplit(":", 1)
        if len(out) == 1:
            # No port
            port = default_port
        else:
            try:
                port = int(out[1])
            except ValueError:
                raise ValueError("Invalid host:port '%s'" % param)
        return (out[0], port)

    def apply_destination(self, msg):
        '''
        Searches for destination address/socket and set the `tinfo_dst` of the message
        to the AddressBook.Endpoint object if destination was found. In this case the result is True.
        If detination was not found `tinfo_dst` will not be changed and False is returned.

        :rtype: bool
        '''
        try:
            # try first in discovered addresses
            msg.tinfo_dst = self._map[msg.dst_id]
            return True
        except KeyError:
            # lookup in configured addresses, take first found. UDP is preferred.
            for jaus_id, entry in self._static_udp:
                if msg.dst_id.match(jaus_id):
                    # take first found
                    msg.tinfo_dst = entry
                    return True
            # lookup in configured TCP addresses, take first found. UDP is preferred.
            for jaus_id, entry in self._static_tcp:
                if msg.dst_id.match(jaus_id):
                    # take first found
                    msg.tinfo_dst = entry
                    return True
        return False

    def add(self, msg):
        '''
        tinfo_src should be valid.
        '''
        if not msg.src_id.has_wildcards():
            self._add(msg.src_id, msg.tinfo_src)

    def add_jaus_address(self, jaus_address, address, port, ep_type):
        if not jaus_address.has_wildcards():
            endpoint = AddressBook.Endpoint(ep_type, address, port)
            self._add(jaus_address, endpoint)

    def _add(self, jaus_address, endpoint):
        if jaus_address in self._map:
            current_addr = self._map[jaus_address]
            if endpoint != current_addr:
                self.logger.warning("Changed address for %s: old=%s, new %s" % (jaus_address, current_addr, endpoint))
                self._map[jaus_address] = endpoint
        else:
            self.logger.info("Added new address for %s: %s" % (jaus_address, endpoint))
            self._map[jaus_address] = endpoint

    def remove(self, jaus_address):
        if not jaus_address.has_wildcards():
            if jaus_address in self._map:
                try:
                    del self._map[jaus_address]
                except KeyError as err:
                    self.logger.warning("Can not remove %s: %s" % (jaus_address, err))
