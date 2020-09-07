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

import os
import unittest

from fkie_iop_node_manager.addrbook import AddressBook
from fkie_iop_node_manager.jaus_address import JausAddress
from fkie_iop_node_manager.message import Message
from fkie_iop_node_manager.message_parser import MessageParser

PKG = 'fkie_iop_node_manager'


class TestMessageLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        self.msg_query_identification = b'\x02\x00\x11\x00\t\xff\xff\xff\xff\xc8@\x96\x00\x00+\x04\xed\r'
        self.msg_connect = b'\x01\x00\x00\x00\x10\x06\x02\x01\x00\x00\x00\x00\x00\x03\x01\x01\x00\x00\x00\x00\x00'

    def tearDown(self):
        pass

    def test_create(self):
        # create QueryIdentification message
        msg = Message(int('0x2b00', 16), version=Message.AS5669A)
        msg.bcast = 2
        msg.priority = 1
        msg.seqnr = 3565
        msg.src_id = JausAddress.from_string('150.64.200')
        msg.dst_id = JausAddress.from_string('65535.255.255')
        msg.payload = b'\x00+\x04'
        self.assertEqual(self.msg_query_identification, msg.bytes(), "wrong QueryIdentification serialization, expected: %s, got: %s" % ([ord(char) for char in self.msg_query_identification], [ord(char) for char in msg.bytes()]))
        try:
            '%s' % msg
        except Exception as err:
            self.fail("Message.__repr__() raised Exception unexpectedly: %s" % err)

    def test_create_v1(self):
        # create QueryIdentification message
        msg = Message(0, version=Message.AS5669)
        msg.cmd_code = Message.CODE_CONNECT
        msg.bcast = 0
        msg.priority = 1
        msg.seqnr = 0
        msg.src_id = JausAddress.from_string('1.1.3')
        msg.dst_id = JausAddress.from_string('0.0.0')
        self.assertEqual(self.msg_connect, msg.bytes(), "wrong V1 connect message serialization, expected: %s, got: %s" % ([ord(char) for char in self.msg_connect], [ord(char) for char in msg.bytes()]))
        try:
            '%s' % msg
        except Exception as err:
            self.fail("Message.__repr__() raised Exception unexpectedly: %s" % err)

    def test_message_parser_v2(self):
        # create message with Connect command code
        mp = MessageParser(AddressBook.Endpoint(AddressBook.Endpoint.UDS, 'test_v2'))
        msgs = mp.unpack(self.msg_query_identification)
        self.assertEqual(1, len(msgs), "wrong parsed message count, expected: %d, got: %d" % (1, len(msgs)))
        msg = msgs[0]
        self.assertEqual(2, msg.bcast, "wrong bcast value after deserialization, expected: %d, got: %d" % (2, msg.bcast))
        self.assertEqual(1, msg.priority, "wrong priority value after deserialization, expected: %d, got: %d" % (1, msg.priority))
        self.assertEqual(3565, msg.seqnr, "wrong seqnr value after deserialization, expected: %d, got: %d" % (3565, msg.seqnr))
        self.assertEqual(0, msg.data_flags, "wrong data_flags value after deserialization, expected: %d, got: %d" % (0, msg.data_flags))
        src_id = JausAddress.from_string('150.64.200')
        self.assertEqual(src_id.value, msg.src_id.value, "wrong src_id value after deserialization, expected: %d, got: %d" % (src_id.value, msg.src_id.value))
        dst_id = JausAddress.from_string('65535.255.255')
        self.assertEqual(dst_id.value, msg.dst_id.value, "wrong dst_id value after deserialization, expected: %d, got: %d" % (dst_id.value, msg.dst_id.value))

    def test_message_parser_v1(self):
        mp = MessageParser(AddressBook.Endpoint(AddressBook.Endpoint.UDS, 'test_v1'))
        msgs = mp.unpack(self.msg_connect)
        self.assertEqual(1, len(msgs), "v1: wrong parsed message count, expected: %d, got: %d" % (1, len(msgs)))
        msg = msgs[0]
        self.assertEqual(0, msg.bcast, "v1: wrong bcast value after deserialization, expected: %d, got: %d" % (0, msg.bcast))
        self.assertEqual(1, msg.priority, "v1: wrong priority value after deserialization, expected: %d, got: %d" % (1, msg.priority))
        self.assertEqual(0, msg.seqnr, "v1: wrong seqnr value after deserialization, expected: %d, got: %d" % (0, msg.seqnr))
        self.assertEqual(0, msg.data_flags, "v1: wrong data_flags value after deserialization, expected: %d, got: %d" % (0, msg.data_flags))
        src_id = JausAddress.from_string('1.1.3')
        self.assertEqual(src_id.value, msg.src_id.value, "v1: wrong src_id value after deserialization, expected: %d, got: %d" % (src_id.value, msg.src_id.value))
        dst_id = JausAddress.from_string('0.0.0')
        self.assertEqual(dst_id.value, msg.dst_id.value, "v1: wrong dst_id value after deserialization, expected: %d, got: %d" % (dst_id.value, msg.dst_id.value))

    def test_message_change_priority(self):
        mp = MessageParser(AddressBook.Endpoint(AddressBook.Endpoint.UDS, 'test_cp'))
        msg = mp.unpack(self.msg_query_identification)[0]
        msg.priority = 3
        self.assertEqual(3, msg.priority, "wrong priority value after set new value, expected: %d, got: %d" % (3, msg.priority))
        msg = mp.unpack(msg.bytes())[0]
        self.assertEqual(3, msg.priority, "wrong priority value after serialization, expected: %d, got: %d" % (3, msg.priority))

    def test_message_change_seqnr(self):
        mp = MessageParser(AddressBook.Endpoint(AddressBook.Endpoint.UDS, 'test_sn'))
        msg = mp.unpack(self.msg_query_identification)[0]
        msg.seqnr = 3
        self.assertEqual(3, msg.seqnr, "wrong seqnr value after set new value, expected: %d, got: %d" % (3, msg.seqnr))
        msg = mp.unpack(msg.bytes())[0]
        self.assertEqual(3, msg.seqnr, "wrong seqnr value after serialization, expected: %d, got: %d" % (3, msg.seqnr))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestMessageLib)
