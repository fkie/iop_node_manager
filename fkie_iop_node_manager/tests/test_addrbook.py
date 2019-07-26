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

PKG = 'fkie_iop_node_manager'


class TestAddrbookLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_endpoint(self):
        endpoint = AddressBook.Endpoint(AddressBook.Endpoint.UDP, '192.168.0.1', 12345)
        endpoint_equal = AddressBook.Endpoint(AddressBook.Endpoint.UDP, '192.168.0.1', 12345)
        self.assertEqual(endpoint, endpoint_equal, "equal AddressBook.Endpoint are not equal, expected: True, got: False")
        endpoint_nequal = AddressBook.Endpoint(AddressBook.Endpoint.UDP, '192.168.0.1', 54321)
        self.assertNotEqual(endpoint, endpoint_nequal, "not equal AddressBook.Endpoint are equal, expected: False, got: True")
        self.assertEqual('UDP', endpoint.etype_str(), "wrong etype as str in AddressBook.Endpoint, expected: %s, got: %s" % ('UDP', endpoint.etype_str()))
        self.assertEqual('192.168.0.1:12345', endpoint.address_str(), "wrong address as str in AddressBook.Endpoint, expected: %s, got: %s" % ('192.168.0.1:12345', endpoint.address_str()))
        try:
            '%s' % endpoint
        except Exception as err:
            self.fail("AddressBook.Endpoint.__repr__() raised Exception unexpectedly: %s" % err)

    def test_create(self):
        addrbook = AddressBook(default_port=1234, addrbook_udp={})
        self.assertEqual(1234, addrbook._default_port, "wrong default_port after creation AddressBook, expected: %d, got: %d" % (1234, addrbook._default_port))
        self.assertEqual(0, len(addrbook._static_udp), "wrong count of static UDP addresses after creation AddressBook with empty address book, expected: %d, got: %d" % (0, len(addrbook._static_udp)))
        addrbook = AddressBook(default_port=1234, addrbook_udp={'192.168.0.1:2345': '1.1.1'})
        self.assertEqual(1, len(addrbook._static_udp), "wrong count of static UDP addresses after creation AddressBook, expected: %d, got: %d" % (1, len(addrbook._static_udp)))
        addrbook = AddressBook(default_port=1234, addrbook_udp={'192.168.0.1': '1.1.1'})
        self.assertEqual(1, len(addrbook._static_udp), "wrong count of static UDP addresses after creation AddressBook without port, expected: %d, got: %d" % (1, len(addrbook._static_udp)))
        addrbook = AddressBook(default_port=1234, addrbook_udp={'192.168.0.1:2345': ['1.1.1', '1.1.2']})
        self.assertEqual(2, len(addrbook._static_udp), "wrong count of static UDP addresses after creation AddressBook with list, expected: %d, got: %d" % (2, len(addrbook._static_udp)))
        try:
            '%s' % addrbook
        except Exception as err:
            self.fail("AddressBook.__repr__() raised Exception unexpectedly: %s" % err)
        # create with invalid port
        try:
            addrbook = AddressBook(default_port=1234, addrbook_udp={'192.168.0.1:2345s': ['1.1.1', '1.1.2']})
            self.fail("AddressBook raises no Exception on parse not valid port!")
        except Exception:
            pass

    def test_add_jaus_address(self):
        addrbook = AddressBook(default_port=1234, addrbook_udp={})
        addrbook.add_jaus_address(JausAddress.from_string('1.1.1'), '192.168.0.1', 12345, AddressBook.Endpoint.UDP)
        self.assertEqual(1, len(addrbook._map), "wrong count of discovered UDP addresses after add address, expected: %d, got: %d" % (1, len(addrbook._map)))
        addrbook.add_jaus_address(JausAddress.from_string('1.1.1'), '192.168.0.2', 12345, AddressBook.Endpoint.UDP)
        self.assertEqual(1, len(addrbook._map), "wrong count of discovered UDP addresses after add same address, expected: %d, got: %d" % (1, len(addrbook._map)))
        addrbook.remove(JausAddress.from_string('1.1.2'))
        self.assertEqual(1, len(addrbook._map), "wrong count of discovered UDP addresses after remove wrong jaus address, expected: %d, got: %d" % (1, len(addrbook._map)))
        addrbook.remove(JausAddress.from_string('1.1.1'))
        self.assertEqual(0, len(addrbook._map), "wrong count of discovered UDP addresses after remove, expected: %d, got: %d" % (0, len(addrbook._map)))

    def test_apply_destination(self):
        addrbook = AddressBook(default_port=1234, addrbook_udp={'192.168.0.2:12345': '2.255.255'})
        addrbook.add_jaus_address(JausAddress.from_string('1.1.1'), '192.168.0.1', 12345, AddressBook.Endpoint.UDP)
        # from dynamic address
        msg = Message()
        msg.dst_id = JausAddress.from_string('1.1.1')
        apply_res = addrbook.apply_destination(msg)
        self.assertEqual(True, apply_res, "can not appy destination")
        dst_endpoint = AddressBook.Endpoint(AddressBook.Endpoint.UDP, '192.168.0.1', 12345)
        self.assertEqual(dst_endpoint, msg.tinfo_dst, "wrong endpoint applied, expected: %s, got: %s" % (dst_endpoint, msg.tinfo_dst))
        # from static udp
        msg = Message()
        msg.dst_id = JausAddress.from_string('2.1.1')
        apply_res = addrbook.apply_destination(msg)
        self.assertEqual(True, apply_res, "can not appy destination from static udp")
        dst_endpoint2 = AddressBook.Endpoint(AddressBook.Endpoint.UDP, '192.168.0.2', 12345)
        self.assertEqual(dst_endpoint2, msg.tinfo_dst, "wrong endpoint applied, should be from static udp, expected: %s, got: %s" % (dst_endpoint, msg.tinfo_dst))
        # test for not exiting jaus id
        msg = Message()
        msg.dst_id = JausAddress.from_string('3.1.1')
        apply_res = addrbook.apply_destination(msg)
        self.assertEqual(False, apply_res, "applied destination for not existing jaus id")


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestAddrbookLib)
