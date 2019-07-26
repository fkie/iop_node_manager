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
from fkie_iop_node_manager.message import Message
from fkie_iop_node_manager.queue import PQueue, Full

PKG = 'fkie_iop_node_manager'


class TestQueueLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_put_without_priority(self):
        queue = PQueue()
        msg = Message()
        self.assertRaises(Exception, queue.put, msg)
        first_src = AddressBook.Endpoint(AddressBook.Endpoint.UDP, '192.168.0.1', 12345)
        msg.tinfo_src = first_src
        queue.put(msg)
        self.assertEqual(1, queue.size(), "PQueue: wrong count of messages, expected: %d, got: %d" % (1, queue.size()))
        msg2 = Message()
        second_src = AddressBook.Endpoint(AddressBook.Endpoint.UDS, '/name')
        msg2.tinfo_src = second_src
        queue.put(msg2)
        self.assertEqual(2, queue.size(), "PQueue: wrong count of messages after second insert, expected: %d, got: %d" % (2, queue.size()))
        msgr = queue.get()
        self.assertEqual(1, queue.size(), "PQueue: wrong count of messages after get, expected: %d, got: %d" % (1, queue.size()))
        self.assertEqual(first_src, msgr.tinfo_src, "PQueue: got wrong message, expected: %s, got: %s" % (first_src, msgr.tinfo_src))
        msgr = queue.get()
        self.assertEqual(0, queue.size(), "PQueue: wrong count of messages after second get, expected: %d, got: %d" % (0, queue.size()))
        self.assertEqual(second_src, msgr.tinfo_src, "PQueue: got wrong second message, expected: %s, got: %s" % (second_src, msgr.tinfo_src))

    def test_put_with_priority(self):
        queue = PQueue()
        msg = Message()
        self.assertRaises(Exception, queue.put, msg)
        first_src = AddressBook.Endpoint(AddressBook.Endpoint.UDP, '192.168.0.1', 12345)
        msg.tinfo_src = first_src
        msg.priority = 1
        queue.put(msg)
        self.assertEqual(1, queue.size(), "PQueue with prio: wrong count of messages, expected: %d, got: %d" % (1, queue.size()))
        msg2 = Message()
        second_src = AddressBook.Endpoint(AddressBook.Endpoint.UDS, '/name')
        msg2.tinfo_src = second_src
        msg2.priority = 3
        queue.put(msg2)
        self.assertEqual(2, queue.size(), "PQueue with prio: wrong count of messages after second insert, expected: %d, got: %d" % (2, queue.size()))
        msgr = queue.get()
        self.assertEqual(1, queue.size(), "PQueue with prio: wrong count of messages after get, expected: %d, got: %d" % (1, queue.size()))
        self.assertEqual(second_src, msgr.tinfo_src, "PQueue with prio: got wrong first message, expected: %s, got: %s" % (second_src, msgr.tinfo_src))
        msgr = queue.get()
        self.assertEqual(0, queue.size(), "PQueue with prio: wrong count of messages after second get, expected: %d, got: %d" % (0, queue.size()))
        self.assertEqual(first_src, msgr.tinfo_src, "PQueue with prio: got wrong second message, expected: %s, got: %s" % (first_src, msgr.tinfo_src))

    def test_clear(self):
        queue = PQueue()
        msg = Message()
        first_src = AddressBook.Endpoint(AddressBook.Endpoint.UDP, '192.168.0.1', 12345)
        msg.tinfo_src = first_src
        queue.put(msg)
        queue.put(msg)
        queue.put(msg)
        self.assertEqual(3, queue.size(), "PQueue: wrong count of messages, expected: %d, got: %d" % (3, queue.size()))
        queue.clear()
        self.assertEqual(0, queue.size(), "PQueue: wrong count of messages after clear, expected: %d, got: %d" % (0, queue.size()))

    def test_queue_full(self):
        queue = PQueue(maxsize=2)
        msg = Message()
        first_src = AddressBook.Endpoint(AddressBook.Endpoint.UDP, '192.168.0.1', 12345)
        msg.tinfo_src = first_src
        queue.put(msg)
        queue.put(msg)
        self.assertRaises(Full, queue.put, msg)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestQueueLib)
