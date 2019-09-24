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

import time

from .reader import Reader


class EvalConnections:

    '''
    Reads statistics from file and prints connection stats to stdout.
    '''

    def __init__(self, input_filename, interval=3):
        self._stop = False
        self._input_filename = input_filename
        self.interval = interval
        self.ts = 0
        self.received_bc_nd = {}
        self.conns = {}

    def stop(self):
        self._stop = True

    def evaluate(self):
        reader = Reader(self._input_filename)
        entries = reader.readline()
        for entry in entries:
            if self.ts == 0:
                self.ts = entry.ts_receive
            if entry.ts_receive - self.ts > self.interval:
                # print current state
                self._print_stats()
                self.ts = entry.ts_receive
            if entry.dst_address in ['Broadcast', 'NotForward']:
                if entry.src_address not in self.received_bc_nd:
                    self.received_bc_nd[entry.src_address] = (1, entry.raw_size)
                else:
                    cs, bs = self.received_bc_nd[entry.src_address]
                    self.received_bc_nd[entry.src_address] = (cs + 1, bs + entry.raw_size)
            else:
                if entry.src_address in self.received_bc_nd:
                    del self.received_bc_nd[entry.src_address]
                key = (entry.src_id, entry.src_address, entry.dst_id, entry.dst_address)
                if entry.src_address and entry.dst_address:
                    if key not in self.conns:
                        self.conns[key] = (1, entry.raw_size)
                    else:
                        cs, bs = self.conns[key]
                        self.conns[key] = (cs + 1, bs + entry.raw_size)
            if self._stop:
                break

    def _print_stats(self):
        print("Connections: %d" % len(self.conns))
        for (src_addr, src_address, dst_addr, dst_address), (count, bytes_sent) in self.conns.items():
            print("  %s (%s) -> %s (%s)\n    count packets: %d, %s" % (src_addr, src_address, dst_addr, dst_address, count, self.sizeof_fmt(bytes_sent)))
        print("Broadcast or not discovered connections: %d" % len(self.received_bc_nd))
        for received_bc_nd, (count, bytes_sent) in self.received_bc_nd.items():
            print("  %s -> count packets: %d %s" % (received_bc_nd, count, self.sizeof_fmt(bytes_sent)))

    def sizeof_fmt(self, num, suffix='B'):
        for unit in ['', 'Ki', 'Mi', 'Gi', 'Ti', 'Pi', 'Ei', 'Zi']:
            if abs(num) < 1024.0:
                return "%.0f%s%s" % (num, unit, suffix)
            num /= 1024.0
        return "%.0%s%s" % (num, 'YiB', suffix)
