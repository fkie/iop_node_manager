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
        self.interval_nr = 0
        self.ts = 0
        self.received_bc = {}
        self.received_nf = {}
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
                self.interval_nr = self.interval_nr + self.interval
                # print current state
                self._print_stats()
                self.ts = entry.ts_receive
            key = (entry.src_id, entry.src_address, entry.dst_id, entry.dst_address)
            if entry.dst_address in ['Broadcast']:
                if key not in self.received_bc:
                    self.received_bc[key] = (1, entry.raw_size)
                else:
                    cs, bs = self.received_bc[key]
                    self.received_bc[key] = (cs + 1, bs + entry.raw_size)
            elif entry.dst_address in ['NotForward']:
                if entry.cmd_code > 0:
                    # do not handle connection requests
                    continue
                if key not in self.received_nf:
                    self.received_nf[key] = (1, entry.raw_size)
                else:
                    cs, bs = self.received_nf[key]
                    self.received_nf[key] = (cs + 1, bs + entry.raw_size)
            else:
                if key in self.received_nf:
                    del self.received_nf[key]
                if key and entry.dst_address:
                    if key not in self.conns:
                        self.conns[key] = (1, entry.raw_size)
                    else:
                        cs, bs = self.conns[key]
                        self.conns[key] = (cs + 1, bs + entry.raw_size)
            if self._stop:
                break

    def _print_stats(self):
        print("--- %d sec:" % self.interval_nr)
        print("Connections: %d" % len(self.conns))
        for (src_id, src_address, dst_id, dst_address), (count, bytes_sent) in self.conns.items():
            print("  %s (%s) -> %s (%s)\n    count packets: %d, %s" % (src_id, src_address, dst_id, dst_address, count, self.sizeof_fmt(bytes_sent)))
        print("Broadcast connections: %d" % len(self.received_bc))
        for (src_id, src_address, dst_id, dst_address), (count, bytes_sent) in self.received_bc.items():
            print("  %s (%s) -> *, count packets: %d %s" % (src_id, src_address, count, self.sizeof_fmt(bytes_sent)))
        print("not forwarded: %d" % len(self.received_nf))
        for (src_id, src_address, dst_id, dst_address), (count, bytes_sent) in self.received_nf.items():
            print("  %s (%s) -> %s (%s), count packets: %d %s" % (src_id, src_address, dst_id, dst_address, count, self.sizeof_fmt(bytes_sent)))

    def sizeof_fmt(self, num, suffix='B'):
        for unit in ['', 'Ki', 'Mi', 'Gi', 'Ti', 'Pi', 'Ei', 'Zi']:
            if abs(num) < 1024.0:
                return "%.0f%s%s" % (num, unit, suffix)
            num /= 1024.0
        return "%.0%s%s" % (num, 'YiB', suffix)
