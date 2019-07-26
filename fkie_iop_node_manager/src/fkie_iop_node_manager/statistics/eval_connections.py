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
        self.ts = time.time()
        self.sender = {}

    def stop(self):
        self._stop = True

    def evaluate(self):
        reader = Reader(self._input_filename)
        entries = reader.readline()
        for entry in entries:
            if time.time() - self.ts > self.interval:
                # print current state
                self._print_stats()
                self.ts = time.time()
            if entry.dst_address in ['Broadcast', 'NotForward']:
                if entry.src_address not in self.sender:
                    self.sender[entry.src_address] = (1, entry.raw_size)
                else:
                    cs, bs = self.sender[entry.src_address]
                    self.sender[entry.src_address] = (cs + 1, bs + entry.raw_size)
            else:
                if entry.src_address in self.sender:
                    del self.sender[entry.src_address]
            if self._stop:
                break

    def _print_stats(self):
        print("Broadcast or not discovered connections: %d" % len(self.sender))
        for sender, (count, bytes_sent) in self.sender.items():
            print("  %s -> %d %s" % (sender, count, self.sizeof_fmt(bytes_sent)))

    def sizeof_fmt(self, num, suffix='B'):
        for unit in ['', 'Ki', 'Mi', 'Gi', 'Ti', 'Pi', 'Ei', 'Zi']:
            if abs(num) < 1024.0:
                return "%.0f%s%s" % (num, unit, suffix)
            num /= 1024.0
        return "%.0%s%s" % (num, 'YiB', suffix)
