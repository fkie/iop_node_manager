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

from .reader import Reader


class EvalBytes:

    '''
    Reads statistics from file and prints evaluated receive and send stats to stdout.
    '''

    def __init__(self, input_filename, interval=1):
        self._stop = False
        self._input_filename = input_filename
        if interval <= 0:
            interval = 1
        self.interval = 1 / interval
        self._current_interval = 0
        self.ts = 0
        self.udp_count_recv = 0
        self.udp_count_sent = 0
        self.udp_count_mcast_sent = 0
        self.udp_bytes_recv = 0
        self.udp_bytes_sent = 0
        self.udp_bytes_mcast_sent = 0

        self.uds_count_recv = 0
        self.uds_count_sent = 0
        self.uds_count_mcast_sent = 0
        self.uds_bytes_recv = 0
        self.uds_bytes_sent = 0
        self.uds_bytes_mcast_sent = 0

    def stop(self):
        self._stop = True

    def evaluate(self):
        reader = Reader(self._input_filename)
        entries = reader.readline()
        print("# interval,   count recv, count sent, bytes recv, bytes sent, count mcast sent, bytes mcast sent,   UDS count recv, UDS count sent, UDS bytes recv, UDS bytes sent, UDS count mcast sent, UDS bytes mcast sent")
        for entry in entries:
            if self.ts == 0:
                self.ts = entry.ts_receive
            while self.ts + self.interval < entry.ts_receive:
                self.ts += self.interval
                self._current_interval += self.interval
                self._stats_line(self._current_interval)
            if entry.src_etype == 'UDP':
                self.udp_count_recv += 1
                self.udp_bytes_recv += entry.raw_size
            if entry.dst_etype == 'UDP':
                if entry.dst_address == 'Broadcast':
                    self.udp_count_mcast_sent += 1
                    self.udp_bytes_mcast_sent += entry.raw_size
                else:
                    self.udp_count_sent += 1
                    self.udp_bytes_sent += entry.raw_size
            if entry.src_etype == 'UDS':
                self.uds_count_recv += 1
                self.uds_bytes_recv += entry.raw_size
            if entry.dst_etype == 'UDS':
                if entry.dst_address == 'Broadcast':
                    self.uds_count_mcast_sent += 1
                    self.uds_bytes_mcast_sent += entry.raw_size
                else:
                    self.uds_count_sent += 1
                    self.uds_bytes_sent += entry.raw_size
            if self._stop:
                break

    def _stats_line(self, ts, clear_stats=True):
        line = "%.2f" % ts
        line += "  "
        line += " %d" % self.udp_count_recv
        line += " %d" % self.udp_count_sent
        line += " %d" % self.udp_bytes_recv
        line += " %d" % self.udp_bytes_sent
        line += " %d" % self.udp_count_mcast_sent
        line += " %d" % self.udp_bytes_mcast_sent
        line += "  "
        line += " %d" % self.uds_count_recv
        line += " %d" % self.uds_count_sent
        line += " %d" % self.uds_bytes_recv
        line += " %d" % self.uds_bytes_sent
        line += " %d" % self.uds_count_mcast_sent
        line += " %d" % self.uds_bytes_mcast_sent
        if clear_stats:
            self.udp_count_recv = 0
            self.udp_count_sent = 0
            self.udp_count_mcast_sent = 0
            self.udp_bytes_recv = 0
            self.udp_bytes_sent = 0
            self.udp_bytes_mcast_sent = 0
            self.uds_count_recv = 0
            self.uds_count_sent = 0
            self.uds_count_mcast_sent = 0
            self.uds_bytes_recv = 0
            self.uds_bytes_sent = 0
            self.uds_bytes_mcast_sent = 0
        print(line)

    def _print_stats(self):
        print("statistics (total):")
        print("  udp_count_recv: %d" % self.udp_count_recv)
        print("  udp_count_sent: %d" % self.udp_count_sent)
        print("  udp_count_sent_mcast: %d" % self.udp_count_mcast_sent)
        print("  udp_bytes_recv: %s" % self.sizeof_fmt(self.udp_bytes_recv))
        print("  udp_bytes_sent: %s" % self.sizeof_fmt(self.udp_bytes_sent))
        print("  udp_bytes_sent_mcast: %s" % self.sizeof_fmt(self.udp_bytes_mcast_sent))

    def sizeof_fmt(self, num, suffix='B'):
        for unit in ['', 'Ki', 'Mi', 'Gi', 'Ti', 'Pi', 'Ei', 'Zi']:
            if abs(num) < 1024.0:
                return "%.0f%s%s" % (num, unit, suffix)
            num /= 1024.0
        return "%.0%s%s" % (num, 'YiB', suffix)
