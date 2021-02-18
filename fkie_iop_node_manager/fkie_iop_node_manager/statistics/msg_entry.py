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

from fkie_iop_node_manager.jaus_address import JausAddress


class MsgEntry:

    def __init__(self):
        self.ts_receive = 0
        self.src_etype = ''
        self.src_address = ''
        self.src_id = JausAddress(0)
        self.dst_etype = ''
        self.dst_address = ''
        self.dst_id = JausAddress(0)
        self.msg_id = 0
        self.version = 0
        self.raw_size = 0
        self.cmd_code = 0
        self.priority = 0
        self.acknak = 0
        self.data_flags = 0
        self.seqnr = 0

    def __repr__(self):
        result = "MsgEntry<"
        result += " ts_receive=%.2f" % self.ts_receive
        result += " src_etype=%s" % self.src_etype
        result += " src_address=%s" % self.src_address
        result += " src_id=%s" % self.src_id
        result += " dst_etype=%s" % self.dst_etype
        result += " dst_address=%s" % self.dst_address
        result += " dst_id=%s" % self.dst_id
        result += " msg_id=0x%x" % self.msg_id
        result += " version=%d" % self.version
        result += " raw_size=%d" % self.raw_size
        result += " cmd_code=%d" % self.cmd_code
        result += " priority=%d" % self.priority
        result += " acknak=%d" % self.acknak
        result += " data_flags=%d" % self.data_flags
        result += " seqnr=%d" % self.seqnr
        result += ">"
        return result

    @staticmethod
    def title(with_newline=True):
        title = "# timestamp, from socket, from address, from JAUS id, to address, to JAUS id, message id, message name, version, bytes, cmd_code, priority, acknak, data flags, seqnr"
        if with_newline:
            title += '\n'
        return title

    @staticmethod
    def toline(msg, cfg, with_newline=True):
        line = '%f' % msg.ts_receive
        line += ' %s' % msg.tinfo_src.etype_str()
        line += ' %s' % msg.tinfo_src.address_str()
        line += ' %s' % msg.src_id.jaus_id
        if msg.tinfo_dst is not None:
            line += ' %s' % msg.tinfo_dst.etype_str()
            line += ' %s' % msg.tinfo_dst.address_str()
        elif msg.forward:
            line += ' UDP'
            line += ' Broadcast'
        else:
            line += ' None'
            line += ' NotForward'
        line += ' %s' % msg.dst_id.jaus_id
        line += ' 0x%.4x' % msg.msg_id
        line += ' %s' % cfg.msg_name(msg.msg_id)
        line += ' %d' % msg.version
        line += ' %d' % msg.raw_size if msg.raw_size > 0 else self._bcast_recved_size
        line += ' %d' % msg.cmd_code
        line += ' %d' % msg.priority
        line += ' %d' % msg.acknak
        line += ' %d' % msg.data_flags
        line += ' %d' % msg.seqnr
        if with_newline:
            line += '\n'
        return line

    @staticmethod
    def fromline(line):
        result = MsgEntry()
        if not line.startswith('#'):
            items = line.split(' ')
            result.ts_receive = float(items[0])
            result.src_etype = items[1]
            result.src_address = items[2]
            result.src_id = JausAddress.from_string(items[3])
            result.dst_etype = items[4]
            result.dst_address = items[5]
            result.dst_id = JausAddress.from_string(items[6])
            result.msg_id = int(items[7], 16)
            result.msg_name = items[8]
            result.version = int(items[9])
            result.raw_size = int(items[10])
            result.cmd_code = int(items[11])
            result.priority = int(items[12])
            result.acknak = int(items[13])
            result.data_flags = int(items[14])
            result.seqnr = int(items[15])
        return result
