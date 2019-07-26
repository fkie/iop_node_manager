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

import os
import time
from .msg_entry import MsgEntry


class Reader:
    '''
    Continuous read from given file and returns for each line a MsgEntry.
    '''

    def __init__(self, filename):
        self._filename = filename
        self._fileio = None

    def readline(self):
        try:
            while True:
                line = None
                if self._fileio is None:
                    if os.path.exists(self._filename):
                        self._fileio = open(self._filename, "r")
                if self._fileio is not None:
                    line = self._fileio.readline()
                    if not line:
                        time.sleep(0.1)
                        continue
                else:
                    time.sleep(0.1)
                if line is not None:
                    yield MsgEntry.fromline(line)
        except KeyboardInterrupt:
            pass
