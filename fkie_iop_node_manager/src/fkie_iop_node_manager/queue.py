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
import threading


class Full(Exception):
    pass


class PQueue(object):

    def __init__(self, maxsize=0, logger_name='queue'):
        '''
        :param int maxsize: The maximal queue length for each priority. No new items are added if this size is reached. Zero to disable the limit for each priority.
        :param str logger_name: the name of this priority queue used for logging or exceptions.
        :param dict(str:int) priority_map: Map to overried priority. The key is message id represented as HEX string. Value is priority (0-3).
        '''
        self._logger_name = logger_name
        self.logger = logging.getLogger(logger_name)
        self._lock = threading.RLock()
        self._cv = threading.Condition()
        self._maxsize = maxsize
        self._pq = {3: [],
                    2: [],
                    1: [],
                    0: []}
        self._counts = {3: 0,
                        2: 0,
                        1: 0,
                        0: 0}
        self._idx = [3, 2, 1, 0]
        self._count = 0

    def clear(self):
        self.logger.debug("Clear queue")
        with self._lock:
            for idx in self._idx:
                del self._pq[idx][:]
                self._count = 0
                self._counts[idx] = 0
        self._cv.acquire()
        self._cv.notify()
        self._cv.release()

    def put(self, item):
        if item.tinfo_src is None:
            self.logger.warning("tinfo_src is empty")
            raise Exception("Queue %s: tinfo_src in item is empty" % self._logger_name)
        if self._maxsize > 0 and self._counts[item.priority] >= self._maxsize:
            raise Full("Queue `%s` for priority %d is full" % (self._logger_name, item.priority))
        with self._lock:
            self._pq[item.priority].append(item)
            self._count += 1
            self._counts[item.priority] += 1
        with self._cv:
            self._cv.notify()

    def get(self, block=True):
        if self.size() == 0:
            if block:
                with self._cv:
                    self._cv.wait()
        with self._lock:
            item = None
            for idx in self._idx:
                if self._counts[idx]:
                    item = self._pq[idx].pop(0)
                    self._count -= 1
                    self._counts[item.priority] -= 1
                    # self.logger.debug("get %s" % item)
                    return item
        return None

    def size(self, priority=None):
        if priority is None:
            return self._count
        if priority in self._counts:
            return self._counts[priority]
        return 0
