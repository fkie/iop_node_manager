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
import os
import threading
import time
import traceback

from .msg_entry import MsgEntry


class Empty(Exception):
    pass


class Full(Exception):
    pass


class Collector(object):

    def __init__(self, cfg, logger_name='collector'):
        '''
        :param fkie_iop_node_manager.config.Config cfg: configuration
        '''
        self.cfg = cfg
        self.logger = logging.getLogger(logger_name)
        self._stop = False
        self._lock = threading.RLock()
        self._cv = threading.Condition()
        self._msgs_recv = []
        self._idx_current = 0
        self._count = 0
        self.stats_file = None
        self.stats_path = ''
        self.stats_enabled = self.cfg.param('statistics/enable', False)
        self.logger.info("Statistics enabled: %s" % self.stats_enabled)
        if self.stats_enabled:
            self._stats_file_open()
        self.cfg.add_param_listener('global/statistics/enable', self._callback_param_enable)
        self._thread_analyse = threading.Thread(target=self._loop_write_to_file)
        self._thread_analyse.start()

    def stop(self):
        self._stop = True
        self.clear()
        self._stats_file_close()

    def clear(self):
        self.logger.debug("Clear collector")
        with self._lock:
            del self._msgs_recv[:]
        self._cv.acquire()
        self._cv.notify()
        self._cv.release()

    def _stats_file_open(self):
        if self.stats_file is not None:
            return
        try:
            self.stats_dir = os.path.expanduser(self.cfg.param('statistics/path', False))
            if not os.path.isdir(self.stats_dir):
                os.makedirs(self.stats_dir)
            self.stats_path = os.path.join(self.stats_dir, 'last.msgs')
            if os.path.exists(self.stats_path):
                os.rename(self.stats_path, os.path.join(self.stats_dir, 'prev_%.0f.msgs' % time.time()))
            self.logger.info("  write statistics to '%s'" % self.stats_path)
            self.stats_file = open(self.stats_path, 'w+')
            self.stats_file.write(MsgEntry.title())
            self.stats_file.flush()
        except Exception as err:
            self.stats_file = None
            self.logger.warning("Error while open statistics file: %s" % err)

    def _stats_file_close(self):
        try:
            if self.stats_file is not None:
                self.stats_file.close()
                self.stats_file = None
        except Exception as err:
            self.logger.warning("Error while close statistics file: %s" % err)

    def str2bool(self, v):
        if isinstance(v, bool):
            return v
        return v.lower() in ["yes", "true", "t", "1"]

    def _callback_param_enable(self, param, value):
        boolval = self.str2bool(value)
        if boolval:
            self._stats_file_open()
        else:
            self._stats_file_close()

    def add(self, msg):
        '''
        Adds a message to the queue. The items of this queue are written in a separate thread to a file.

        :param fkie_iop_node_manager.message.Mesage msg: received or sent message.
        '''
        if self.stats_file is None:
            return
        msg.ts_recv = time.time()
        with self._lock:
            self._msgs_recv.append(msg)
            self._count += 1
        self._cv.acquire()
        self._cv.notify()
        self._cv.release()

    def get(self, block=True):
        if self.size() == 0:
            if block:
                self._cv.acquire()
                self._cv.wait()
                self._cv.release()
        with self._lock:
            item = None
            try:
                item = self._msgs_recv.pop(0)
                self._count -= 1
            except IndexError:
                pass
            if item is None:
                raise Empty()
            self.logger.debug("get %s" % item)
            return item
        return None

    def size(self):
        return self._count

    def _loop_write_to_file(self):
        '''
        Writes the messages of the queue to a file.
        '''
        while not self._stop:
            try:
                msg = self.get()
                if msg.tinfo_src is None:
                    self.logger.warning("Collects a message without valid tinfo_src, ignore...")
                    continue
                try:
                    self.stats_file.write(MsgEntry.toline(msg, self.cfg))
                    self.stats_file.flush()
                except Exception as err:
                    print(traceback.format_exc())
                    self.logger.warning("Error write message to statistics file: %s" % err)
            except Empty:
                pass
            except Exception as e:
                print(traceback.format_exc())
                self.logger.warning("Error while get send item from collector queue: %s" % e)
