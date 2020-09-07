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

ROSPY = False
import logging
from logging import Logger
try:
    import rospy
    ROSPY = True
except Exception:
    pass


# Workaround for logging with and without ROS
# see https://github.com/ros/ros_comm/issues/1384
class NMLogger:
    def __init__(self, name, loglevel='info'):
        self._rospy = False
        if ROSPY:
            self.logger = logging.getLogger('rosout.%s' % name)
            self.debug('Use ROS logger')
        else:
            self.logger = logging.getLogger(name)
        level = self.str2level(loglevel)
        self.logger.setLevel(level)

    def debug(self, msg):
        self.logger.debug(msg)

    def info(self, msg):
        self.logger.info(msg)

    def warning(self, msg):
        self.logger.warning(msg)

    def error(self, msg):
        self.logger.error(msg)

    def critical(self, msg):
        self.logger.critical(msg)

    def level(self):
        return self.level2str(self.logger.level)

    @classmethod
    def setall_loglevel(cls, loglevel):
        level = cls.str2level(loglevel)
        for _lname, logger in Logger.manager.loggerDict.items():
            if not hasattr(logger, 'setLevel'):
                continue
            logger.setLevel(level=level)

    @classmethod
    def str2level(cls, loglevel):
        result = logging.INFO
        if loglevel == 'debug':
            result = logging.DEBUG
        elif loglevel == 'info':
            result = logging.INFO
        elif loglevel == 'warning':
            result = logging.WARNING
        elif loglevel == 'error':
            result = logging.ERROR
        elif loglevel == 'critical':
            result = logging.CRITICAL
        return result

    @classmethod
    def level2str(cls, loglevel):
        result = 'info'
        if loglevel == logging.DEBUG:
            result = 'debug'
        elif loglevel == logging.INFO:
            result = 'info'
        elif loglevel == logging.WARNING:
            result = 'warning'
        elif loglevel == logging.ERROR:
            result = 'error'
        elif loglevel == logging.CRITICAL:
            result = 'critical'
        return result


