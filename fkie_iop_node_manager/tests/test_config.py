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
import socket
import time
import unittest

from fkie_iop_node_manager.config import Config

PKG = 'fkie_iop_node_manager'


class TestConfigLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        self._param_received = False
        self._param_changed = False

    def tearDown(self):
        pass

    def test_create(self):
        try:
            cfg = Config()
            '%s' % cfg
        except Exception as err:
            self.fail("Config() raised Exception unexpectedly: %s" % err)

    def test_param(self):
        cfg = Config()
        reset = cfg.param('global/reset')
        self.assertEqual(False, reset, "wrong reset value, expected: %s, got: %s" % (False, reset))
        none_value = cfg.param('global/invalid')
        self.assertEqual(None, none_value, "wrong result for not existing parameter, expected: %s, got: %s" % (None, none_value))
        default_value = cfg.param('global/invalid', 123)
        self.assertEqual(123, default_value, "wrong result for not existing parameter with default value, expected: %d, got: %s" % (123, default_value))
        reset_dict = cfg.param('global/reset', extract_value=False)
        is_dict = isinstance(reset_dict, dict)
        self.assertEqual(True, is_dict, "wrong result value, expected: dict, got: %s" % (type(reset_dict)))

    def test_set_param(self):
        cfg = Config()
        # change existing parameter
        cfg.set_param('global/reset', True, save_on_change=False)
        reset = cfg.param('global/reset')
        self.assertEqual(True, reset, "wrong reset value after set_param, expected: %s, got: %s" % (True, reset))
        # set new parameter
        cfg.set_param('global/new_param', 123, save_on_change=False)
        new_param = cfg.param('global/new_param')
        self.assertEqual(123, new_param, "not found new param after set_param, expected: %d, got: %s" % (123, new_param))
        # change read_only parameter
        try:
            cfg.set_param('global/version', 123, save_on_change=False)
            self.fail("change read only parameter not throws an exception.")
        except Exception:
            pass

    def test_set_param_notification(self):
        cfg = Config()
        cfg.add_param_listener('global/reset', self._dynamic_set_param_callback)
        cfg.set_param('global/reset', True, save_on_change=False)
        if not self._param_changed:
            self.fail("dynamic parameter change: no notification on new value.")

    def _dynamic_set_param_callback(self, param, value):
        self._param_changed = True

    def test_dynamic_tcp_param(self):
        cfg = Config()
        cfg.add_param_listener('global/reset', self._dynamic_recv_callback)
        cfg.init_cfgif(cfgif=':54321')
        # connects to the configuration port and send new parameter value
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(('localhost', 54321))
            sock.sendall(b'global/reset: True\n', socket.MSG_DONTWAIT)
            sock.close()
        except Exception as err:
            print("WARNIG: failure while send parameter: %s" % err)
        # wait some time to transmitt new parameter value
        time.sleep(0.5)
        cfg.close()
        if not self._param_received:
            self.fail("dynamic parameter change: no notification by set through TCP.")

    def _dynamic_recv_callback(self, param, value):
        self._param_received = True


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestConfigLib)
