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
import logging
import ruamel.yaml
import socket
import threading
import traceback


class Config:

    RECV_BUFFER = 5000  # 4094 should be also enough if MTU_Size = "4079"
    RECV_TIMEOUT = 1

    def __init__(self, filename='', version='', params={}):
        self._stop = False
        self.logger = logging.getLogger('config')
        self._mutex = threading.RLock()
        self.version = version
        self.filename = filename
        if not self.filename:
            self.filename = os.path.expanduser('~/.config/iop.fkie/iop_node_manager.yaml')
        cfg_path = os.path.dirname(self.filename)
        if not os.path.isdir(cfg_path):
            os.makedirs(cfg_path)
        self._reload_callbacks = []
        self._param_callbacks = {}
        self._cfg = None
        self.reload()
        self.apply(params, save=False)
        self.msg_ids = {}  # (int)id: (str)Name
        self._read_msg_ids()
        self._cfgif = None
        self._cfgif_address = None

    def init_cfgif(self, cfgif=''):
        try:
            p_cfgif = cfgif
            if not p_cfgif:
                p_cfgif = self.param('global/cfgif', ':37940')
            sliptres = p_cfgif.split(':')
            host = ''
            port = 37940
            if len(sliptres) == 1:
                if sliptres[0]:
                    port = int(sliptres[0])
                else:
                    # empty: disable configuration interface
                    self.logger.info("Dynamic configuration disabled by empty parameter `global/cfgif`")
                    return
            else:
                host = sliptres[0]
                port = int(sliptres[1])
            self.logger.info("Listen for dynamic parameter @%s:%d" % (host, port))
            # self._cfgif = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._cfgif = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self._cfgif.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except Exception:
                self.logger.warning("SO_REUSEPORT not available.")
            self._cfgif_address = (host, port)
            self._cfgif.bind((host, port))
            self._cfgif.listen(2)
            self._thread_listen_for_config = threading.Thread(target=self._recv_dynamic_parameter)
            self._thread_listen_for_config.start()
        except Exception as err:
            print(traceback.format_exc())
            self.logger.warning("Can not initilaize configuration UDP interface with param `%s`: %s" % (p_cfgif, err))

    def close(self):
        self._stop = True
        if self._cfgif is not None:
            self.logger.info("Close configuration socket")
            try:
                # we call socket.SHUT_RDWR to cancel bloking recv method
                self._cfgif.shutdown(socket.SHUT_RD)
                # connect to cancel accept()
                # close_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # close_socket.connect(self._cfgif_address)
                # close_socket.close()
            except Exception:
                self.logger.debug(traceback.format_exc())
            self._cfgif.close()

    def default(self):
        '''
        Creates a new default configuration.
        Value supports follow tags: {:value, :min, :max, :default, :hint(str), :ro(bool)}
        '''
        result = {
            'global': {
                'version': {':value': self.version, ':ro': True},
                'file': {':value': self.filename, ':ro': True},
                'reset': {':value': False, ':hint': 'if this flag is set to True the configuration will be reseted'},
                'loglevel': {':value': 'info', ':default': 'info', ':hint': 'Possible values: debug, info, warning, error, critical. (Default: info)'},
                'cfgif': {':value': ':37940', ':default': ':37940', ':hint': 'Configuration interface. Disabled if empty. (Default: :37940)'}
            },
            'transport':
            {
                'local':
                {
                    'root': {':value': '/tmp', ':default': '/tmp', ':hint': "The components communicate to the node manager through file sockets. For each component a new socket with it`s id will be created in this path."},
                    'nm_path': {':value': 'JuniorRTE', ':default': 'JuniorRTE', ':hint': "Contact socket name to the node manager."},
                    'queue_length': {':value': 100, ':default': 100, ':hint': "Maximal message count for each priority in the send queue."}
                },
                'udp':
                {
                    'port': {':value': 3794, ':default': 3794, ':hint': "By default, uses the port reserved for JAUS as assigned by the IANA for all UDP traffic. Changing the UDP port assignment is not recommended. (Default: 3794)."},
                    'use_mcast': {':value': True, ':default': True, ':hint': "If disabled, only unicast communication will be used. You need to specify the address book!"},
                    'group': {':value': '239.255.0.1', ':default': '239.255.0.1', ':hint': "Broadcasts are restricted to a multicast IP address. (Default: 239.255.0.1)."},
                    'ttl': {':value': 16, ':default': 16, ':hint': "Time to leave (Default: 16)."},
                    'queue_length': {':value': 100, ':default': 100, ':hint': "Maximal message count for each priority in the send queue."},
                    'interface': {':value': '', ':default': '', ':hint': "By default it binds to all network interfaces. You can bind it to specific one by specifying the address like 192.168.101.10"},
                    'buffer_size': {':value': 0, ':default': 0, ':hint': "Size of the send buffer. Zero do not changes the default buffer."},
                },
                'loopback_debug':
                {
                    'enable': {':value': False, ':default': False, ':hint': "Enable to mirror messages sent on its local socket to a UDP port. (Default: False)"},
                    'port': {':value': 55555, ':default': 55555, ':hint': "Destination port (Default: 55555)."},
                    'address': {':value': '127.0.0.1', ':default': '127.0.0.1', ':hint': "IP that will show in Wireshark, used if `use_mcast` is False."},
                    'use_mcast': {':value': False, ':hint': "If disabled, unicast address is used in other case the message will be broadcasted to a multicast IP. (Default: False)"},
                    'group': {':value': '239.255.0.100', ':default': '239.255.0.100', ':hint': "If `use_mcast` is True, the messages are broadcasted to a multicast IP address. (Default: 239.255.0.100)."},
                    'interface': {':value': '', ':default': '', ':hint': "By default it binds to all network interfaces. You can bind it to specific one by specifying the address like 192.168.101.10. Only if use_mcast is set to `True`."},
                    'queue_length': {':value': 100, ':default': 100, ':hint': "Maximal message count for each priority in the send queue."}
                },
                'tcp':
                {
                    'enable': {':value': False, ':default': False, ':hint': "Enable TCP communication. (Default: False)"},
                    'port': {':value': 3794, ':default': 3794, ':hint': "By default, uses the port reserved for JAUS as assigned by the IANA for all TCP traffic. Changing the TCP port assignment is not recommended. (Default: 3794)."},
                    'queue_length': {':value': 100, ':default': 100, ':hint': "Maximal message count for each priority in the send queue."},
                    'interface': {':value': '', ':default': '', ':hint': "By default it binds to all network interfaces. You can bind it to specific one by specifying the address like 192.168.101.10"},
                },
            },
            'addrbook':
            {
                'udp': {
                    ':value': {},  # {'192.168.0.2:3794': ['101.1.15', '101.255.255']},
                    ':hint': "Dictionary entries: {IP:PORT : single or list Jaus ID}. Jaus ID can be unique or contain wildcards (subsystem: 65535, node or component: 255)."
                },
                'tcp': {
                    ':value': {},  # {'128.7.92.233': '1.1.1', '128.7.92.114': '1.1.3'},  # {'192.168.0.2:3794': ['101.1.15', '101.255.255']},
                    ':hint': "Dictionary entries: {IP:PORT : single or list Jaus ID}. Jaus ID can be unique or contain wildcards (subsystem: 65535, node or component: 255)."
                }
            },
            'priority':
            {
                'override': {':value': False, ':hint': "Override priority for messages defined in config list."},
                'map': {
                    ':value': {
                        '0x000D': 3,  # RequestControl: critical
                        '0x000F': 3,  # ConfirmControl: critical
                        '0x4403': 1,  # ReportLocalPose: high
                        '0xD742': 0   # ReportCostMap2D: low
                    },
                    ':hint': "A dictionary of message id as HEX (0x1234) and priority (0: low, 1 standard, 2: hight, 3: critical)"
                },
            },
            'statistics':
            {
                'enable': {':value': False, ':default': False, ':hint': "Enable statistics of routed messages. (Default: False)"},
                'path': {':value': '~/.iop/statistics', ':default': '~/.iop/statistics', ':hint': "Directory where to save statistics", ':path': 'dir'},
                'msg_names': {':value': 'msg.ids', ':default': 'msg.ids', ':hint': "File with mapping of message id to their name. In case of relative path the configuration path is taken.", ':path': 'file'}
            }
        }
        return result

    def param(self, param_name, default_value=None, extract_value=True):
        '''
        Returns parameter value for given param_name.

        :param str param_name: name of the parameter. Namespace is separated by '/'.
        :param default_value: returns this value if parameter was not found (Default: None)
        :param bool extract_value: Since value is a dictionary with additional informations,
            try to extract value by default on True or return all options by False (Default: True).
        '''
        result = default_value
        try:
            path = param_name.split('/')
            value = self._cfg
            # go through the path
            for item in path:
                value = value[item]
            # extract value
            if isinstance(value, dict):
                if extract_value and ':value' in value:
                    result = value[':value']
                else:
                    result = value
            else:
                result = value
        except Exception as exc:
            self.logger.debug("Cant't get parameter '%s', full parameter path: '%s'; return default: %s" % (exc, param_name, result))
        return result

    def set_param(self, param_name, value, tag=':value', save_on_change=True):
        '''
        Sets new value to a parameter. The parameter can contain namespaces separated by '/'.
        Since a value can contain different tags, you can change the tag value
        by specifying the tag parameter.

        :param: str param_name: parameter name with namespaces.
        :param: value: new value.
        :param: str tag: tag name of parameter. It should begin with ':'.
        '''
        changed = False
        try:
            path = os.path.dirname(param_name).split('/')
            val_tag = tag if tag else ':value'
            cfg_item = self._cfg
            for item in path:
                if item:
                    if item in cfg_item:
                        cfg_item = cfg_item[item]
                    else:
                        cfg_item[item] = {}
                        cfg_item = cfg_item[item]
                        changed = True
            pname = os.path.basename(param_name)
            if pname in cfg_item:
                if isinstance(cfg_item[pname], dict):
                    if self._is_writable(cfg_item[pname]):
                        changed = cfg_item[pname][val_tag] != value
                        cfg_item[pname][val_tag] = value
                    else:
                        raise Exception('%s is a read only parameter!' % param_name)
                else:
                    changed = cfg_item[pname] != value
                    cfg_item[pname] = value
            else:
                # create new parameter entry
                cfg_item[pname] = {val_tag: value}
                changed = True
            if changed:
                if save_on_change:
                    self.save()
                self._notify_param_listener(param_name, value)
            return True
        except Exception as exc:
            self.logger.debug("Cant't set parameter '%s': '%s'" % (param_name, exc))
            return False

    def reload(self):
        '''
        Load the configuration from file. If file does not exists default configuration will be used.
        After configuration is loaded all subscribers are notified.
        '''
        with self._mutex:
            try:
                with open(self.filename, 'r') as stream:
                    result = ruamel.yaml.load(stream, Loader=ruamel.yaml.Loader)
                    if result is None:
                        self.logger.info('reset configuration file %s' % self.filename)
                        self._cfg = self.default()
                        self.save()
                    else:
                        self.logger.info('loaded configuration from %s' % self.filename)
                        self._cfg = result
            except (ruamel.yaml.YAMLError, IOError) as exc:
                self.logger.info('%s: use default configuration!' % exc)
                self._cfg = self.default()
            self._notify_reload_listener()

    def save(self, reset=False, save_msg_ids=False):
        '''
        Saves current configuration to file.
        '''
        if reset:
            self._cfg = self.default()
        with open(self.filename, 'w') as stream:
            try:
                ruamel.yaml.dump(self._cfg, stream, Dumper=ruamel.yaml.RoundTripDumper)
                self.logger.debug("Configuration saved to '%s'" % self.filename)
            except ruamel.yaml.YAMLError as exc:
                self.logger.warn("Cant't save configuration to '%s': %s" % (self.filename, exc))
        if save_msg_ids:
            filename = self.param('statistics/msg_names', 'msg.ids')
            if not os.path.isabs(filename):
                filename = os.path.join(os.path.dirname(self.filename), filename)
                with open(filename, 'w+') as fp:
                    for key, name in self.msg_ids.items():
                        line = '0x%.4x %s\n' % (key, name)
                        fp.write(line)

    def yaml(self, _nslist=[]):
        '''
        :param list nslist: Filter option. Currently not used!
        :return: Create YAML string representation from configuration dictionary structure.
        :rtype: str
        '''
        return ruamel.yaml.dump(self._cfg)

    def apply(self, data, save=True):
        '''
        Applies data (string representation of YAML).
        After new data are set the configuration will be saved to file.
        All subscribers are notified.

        :param str data: YAML as string representation.
        '''
        with self._mutex:
            data_dict = data
            if type(data) != dict:
                data_dict = ruamel.yaml.load(data, Loader=ruamel.yaml.Loader)
            self._cfg = self._apply_recursive(data_dict, self._cfg)
            do_reset = self.param('global/reset', False)
            if do_reset:
                self.logger.info("Reset configuration requested!")
                self._cfg = self.default()
            else:
                self.logger.debug("new configuration applied, save now.")
            if save:
                self.save()
            self._notify_reload_listener()

    def _apply_recursive(self, new_data, curr_value):
        new_cfg = dict()
        if not curr_value:
            return new_data
        for key, value in curr_value.items():
            try:
                if isinstance(value, dict):
                    if self._is_writable(value) and key in new_data:
                        new_cfg[key] = self._apply_recursive(new_data[key], value)
                    else:
                        new_cfg[key] = value
                elif key not in [':hint', ':default', ':ro', ':min', ':max', ':alt']:
                    if isinstance(new_data, dict):
                        new_cfg[key] = new_data[key]
                    else:
                        new_cfg[key] = new_data
                else:
                    new_cfg[key] = value
            except Exception:
                import traceback
                self.logger.warning("_apply_recursive error: %s, use old value: %s" % (traceback.format_exc(), str(value)))
                new_cfg[key] = value
        return new_cfg

    def _is_writable(self, value):
        if ':ro' in value:
            return value[':ro']
        return True

    def add_reload_listener(self, callback, call=True):
        '''
        Adds a subscriber to change notifications. All subscribers are notified on any changes.

        :param callback: Method of type callback(Settings)
        :param call: if True the callback is called after adding. (Default: True)
        '''
        with self._mutex:
            if callback not in self._reload_callbacks:
                self._reload_callbacks.append(callback)
                if call:
                    callback(self)

    def _notify_reload_listener(self):
        with self._mutex:
            for callback in self._reload_callbacks:
                callback(self)

    def add_param_listener(self, paramname, callback):
        '''
        Adds a subscriber to dynamic changes of a specific parameter.

        :param str paramname: Name of the parameter
        :param callback: Method of type callback(paramname[str], value)
        :param call: if True the callback is called after adding. (Default: True)
        '''
        if paramname not in self._param_callbacks:
            self._param_callbacks[paramname] = []
        self.logger.debug("Add parameter listener %s for %s" % (callback, paramname))
        self._param_callbacks[paramname].append(callback)

    def _notify_param_listener(self, paramname, value):
        for pn, callbacks in self._param_callbacks.items():
            if paramname.startswith(pn):
                for callback in callbacks:
                    callback(paramname, value)

    def _recv_dynamic_parameter(self):
        while not self._stop:
            try:
                connection, addr = self._cfgif.accept()
                if not self._stop:
                    data = connection.recv(65535)
                    for line in data.splitlines():
                        pv = line.decode("utf-8") .split(': ')
                        self.logger.info("Parameter change requested from %s: %s" % (addr, pv))
                        if len(pv) == 2:
                            try:
                                if pv[0] in self._param_callbacks:
                                    for clbk in self._param_callbacks[pv[0]]:
                                        clbk(pv[0], pv[1])
                                else:
                                    self.logger.warning("No callback for dynamic parameter found: %s" % pv)
                            except Exception as perr:
                                self.logger.warning("Error while notify parameter listener on changed parameter '%s': %s" % (pv[0], perr))
                connection.close()
            except Exception as err:
                if err.errno == 22:
                    # handle shutdown
                    return
                print(traceback.format_exc())
                self.logger.warning("Error while receive configuration through UDP interface on %s: %s" % (self._cfgif_address, err))
                # raise Exception(traceback.format_exc())

    def msg_name(self, msgid):
        '''
        Returns to given message id a name if exists.
        Returns `Unknown` if id not found.
        '''
        try:
            return self.msg_ids[msgid]
        except KeyError:
            pass
        return 'Unknown'

    def _read_msg_ids(self):
        # try load from file
        filename = self.param('statistics/msg_names', 'msg.ids')
        incfg = False
        if not os.path.isabs(filename):
            filename = os.path.join(os.path.dirname(self.filename), filename)
            incfg = True
        if os.path.exists(filename):
            with open(filename, 'r') as fp:
                for line in fp.readlines():
                    try:
                        kn = line.rstrip('\n').split()
                        self.msg_ids[int(kn[0], 16)] = kn[1]
                    except Exception:
                        print(traceback.format_exc())
        elif incfg:
            # create default list
            self._init_msgs_ids()

    def _init_msgs_ids(self):
        self.msg_ids[int('2B00', 16)] = 'QueryIdentification'
        self.msg_ids[int('000D', 16)] = 'RequestControl'
        self.msg_ids[int('200D', 16)] = 'QueryControl'
        self.msg_ids[int('21F0', 16)] = 'QueryEvents'
