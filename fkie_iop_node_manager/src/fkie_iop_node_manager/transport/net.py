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

import array
import fcntl
import platform
import socket
import struct

try:
    import netifaces
    _USE_NETIFACES = True
except Exception:
    _USE_NETIFACES = False


def localifs():
    '''
    Used to get a list of the up interfaces and associated IP addresses
    on local machine. (linux only if netifaces not available)

    :return: List of interface tuples.  Each tuple consists of ``(interface name, interface IP)``
    :rtype: list of ``(str, str)``
    '''
    if _USE_NETIFACES:
        # #addresses on multiple platforms (OS X, Unix, Windows)
        local_addrs = []
        # see http://alastairs-place.net/netifaces/
        for i in netifaces.interfaces():
            try:
                local_addrs.extend([(i, d['addr']) for d in netifaces.ifaddresses(i)[netifaces.AF_INET]])
            except KeyError:
                pass
        return local_addrs
    else:
        SIOCGIFCONF = 0x8912
        MAXBYTES = 8096
        arch = platform.architecture()[0]
        # I really don't know what to call these right now
        var1 = -1
        var2 = -1
        if arch == '32bit':
            var1 = 32
            var2 = 32
        elif arch == '64bit':
            var1 = 16
            var2 = 40
        else:
            raise OSError("Unknown architecture: %s" % arch)
        sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        names = array.array('B', '\0' * MAXBYTES)
        outbytes = struct.unpack('iL', fcntl.ioctl(sockfd.fileno(),
                                                   SIOCGIFCONF,
                                                   struct.pack('iL', MAXBYTES, names.buffer_info()[0])
                                                   ))[0]
        namestr = names.tostring()
        return [(namestr[i:i + var1].split('\0', 1)[0], socket.inet_ntoa(namestr[i + 20:i + 24])) for i in range(0, outbytes, var2)]


def is_local_iface(ip):
    return ip in [ip for _ifname, ip in localifs()]


def getaddrinfo(addr, family=None):
    '''
    :param addr: the addess to get the info for
    :param family: type of the address family (e.g. socket.AF_INET)
    '''
    # get info about the IP version (4 or 6)
    addrinfos = socket.getaddrinfo(addr, None)
    addrinfo = None
    if family is None and len(addrinfos) > 0:
        addrinfo = addrinfos[0]
    elif family:
        for ainfo in addrinfos:
            if ainfo[0] == family:
                return ainfo
    return addrinfo
