#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Jimy Zhang <jimy.zhang@ufactory.cc> <jimy92@163.com>
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


import queue
import os
import socket
import struct
import platform
import threading
import time
from ..utils.log import logger
from .base import Port
from ..config.x_config import XCONF

# try:
#     if platform.system() == 'Linux':
#         import fcntl
#     else:
#         fcntl = None
# except:
#     fcntl = None
#
#
# def is_xarm_local_ip(ip):
#     try:
#         if platform.system() == 'Linux' and fcntl:
#             def _get_ip(s, ifname):
#                 try:
#                     return socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', ifname[:15]))[20:24])
#                 except:
#                     pass
#                 return ''
#             sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#             # gentoo system netcard name
#             if ip == _get_ip(sock, b'enp1s0'):
#                 return True
#             # rasp system netcard name
#             if ip == _get_ip(sock, b'eth0'):
#                 return True
#     except:
#         pass
#     return False


def get_all_ips():
    addrs = ['localhost', '127.0.0.1']
    addrs = set(addrs)
    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            try:
                if not ip.startswith('127.'):
                    addrs.add(ip)
            except:
                pass
    except:
        pass
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(3)
        sock.connect(('8.8.8.8', 53))
        addrs.add(sock.getsockname()[0])
    except:
        pass
    return addrs


class HeartBeatThread(threading.Thread):
    def __init__(self, sock_class):
        threading.Thread.__init__(self)
        self.sock_class = sock_class
        self.daemon = True

    def run(self):
        logger.debug('{} heartbeat thread start'.format(self.sock_class.port_type))
        heat_data = bytes([0, 0, 0, 1, 0, 2, 0, 0])

        while self.sock_class.connected:
            if self.sock_class.write(heat_data) == -1:
                break
            time.sleep(1)
        logger.debug('{} heartbeat thread had stopped'.format(self.sock_class.port_type))


class SocketPort(Port):
    def __init__(self, server_ip, server_port, rxque_max=XCONF.SocketConf.TCP_RX_QUE_MAX, heartbeat=False,
                 buffer_size=XCONF.SocketConf.TCP_CONTROL_BUF_SIZE, forbid_uds=False, fb_que=None):
        is_main_tcp = server_port == XCONF.SocketConf.TCP_CONTROL_PORT or server_port == XCONF.SocketConf.TCP_CONTROL_PORT + 1
        super(SocketPort, self).__init__(rxque_max, fb_que)
        if is_main_tcp:
            self.port_type = 'main-socket'
            # self.com.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, 5)
        else:
            self.port_type = 'report-socket'
        try:
            socket.setdefaulttimeout(1)
            use_uds = False
            # if not forbid_uds and platform.system() == 'Linux' and is_xarm_local_ip(server_ip):
            # if not forbid_uds and platform.system() == 'Linux' and server_ip in get_all_ips():
            if not forbid_uds and platform.system() == 'Linux':
                uds_path = os.path.join('/tmp/xarmcontroller_uds_{}'.format(server_port))
                if os.path.exists(uds_path):
                    try:
                        self.com = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                        self.com.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                        self.com.setblocking(True)
                        self.com.settimeout(1)
                        self.com.connect(uds_path)
                        logger.info('{} connect {} success, uds_{}'.format(self.port_type, server_ip, server_port))
                        use_uds = True
                    except Exception as e:
                        pass
                        # logger.error('use uds error, {}'.format(e))
            else:
                pass
            if not use_uds:
                self.com = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.com.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                # self.com.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                # self.com.setsockopt(socket.SOL_TCP, socket.TCP_KEEPIDLE, 30)
                # self.com.setsockopt(socket.SOL_TCP, socket.TCP_KEEPINTVL, 10)
                # self.com.setsockopt(socket.SOL_TCP, socket.TCP_KEEPCNT, 3)
                self.com.setblocking(True)
                self.com.settimeout(1)
                self.com.connect((server_ip, server_port))
                logger.info('{} connect {} success'.format(self.port_type, server_ip))
                # logger.info('{} connect {}:{} success'.format(self.port_type, server_ip, server_port))

            self._connected = True
            self.buffer_size = buffer_size
            # time.sleep(1)

            self.com_read = self.com.recv
            self.com_write = self.com.send
            self.write_lock = threading.Lock()
            self.start()
            if heartbeat:
                self.heartbeat_thread = HeartBeatThread(self)
                self.heartbeat_thread.start()
        except Exception as e:
            logger.info('{} connect {} failed, {}'.format(self.port_type, server_ip, e))
            # logger.error('{} connect {}:{} failed, {}'.format(self.port_type, server_ip, server_port, e))
            self._connected = False

