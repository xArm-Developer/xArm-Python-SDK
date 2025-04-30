# !/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import re
import sys
import socket
import logging
import threading

def create_logger(name):
    logger = logging.Logger(name)
    logger_fmt = '[%(levelname)s][%(asctime)s][%(filename)s:%(lineno)d] - - %(message)s'
    logger_date_fmt = '%Y-%m-%d %H:%M:%S'
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setLevel(logging.DEBUG)
    stream_handler.setFormatter(logging.Formatter(logger_fmt, logger_date_fmt))
    logger.handlers.clear()
    logger.addHandler(stream_handler)
    logger.setLevel(logging.INFO)
    return logger


GCODE_PATTERN = r'([A-Z])([-+]?[0-9.]+)'
CLEAN_PATTERN = r'\s+|\(.*?\)|;.*'


class GcodeClient(object):
    def __init__(self, robot_ip, logger=None):
        if isinstance(logger, logging.Logger):
            self.logger = logger
        else:
            self.logger = create_logger('gcode')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setblocking(True)
        self.sock.connect((robot_ip, 504))
        self.logger.info('Connetc to GcodeServer({}) success'.format(robot_ip))
        self._lock = threading.Lock()

    def close(self):
        self.sock.close()

    def execute(self, cmd):
        data = re.sub(CLEAN_PATTERN, '', cmd.strip().upper())
        if not data:
            # self.logger.warning('[E] null after clean {}'.format(cmd))
            return -1, []
        if data[0] == '%':
            # self.logger.warning('[E] starts with % ({})'.format(cmd))
            return -2, []
        if not re.findall(GCODE_PATTERN, data):
            # self.logger.warning('[E] not found {}'.format(cmd))
            return -3, []
        data = data.encode('utf-8', 'replace')
        with self._lock:
            self.sock.send(data + b'\n')
            ret = self.sock.recv(5)
        code, mode_state, err = ret[0:3]
        state, mode = mode_state & 0x0F, mode_state >> 4
        cmdnum = ret[3] << 8 | ret[4]
        if code != 0 or err != 0:
            self.logger.error('[{}], code={}, err={}, mode={}, state={}, cmdnum={}'.format(cmd, code, err, mode, state, cmdnum))
        elif state >= 4:
            self.logger.warning('[{}], code={}, err={}, mode={}, state={}, cmdnum={}'.format(cmd, code, err, mode, state, cmdnum))
        return code, [mode, state, err, cmdnum]
    
    def execute_file(self, filepath):
        if not os.path.exists(filepath) or os.path.isdir(filepath):
            return -99
        with open(filepath, 'r') as f:
            for line in f.readlines():
                cmd = line.strip()
                if not cmd:
                    continue
                code, info = self.execute(cmd)
                if code < 0:
                    continue
                if code != 0 or info[2] != 0:
                    if code != 1 and code != 2:
                        return code
                if cmd in ['M2', 'M02', 'M30']:
                    self.logger.info('[{}] Program End'.format(cmd))
                    break
        return 0
