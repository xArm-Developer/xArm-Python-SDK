#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc>


import threading
import queue
import socket
import select
import time
from ..utils.log import logger


class RxParse(object):
    def __init__(self, rx_que):
        self.rx_que = rx_que

    def flush(self, fromid=-1, toid=-1):
        pass

    def put(self, data):
        self.rx_que.put(data)


class Port(threading.Thread):
    def __init__(self, rxque_max):
        super(Port, self).__init__()
        self.daemon = True
        self.rx_que = queue.Queue(rxque_max)
        self.write_lock = threading.Lock()
        self._connected = False
        self.com = None
        self.rx_parse = RxParse(self.rx_que)
        self.com_read = None
        self.com_write = None
        self.port_type = ''
        self.buffer_size = 1
        self.heartbeat_thread = None
        self.alive = True

    @property
    def connected(self):
        return self._connected

    def run(self):
        self.recv_proc()
        # self.recv_loop()

    def close(self):
        self.alive = False
        if 'socket' in self.port_type:
            try:
                self.com.shutdown(socket.SHUT_RDWR)
            except:
                pass
        try:
            self.com.close()
        except:
            pass
        # start_time = time.time()
        # while self.connected and time.time() - start_time < 5:
        #     time.sleep(0.01)

    def flush(self, fromid=-1, toid=-1):
        if not self.connected:
            return -1
        while not(self.rx_que.empty()):
            self.rx_que.queue.clear()
        self.rx_parse.flush(fromid, toid)
        return 0

    def write(self, data):
        if not self.connected:
            return -1
        try:
            with self.write_lock:
                logger.verbose('[{}] send: {}'.format(self.port_type, data))
                self.com_write(data)
            return 0
        except Exception as e:
            self._connected = False
            logger.error("[{}] send error: {}".format(self.port_type, e))
            return -1

    def read(self, timeout=None):
        if not self.connected:
            return -1
        try:
            buf = self.rx_que.get(timeout=timeout)
            logger.verbose('[{}] recv: {}'.format(self.port_type, buf))
            return buf
        except:
            return -1
        # if not self.connected:
        #     return -1
        # if not self.rx_que.empty():
        #     buf = self.rx_que.get(timeout=timeout)
        #     logger.verbose('[{}] recv: {}'.format(self.port_type, buf))
        #     return buf
        # else:
        #     return -1

    def recv_loop(self):
        self.alive = True
        logger.debug('[{}] recv thread start'.format(self.port_type))
        try:
            while self.connected and self.alive:
                if 'socket' in self.port_type:
                    ready_input, ready_output, ready_exception = select.select([self.com], [], [])
                    for indata in ready_input:
                        if indata == self.com:
                            rx_data = self.com_read(self.buffer_size)
                            break
                    else:
                        continue
                else:
                    rx_data = self.com_read(self.com.in_waiting or self.buffer_size)
                self.rx_parse.put(rx_data)
        except Exception as e:
            if self.alive:
                logger.error('[{}] recv error: {}'.format(self.port_type, e))
        finally:
            self.close()
        logger.debug('[{}] recv thread had stopped'.format(self.port_type))
        self._connected = False

    def recv_proc(self):
        self.alive = True
        logger.debug('[{}] recv thread start'.format(self.port_type))
        try:
            failed_read_count = 0
            timeout_count = 0
            while self.connected and self.alive:
                if self.port_type == 'main-socket':
                    try:
                        rx_data = self.com_read(self.buffer_size)
                    except socket.timeout:
                        continue
                    if len(rx_data) == 0:
                        failed_read_count += 1
                        if failed_read_count > 5:
                            self._connected = False
                            break
                        time.sleep(0.1)
                        continue
                elif self.port_type == 'report-socket':
                    try:
                        rx_data = self.com_read(self.buffer_size)
                    except socket.timeout:
                        timeout_count += 1
                        if timeout_count > 3:
                            self._connected = False
                            break
                        continue
                    if len(rx_data) == 0:
                        failed_read_count += 1
                        if failed_read_count > 5:
                            self._connected = False
                            break
                        time.sleep(0.1)
                        continue
                elif self.port_type == 'main-serial':
                    rx_data = self.com_read(self.com.in_waiting or self.buffer_size)
                else:
                    break
                timeout_count = 0
                failed_read_count = 0
                self.rx_parse.put(rx_data)
        except Exception as e:
            if self.alive:
                logger.error('[{}] recv error: {}'.format(self.port_type, e))
        finally:
            self.close()
        logger.debug('[{}] recv thread had stopped'.format(self.port_type))
        self._connected = False
        # if self.heartbeat_thread:
        #     try:
        #         self.heartbeat_thread.join()
        #     except:
        #         pass

