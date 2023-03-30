#!/usr/bin/env python3
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc>

import time
import queue
import socket
import select
import threading
from ..utils.log import logger
from ..utils import convert


class RxParse(object):
    def __init__(self, rx_que, fb_que=None):
        self.rx_que = rx_que
        self.fb_que = fb_que

    def flush(self, fromid=-1, toid=-1):
        pass

    def put(self, data, is_report=False):
        if not is_report and data[6] == 0xFF:
            if not self.fb_que:
                return
            self.fb_que.put(data)
        else:
            self.rx_que.put(data)


class Port(threading.Thread):
    def __init__(self, rxque_max, fb_que=None):
        super(Port, self).__init__()
        self.daemon = True
        self.rx_que = queue.Queue(rxque_max)
        self.fb_que = fb_que
        self.write_lock = threading.Lock()
        self._connected = False
        self.com = None
        self.rx_parse = RxParse(self.rx_que, self.fb_que)
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
        if self.port_type == 'report-socket':
            self.recv_report_proc()
        else:
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

    # def recv_loop(self):
    #     self.alive = True
    #     logger.debug('[{}] recv thread start'.format(self.port_type))
    #     try:
    #         while self.connected and self.alive:
    #             if 'socket' in self.port_type:
    #                 ready_input, ready_output, ready_exception = select.select([self.com], [], [])
    #                 for indata in ready_input:
    #                     if indata == self.com:
    #                         rx_data = self.com_read(self.buffer_size)
    #                         break
    #                 else:
    #                     continue
    #             else:
    #                 rx_data = self.com_read(self.com.in_waiting or self.buffer_size)
    #             self.rx_parse.put(rx_data)
    #     except Exception as e:
    #         if self.alive:
    #             logger.error('[{}] recv error: {}'.format(self.port_type, e))
    #     finally:
    #         self.close()
    #     logger.debug('[{}] recv thread had stopped'.format(self.port_type))
    #     self._connected = False

    def recv_report_proc(self):
        self.alive = True
        logger.debug('[{}] recv thread start'.format(self.port_type))
        failed_read_count = 0
        timeout_count = 0
        size = 0
        data_num = 0
        buffer = b''
        size_is_not_confirm = False

        data_prev_us = 0
        data_curr_us = 0
        data_max_interval_us = 0
        data_over_us = 205 * 1000  # over 205ms, cnts++
        data_over_cnts = 0

        recv_prev_us = 0
        recv_curr_us = 0
        recv_max_interval_us = 0
        recv_over_us = 300 * 1000  # over 300ms, cnts++
        recv_over_cnts = 0

        try:
            while self.connected and self.alive:
                try:
                    data = self.com_read(4 - data_num if size == 0 else (size - data_num))
                except socket.timeout:
                    timeout_count += 1
                    if timeout_count > 3:
                        self._connected = False
                        logger.error('[{}] socket read timeout'.format(self.port_type))
                        break
                    continue
                else:
                    if len(data) == 0:
                        failed_read_count += 1
                        if failed_read_count > 5:
                            self._connected = False
                            logger.error('[{}] socket read failed, len=0'.format(self.port_type))
                            break
                        time.sleep(0.1)
                        continue
                    data_num += len(data)
                    buffer += data
                    if size == 0:
                        if data_num != 4:
                            continue
                        size = convert.bytes_to_u32(buffer[0:4])
                        if size == 233:
                            size_is_not_confirm = True
                            size = 245
                        logger.info('report_data_size: {}, size_is_not_confirm={}'.format(size, size_is_not_confirm))
                    else:
                        if data_num < size:
                            continue
                        if size_is_not_confirm:
                            size_is_not_confirm = True
                            if convert.bytes_to_u32(buffer[233:237]) == 233:
                                size = 233
                                buffer = buffer[233:]
                                continue

                        if convert.bytes_to_u32(buffer[0:4]) != size:
                            logger.error('report data error, close, length={}, size={}'.format(convert.bytes_to_u32(buffer[0:4]), size))
                            break

                        # # buffer[494:502]
                        # data_curr_us = convert.bytes_to_u64(buffer[-8:])
                        # recv_curr_us = time.monotonic() * 1000000
                        #
                        # if data_prev_us != 0 and recv_prev_us != 0:
                        #     data_interval_us = data_curr_us - data_prev_us
                        #     data_over_cnts += 1 if data_interval_us > data_over_us else 0
                        #
                        #     recv_interval_us = recv_curr_us - recv_prev_us
                        #     recv_over_cnts += 1 if recv_interval_us > recv_over_us else 0
                        #
                        #     print_flag = False
                        #
                        #     if data_interval_us > data_max_interval_us:
                        #         data_max_interval_us = data_interval_us
                        #         print_flag = True
                        #     elif data_interval_us > data_over_us:
                        #         print_flag = True
                        #
                        #     if recv_interval_us > recv_max_interval_us:
                        #         recv_max_interval_us = recv_interval_us
                        #         print_flag = True
                        #     elif recv_interval_us > recv_over_us:
                        #         print_flag = True
                        #
                        #     if print_flag:
                        #         print('[RECV] Di={}, Dmax={}, Dcnts={}, Ri={}, Rmax={}, Rcnts={}'.format(
                        #             data_interval_us / 1000, data_max_interval_us / 1000, data_over_cnts,
                        #             recv_interval_us / 1000, recv_max_interval_us / 1000, recv_over_cnts
                        #         ))
                        # data_prev_us = data_curr_us
                        # recv_prev_us = recv_curr_us

                        if self.rx_que.qsize() > 1:
                            self.rx_que.get()
                        self.rx_parse.put(buffer, True)
                        buffer = b''
                        data_num = 0

                    timeout_count = 0
                    failed_read_count = 0
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
        is_main_tcp = self.port_type == 'main-socket'
        is_main_serial = self.port_type == 'main-serial'
        try:
            failed_read_count = 0
            buffer = b''
            while self.connected and self.alive:
                if is_main_tcp:
                    try:
                        rx_data = self.com_read(self.buffer_size)
                    except socket.timeout:
                        continue
                    if len(rx_data) == 0:
                        failed_read_count += 1
                        if failed_read_count > 5:
                            self._connected = False
                            logger.error('[{}] socket read failed, len=0'.format(self.port_type))
                            break
                        time.sleep(0.1)
                        continue
                    buffer += rx_data
                    while True:
                        if len(buffer) < 6:
                            break
                        length = convert.bytes_to_u16(buffer[4:6]) + 6
                        if len(buffer) < length:
                            break
                        rx_data = buffer[:length]
                        buffer = buffer[length:]
                        self.rx_parse.put(rx_data)
                elif is_main_serial:
                    rx_data = self.com_read(self.com.in_waiting or self.buffer_size)
                    self.rx_parse.put(rx_data)
                else:
                    break
                failed_read_count = 0
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

