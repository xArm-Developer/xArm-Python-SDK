#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Jimy Zhang <jimy.zhang@ufactory.cc> <jimy92@163.com>
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


from ..utils import crc16
from ..utils.log import logger

# ux2_hex_protocol define
UX2HEX_RXSTART_FROMID = 0
UX2HEX_RXSTART_TOID = 1
UX2HEX_RXSTATE_LEN = 2
UX2HEX_RXSTATE_DATA = 3
UX2HEX_RXSTATE_CRC1 = 4
UX2HEX_RXSTATE_CRC2 = 5
UX2HEX_RXLEN_MAX = 50


class Ux2HexProtocol(object):
    """
    fromid and toid: broadcast address is 0xFF
    """
    def __init__(self, rx_que, fromid, toid):
        self.rx_que = rx_que
        self.rxstate = UX2HEX_RXSTART_FROMID
        self.data_idx = 0
        self.len = 0
        self.fromid = fromid
        self.toid = toid
        self.rxbuf = None

    # wipe cache , set from_id and to_id
    def flush(self, fromid=-1, toid=-1):
        self.rxstate = UX2HEX_RXSTART_FROMID
        self.data_idx = 0
        self.len = 0
        if fromid != -1:
            self.fromid = fromid
        if toid != -1:
            self.toid = toid

    def put(self, rxstr, length=0):
        if length == 0:
            length = len(rxstr)
        if len(rxstr) < length:
            logger.error('len(rxstr) < length')

        for i in range(length):
            rxch = bytes([rxstr[i]])
            # print_hex(self.DB_FLG, rxch, 1)
            # print('state:%d' % (self.rxstate))
            if UX2HEX_RXSTART_FROMID == self.rxstate:
                if self.toid == rxch[0] or 255 == self.toid:
                    self.rxbuf = rxch
                    self.rxstate = UX2HEX_RXSTART_TOID

            elif UX2HEX_RXSTART_TOID == self.rxstate:
                if self.fromid == rxch[0] or self.fromid == 0xFF:
                    self.rxbuf += rxch
                    self.rxstate = UX2HEX_RXSTATE_LEN
                else:
                    self.rxstate = UX2HEX_RXSTART_FROMID

            elif UX2HEX_RXSTATE_LEN == self.rxstate:
                if rxch[0] < UX2HEX_RXLEN_MAX:
                    self.rxbuf += rxch
                    self.len = rxch[0]
                    self.data_idx = 0
                    self.rxstate = UX2HEX_RXSTATE_DATA
                else:
                    self.rxstate = UX2HEX_RXSTART_FROMID

            elif UX2HEX_RXSTATE_DATA == self.rxstate:
                if self.data_idx < self.len:
                    self.rxbuf += rxch
                    self.data_idx += 1
                    if self.data_idx == self.len:
                        self.rxstate = UX2HEX_RXSTATE_CRC1
                else:
                    self.rxstate = UX2HEX_RXSTART_FROMID

            elif UX2HEX_RXSTATE_CRC1 == self.rxstate:
                self.rxbuf += rxch
                self.rxstate = UX2HEX_RXSTATE_CRC2

            elif UX2HEX_RXSTATE_CRC2 == self.rxstate:
                self.rxbuf += rxch
                self.rxstate = UX2HEX_RXSTART_FROMID
                crc = crc16.crc_modbus(self.rxbuf[:self.len + 3])
                if crc[0] == self.rxbuf[self.len + 3] and crc[1] == self.rxbuf[self.len + 4]:
                    if self.rx_que.full():
                        self.rx_que.get()
                    self.rx_que.put(self.rxbuf)
                    # print(self.rxbuf)

