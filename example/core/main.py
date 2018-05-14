#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#
#  ./main.py
#  Copyright (C) 2018.4 -  UFactory.
#  Author: Jimy Zhang   <jimy.zhang@ufactory.cc>
#                       <jimy92@163.com>
#

import time
from argparse import ArgumentParser
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from xarm.core.utils import convert
from xarm.core.utils.debug_print import print_nvect
from xarm.core.comm import SerialPort, SocketPort
from xarm.core.wrapper import UX2HexCmd, TX2HexCmd
from xarm.core.config import x2_config


DB_FLG = '[main    ] '


def args_parse():
    parser = ArgumentParser()
    parser.add_argument('--connect', dest='connect', default='tcp', help='Hardware connection mode,tcp or 485')
    parser.add_argument('--ip', dest='ip', default=x2_config.SERVER_IP, help='modbus ip')
    parser.add_argument('--com', dest='com', default=x2_config.CTSER_COM_NAME, help='serial com')
    args = parser.parse_args()
    return args


def print_report_norm(rx_data):
    angle = [0] * 7
    pose = [0] * 6
    tmp_u8 = [0] * 5
    tcp_pose = [0] * 6
    if rx_data != -1 and len(rx_data) > 5:
        tmp_u8[0:4] = rx_data[0:4]
        angle = convert.bytes_to_fp32s(rx_data[5:7 * 4 + 5], 7)
        pose = convert.bytes_to_fp32s(rx_data[33:6 * 4 + 33], 6)
        cmdnum = convert.bytes_to_u16(rx_data[57:59])
        tcp_pose = convert.bytes_to_fp32s(rx_data[59:6 * 4 + 59], 6)
        print("runing = %d, mtbrake = %d, maable = %d, err = %d, war = %d, cmdnum = %d" % \
              (tmp_u8[0], tmp_u8[1], tmp_u8[2], tmp_u8[3],tmp_u8[4], cmdnum))
        print_nvect("angle  : ", angle, 7)
        print_nvect("pose   : ", pose, 6)
        print_nvect("tcp_pos: ", tcp_pose, 6)


def print_report_rich(rx_data):
    trs_msg = [0] * 5
    p2p_msg = [0] * 5
    ros_msg = [0] * 2
    ver_msg = [0] * 20
    if rx_data != -1 and len(rx_data) > 5:
        print_report_norm(rx_data)
        arm_type  = rx_data[83]
        arm_axis  = rx_data[84]
        arm_mid   = rx_data[85]
        arm_sid   = rx_data[86]
        arm_mttid = rx_data[87]
        arm_mtfid = rx_data[88]
        ver_msg   = rx_data[89:108]
        trs_msg = convert.bytes_to_fp32s(rx_data[109:129], 5)
        p2p_msg = convert.bytes_to_fp32s(rx_data[129:149], 5)
        ros_msg = convert.bytes_to_fp32s(rx_data[149:157], 2)
        print("arm_type = %d, arm_axis = %d, arm_mid = %d, arm_sid = %d, arm_mttid = %d, arm_mtfid = %d" % \
              (arm_type, arm_axis, arm_mid, arm_sid, arm_mttid, arm_mtfid))
        print_nvect("trs_msg: ", trs_msg, 5)
        print_nvect("p2p_msg: ", p2p_msg, 5)
        print_nvect("ros_msg: ", ros_msg, 2)
        str1 = str(ver_msg, "utf-8")
        print("ver_msg: " + str1)


def connect_485(port):
    arm_port = SerialPort(port, x2_config.CTSER_COM_BAUD, x2_config.UXBUS_RXQUE_MAX, x2_config.UX2_HEX_PROTOCOL)
    if 0 != arm_port.is_ok():
        print(DB_FLG + "error: serial not connect")
        return -1
    arm_cmd = UX2HexCmd(arm_port)
    print(DB_FLG + "rs485 connect")
    return arm_cmd


def connect_tcp(ip):
    arm_port = SocketPort(ip, x2_config.SERVER_PORT, x2_config.TCP_RX_QUE_MAX, 1)
    if 0 != arm_port.is_ok():
        print(DB_FLG + "error: tcp not connect")
        return -1
    arm_cmd = TX2HexCmd(arm_port)
    print(DB_FLG + "tcp connect")
    return arm_cmd


def connect_report_norm(ip):
    rxcnt = 0
    arm_report = SocketPort(ip, x2_config.SERVER_REPORT_NORM, x2_config.TCP_RX_QUE_MAX)
    if 0 != arm_report.is_ok():
        print(DB_FLG + "error: tcp report norm not connect")
        return -1
    print(DB_FLG + "tcp report norm connect")
    while (1):
        rx_data = arm_report.read()
        if rx_data != -1 and len(rx_data) > 5:
            rxcnt += 1
            print("【normal report】: len = %d, rxcnt = %d" % (len(rx_data), rxcnt))
            print_report_norm(rx_data)
            print(" ")
        time.sleep(0.001)


def connect_report_rich(ip):
    rxcnt = 0
    arm_report = SocketPort(ip, x2_config.SERVER_REPORT_RICH, x2_config.TCP_RX_QUE_MAX)
    if 0 != arm_report.is_ok():
        print(DB_FLG + "error: tcp report rich not connect")
        return -1
    print(DB_FLG + "tcp report rich connect")
    while (1):
        rx_data = arm_report.read()
        if rx_data != -1 and len(rx_data) > 5:
            rxcnt += 1
            print("【rich   report】: len = %d, rxcnt = %d" % (len(rx_data), rxcnt))
            print_report_rich(rx_data)
            print(" ")
        time.sleep(0.001)


def connect_report_realt(ip):
    rxcnt = 0
    arm_report = SocketPort(ip, x2_config.SERVER_REPORT_REALT, x2_config.TCP_RX_QUE_MAX)
    if 0 != arm_report.is_ok():
        print(DB_FLG + "error: tcp report real not connect")
        return -1
    print(DB_FLG + "tcp report realt connect")
    while (1):
        rx_data = arm_report.read()
        if rx_data != -1 and len(rx_data) > 5:
            rxcnt += 1
            print("【real   report】: len = %d, rxcnt = %d" % (len(rx_data), rxcnt))
            print_report_norm(rx_data)
            print(" ")
        time.sleep(0.001)


def main():
    arm_cmd = -1
    if args.connect == '485':
        arm_cmd = connect_485(args.com)
    elif args.connect == 'tcp':
        arm_cmd = connect_tcp(args.ip)
    elif args.connect == 'tcp_normal':
        connect_report_norm(args.ip)
    elif args.connect == 'tcp_rich':
        connect_report_rich(args.ip)
    elif args.connect == 'tcp_real':
        connect_report_realt(args.ip)
    if -1 == arm_cmd:
        return

    while True:
        time.sleep(1)

if __name__ == '__main__':
    args = args_parse()
    print(DB_FLG + "main thread start")
    main()
    print(DB_FLG + "main thread end")
