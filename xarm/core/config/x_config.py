#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import math


class XCONF(object):
    ARM_AXIS_NUM = 7
    GRIPPER_ID = 8
    TGPIO_ID = 9
    MAX_CMD_NUM = 1000

    def __init__(self):
        pass

    class Robot:
        class Axis:
            XARM5 = 5
            XARM6 = 6
            XARM7 = 7

        class Type:
            XARM6_X1 = 1
            XARM7_X2 = 2
            XARM7_X3 = 3
            XARM7_X3MIR = 4
            XARM5_X4 = 5
            XARM6_X4 = 6
            XARM7_X4 = 7

        JOINT_LIMITS = {
            Axis.XARM5: {
                Type.XARM5_X4: [
                    (-2 * math.pi, 2 * math.pi),
                    (-2.059488, 2.094395),  # (-2.18, 2.18),
                    (-3.92699, 0.191986),  # (-4.01, 0.1),
                    (-1.692969, math.pi),  # (-1.75, math.pi),
                    (-2 * math.pi, 2 * math.pi)
                ],
            },
            Axis.XARM6: {
                Type.XARM6_X4: [
                    (-2 * math.pi, 2 * math.pi),
                    (-2.059488, 2.094395),  # (-2.18, 2.18),
                    (-3.92699, 0.191986),  # (-4.01, 0.1),
                    (-2 * math.pi, 2 * math.pi),
                    (-1.692969, math.pi),  # (-1.75, math.pi),
                    (-2 * math.pi, 2 * math.pi)
                ],
            },
            Axis.XARM7: {
                Type.XARM7_X3: [
                    (-2 * math.pi, 2 * math.pi),
                    (-2.059488, 2.094395),  # (-2.18, 2.18),
                    (-2 * math.pi, 2 * math.pi),
                    (-3.92699, 0.191986),  # (-4.01, 0.1),
                    (-2 * math.pi, 2 * math.pi),
                    (-1.692969, math.pi),  # (-1.75, math.pi),
                    (-2 * math.pi, 2 * math.pi)
                ],
                Type.XARM7_X4: [
                    (-2 * math.pi, 2 * math.pi),
                    (-2.059488, 2.094395),  # (-2.18, 2.18),
                    (-2 * math.pi, 2 * math.pi),
                    (-0.191986, 3.92699),  # (-0.1, 4.01),
                    (-2 * math.pi, 2 * math.pi),
                    (-1.692969, math.pi),  # (-1.75, math.pi),
                    (-2 * math.pi, 2 * math.pi)
                ],
            }
        }
        TCP_LIMITS = {
            Axis.XARM5: {
                Type.XARM5_X4: [
                    (-750, 750),
                    (-750, 750),
                    (-400, 1000),
                    (math.pi, math.pi),
                    (0, 0),
                    (-math.pi, math.pi)
                ],
            },
            Axis.XARM6: {
                Type.XARM6_X1: [
                    (-750, 750),
                    (-750, 750),
                    (-400, 1000),
                    (-math.pi, math.pi),
                    (-math.pi, math.pi),
                    (-math.pi, math.pi)
                ],
                Type.XARM6_X4: [
                    (-750, 750),
                    (-750, 750),
                    (-400, 1000),
                    (-math.pi, math.pi),
                    (-math.pi, math.pi),
                    (-math.pi, math.pi)
                ],
            },
            Axis.XARM7: {
                Type.XARM7_X3: [
                    (-750, 750),
                    (-750, 750),
                    (-400, 1000),
                    (-math.pi, math.pi),
                    (-math.pi, math.pi),
                    (-math.pi, math.pi)
                ],
                Type.XARM7_X4: [
                    (-750, 750),
                    (-750, 750),
                    (-400, 1000),
                    (-math.pi, math.pi),
                    (-math.pi, math.pi),
                    (-math.pi, math.pi)
                ],
            }
        }

    class SerialConf:
        SERIAL_BAUD = 2000000  # 921600
        UXBUS_RXQUE_MAX = 10
        UXBUS_DEF_FROMID = 0xAA
        UXBUS_DEF_TOID = 0x55
        UX2_HEX_PROTOCOL = 1
        UX2_STR_PROTOCOL = 2
        UX1_HEX_PROTOCOL = 3
        UX1_STR_PROTOCOL = 4

    class SocketConf:
        TCP_CONTROL_PORT = 502
        TCP_REPORT_NORM_PORT = 30001
        TCP_REPORT_RICH_PORT = 30002
        TCP_REPORT_REAL_PORT = 30003
        TCP_RX_QUE_MAX = 1024
        TCP_CONTROL_BUF_SIZE = 1024
        TCP_REPORT_REAL_BUF_SIZE = 87
        TCP_REPORT_NORMAL_BUF_SIZE = 133
        TCP_REPORT_RICH_BUF_SIZE = 233

    class UxbusReg:
        GET_VERSION = 1
        GET_ROBOT_SN = 2
        CHECK_VERIFY = 3
        RELOAD_DYNAMICS = 4
        SHUTDOWN_SYSTEM = 10
        MOTION_EN = 11
        SET_STATE = 12
        GET_STATE = 13
        GET_CMDNUM = 14
        GET_ERROR = 15
        CLEAN_ERR = 16
        CLEAN_WAR = 17
        SET_BRAKE = 18
        SET_MODE = 19

        MOVE_LINE = 21
        MOVE_LINEB = 22
        MOVE_JOINT = 23
        MOVE_HOME = 25
        SLEEP_INSTT = 26
        MOVE_CIRCLE = 27
        MOVE_LINE_TOOL = 28
        MOVE_SERVOJ = 29
        MOVE_SERVO_CART = 30

        SET_TCP_JERK = 31
        SET_TCP_MAXACC = 32
        SET_JOINT_JERK = 33
        SET_JOINT_MAXACC = 34
        SET_TCP_OFFSET = 35
        SET_LOAD_PARAM = 36
        SET_COLLIS_SENS = 37
        SET_TEACH_SENS = 38
        CLEAN_CONF = 39
        SAVE_CONF = 40

        GET_TCP_POSE = 41
        GET_JOINT_POS = 42
        GET_IK = 43
        GET_FK = 44
        IS_JOINT_LIMIT = 45
        IS_TCP_LIMIT = 46

        SET_REDUCED_TRSV = 47
        SET_REDUCED_P2PV = 48
        GET_REDUCED_MODE = 49
        SET_REDUCED_MODE = 50
        SET_GRAVITY_DIR = 51
        SET_LIMIT_XYZ = 52
        GET_REDUCED_STATE = 53

        SET_SERVOT = 54
        GET_JOINT_TAU = 55
        SET_SAFE_LEVEL = 56
        GET_SAFE_LEVEL = 57

        SET_REDUCED_JRANGE = 58
        SET_FENSE_ON = 59
        SET_COLLIS_REB = 60

        SET_TRAJ_RECORD = 61
        SAVE_TRAJ = 62
        LOAD_TRAJ = 63
        PLAY_TRAJ = 64
        GET_TRAJ_RW_STATUS = 65

        SET_TIMER = 71
        CANCEL_TIMER = 72
        SET_WORLD_OFFSET = 73
        CNTER_RESET = 74
        CNTER_PLUS = 75

        SERVO_W16B = 101
        SERVO_R16B = 102
        SERVO_W32B = 103
        SERVO_R32B = 104
        SERVO_ZERO = 105
        SERVO_DBMSG = 106

        TGPIO_MB_TIOUT = 123
        TGPIO_MODBUS = 124
        TGPIO_ERR = 125
        TGPIO_W16B = 127
        TGPIO_R16B = 128
        TGPIO_W32B = 129
        TGPIO_R32B = 130

        CGPIO_GET_DIGIT = 131
        CGPIO_GET_ANALOG1 = 132
        CGPIO_GET_ANALOG2 = 133
        CGPIO_SET_DIGIT = 134
        CGPIO_SET_ANALOG1 = 135
        CGPIO_SET_ANALOG2 = 136
        CGPIO_SET_IN_FUN = 137
        CGPIO_SET_OUT_FUN = 138
        CGPIO_GET_STATE = 139

        GET_HD_TYPES = 141

    class UxbusConf:
        SET_TIMEOUT = 1000  # ms
        GET_TIMEOUT = 1000  # ms

    class ServoConf:
        CON_EN = 0x0100
        CON_MODE = 0x0101
        CON_DIR = 0x0102

        SV3MOD_POS = 0
        SV3MOD_SPD = 1
        SV3MOD_FOS = 2
        SV3_SAVE = 0x1000

        BRAKE = 0x0104
        GET_TEMP = 0x000E
        ERR_CODE = 0x000F
        OVER_TEMP = 0x0108
        CURR_CURR = 0x0001
        POS_KP = 0x0200
        POS_FWDKP = 0x0201
        POS_PWDTC = 0x0202
        SPD_KP = 0x0203
        SPD_KI = 0x0204
        CURR_KP = 0x090C
        CURR_KI = 0x090D
        SPD_IFILT = 0x030C
        SPD_OFILT = 0x030D
        POS_CMDILT = 0x030E
        CURR_IFILT = 0x0401
        POS_KD = 0x0205
        POS_ACCT = 0x0300
        POS_DECT = 0x0301
        POS_STHT = 0x0302
        POS_SPD = 0x0303
        MT_ID = 0x1600
        BAUDRATE = 0x0601
        SOFT_REBOOT = 0x0607
        TAGET_TOQ = 0x050a
        CURR_TOQ = 0x050c
        TOQ_SPD = 0x050e
        TAGET_POS = 0x0700
        CURR_POS = 0x0702
        HARD_VER = 0x0800
        SOFT_VER = 0x0801
        MT_TYPE = 0x0802
        MT_ZERO = 0x0817
        RESET_PVL = 0x0813
        CAL_ZERO = 0x080C
        ERR_SWITCH = 0x0910
        RESET_ERR = 0x0109
        SV3_BRO_ID = 0xFF

        MODBUS_BAUDRATE = 0x0A0B
        TOOL_MB_TIMEOUT = 0x0A0E
        DIGITAL_IN = 0x0A14
        DIGITAL_OUT = 0x0A15
        ANALOG_IO1 = 0x0A16
        ANALOG_IO2 = 0x0A17

    class UxbusState:
        ERR_CODE = 1  # 有尚未清除的错误
        WAR_CODE = 2  # 有尚未清除的警告
        ERR_TOUT = 3  # 获取结果超时
        ERR_LENG = 4  # TCP回复长度错误
        ERR_NUM = 5  # TCP回复序号错误
        ERR_PROT = 6  # TCP协议标志错误
        ERR_FUN = 7  # TCP回复指令和发送指令不匹配
        ERR_NOTTCP = 8  # 发送错误
        ERR_OTHER = 11  # 其它错误
        ERR_PARAM = 12  # 参数错误

    class TrajState:
        IDLE = 0
        LOADING = 1
        LOAD_SUCCESS = 2
        LOAD_FAIL = 3
        SAVING = 4
        SAVE_SUCCESS = 5
        SAVE_FAIL = 6



