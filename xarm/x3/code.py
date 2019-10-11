#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


from ..core.config.x_config import XCONF


class APIState(object):
    NOT_CONNECTED = -1  # 已断开或未连接
    NOT_READY = -2  # 未使能或者设置状态
    API_EXCEPTION = -3  # 接口异常，可能是参数错误
    CMD_NOT_EXIST = -4  # 命令不存在
    TCP_LIMIT = -6  # 笛卡尔限位
    JOINT_LIMIT = -7  # 关节角度限位
    OUT_OF_RANGE = -8  # 超出范围
    EMERGENCY_STOP = -9  # 紧急停止
    SERVO_NOT_EXIST = -10  # 不存在此ID的关节
    CONVERT_FAILED = -11  # 转换Blockly失败
    NORMAL = 0  # 正常
    HAS_ERROR = XCONF.UxbusState.ERR_CODE  # 有尚未清除的错误
    HAS_WARN = XCONF.UxbusState.WAR_CODE  # 有尚未清除的警告
    RES_TIMEOUT = XCONF.UxbusState.ERR_TOUT  # 命令回复超时
    RES_LENGTH_ERROR = XCONF.UxbusState.ERR_LENG  # TCP长度错误
    CMD_NUM_ERROR = XCONF.UxbusState.ERR_NUM  # TCP序号错误
    CMD_PROT_ERROR = XCONF.UxbusState.ERR_PROT  # TCP协议标志错误
    FUN_ERROR = XCONF.UxbusState.ERR_FUN  # TCP回复指令和发送指令不匹配
    NO_TCP = XCONF.UxbusState.ERR_NOTTCP  # 写数据异常
    OTHER = XCONF.UxbusState.ERR_OTHER  # 其它错误
    PARAM_ERROR = XCONF.UxbusState.ERR_PARAM  # 参数错误
    TRAJ_RW_FAILED = 31  # 读写轨迹失败(加载轨迹或保存轨迹)
    TRAJ_RW_TOUT = 32  # 读写轨迹等待超时(加载轨迹或保存轨迹)
    TRAJ_PLAYBACK_TOUT = 33  # 回放轨迹超时(多种情况)
    SUCTION_CUP_TOUT = 41  # 等待吸泵设置超时


