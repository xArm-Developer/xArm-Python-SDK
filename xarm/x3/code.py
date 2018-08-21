#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


from ..core.config.x_config import XCONF


class APIState(object):
    NOT_CONNECTED = -1  # xArmCore与控制器断开
    NOT_READY = -2  # 未使能或者设置状态
    API_EXCEPTION = -3  # 接口异常，可能是参数错误
    CMD_NOT_EXIST = -4  # 命令不存在
    TCP_LIMIT = -6  # 笛卡尔限位
    JOINT_LIMIT = -6  # 关节角度限位
    HAS_ERROR = XCONF.UxbusState.ERR_CODE  # 有错误
    HAS_WARN = XCONF.UxbusState.WAR_CODE  # 有警告
    RES_TIMEOUT = XCONF.UxbusState.ERR_TOUT  # 命令回复超时
    RES_LENGTH_ERROR = XCONF.UxbusState.ERR_LENG  # 命令回复的数据长度不对
    CMD_NUM_ERROR = XCONF.UxbusState.ERR_NUM  # 命令序号出错
    CMD_PROT_ERROR = XCONF.UxbusState.ERR_PROT  # TCP命令标志出错
    FUN_ERROR = XCONF.UxbusState.ERR_FUN  # TCP命令不对应
    NO_TCP = XCONF.UxbusState.ERR_NOTTCP  # xArmCore和控制器断开或写数据异常
    OTHER = XCONF.UxbusState.ERR_OTHER  # 其它错误


