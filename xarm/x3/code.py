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
    RUN_BLOCKLY_EXCEPTION = -12  # 运行blockly app异常
    NORMAL = 0  # 正常
    HAS_ERROR = XCONF.UxbusState.ERR_CODE  # 有尚未清除的错误
    HAS_WARN = XCONF.UxbusState.WAR_CODE  # 有尚未清除的警告
    RES_TIMEOUT = XCONF.UxbusState.ERR_TOUT  # 命令回复超时
    RES_LENGTH_ERROR = XCONF.UxbusState.ERR_LENG  # TCP长度错误
    CMD_NUM_ERROR = XCONF.UxbusState.ERR_NUM  # TCP序号错误
    CMD_PROT_ERROR = XCONF.UxbusState.ERR_PROT  # TCP协议标志错误
    FUN_ERROR = XCONF.UxbusState.ERR_FUN  # TCP回复指令和发送指令不匹配
    NO_TCP = XCONF.UxbusState.ERR_NOTTCP  # 写数据异常
    STATE_NOT_READY = XCONF.UxbusState.STATE_NOT_READY  # 参数错误
    RET_IS_INVALID = XCONF.UxbusState.INVALID  # 结果无效
    OTHER = XCONF.UxbusState.ERR_OTHER  # 其它错误
    PARAM_ERROR = XCONF.UxbusState.ERR_PARAM  # 参数错误

    HOST_ID_ERR = 20  # 主机ID错误, 看使用的接口，可能是末端IO也可能是滑轨
    MODBUS_BAUD_NOT_SUPPORT = 21  # modbus不支持此波特率
    MODBUS_BAUD_NOT_CORRECT = 22  # 末端modbus波特率不正确
    MODBUS_ERR_LENG = 23  # modbus回复数据长度错误

    TRAJ_RW_FAILED = 31  # 读写轨迹失败(加载轨迹或保存轨迹)
    TRAJ_RW_TOUT = 32  # 读写轨迹等待超时(加载轨迹或保存轨迹)
    TRAJ_PLAYBACK_TOUT = 33  # 回放轨迹超时(多种情况)
    TRAJ_PLAYBACK_FAILED = 34  # 回放轨迹失败(多种情况)
    SUCTION_CUP_TOUT = 41  # 等待吸泵设置超时

    MODE_IS_NOT_CORRECT = 51  # 模式不正确

    LINEAR_TRACK_HAS_FAULT = 80  # 滑轨有错误
    LINEAR_TRACK_SCI_IS_LOW = 81  # 滑轨的SCI被置低了
    LINEAR_TRACK_NOT_INIT = 82  # 直线滑轨未初始化

    WAIT_FINISH_TIMEOUT = 100  # 等待操作完成超时
    CHECK_FAILED = 101  # 等待操作完成过程检测状态连续失败次数过多
    END_EFFECTOR_HAS_FAULT = 102  # 末端配件有错误
    END_EFFECTOR_NOT_ENABLED = 103  # 末端配件未使能

    # 129 ~ 144: 标准modbus tcp的异常码，实际异常码(api_code - 0x80)

