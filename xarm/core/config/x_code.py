#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


ServoCodeMap = {
    0x01: {
        'description': '通讯地址无效或参数写入只读地址',
        'handle': ['检查通讯地址', '检查通讯地址对应的属性']
    },
    0x02: {
        'description': '写入数值超限或不能写入EEPROM',
        'handle': ['检查通讯地址对应的参数范围', '检查通讯地址对应的属性']
    },
    0x0A: {
        'description': '电流零点偏置错误',
        'handle': ['重新上电(如多次上电无效，更新驱动板)']
    },
    0x0B: {
        'description': '过流保护',
        'handle': ['检查相序是否正确', '检查电机类型是否正确', '检查抱闸是否正常打开', '检查是否堵转', '设置的控制命令是否变动很大', '电机零点是否正确']
    },
    0x0C: {
        'description': '电机相序错误或速度输入值大于过速值或速度超调太大或电机零点错误',
        'handle': ['检查相序是否正确', '降低速度指令', '降低调节器增益', '设置零点']
    },
    0x0D: {
        'description': '位置偏差过大',
        'handle': ['降低位置指令', '加大位置滤波时间', '位置偏差报警值设置过小(Fn308)', '电机是否堵转', '适当加大位置增益']
    },
    0x0E: {
        'description': '位置指令输入过大',
        'handle': ['是否未使能', '是否位置设置值太大']
    },
    0x0F: {
        'description': '温度过高报警',
        'handle': ['检查温度传感器类型和设置值(Fn830)是否一致', '检查温度报警设置值是否过小', '是否长时间运行温度过高']
    },
    0x10: {
        'description': '上电时两次读取编码器位置相差超过设定值',
        'handle': ['检查MU和PVL参数', '重新上电，多次上电无效，驱动板故障']
    },
    0x11: {
        'description': 'iC-MU故障',
        'handle': ['根据故障码(EP1命令)处理']
    },
    0x12: {
        'description': 'iC-PVL故障',
        'handle': ['检查电池是否正常', '复位iC-PVL，重新上电', 'iC-PVL故障']
    },
    0x13: {
        'description': '电池电压过低警告',
        'handle': ['更换电池']
    },
    0x14: {
        'description': 'DRV8323硬件报警',
        'handle': ['是否堵转', '检查电机相序', '减小调节器增益']
    },
    0x15: {
        'description': 'DRV8323通讯错误',
        'handle': ['检查DRV8323']
    },
    0x16: {
        'description': 'iC-MU、iC-PVL配置参数错误',
        'handle': ['写入配置，重新校准iC-MU和iC-PVL', '重新上电']
    },
    0x17: {
        'description': '位置命令值和电机反馈值相差过大',
        'handle': ['是否堵转', '适当增大调节器增益', '减小位置命令输入值']
    },
    0x23: {
        'description': '电机类型错误',
        'handle': ['重新设置电机类型']
    },
    0x24: {
        'description': '驱动器类型错误',
        'handle': ['重新设置驱动器类型']
    },
    0x31: {
        'description': 'EEPROM读写错误',
        'handle': ['是否频繁写入EEPROM', '多次上电无效，EEPROM故障']
    },
    0x34: {
        'description': '电角度初始化失败',
        'handle': ['重新上电，多次上电无效，寻找支持']
    },
}

ControllerErrorCodeMap = {
    10: {
        'description': '伺服电机报错(Servo motor error)'
    },
    11: {
        'description': '伺服电机1报错(Servo motor 1 error)'
    },
    12: {
        'description': '伺服电机2报错(Servo motor 2 error)'
    },
    13: {
        'description': '伺服电机3报错(Servo motor 3 error)'
    },
    14: {
        'description': '伺服电机4报错(Servo motor 4 error)'
    },
    15: {
        'description': '伺服电机5报错(Servo motor 5 error)'
    },
    16: {
        'description': '伺服电机6报错(Servo motor 6 error)'
    },
    17: {
        'description': '伺服电机7报错(Servo motor 7 error)'
    },
    21: {
        'description': '逆解错误(Inverse kinematics)',
    },
    22: {
        'description': '机械臂碰撞限位(collision limit)',
    },
    23: {
        'description': '机械臂角度限位(angle limit)',
    },
    24: {
        'description': '机械臂关节输出速度限位(angle speed limit)',
    },
    25: {
        'description': '速度规划错误(planning error)',
    },
    26: {
        'description': 'linux-rt系统错误(Rtlinux set timing error)',
    },
    27: {
        'description': '指令回复错误(Reply command failed)',
    },
    28: {
        'description': '爪子错误(gripper error)',
    },
    29: {
        'description': '其它错误(other error)',
    }
}

ControllerWarnCodeMap = {
    11: {
        'description': '当前控制器缓存已满(Uxbus que is full)',
    },
    12: {
        'description': '用户指令参数错误(Parameter error)',
    },
    13: {
        'description': '用户指令控制码不存在(The instruction does not exist)',
    },
    14: {
        'description': '用户指令和参数无解(command has no solution)',
    }
}


class BaseCode(object):
    def __init__(self, code):
        self._code = code
        self._description = None

    @property
    def code(self):
        return self._code

    @property
    def description(self):
        if self._description is None:
            self._description = self._code_map.get(self.code, {}).get('description', None)
        return self._description

    @description.setter
    def description(self, desc):
        self._description = desc

    @property
    def handle(self):
        return self._code_map.get(self.code, {}).get('handle', None)


class ControllerError(BaseCode):
    def __init__(self, code):
        self._code_map = ControllerErrorCodeMap
        super(ControllerError, self).__init__(code)


class ControllerWarn(BaseCode):
    def __init__(self, code):
        self._code_map = ControllerWarnCodeMap
        super(ControllerWarn, self).__init__(code)


class ServoError(BaseCode):
    def __init__(self, code):
        self._code_map = ServoCodeMap
        super(ServoError, self).__init__(code)

