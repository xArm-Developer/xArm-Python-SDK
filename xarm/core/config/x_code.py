#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


ServoCodeMap = {
    10: {
        'en': {
            'title': 'Current Detection Error',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '电流检测异常',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。',
        }
    },
    11: {
        'en': {
            'title': 'Joint Current Overlimit',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '关节电流过大',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。',
        }
    },
    12: {
        'en': {
            'title': 'Joint Speed Overlimit',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '关节速度过大',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。',
        }
    },
    14: {
        'en': {
            'title': 'Position Command Overlimit',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '位置指令过大',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。',
        }
    },
    15: {
        'en': {
            'title': 'Joints Overheat',
            'desc': 'If the robot arm is running for a long time, please stop running and restart the xArm after it\'s cool down. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '关节过热',
            'desc': '如果机械臂长时间运行温度过高，请停并机冷却后重启机械臂。如多次重启无效，请联系技术支持。',
        }
    },
    16: {
        'en': {
            'title': 'Encoder Initialization Error',
            'desc': 'Please ensure that there is no external force to push the robotic arm when the  it\'s energized. Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '编码器初始化异常',
            'desc': '请确保机械臂通电时，无外力推动机械臂运动。请通过控制器上的紧急停止按钮重启机械臂，如多次重启无效，请联系技术支持。',
        }
    },
    17: {
        'en': {
            'title': 'Single Ring Encoder Error',
            'desc': 'Please re-enable the robot.',
        },
        'cn': {
            'title': '单圈编码器故障',
            'desc': '请重新使能机械臂。',
        }
    },
    18: {
        'en': {
            'title': 'Multi-turn Encoder Error ',
            'desc': 'Please contact technical support.',
        },
        'cn': {
            'title': '多圈编码器故障',
            'desc': '请联系技术支持。',
        }
    },
    19: {
        'en': {
            'title': 'Low Battery Voltage',
            'desc': 'Please contact technical support.',
        },
        'cn': {
            'title': '电池电压过低',
            'desc': '请联系技术支持。',
        }
    },
    20: {
        'en': {
            'title': 'Driver IC Hardware Error',
            'desc': 'Please re-enable the robot. If it appears frequently, please contact technical support.',
        },
        'cn': {
            'title': '驱动IC硬件异常',
            'desc': '请重新使能机械臂。如频繁出现，请联系技术支持。',
        }
    },
    21: {
        'en': {
            'title': 'Driver IC Initialization Error',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.If multiple reboots are invalid, please contact technical support.',
        },
        'cn': {
            'title': '驱动IC初始化异常',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂，如多次重启无效，请联系技术支持。',
        }
    },
    22: {
        'en': {
            'title': 'Encoder Configuration Error',
            'desc': 'Please contact technical support.',
        },
        'cn': {
            'title': '编码器配置错误',
            'desc': '请联系技术支持。',
        }
    },
    23: {
        'en': {
            'title': 'Large Motor Position Deviation',
            'desc': 'Please check whether the xArm movement is blocked, whether the payload exceeds the rated payload of xArm, and whether the acceleration value is too large. If it appears frequently, please contact technical support.',
        },
        'cn': {
            'title': '电机位置偏差过大',
            'desc': '请检查机械臂运动是否受阻，末端负载是否超过机械臂额定负载，机械臂加速度值是否设置过大。如频繁出现，请联系技术支持。',
        }
    },
    26: {
        'en': {
            'title': 'Joint N Positive Overrun',
            'desc': 'Please check if angle value of the joint N is too large.',
        },
        'cn': {
            'title': '第N关节正向超限',
            'desc': '请检测N关节角度值是否设置过大。',
        }
    },
    27: {
        'en': {
            'title': 'Joint N Negative Overrun',
            'desc': 'Please check if the angle value of  joint N is too large, if so, please click Clear Error and manually unlock the joint and rotate the joint to the allowed range of motion.',
        },
        'cn': {
            'title': '第N关节负向超限',
            'desc': '请检测第N关节角度值是否设置过大，如果是，请点击清除报错后，手动解锁该关节并转动该关节至其运动范围内。',
        }
    },
    28: {
        'en': {
            'title': 'Joint Commands Error',
            'desc': 'The xArm is not enabled, please click Enable Robot.',
        },
        'cn': {
            'title': '关节指令错误',
            'desc': '机械臂未使能,请点击“使能机械臂”。',
        }
    },
    33: {
        'en': {
            'title': 'Drive Overloaded',
            'desc': 'Please make sure the payload is within the rated load.',
        },
        'cn': {
            'title': '驱动器过载',
            'desc': '请确保机械臂负载处于额定负载内。',
        }
    },
    34: {
        'en': {
            'title': 'Motor Overload',
            'desc': 'Please make sure the payload is within the rated load.',
        },
        'cn': {
            'title': '电机过载',
            'desc': '请确保机械臂负载处于额定负载内。',
        }
    },
    35: {
        'en': {
            'title': 'Motor Type Error',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '电机类型错误',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。',
        }
    },
    36: {
        'en': {
            'title': 'Driver Type Error',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '驱动器类型错误',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。',
        }
    },
    39: {
        'en': {
            'title': 'Joint Voltage Overload',
            'desc': 'Please reduce the acceleration value in the Motion Settings.',
        },
        'cn': {
            'title': '关节过压',
            'desc': '请在运动设置中减少加速度值。',
        }
    },
    40: {
        'en': {
            'title': 'Joint Voltage Insufficient',
            'desc': 'Please reduce the acceleration value in the Motion Settings.Please check if the controller emergency stop switch is released.',
        },
        'cn': {
            'title': '关节欠压',
            'desc': '请在运动设置中减少加速度值。请检查控制器紧急停止开关是否松开。',
        }
    },
    49: {
        'en': {
            'title': 'EEPROM Read and Write Error.',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': 'EEPROM读写错误',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。',
        }
    },
    52: {
        'en': {
            'title': 'Motor Angle Initialization Error',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '电机角度初始化失败',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。',
        }
    },
    'other': {
        'en': {
            'title': 'Joint Error',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '关节异常',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。',
        }
    },
    'failed': {
        'en': {
            'title': 'Joint Communication failure',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '关节通信失败',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。',
        }
    }
}

GripperErrorCodeMap = {
    9: {
        'en': {
            'title': 'Gripper Current Detection Error',
            'desc': 'Please restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '机械爪电流检测异常',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。',
        }
      },
    11: {
        'en': {
            'title': 'Gripper Current Overlimit',
            'desc': 'Please click “OK” to re-enable the Gripper. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '机械爪电流过大',
            'desc': '请点击“确认”重新使能机械爪。如反复报错，请联系技术支持。',
        }
    },
    12: {
        'en': {
            'title': 'Gripper Speed Overlimit',
            'desc': 'Please click “OK” to re-enable the Gripper. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '机械爪速度过大',
            'desc': '请点击“确认”重新使能机械爪。如反复报错，请联系技术支持。',
        }
    },
    14: {
        'en': {
            'title': 'Gripper Position Command Overlimit',
            'desc': 'Please click “OK” to re-enable the Gripper. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '机械爪位置指令过大',
            'desc': '请点击“确认”重新使能机械爪。如反复报错，请联系技术支持。',
        }
    },
    15: {
        'en': {
            'title': 'Gripper EEPROM Read and Write Error',
            'desc': 'Please click “OK” to re-enable the Gripper. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '机械爪EEPROM读写错误',
            'desc': '请点击“确认”重新使能机械爪。如反复报错，请联系技术支持。',
        }
    },
    20: {
        'en': {
            'title': 'Gripper Driver IC Hardware Error',
            'desc': 'Please click “OK” to re-enable the Gripper. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '机械爪驱动IC硬件异常',
            'desc': '请点击“确认”重新使能机械爪。如反复报错，请联系技术支持。',
        }
    },
    21: {
        'en': {
            'title': 'Gripper Driver IC Initialization Error',
            'desc': 'Please click “OK” to re-enable the Gripper. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '机械爪驱动IC初始化异常',
            'desc': '请点击“确认”重新使能机械爪。如反复报错，请联系技术支持。',
        }
    },
    23: {
        'en': {
            'title': 'Gripper Large Motor Position Deviation',
            'desc': 'Please check if the movement of the Gripper is blocked, if not, please click “OK” to re-enable the Gripper. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '机械爪电机位置偏差过大',
            'desc': '请检查机械爪运动是否受阻，如机械爪运动未受阻，请点击“确认”重新使能机械爪。如频繁出现，请联系技术支持。',
        }
    },
    25: {
        'en': {
            'title': 'Gripper Command Over Software Limit',
            'desc': 'Please check if the gripper command is set beyond the software limit. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '机械爪指令超软件限位',
            'desc': '请检测机械爪指令是否设置超出软件限制。如频繁出现，请联系技术支持。',
        }
    },
    26: {
        'en': {
            'title': 'Gripper Feedback Position Software Limit',
            'desc': 'Please contact technical support.',
        },
        'cn': {
            'title': '机械爪反馈位置超限软件限位',
            'desc': '请联系技术支持。',
        }
    },
    33: {
        'en': {
            'title': 'Gripper Drive Overloaded',
            'desc': 'Please contact technical support.',
        },
        'cn': {
            'title': '机械爪驱动器过载',
            'desc': '请联系技术支持。',
        }
    },
    34: {
        'en': {
            'title': 'Gripper Motor Overload',
            'desc': 'Please contact technical support.',
        },
        'cn': {
            'title': '机械爪电机过载',
            'desc': '请联系技术支持。',
        }
    },
    36: {
        'en': {
            'title': 'Gripper Driver Type Error',
            'desc': 'Please click “OK” to re-enable the Gripper. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '机械爪驱动器类型错误',
            'desc': '请点击“确认”重新使能机械爪。如频繁出现，请联系技术支持。',
        }
    },
    'other': {
        'en': {
            'title': 'Gripper Error',
            'desc': 'Please click “OK” to re-enable the Gripper. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '机械爪异常',
            'desc': '请点击“确认”重新使能机械爪。如频繁出现，请联系技术支持。',
        }
    },
    'failed': {
        'en': {
            'title': 'Gripper Communication failure',
            'desc': 'Please confirm that the mechanical grip is properly installed, or cancel the installation of the mechanical claws on the software.',
        },
        'cn': {
            'title': '机械爪通信失败',
            'desc': '请确认机械爪正确安装，或在软件上取消机械爪的安装',
        }
    }
}

ControllerErrorCodeMap = {
    10: {
        'en': {
            'title': 'Servo motor error',
            'desc': ''
        },
        'cn': {
            'title': '关节错误',
            'desc': ''
        }
    },
    11: {
        'en': {
            'title': 'Servo motor 1 error',
            'desc': ''
        },
        'cn': {
            'title': '关节1错误',
            'desc': ''
        }
    },
    12: {
        'en': {
            'title': 'Servo motor 2 error',
            'desc': ''
        },
        'cn': {
            'title': '关节2错误',
            'desc': ''
        }
    },
    13: {
        'en': {
            'title': 'Servo motor 3 error',
            'desc': ''
        },
        'cn': {
            'title': '关节3错误',
            'desc': ''
        }
    },
    14: {
        'en': {
            'title': 'Servo motor 4 error',
            'desc': ''
        },
        'cn': {
            'title': '关节4错误',
            'desc': ''
        }
    },
    15: {
        'en': {
            'title': 'Servo motor 5 error',
            'desc': ''
        },
        'cn': {
            'title': '关节5错误',
            'desc': ''
        }
    },
    16: {
        'en': {
            'title': 'Servo motor 6 error',
            'desc': ''
        },
        'cn': {
            'title': '关节6错误',
            'desc': ''
        }
    },
    17: {
        'en': {
            'title': 'Servo motor 7 error',
            'desc': ''
        },
        'cn': {
            'title': '关节7错误',
            'desc': ''
        }
    },
    19: {
        'en': {
            'title': 'Gripper Communication Error',
            'desc': ''
        },
        'cn': {
            'title': '机械爪通信失败',
            'desc': ''
        }
    },
    21: {
        'en': {
            'title': 'Kinematic Error',
            'desc': 'Please re-plan the path.'
        },
        'cn': {
            'title': '运动学错误',
            'desc': '请重新规划路径。'
        }
    },
    22: {
        'en': {
            'title': 'Collision Error',
            'desc': 'Please click the "ZERO" button to return to the zero pozition.'
        },
        'cn': {
            'title': '自碰撞错误',
            'desc': '请点击”零点“按钮回到关节零点。'
        }
    },
    23: {
        'en': {
            'title': 'Joints Angle Exceed Limit',
            'desc': 'Please click the "ZERO" button to return to the zero pozition.'
        },
        'cn': {
            'title': '关节角度超出限制',
            'desc': '请点击”零点“按钮回到关节零点。'
        }
    },
    24: {
        'en': {
            'title': 'Speed Exceeds Limit',
            'desc': 'Please check if the xArm is out of working range, or reduce the speed and acceleration values.'
        },
        'cn': {
            'title': '速度超出限制',
            'desc': '请检查机械臂是否超出运动范围，或减小运动速度和加速度值。'
        }
    },
    25: {
        'en': {
            'title': 'Planning Error',
            'desc': 'Please re-plan the path or reduce the speed.'
        },
        'cn': {
            'title': '规划错误',
            'desc': '请重新规划路径或者减小运动速度。'
        }
    },
    26: {
        'en': {
            'title': 'Linux RT Error',
            'desc': 'Please contact technical support.'
        },
        'cn': {
            'title': 'Linux RT 错误',
            'desc': '请联系技术支持。'
        }
    },
    27: {
        'en': {
            'title': 'Command Reply Error',
            'desc': 'Pleas retry, or restart the xArm with the Emergency Stop Button on the xArm Controller. If multiple reboots are not working, please contact technical support.'
        },
        'cn': {
            'title': '回复指令错误 ',
            'desc': '请重试，或通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。'
        }
    },
    28: {
        'en': {
            'title': 'End Module Communication Error',
            'desc': ''
        },
        'cn': {
            'title': '末端通信失败',
            'desc': ''
        }
    },
    30: {
        'en': {
            'title': 'Feedback Speed Exceeds limit',
            'desc': 'Please contact technical support.'
        },
        'cn': {
            'title': '反馈速度超出限制',
            'desc': '请联系技术支持。'
        }
    },
    31: {
        'en': {
            'title': 'Collision Caused Abnormal Current',
            'desc': 'Please check for collisions, check that the payload settings are correct, and that the collision sensitivity matches the speed.'
        },
        'cn': {
            'title': '碰撞导致电流异常',
            'desc': '请检查是否碰撞、负载设置是否正确，碰撞灵敏度与速度是否匹配。'
        }
    },
    32: {
        'en': {
            'title': 'Three-point drawing circle calculation error',
            'desc': 'Three-point drawing circle calculation error, please reset the arc command.'
        },
        'cn': {
            'title': '三点圆弧指令计算出错',
            'desc': '三点圆弧指令计算出错，请重新设置圆弧指令。'
        }
    },
    33: {
        'en': {
            'title': 'Controller GPIO Error',
            'desc': 'Please check the connection of the controller GPIO module and power on and off again. If the error occurs repeatedly, please contact technical support.'
        },
        'cn': {
            'title': '控制器GPIO模块报错',
            'desc': '请检查控制器GPIO模块的连接，并重新上下电。如该错误反复出现，请联系技术支持。'
        }
    },
    34: {
        'en': {
            'title': 'Recording Timeout',
            'desc': 'The track recording duration exceeds the maximum duration limit of 5 minutes. It is recommended to re-record.'
        },
        'cn': {
            'title': '轨迹录制超时',
            'desc': '轨迹录制时间超过最大限制5分钟，建议重新录制。'
        }
    },
    35: {
        'en': {
            'title': 'Safety Boundary Limit',
            'desc': 'The xArm reaches the safety boundary. Please move the xArm to the safety boundary after turning on the Manual mode on the Live Control interface.'
        },
        'cn': {
            'title': '机械臂到达安全边界',
            'desc': '机械臂到达安全边界，请到实时控制界面开启手动模式后将机械臂移动到安全边界内。'
        }
    },
    'other': {
        'en': {
            'title': 'Other Errors',
            'desc': ''
        },
        'cn': {
            'title': '其他错误',
            'desc': ''
        }
    },
}

ControllerWarnCodeMap = {
    11: {
        'en': {
            'title': 'Current controller cache is full',
            'desc': ''
        },
        'cn': {
            'title': '当前控制器缓存已满',
            'desc': ''
        }
    },
    12: {
        'en': {
            'title': 'User instruction parameter error',
            'desc': ''
        },
        'cn': {
            'title': '用户指令参数错误',
            'desc': ''
        }
    },
    13: {
        'en': {
            'title': 'User command control code does not exist',
            'desc': ''
        },
        'cn': {
            'title': '用户指令控制码不存在',
            'desc': ''
        }
    },
    14: {
        'en': {
            'title': 'User instructions and parameters have no solution',
            'desc': ''
        },
        'cn': {
            'title': '用户指令和参数无解',
            'desc': ''
        }
    },
    'other': {
        'en': {
            'title': 'Other Warnings',
            'desc': ''
        },
        'cn': {
            'title': '其它警告',
            'desc': ''
        }
    },
}


class BaseCode(object):
    def __init__(self, code, status=0):
        self._code = code
        self._status = status
        if status in [0, 1]:
            if code != 0:
                self.info = self._code_map.get(code, self._code_map.get('other'))
            else:
                self.info = {'en': {'title': 'Normal', 'desc': ''}, 'cn': {'title': '正常', 'desc': ''}}
        else:
            self.info = self._code_map.get('failed', self._code_map.get('other'))

    @property
    def status(self):
        return self._status

    @property
    def code(self):
        return self._code

    @property
    def title(self):
        return {
            'en': self.info['en']['title'],
            'cn': self.info['cn']['title']
        }

    @property
    def description(self):
        return {
            'en': self.info['en']['desc'],
            'cn': self.info['cn']['desc']
        }


class ControllerError(BaseCode):
    def __init__(self, code, status=0):
        self._code_map = ControllerErrorCodeMap
        super(ControllerError, self).__init__(code, status=status)


class ControllerWarn(BaseCode):
    def __init__(self, code, status=0):
        self._code_map = ControllerWarnCodeMap
        super(ControllerWarn, self).__init__(code, status=status)


class ServoError(BaseCode):
    def __init__(self, code, status=0):
        self._code_map = ServoCodeMap
        super(ServoError, self).__init__(code, status=status)


class GripperError(BaseCode):
    def __init__(self, code, status=0):
        self._code_map = GripperError
        super(GripperError, self).__init__(code, status=status)
