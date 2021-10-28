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
    1: {
        'en': {
            'title': 'The Emergency Stop Button on the xArm Controller is pushed in to stop',
            'desc': 'Please release the Emergency Stop Button, and then re-enable the robot'
        },
        'cn': {
            'title': '控制器上的紧急停止按钮被按下',
            'desc': '请释放紧急停止按钮，然后重新使能机械臂'
        }
    },
    2: {
        'en': {
            'title': 'The Emergency IO of the Control Box is triggered',
            'desc': 'Please ground the 2 EIs of the Control Box, and then re-enable the robot'
        },
        'cn': {
            'title': '控制器上的紧急停止IO被触发',
            'desc': '请将控制器的2组EI接地，然后重新使能机械臂'
        }
    },
    3: {
        'en': {
            'title': 'The Emergency Stop Button of the Three-state Switch is pressed',
            'desc': 'Please release the Emergency Stop Button of the Three-state Switch, and then re-enable the robot'
        },
        'cn': {
            'title': '三态开关的紧急停止按钮被按下',
            'desc': '请释放三态开关的紧急停止按钮，然后重新使能机械臂'
        }
    },
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
            'desc': 'Please check whether gripper is installed and the baud rate setting is correct'
        },
        'cn': {
            'title': '机械爪通信失败',
            'desc': '请检查机械爪是否安装，波特率设置是否正确'
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
            'title': 'Self-Collision Error',
            'desc': 'The robot is about to collide with itself. Please re-plan the path. If the robot reports the self-collision error continually, please turn on the manual mode and drag the robotic back to the normal area.'
        },
        'cn': {
            'title': '自碰撞错误',
            'desc': '机械臂即将发生自碰撞，请重新规划路径。如果机械臂持续报自碰撞错误，请开启手动模式将机械臂拖回正常位置。'
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
            'desc': 'Please restart the xArm with the Emergency Stop Button on the Control Box. If multiple reboots are not working, please contact technical support.'
        },
        'cn': {
            'title': '末端通信失败',
            'desc': '请通过控制器上的紧急停止按钮重启机械臂。如多次重启无效，请联系技术支持。'
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
    36: {
        'en': {
            'title': 'The number of delay commands exceeds the limit',
            'desc': 'The number of delay commands or position detection commands to be executed cannot exceed 36, please check whether there are too many delay commands or position detection commands in the code.'
        },
        'cn': {
            'title': '延时指令数量超限',
            'desc': '待执行的延时指令或位置检测指令超过36个，请检查代码中延时指令或位置检测指令是否过多。'
        }
    },
    37: {
        'en': {
            'title': 'Abnormal movement in Manual Mode',
            'desc': 'Please check whether the TCP payload setting and mounting setting of the robot arm are correct.'
        },
        'cn': {
            'title': '手动模式运动异常',
            'desc': '请检查机械臂的TCP负载设置和机械臂安装方式是否与实际匹配。'
        }
    },
    38: {
        'en': {
            'title': 'Abnormal Joint Angle',
            'desc': 'Please stop the xArm by pressing the Emergency Stop Button on the Control Box and then contact technical support.'
        },
        'cn': {
            'title': '关节角度异常',
            'desc': '请通过控制器上的紧急停止按钮停止机械臂，并联系技术支持。'
        }
    },
    39: {
        'en': {
            'title': 'Abnormal Communication Between Master and Slave IC of Power Board',
            'desc': 'Please contact technical support'
        },
        'cn': {
            'title': '电源板主从IC通信异常',
            'desc': '请联系技术支持。'
        }
    },
    50: {
        'en': {
            'title': 'Six-axis Force Torque Sensor read error',
            'desc': ''
        },
        'cn': {
            'title': '六维力矩传感器读取数据错误',
            'desc': ''
        }
    },
    51: {
        'en': {
            'title': 'Six-axis Force Torque Sensor set mode error',
            'desc': ''
        },
        'cn': {
            'title': '六维力矩传感器设置模式错误',
            'desc': ''
        }
    },
    52: {
        'en': {
            'title': 'Six-axis Force Torque Sensor set zero error',
            'desc': ''
        },
        'cn': {
            'title': '六维力矩传感器设置零点错误',
            'desc': ''
        }
    },
    53: {
        'en': {
            'title': 'Six-axis Force Torque Sensor is overloaded or the reading exceeds the limit',
            'desc': ''
        },
        'cn': {
            'title': '六维力矩传感器过载或读数超限',
            'desc': ''
        }
    },
    110: {
        'en': {
            'title': 'Robot Arm Base Board Communication Error',
            'desc': 'Please contact technical support.'
        },
        'cn': {
            'title': '机械臂底座板通信异常',
            'desc': '请联系技术支持。'
        }
    },
    111: {
        'en': {
            'title': 'Control Box External 485 Device Communication Error',
            'desc': 'Please contact technical support.'
        },
        'cn': {
            'title': '控制器外接485设备通信异常',
            'desc': '请联系技术支持。'
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
    15: {
        'en': {
            'title': 'Modbus cmd full',
            'desc': ''
        },
        'cn': {
            'title': 'Modbus指令已满',
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


RobotiqErrorCodeMap = {
    0x05: {
        'en': {
            'title': 'Robotiq Gripper',
            'desc': 'Action delayed, activation(reactivation) must be completed prior to perfmoring the action'
        },
        'cn': {
            'title': 'Robotiq 机械爪',
            'desc': '运动延迟, 机械爪运动之前必须先完成激活（重新激活）'
        }
    },
    0x07: {
        'en': {
            'title': 'Robotiq Gripper',
            'desc': 'The activation bit must be set prior to action'
        },
        'cn': {
            'title': 'Robotiq 机械爪',
            'desc': '激活位必须在机械爪运动前设置'
        }
    },
    0x08: {
        'en': {
            'title': 'Robotiq Gripper',
            'desc': 'Maximum operating temperature exceeded, wait for cool-down.'
        },
        'cn': {
            'title': 'Robotiq 机械爪',
            'desc': '超过最高工作温度，请等待机械爪冷却'
        }
    },
    0x09: {
        'en': {
            'title': 'Robotiq Gripper',
            'desc': 'No communication during at least 1 second'
        },
        'cn': {
            'title': 'Robotiq 机械爪',
            'desc': '通信中断超过1秒'
        }
    },
    0x0A: {
        'en': {
            'title': 'Robotiq Gripper',
            'desc': 'Under minimum operating voltage'
        },
        'cn': {
            'title': 'Robotiq 机械爪',
            'desc': '低于最小工作电压'
        }
    },
    0x0B: {
        'en': {
            'title': 'Robotiq Gripper',
            'desc': 'Automatic release in progress'
        },
        'cn': {
            'title': 'Robotiq 机械爪',
            'desc': '正在自动释放'
        }
    },
    0x0C: {
        'en': {
            'title': 'Robotiq Gripper',
            'desc': 'Internal fault, please contact support@robotiq.com'
        },
        'cn': {
            'title': 'Robotiq 机械爪',
            'desc': '内部故障，请联系技术支持 support@robotiq.com'
        }
    },
    0x0D: {
        'en': {
            'title': 'Robotiq Gripper',
            'desc': 'Activation fault, please verify that no interference or other erroro ccurred'
        },
        'cn': {
            'title': 'Robotiq 机械爪',
            'desc': '激活故障，请确认没有干扰或其他错误发生'
        }
    },
    0x0E: {
        'en': {
            'title': 'Robotiq Gripper',
            'desc': 'Over current triggered'
        },
        'cn': {
            'title': 'Robotiq 机械爪',
            'desc': '过流'
        }
    },
    0x0F: {
        'en': {
            'title': 'Robotiq Gripper',
            'desc': 'Automatic release completed'
        },
        'cn': {
            'title': 'Robotiq 机械爪',
            'desc': '自动松开完成'
        }
    },
    'other': {
        'en': {
            'title': 'Robotiq Gripper',
            'desc': 'Other fault'
        },
        'cn': {
            'title': 'Robotiq 机械爪',
            'desc': '其它故障'
        }
    },
}


BioGripperErrorCodeMap = {
    0x0B: {
        'en': {
            'title': 'BIO Gripper Current Overlimit',
            'desc': 'Current Overlimit, please click “OK” to re-enable the Gripper. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': 'BIO 机械爪过流',
            'desc': '电流过大，请点击“确认”重新使能机械爪。如反复报错，请联系技术支持。',
        }
    },
    0x0C: {
        'en': {
            'title': 'The object slipped from the BIO Gripper',
            'desc': 'The object slipped from the BIO Gripper, please clear the error and try again',
        },
        'cn': {
            'title': 'BIO 机械爪夹取的物体脱落',
            'desc': 'BIO 机械爪夹取的物体脱落，请清除错误后重试',
        }
    },
    'other': {
        'en': {
            'title': 'BIO Gripper',
            'desc': 'Other fault'
        },
        'cn': {
            'title': 'BIO 机械爪',
            'desc': '其它故障'
        }
    },
}

LinearTrackErrorCodeMap = {
    10: {
        'en': {
            'title': 'Linear Motor Current Detection Error',
            'desc': 'Please restart the Controller. If multiple reboots are not working, please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨电流检测异常',
            'desc': '请重启控制器。如多次重启无效，请联系技术支持。',
        }
      },
    11: {
        'en': {
            'title': 'Linear Motor Current Overlimit',
            'desc': 'Please clear the Linear Motor error. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨电流过大',
            'desc': '请清除直线滑轨报错。如反复报错，请联系技术支持。',
        }
    },
    12: {
        'en': {
            'title': 'Linear Motor Speed Overlimit',
            'desc': 'Please clear the Linear Motor error. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨速度过大',
            'desc': '请清除直线滑轨报错。如反复报错，请联系技术支持。',
        }
    },
    13: {
        'en': {
            'title': 'Linear Motor Large Motor Position Deviation',
            'desc': 'Please check if the movement of the Linear Motor is blocked, if not, please clear the Linear Motor error. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨电机位置偏差过大',
            'desc': '请检查直线滑轨运动是否受阻，如直线滑轨运动未受阻，请清除直线滑轨报错。如反复报错，请联系技术支持。',
        }
    },
    14: {
        'en': {
            'title': 'Linear Motor Position Command Overlimit',
            'desc': 'Please clear the Linear Motor error. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨位置指令过大',
            'desc': '请清除直线滑轨报错。如反复报错，请联系技术支持。',
        }
    },
    20: {
        'en': {
            'title': 'Linear Motor Driver IC Hardware Error',
            'desc': 'Please clear the Linear Motor error. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨驱动IC硬件异常',
            'desc': '请清除直线滑轨报错。如反复报错，请联系技术支持。',
        }
    },
    21: {
        'en': {
            'title': 'Linear Motor Driver IC Initialization Error',
            'desc': 'Please clear the Linear Motor error. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨驱动IC初始化异常',
            'desc': '请清除直线滑轨报错。如反复报错，请联系技术支持。',
        }
    },
    25: {
        'en': {
            'title': 'Linear Motor Command Over Software Limit',
            'desc': 'Please check if the Linear Motor command is set beyond the software limit. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨指令超软件限位',
            'desc': '请检测直线滑轨指令是否设置超出软件限制。如频繁出现，请联系技术支持。',
        }
    },
    26: {
        'en': {
            'title': 'Linear Motor Feedback Position Software Limit',
            'desc': 'Please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨反馈位置超限软件限位',
            'desc': '请联系技术支持。',
        }
    },
    33: {
        'en': {
            'title': 'Linear Motor Drive Overloaded',
            'desc': 'Please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨驱动器过载',
            'desc': '请联系技术支持。',
        }
    },
    34: {
        'en': {
            'title': 'Linear Motor Motor Overload',
            'desc': 'Please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨电机过载',
            'desc': '请联系技术支持。',
        }
    },
    35: {
        'en': {
            'title': 'Linear Motor type error',
            'desc': 'Please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨电机类型错误',
            'desc': '请联系技术支持。',
        }
    },
    36: {
        'en': {
            'title': 'Linear Motor Driver Type Error',
            'desc': 'Please clear the Linear Motor error. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨驱动器类型错误',
            'desc': '请清除直线滑轨报错。如反复报错，请联系技术支持。',
        }
    },
    39: {
        'en': {
            'title': 'Linear Motor over voltage',
            'desc': 'please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨过压',
            'desc': '请联系技术支持。',
        }
    },
    40: {
        'en': {
            'title': 'Linear Moter undervoltage',
            'desc': 'please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨欠压',
            'desc': '请联系技术支持。',
        }
    },
    49: {
        'en': {
            'title': 'Linear Motor EEPROM Read and Write Error',
            'desc': 'Please clear the Linear Motor error. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨EEPROM读写错误',
            'desc': '请清除直线滑轨报错。如反复报错，请联系技术支持。',
        }
    },
    'other': {
        'en': {
            'title': 'Linear Motor Error',
            'desc': 'Please clear the Linear Motor error. If it reports the same error repeatedly, please contact technical support.',
        },
        'cn': {
            'title': '直线滑轨异常',
            'desc': '请清除直线滑轨报错。如反复报错，请联系技术支持。',
        }
    },
}

FtSensorErrorCodeMap = {
    64: {
        'en': {
            'title': 'Six-axis Force Torque Sensor Communication Failure',
            'desc': '',
        },
        'cn': {
            'title': '力矩出现通讯中断',
            'desc': '',
        }
    },
    65: {
        'en': {
            'title': 'The Data Collected by the Six-axis Force Torque Sensor is Abnormal',
            'desc': '',
        },
        'cn': {
            'title': '力矩采集数据不变化',
            'desc': '',
        }
    },
    66: {
        'en': {
            'title': 'Six-axis Force Torque Sensor X-direction Torque Exceeds Limit',
            'desc': '',
        },
        'cn': {
            'title': '六维力矩传感器的Fx超限',
            'desc': '',
        }
    },
    67: {
        'en': {
            'title': 'Six-axis Force Torque Sensor Y-direction Torque Exceeds Limit',
            'desc': '',
        },
        'cn': {
            'title': '六维力矩传感器的Fy超限',
            'desc': '',
        }
    },
    68: {
        'en': {
            'title': 'Six-axis Force Torque Sensor Z-direction Torque Exceeds Limitrection',
            'desc': '',
        },
        'cn': {
            'title': '六维力矩传感器的Fz超限',
            'desc': '',
        }
    },
    69: {
        'en': {
            'title': 'Six-axis Force Torque Sensor Tx Torque Exceeds Limit',
            'desc': '',
        },
        'cn': {
            'title': '六维力矩传感器的Tx超限',
            'desc': '',
        }
    },
    70: {
        'en': {
            'title': 'Six-axis Force Torque Sensor Ty direction Torque Exceeds Limit',
            'desc': '',
        },
        'cn': {
            'title': '六维力矩传感器的Ty超限',
            'desc': '',
        }
    },
    71: {
        'en': {
            'title': 'Six-axis Force Torque Sensor Tz direction Torque Exceeds Limit',
            'desc': '',
        },
        'cn': {
            'title': '六维力矩传感器的Tz超限',
            'desc': '',
        }
    },
    73: {
        'en': {
            'title': 'Six-axis Force Torque Sensor Failed to Initialize',
            'desc': '',
        },
        'cn': {
            'title': '六维力矩传感器初始化不成功',
            'desc': '',
        }
    },
    'other': {
        'en': {
            'title': 'Six-axis Force Torque Sensor Error',
            'desc': '',
        },
        'cn': {
            'title': '六维力矩传感器异常',
            'desc': '',
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


class BioGripperError(BaseCode):
    def __init__(self, code, status=0):
        self._code_map = BioGripperErrorCodeMap
        super(BioGripperError, self).__init__(code, status=status)


class RobotIqError(BaseCode):
    def __init__(self, code, status=0):
        self._code_map = RobotiqErrorCodeMap
        super(RobotIqError, self).__init__(code, status=status)


class LinearTrackError(BaseCode):
    def __init__(self, code, status=0):
        self._code_map = LinearTrackErrorCodeMap
        super(LinearTrackError, self).__init__(code, status=status)


class FtSensorError(BaseCode):
    def __init__(self, code, status=0):
        self._code_map = FtSensorErrorCodeMap
        super(FtSensorError, self).__init__(code, status=status)
