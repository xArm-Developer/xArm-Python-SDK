#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import re
import sys
import time
import math
import uuid
import queue
import struct
import threading
try:
    from multiprocessing.pool import ThreadPool
except:
    ThreadPool = None
try:
    import asyncio

    if sys.version_info.major >= 3 and sys.version_info.minor >= 5:
        from .grammar_async import AsyncObject as BaseObject
    else:
        from .grammar_coroutine import CoroutineObject as BaseObject
except:
    asyncio = None
if not hasattr(math, 'inf'):
    setattr(math, 'inf', float('inf'))
from .events import Events
from ..core.config.x_config import XCONF
from ..core.comm import SocketPort
try:
    from ..core.comm import SerialPort
except:
    SerialPort = None 
from ..core.wrapper import UxbusCmdSer, UxbusCmdTcp
from ..core.utils.log import logger, pretty_print
from ..core.utils import convert
from ..core.config.x_code import ControllerWarn, ControllerError, ControllerErrorCodeMap, ControllerWarnCodeMap
from .utils import compare_time, compare_version, filter_invaild_number
from .decorator import xarm_is_connected, xarm_is_ready, xarm_is_not_simulation_mode, xarm_wait_until_cmdnum_lt_max, xarm_wait_until_not_pause
from .code import APIState
from ..tools.threads import ThreadManage
from ..version import __version__

controller_error_keys = ControllerErrorCodeMap.keys()
controller_warn_keys = ControllerWarnCodeMap.keys()

print('SDK_VERSION: {}'.format(__version__))


class Base(BaseObject, Events):
    def __init__(self, port=None, is_radian=False, do_not_open=False, **kwargs):
        if kwargs.get('init', False):
            super(Base, self).__init__()
            self._port = port
            self._debug = kwargs.get('debug', False)
            self._baudrate = kwargs.get('baudrate', XCONF.SerialConf.SERIAL_BAUD)
            self._timeout = kwargs.get('timeout', None)
            self._filters = kwargs.get('filters', None)
            self._enable_heartbeat = kwargs.get('enable_heartbeat', False)
            self._enable_report = kwargs.get('enable_report', True)
            self._report_type = kwargs.get('report_type', 'rich')
            self._forbid_uds = kwargs.get('forbid_uds', False)

            self._check_tcp_limit = kwargs.get('check_tcp_limit', False)
            self._check_joint_limit = kwargs.get('check_joint_limit', True)
            self._check_cmdnum_limit = kwargs.get('check_cmdnum_limit', True)
            self._check_simulation_mode = kwargs.get('check_simulation_mode', True)
            self._max_cmd_num = kwargs.get('max_cmdnum', 512)
            if not isinstance(self._max_cmd_num, int):
                self._max_cmd_num = 512
            self._max_cmd_num = min(XCONF.MAX_CMD_NUM, self._max_cmd_num)
            self._check_robot_sn = kwargs.get('check_robot_sn', False)
            self._check_is_ready = kwargs.get('check_is_ready', True)
            self._check_is_pause = kwargs.get('check_is_pause', True)
            self._timed_comm = kwargs.get('timed_comm', True)
            self._timed_comm_interval = kwargs.get('timed_comm_interval', 30)
            self._timed_comm_t = None
            self._timed_comm_t_alive = False

            self._baud_checkset = kwargs.get('baud_checkset', True)
            self._default_bio_baud = kwargs.get('default_bio_baud', 2000000)
            self._default_gripper_baud = kwargs.get('default_gripper_baud', 2000000)
            self._default_robotiq_baud = kwargs.get('default_robotiq_baud', 115200)
            self._default_linear_track_baud = kwargs.get('default_linear_track_baud', 2000000)

            self._max_callback_thread_count = kwargs.get('max_callback_thread_count', 0)
            self._asyncio_loop = None
            self._asyncio_loop_alive = False
            self._asyncio_loop_thread = None
            self._pool = None
            self._thread_manage = ThreadManage()

            self._rewrite_modbus_baudrate_method = kwargs.get('rewrite_modbus_baudrate_method', True)

            self._min_tcp_speed, self._max_tcp_speed = 0.1, 1000  # mm/s
            self._min_tcp_acc, self._max_tcp_acc = 1.0, 50000  # mm/s^2
            self._tcp_jerk = 1000  # mm/s^3

            # self._min_joint_speed, self._max_joint_speed = 0.01, 4.0  # rad/s
            self._min_joint_speed, self._max_joint_speed = 0.0001, 4.0  # rad/s
            self._min_joint_acc, self._max_joint_acc = 0.01, 20.0  # rad/s^2
            self._joint_jerk = 20.0  # rad/s^3

            self._rot_jerk = 2.3
            self._max_rot_acc = 2.7

            self._stream_type = 'serial'
            self._stream = None
            self.arm_cmd = None
            self._stream_503 = None # 透传使用
            self.arm_cmd_503 = None # 透传使用
            self._stream_report = None
            self._report_thread = None
            self._only_report_err_warn_changed = True

            self._last_position = [201.5, 0, 140.5, 3.1415926, 0, 0]  # [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]
            self._last_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [servo_1(rad), servo_2(rad), servo_3(rad), servo_4(rad), servo_5(rad), servo_6(rad), servo_7(rad)]
            self._last_tcp_speed = 100  # mm/s, rad/s
            self._last_tcp_acc = 2000  # mm/s^2, rad/s^2
            self._last_joint_speed = 0.3490658503988659  # 20 °/s
            self._last_joint_acc = 8.726646259971648  # 500 °/s^2
            self._mvtime = 0

            self._version = None
            self._robot_sn = None
            self._control_box_sn = None
            self._position = [201.5, 0, 140.5, 3.1415926, 0, 0]
            self._pose_aa = [201.5, 0, 140.5, 3.1415926, 0, 0]
            self._angles = [0] * 7
            self._position_offset = [0] * 6
            self._world_offset = [0] * 6
            self._state = 4
            self._mode = 0
            self._joints_torque = [0, 0, 0, 0, 0, 0, 0]  # 力矩
            self._tcp_load = [0, [0, 0, 0]]  # 负载[重量, 重心], [weight, [x, y, z]]
            self._collision_sensitivity = 0  # 碰撞灵敏度
            self._teach_sensitivity = 0  # 示教灵敏度
            self._error_code = 0
            self._warn_code = 0
            self._servo_codes = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
            self._cmd_num = 0
            self._arm_type = XCONF.Robot.Type.XARM7_X4
            self._arm_axis = XCONF.Robot.Axis.XARM7
            axis = kwargs.get('axis', self._arm_axis)
            if axis in [5, 6, 7]:
                self._arm_axis = axis
            arm_type = kwargs.get('type', self._arm_type)
            if arm_type in [3, 5, 6, 7, 8, 11]:
                self._arm_type = arm_type
            self._arm_master_id = 0
            self._arm_slave_id = 0
            self._arm_motor_tid = 0
            self._arm_motor_fid = 0
            self._arm_motor_brake_states = [-1, -1, -1, -1, -1, -1, -1, -1]  # [motor-1-brake-state, ..., motor-7-brake, reserved]
            self._arm_motor_enable_states = [-1, -1, -1, -1, -1, -1, -1, -1]  # [motor-1-enable-state, ..., motor-7-enable, reserved]
            self._gravity_direction = [0, 0, -1]

            self._is_ready = False
            self._is_sync = False
            self._is_first_report = True
            self._first_report_over = False
            self._default_is_radian = is_radian
            self._only_check_type = kwargs.get('only_check_type', 0)

            self._sleep_finish_time = time.monotonic()
            self._is_old_protocol = False

            self._major_version_number = 0  # 固件主版本号
            self._minor_version_number = 0  # 固件次版本号
            self._revision_version_number = 0  # 固件修正版本号

            self._temperatures = [0, 0, 0, 0, 0, 0, 0]
            self._voltages = [0, 0, 0, 0, 0, 0, 0]
            self._currents = [0, 0, 0, 0, 0, 0, 0]

            self._is_set_move = False
            self._pause_cond = threading.Condition()
            self._pause_lock = threading.Lock()
            self._pause_cnts = 0

            self._realtime_tcp_speed = 0
            self._realtime_joint_speeds = [0, 0, 0, 0, 0, 0, 0]

            self._count = -1
            self._last_report_time = time.monotonic()
            self._max_report_interval = 0

            self._cgpio_reset_enable = 0
            self._tgpio_reset_enable = 0
            self._cgpio_states = [0, 0, 256, 65533, 0, 65280, 0, 0, 0.0, 0.0, [0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0]]
            self._iden_progress = 0

            self._ignore_error = False
            self._ignore_state = False
            self.modbus_baud = -1

            self.gripper_is_enabled = False
            self.gripper_speed = 0
            self.gripper_version_numbers = [-1, -1, -1]

            self.bio_gripper_is_enabled = False
            self.bio_gripper_speed = 0
            self.bio_gripper_error_code = 0

            self.robotiq_is_activated = False

            self._cmd_timeout = XCONF.UxbusConf.SET_TIMEOUT / 1000

            self._is_collision_detection = 1
            self._collision_tool_type = 0
            self._collision_tool_params = [0, 0, 0, 0, 0, 0]
            self._is_simulation_robot = False

            self._is_reduced_mode = 0
            self._is_fence_mode = 0
            self._is_report_current = 0  # 针对get_report_tau_or_i的结果
            self._is_approx_motion = 0
            self._is_cart_continuous = 0

            self._reduced_mode_is_on = 0
            self._reduced_tcp_boundary = [9999, -9999, 9999, -9999, 9999, -9999]

            self._last_update_err_time = 0
            self._last_update_state_time = 0
            self._last_update_cmdnum_time = 0

            self._arm_type_is_1300 = False
            self._control_box_type_is_1300 = False

            self.linear_track_baud = -1
            self.linear_track_speed = 1
            self.linear_track_is_enabled = False
            self._ft_ext_force = [0, 0, 0, 0, 0, 0]
            self._ft_raw_force = [0, 0, 0, 0, 0, 0]
            self._only_check_result = 0
            self._keep_heart = True

            self._has_motion_cmd = False
            self._need_sync = False

            self._support_feedback = False
            self._feedback_que = queue.Queue()
            self._feedback_thread = None
            self._fb_key_transid_map = {}
            self._fb_transid_type_map = {}
            self._fb_transid_result_map = {}

            if not do_not_open:
                self.connect()

    def _init(self):
        self._last_position = [201.5, 0, 140.5, 3.1415926, 0, 0]  # [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]
        self._last_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [servo_1(rad), servo_2(rad), servo_3(rad), servo_4(rad), servo_5(rad), servo_6(rad), servo_7(rad)]
        self._last_tcp_speed = 100  # mm/s, rad/s
        self._last_tcp_acc = 2000  # mm/s^2, rad/s^2
        self._last_joint_speed = 0.3490658503988659  # 20 °/s
        self._last_joint_acc = 8.726646259971648  # 500 °/s^2
        self._mvtime = 0
        self._version = None
        self._robot_sn = None
        self._control_box_sn = None
        self._position = [201.5, 0, 140.5, 3.1415926, 0, 0]
        self._pose_aa = [201.5, 0, 140.5, 3.1415926, 0, 0]
        self._angles = [0] * 7
        self._position_offset = [0] * 6
        self._world_offset = [0] * 6
        self._state = 4
        self._mode = 0
        self._joints_torque = [0, 0, 0, 0, 0, 0, 0]  # 力矩
        self._tcp_load = [0, [0, 0, 0]]  # 负载[重量, 重心], [weight, [x, y, z]]
        self._collision_sensitivity = 0  # 碰撞灵敏度
        self._teach_sensitivity = 0  # 示教灵敏度
        self._error_code = 0
        self._warn_code = 0
        self._servo_codes = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
        self._cmd_num = 0
        self._arm_master_id = 0
        self._arm_slave_id = 0
        self._arm_motor_tid = 0
        self._arm_motor_fid = 0
        self._arm_motor_brake_states = [-1, -1, -1, -1, -1, -1, -1,
                                        -1]  # [motor-1-brake-state, ..., motor-7-brake, reserved]
        self._arm_motor_enable_states = [-1, -1, -1, -1, -1, -1, -1,
                                         -1]  # [motor-1-enable-state, ..., motor-7-enable, reserved]
        self._gravity_direction = [0, 0, -1]

        self._is_ready = False
        self._is_sync = False
        self._is_first_report = True
        self._first_report_over = False

        self._sleep_finish_time = time.monotonic()
        self._is_old_protocol = False

        self._major_version_number = 0  # 固件主版本号
        self._minor_version_number = 0  # 固件次版本号
        self._revision_version_number = 0  # 固件修正版本号

        self._temperatures = [0, 0, 0, 0, 0, 0, 0]
        self._voltages = [0, 0, 0, 0, 0, 0, 0]
        self._currents = [0, 0, 0, 0, 0, 0, 0]

        self._is_set_move = False
        self._pause_cond = threading.Condition()
        self._pause_lock = threading.Lock()
        self._pause_cnts = 0

        self._realtime_tcp_speed = 0
        self._realtime_joint_speeds = [0, 0, 0, 0, 0, 0, 0]

        self._count = -1
        self._last_report_time = time.monotonic()
        self._max_report_interval = 0

        self._cgpio_reset_enable = 0
        self._tgpio_reset_enable = 0
        self._cgpio_states = [0, 0, 256, 65533, 0, 65280, 0, 0, 0.0, 0.0, [0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 0, 0]]
        self._iden_progress = 0

        self._ignore_error = False
        self._ignore_state = False
        self.modbus_baud = -1

        self.gripper_is_enabled = False
        self.gripper_speed = 0
        self.gripper_version_numbers = [-1, -1, -1]

        self.bio_gripper_is_enabled = False
        self.bio_gripper_speed = 0
        self.bio_gripper_error_code = 0

        self.robotiq_is_activated = False

        self._cmd_timeout = XCONF.UxbusConf.SET_TIMEOUT / 1000

        self._is_collision_detection = 1
        self._collision_tool_type = 0
        self._collision_tool_params = [0, 0, 0, 0, 0, 0]
        self._is_simulation_robot = False

        self._is_reduced_mode = 0
        self._is_fence_mode = 0
        self._is_report_current = 0  # 针对get_report_tau_or_i的结果
        self._is_approx_motion = 0
        self._is_cart_continuous = 0

        self._reduced_mode_is_on = 0
        self._reduced_tcp_boundary = [9999, -9999, 9999, -9999, 9999, -9999]
        
        self._last_update_err_time = 0
        self._last_update_state_time = 0
        self._last_update_cmdnum_time = 0

        self._arm_type_is_1300 = False
        self._control_box_type_is_1300 = False

        self.linear_track_baud = -1
        self.linear_track_speed = 1
        self.linear_track_is_enabled = False

        self._ft_ext_force = [0, 0, 0, 0, 0, 0]
        self._ft_raw_force = [0, 0, 0, 0, 0, 0]

        self._has_motion_cmd = False
        self._need_sync = False
        self._only_check_result = 0
        self._keep_heart = True

    @staticmethod
    def log_api_info(msg, *args, code=0, **kwargs):
        if code == 0:
            logger.info(msg, *args, **kwargs)
        else:
            logger.error(msg, *args, **kwargs)

    def _check_version(self, is_first=False):
        if is_first:
            self._version = None
            self._robot_sn = None
            self._control_box_sn = None
        try:
            if not self._version:
                self.get_version()
            if is_first:
                fail_cnt = 0
                while not self._version and fail_cnt < 100:
                    code, _ = self.get_version()
                    fail_cnt += 1 if code != 0 else 0
                    if code != 0 or not self._version:
                        time.sleep(0.1)
                if not self._version and fail_cnt >= 100:
                    logger.error('failed to get version')
                    return -2

            if self._version and isinstance(self._version, str):
                # pattern = re.compile(
                #     r'.*(\d+),(\d+),(\S+),(\S+),.*[vV]*(\d+)\.(\d+)\.(\d+)')
                pattern = re.compile(
                    r'.*(\d+),(\d+),(.*),(.*),.*[vV]*(\d+)\.(\d+)\.(\d+).*')
                m = re.match(pattern, self._version)
                if m:
                    (xarm_axis, xarm_type, xarm_sn, ac_version,
                     major_version_number,
                     minor_version_number,
                     revision_version_number) = m.groups()
                    self._arm_axis = int(xarm_axis)
                    self._arm_type = int(xarm_type)
                    self._major_version_number = int(major_version_number)
                    self._minor_version_number = int(minor_version_number)
                    self._revision_version_number = int(revision_version_number)
                    
                    self._robot_sn = xarm_sn
                    self._control_box_sn = ac_version.strip()

                    self._arm_type_is_1300 = int(xarm_sn[2:6]) >= 1300 if xarm_sn[2:6].isdigit() else False
                    self._control_box_type_is_1300 = int(ac_version[2:6]) >= 1300 if ac_version[2:6].isdigit() else False
                else:
                    pattern = re.compile(r'.*[vV]*(\d+)\.(\d+)\.(\d+).*')
                    m = re.match(pattern, self._version)
                    if m:
                        (self._major_version_number,
                         self._minor_version_number,
                         self._revision_version_number) = map(int, m.groups())
                    else:
                        version_date = '-'.join(self._version.split('-')[-3:])
                        self._is_old_protocol = compare_time('2019-02-01', version_date)
                        if self._is_old_protocol:
                            self._major_version_number = 0
                            self._minor_version_number = 0
                            self._revision_version_number = 1
                        else:
                            self._major_version_number = 0
                            self._minor_version_number = 1
                            self._revision_version_number = 0
            if is_first:
                if self._check_robot_sn:
                    count = 2
                    self.get_robot_sn()
                    while not self._robot_sn and count and self.warn_code == 0:
                        self.get_robot_sn()
                        self.get_err_warn_code()
                        if not self._robot_sn and self.warn_code == 0 and count:
                            time.sleep(0.1)
                        count -= 1
                if self.warn_code != 0:
                    self.clean_warn()
                print('ROBOT_IP: {}, VERSION: v{}, PROTOCOL: {}, DETAIL: {}, TYPE1300: [{:d}, {:d}]'.format(
                    self._port,
                    '{}.{}.{}'.format(self._major_version_number, self._minor_version_number, self._revision_version_number),
                    'V0' if self._is_old_protocol else 'V1', self._version, self._control_box_type_is_1300, self._arm_type_is_1300
                ))
            return 0
        except Exception as e:
            print('compare_time: {}, {}'.format(self._version, e))
            return -1

    @property
    def only_check_result(self):
        return self._only_check_result

    @property
    def realtime_tcp_speed(self):
        return self._realtime_tcp_speed

    @property
    def realtime_joint_speeds(self):
        return [speed if self._default_is_radian else math.degrees(speed) for speed in self._realtime_joint_speeds]

    @property
    def version_number(self):
        return self._major_version_number, self._minor_version_number, self._revision_version_number

    @property
    def connected(self):
        return self._stream is not None and self._stream.connected

    @property
    def connected_503(self):
        return self._stream_503 is not None and self._stream_503.connected

    @property
    def reported(self):
        return self._stream_report is not None and self._stream_report.connected

    @property
    def ready(self):
        return self._is_ready

    @property
    def default_is_radian(self):
        return self._default_is_radian

    @property
    def is_simulation_robot(self):
        return self._is_simulation_robot

    def check_is_simulation_robot(self):
        return self._check_simulation_mode and self.is_simulation_robot
        # return self._check_simulation_mode and self.mode != 4
    
    @property
    def is_lite6(self):
        return self.axis == 6 and self.device_type == 9
        
    @property
    def is_850(self):
        return self.axis == 6 and self.device_type == 12

    @property
    def version(self):
        if not self._version:
            self.get_version()
        return self._version
        # return 'v' + '.'.join(map(str, self.version_number))

    @property
    def sn(self):
        return self._robot_sn
    
    @property
    def control_box_sn(self):
        return self._control_box_sn

    @property
    def position(self):
        if not self._enable_report:
            self.get_position()
        return [math.degrees(self._position[i]) if 2 < i < 6 and not self._default_is_radian
                else self._position[i] for i in range(len(self._position))]

    @property
    def position_aa(self):
        if not self._enable_report:
            self.get_position_aa()
        return [math.degrees(self._pose_aa[i]) if 2 < i < 6 and not self._default_is_radian
                else self._pose_aa[i] for i in range(len(self._pose_aa))]

    @property
    def tcp_jerk(self):
        return self._tcp_jerk

    @property
    def tcp_speed_limit(self):
        return [self._min_tcp_speed, self._max_tcp_speed]

    @property
    def tcp_acc_limit(self):
        return [self._min_tcp_acc, self._max_tcp_acc]

    @property
    def last_used_position(self):
        return [math.degrees(self._last_position[i]) if 2 < i < 6 and not self._default_is_radian
                else self._last_position[i] for i in range(len(self._last_position))]

    @property
    def last_used_tcp_speed(self):
        return self._last_tcp_speed

    @property
    def last_used_tcp_acc(self):
        return self._last_tcp_acc

    @property
    def angles(self):
        if not self._enable_report:
            self.get_servo_angle()
        return [angle if self._default_is_radian else math.degrees(angle) for angle in self._angles]

    @property
    def joint_jerk(self):
        return self._joint_jerk if self._default_is_radian else math.degrees(self._joint_jerk)

    @property
    def joint_speed_limit(self):
        limit = [self._min_joint_speed, self._max_joint_speed]
        if not self._default_is_radian:
            limit = [math.degrees(i) for i in limit]
        return limit

    @property
    def joint_acc_limit(self):
        limit = [self._min_joint_acc, self._max_joint_acc]
        if not self._default_is_radian:
            limit = [math.degrees(i) for i in limit]
        return limit

    @property
    def last_used_angles(self):
        return [angle if self._default_is_radian else math.degrees(angle) for angle in self._last_angles]

    @property
    def last_used_joint_speed(self):
        return self._last_joint_speed if self._default_is_radian else math.degrees(self._last_joint_speed)

    @property
    def last_used_joint_acc(self):
        return self._last_joint_acc if self._default_is_radian else math.degrees(self._last_joint_acc)

    @property
    def position_offset(self):
        return [math.degrees(self._position_offset[i]) if 2 < i < 6 and not self._default_is_radian
                else self._position_offset[i] for i in range(len(self._position_offset))]

    @property
    def world_offset(self):
        return [math.degrees(self._world_offset[i]) if 2 < i < 6 and not self._default_is_radian
                else self._world_offset[i] for i in range(len(self._world_offset))]

    @property
    def state(self):
        if not self._enable_report:
            self.get_state()
        return self._state

    @property
    def mode(self):
        return self._mode

    @property
    def joints_torque(self):
        return self._joints_torque

    @property
    def tcp_load(self):
        return self._tcp_load

    @property
    def collision_sensitivity(self):
        return self._collision_sensitivity

    @property
    def teach_sensitivity(self):
        return self._teach_sensitivity

    @property
    def motor_brake_states(self):
        return self._arm_motor_brake_states

    @property
    def motor_enable_states(self):
        return self._arm_motor_enable_states

    @property
    def temperatures(self):
        return self._temperatures

    @property
    def error_code(self):
        if not self._enable_report:
            self.get_err_warn_code()
        return self._error_code

    @property
    def warn_code(self):
        if not self._enable_report:
            self.get_err_warn_code()
        return self._warn_code

    @property
    def has_error(self):
        return self.error_code != 0

    @property
    def has_warn(self):
        return self.warn_code != 0

    @property
    def has_err_warn(self):
        return self.has_error or self._warn_code != 0 or (self.arm_cmd and self.arm_cmd.has_err_warn)

    @property
    def cmd_num(self):
        if not self._enable_report:
            self.get_cmdnum()
        return self._cmd_num

    @property
    def device_type(self):
        return self._arm_type

    @property
    def axis(self):
        return self._arm_axis

    @property
    def master_id(self):
        return self._arm_master_id

    @property
    def slave_id(self):
        return self._arm_slave_id

    @property
    def motor_tid(self):
        return self._arm_motor_tid

    @property
    def motor_fid(self):
        return self._arm_motor_fid

    @property
    def gravity_direction(self):
        return self._gravity_direction

    @property
    def gpio_reset_config(self):
        return [self._cgpio_reset_enable, self._tgpio_reset_enable]

    @property
    def count(self):
        return self._count

    @property
    def servo_codes(self):
        return self._servo_codes

    @property
    def is_stop(self):
        return self.state >= 4

    @property
    def voltages(self):
        return self._voltages

    @property
    def currents(self):
        return self._currents

    @property
    def cgpio_states(self):
        return self._cgpio_states

    @property
    def self_collision_params(self):
        return [self._is_collision_detection, self._collision_tool_type, self._collision_tool_params]

    @property
    def is_reduced_mode(self):
        return self._is_reduced_mode != 0
    
    @property
    def is_fence_mode(self):
        return self._is_fence_mode != 0
    
    @property
    def is_report_current(self):
        return self._is_report_current != 0  # 针对get_report_tau_or_i的结果
    
    @property
    def is_approx_motion(self):
        return self._is_approx_motion != 0

    @property
    def is_cart_continuous(self):
        return self._is_cart_continuous != 0
    
    @property
    def reduced_mode_is_on(self):
        return self._reduced_mode_is_on != 0
    
    @property
    def reduced_tcp_boundary(self):
        return self._reduced_tcp_boundary
    
    @property
    def ft_ext_force(self):
        return self._ft_ext_force

    @property
    def ft_raw_force(self):
        return self._ft_raw_force
    
    @property
    def support_feedback(self):
        return self._support_feedback

    def version_is_ge(self, major, minor=0, revision=0):
        if self._version is None:
            self._check_version()
        return self._major_version_number > major or (
            self._major_version_number == major and self._minor_version_number > minor) or (
            self._major_version_number == major and self._minor_version_number == minor and
            self._revision_version_number >= revision)

    def wait_until_not_pause(self):
        if self._check_is_pause and self.connected and self.state == 3 and self._enable_report:
            with self._pause_cond:
                with self._pause_lock:
                    self._pause_cnts += 1
                self._pause_cond.wait()
                with self._pause_lock:
                    self._pause_cnts -= 1
    
    def wait_until_cmdnum_lt_max(self):
        if not self._check_cmdnum_limit:
            return
        while self.connected and self.cmd_num >= self._max_cmd_num:
            if time.monotonic() - self._last_report_time > 0.4:
                self.get_cmdnum()
            time.sleep(0.05)

    @property
    def check_xarm_is_ready(self):
        if self._check_is_ready and not self.version_is_ge(1, 5, 20):
            return self.ready
        else:
            # no check if version >= 1.5.20
            return True

    def _timed_comm_thread(self):
        self._timed_comm_t_alive = True
        cnt = 0
        last_send_time = 0
        while self.connected and self._timed_comm_t_alive:
            curr_time = time.monotonic()
            if not self._keep_heart:
                time.sleep(1)
                continue
            if self.arm_cmd and curr_time - last_send_time > 10 and curr_time - self.arm_cmd.last_comm_time > self._timed_comm_interval:
                try:
                    if cnt == 0:
                        code, _ = self.get_cmdnum()
                    elif cnt == 1:
                        code, _ = self.get_state()
                    else:
                        code, _ = self.get_err_warn_code()
                    cnt = (cnt + 1) % 3
                    if code >= 0:
                        last_send_time = curr_time
                except:
                    pass
            time.sleep(0.5)

    def _clean_thread(self):
        self._thread_manage.join(1)
        if self._pool:
            try:
                self._pool.close()
                self._pool.join()
            except:
                pass
    
    def connect_503(self):
        self._stream_503 = SocketPort(self._port, XCONF.SocketConf.TCP_CONTROL_PORT + 1,
            heartbeat=self._enable_heartbeat, buffer_size=XCONF.SocketConf.TCP_CONTROL_BUF_SIZE, forbid_uds=self._forbid_uds)
        if not self.connected_503:
            return -1
        self.arm_cmd_503 = UxbusCmdTcp(self._stream_503, set_feedback_key_tranid=self._set_feedback_key_tranid)
        self.arm_cmd_503.set_debug(self._debug)
        return 0

    def connect(self, port=None, baudrate=None, timeout=None, axis=None, arm_type=None):
        if self.connected:
            return
        if axis in [5, 6, 7]:
            self._arm_axis = axis
        if arm_type in [3, 5, 6, 7, 8, 9, 11]:
            self._arm_type = arm_type
        self._is_ready = True
        self._port = port if port is not None else self._port
        self._baudrate = baudrate if baudrate is not None else self._baudrate
        self._timeout = timeout if timeout is not None else self._timeout
        if not self._port:
            raise Exception('can not connect to port/ip {}'.format(self._port))
        if self._timed_comm_t is not None:
            try:
                self._timed_comm_t_alive = False
                self._timed_comm_t.join()
                self._timed_comm_t = None
            except:
                pass
        self._is_first_report = True
        self._first_report_over = False
        self._init()
        if isinstance(self._port, (str, bytes)):
            if self._port == 'localhost' or re.match(
                    r"^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$",
                    self._port):
                self._stream = SocketPort(self._port, XCONF.SocketConf.TCP_CONTROL_PORT,
                                          heartbeat=self._enable_heartbeat,
                                          buffer_size=XCONF.SocketConf.TCP_CONTROL_BUF_SIZE, forbid_uds=self._forbid_uds, fb_que=self._feedback_que)
                if not self.connected:
                    raise Exception('connect socket failed')

                self._report_error_warn_changed_callback()
                self._feedback_thread = threading.Thread(target=self._feedback_thread_handle, daemon=True)
                self._feedback_thread.start()

                self.arm_cmd = UxbusCmdTcp(self._stream, set_feedback_key_tranid=self._set_feedback_key_tranid)
                self.arm_cmd.set_protocol_identifier(2)
                self._stream_type = 'socket'

                try:
                    if self._timed_comm:
                        self._timed_comm_t = threading.Thread(target=self._timed_comm_thread, daemon=True)
                        self._timed_comm_t.start()
                except:
                    pass

                self._stream_report = None

                try:
                    self._connect_report()
                except:
                    self._stream_report = None

                if self._check_version(is_first=True) < 0:
                    self.disconnect()
                    raise Exception('failed to check version, close')
                self._support_feedback = self.version_is_ge(2, 0, 102)
                self.arm_cmd.set_debug(self._debug)

                if self._max_callback_thread_count < 0 and asyncio is not None:
                    self._asyncio_loop = asyncio.new_event_loop()
                    self._asyncio_loop_thread = threading.Thread(target=self._run_asyncio_loop, daemon=True)
                    self._thread_manage.append(self._asyncio_loop_thread)
                    self._asyncio_loop_thread.start()
                elif self._max_callback_thread_count > 0 and ThreadPool is not None:
                    self._pool = ThreadPool(self._max_callback_thread_count)

                if self._stream.connected and self._enable_report:
                    self._report_thread = threading.Thread(target=self._report_thread_handle, daemon=True)
                    self._report_thread.start()
                    self._thread_manage.append(self._report_thread)

                self._report_connect_changed_callback()
            else:
                if SerialPort is None:
                    raise Exception('serial module is not found, if you want to connect to xArm with serial, please `pip install pyserial>=3.4`')
                self._stream = SerialPort(self._port)
                if not self.connected:
                    raise Exception('connect serial failed')
                self._report_error_warn_changed_callback()

                self.arm_cmd = UxbusCmdSer(self._stream)
                self._stream_type = 'serial'

                if self._max_callback_thread_count < 0 and asyncio is not None:
                    self._asyncio_loop = asyncio.new_event_loop()
                    self._asyncio_loop_thread = threading.Thread(target=self._run_asyncio_loop, daemon=True)
                    self._thread_manage.append(self._asyncio_loop_thread)
                    self._asyncio_loop_thread.start()
                elif self._max_callback_thread_count > 0 and ThreadPool is not None:
                    self._pool = ThreadPool(self._max_callback_thread_count)

                if self._enable_report:
                    self._report_thread = threading.Thread(target=self._auto_get_report_thread, daemon=True)
                    self._report_thread.start()
                    self._report_connect_changed_callback(True, True)
                    self._thread_manage.append(self._report_thread)
                else:
                    self._report_connect_changed_callback(True, False)
                self._check_version(is_first=True)
                self.arm_cmd.set_debug(self._debug)
            self.set_timeout(self._cmd_timeout)
            if self._rewrite_modbus_baudrate_method:
                setattr(self.arm_cmd, 'set_modbus_baudrate_old', self.arm_cmd.set_modbus_baudrate)
                setattr(self.arm_cmd, 'set_modbus_baudrate', self._core_set_modbus_baudrate)

    if asyncio:
        def _run_asyncio_loop(self):
            # @asyncio.coroutine
            # def _asyncio_loop():
            #     logger.debug('asyncio thread start ...')
            #     while self.connected:
            #         yield from asyncio.sleep(0.001)
            #     logger.debug('asyncio thread exit ...')

            try:
                asyncio.set_event_loop(self._asyncio_loop)
                self._asyncio_loop_alive = True
                # self._asyncio_loop.run_until_complete(_asyncio_loop())
                self._asyncio_loop.run_until_complete(self._asyncio_loop_func())
            except Exception as e:
                pass

            self._asyncio_loop_alive = False

        # @staticmethod
        # @asyncio.coroutine
        # def _async_run_callback(callback, msg):
        #     yield from callback(msg)

    def _run_callback(self, callback, msg, name='', enable_callback_thread=True):
        try:
            if self._asyncio_loop_alive and enable_callback_thread:
                coroutine = self._async_run_callback(callback, msg)
                asyncio.run_coroutine_threadsafe(coroutine, self._asyncio_loop)
            elif self._pool is not None and enable_callback_thread:
                self._pool.apply_async(callback, args=(msg,))
            else:
                callback(msg)
        except Exception as e:
            logger.error('run {} callback exception: {}'.format(name, e))

    def _core_set_modbus_baudrate(self, baudrate, use_old=False):
        """
        此函数是用于覆盖core.set_modbus_baudrate方法，主要用于兼容旧代码
        新代码建议直接使用set_tgpio_modbus_baudrate此接口
        :param baudrate: 
        :param use_old: 
            为True时调用原来的core.set_modbus_baudrate方法
            为False时使用新的set_tgpio_modbus_baudrate
        :return [code, ...]
        """
        if not use_old:
            ret = self.set_tgpio_modbus_baudrate(baudrate)
            return [ret, self.modbus_baud]
        else:
            return self.arm_cmd.set_modbus_baudrate_old(baudrate)

    def disconnect(self):
        try:
            self._stream.close()
        except:
            pass
        if self._stream_503:
            try:
                self._stream_503.close()
            except:
                pass
        if self._stream_report:
            try:
                self._stream_report.close()
            except:
                pass
        self._is_ready = False
        try:
            self._stream.join()
        except:
            pass
        if self._stream_report:
            try:
                self._stream_report.join()
            except:
                pass
        self._report_connect_changed_callback(False, False)
        with self._pause_cond:
            self._pause_cond.notifyAll()
        self._clean_thread()

    def set_timeout(self, timeout):
        self._cmd_timeout = timeout
        if self.arm_cmd is not None:
            self._cmd_timeout = self.arm_cmd.set_timeout(self._cmd_timeout)
        return self._cmd_timeout
    
    def set_baud_checkset_enable(self, enable):
        self._baud_checkset = enable
        return 0

    def set_checkset_default_baud(self, type_, baud):
        if type_ == 1:
            self._default_gripper_baud = baud
        elif type_ == 2:
            self._default_bio_baud = baud
        elif type_ == 3:
            self._default_robotiq_baud = baud
        elif type_ == 4:
            self._default_linear_track_baud = baud
        else:
            return APIState.API_EXCEPTION
        return 0

    def get_checkset_default_baud(self, type_):
        if type_ == 1:
            return 0, self._default_gripper_baud
        elif type_ == 2:
            return 0, self._default_bio_baud
        elif type_ == 3:
            return 0, self._default_robotiq_baud
        elif type_ == 4:
            return 0, self._default_linear_track_baud
        return APIState.API_EXCEPTION, 0

    def _connect_report(self):
        if self._enable_report:
            if self._stream_report:
                try:
                    self._stream_report.close()
                except:
                    pass
                time.sleep(2)
            if self._report_type == 'real':
                self._stream_report = SocketPort(
                    self._port, XCONF.SocketConf.TCP_REPORT_REAL_PORT,
                    buffer_size=1024 if not self._is_old_protocol else 87,
                    forbid_uds=self._forbid_uds)
            elif self._report_type == 'normal':
                self._stream_report = SocketPort(
                    self._port, XCONF.SocketConf.TCP_REPORT_NORM_PORT,
                    buffer_size=XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE if not self._is_old_protocol else 87,
                    forbid_uds=self._forbid_uds)
            else:
                self._stream_report = SocketPort(
                    self._port, XCONF.SocketConf.TCP_REPORT_RICH_PORT,
                    buffer_size=1024 if not self._is_old_protocol else 187,
                    forbid_uds=self._forbid_uds)

    def __report_callback(self, report_id, item, name=''):
        if report_id in self._report_callbacks.keys():
            for callback in self._report_callbacks[report_id]:
                self._run_callback(callback, item, name=name)

    def _report_connect_changed_callback(self, main_connected=None, report_connected=None):
        if self.REPORT_CONNECT_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[self.REPORT_CONNECT_CHANGED_ID]:
                self._run_callback(callback, {
                    'connected': self._stream and self._stream.connected if main_connected is None else main_connected,
                    'reported': self._stream_report and self._stream_report.connected if report_connected is None else report_connected,
                }, name='connect_changed')

    def _report_state_changed_callback(self):
        if self._ignore_state:
            return
        self.__report_callback(self.REPORT_STATE_CHANGED_ID, {'state': self._state}, name='state_changed')

    def _report_mode_changed_callback(self):
        self.__report_callback(self.REPORT_MODE_CHANGED_ID, {'mode': self._mode}, name='mode_changed')

    def _report_mtable_mtbrake_changed_callback(self):
        self.__report_callback(self.REPORT_MTABLE_MTBRAKE_CHANGED_ID, {
            'mtable': [bool(i) for i in self._arm_motor_enable_states],
            'mtbrake': [bool(i) for i in self._arm_motor_brake_states]
        }, name='(mtable/mtbrake)_changed')

    def _report_error_warn_changed_callback(self):
        if self._ignore_error:
            return
        self.__report_callback(self.REPORT_ERROR_WARN_CHANGED_ID, {
            'warn_code': self._warn_code,
            'error_code': self._error_code,
        }, name='(error/warn)_changed')

    def _report_cmdnum_changed_callback(self):
        self.__report_callback(self.REPORT_CMDNUM_CHANGED_ID, {
            'cmdnum': self._cmd_num
        }, name='cmdnum_changed')

    def _report_temperature_changed_callback(self):
        self.__report_callback(self.REPORT_TEMPERATURE_CHANGED_ID, {
            'temperatures': self.temperatures
        }, name='temperature_changed')

    def _report_count_changed_callback(self):
        self.__report_callback(self.REPORT_COUNT_CHANGED_ID, {'count': self._count}, name='count_changed')

    def _report_iden_progress_changed_callback(self):
        self.__report_callback(self.REPORT_IDEN_PROGRESS_CHANGED_ID, {'progress': self._iden_progress}, name='iden_progress_changed')

    def _report_location_callback(self):
        if self.REPORT_LOCATION_ID in self._report_callbacks.keys():
            for item in self._report_callbacks[self.REPORT_LOCATION_ID]:
                callback = item['callback']
                ret = {}
                if item['cartesian']:
                    ret['cartesian'] = self.position.copy()
                if item['joints']:
                    ret['joints'] = self.angles.copy()
                self._run_callback(callback, ret, name='location')

    def _report_callback(self):
        if self.REPORT_ID in self._report_callbacks.keys():
            for item in self._report_callbacks[self.REPORT_ID]:
                callback = item['callback']
                ret = {}
                if item['cartesian']:
                    ret['cartesian'] = self.position.copy()
                if item['joints']:
                    ret['joints'] = self.angles.copy()
                if item['error_code']:
                    ret['error_code'] = self._error_code
                if item['warn_code']:
                    ret['warn_code'] = self._warn_code
                if item['state']:
                    ret['state'] = self._state
                if item['mtable']:
                    mtable = [bool(i) for i in self._arm_motor_enable_states]
                    ret['mtable'] = mtable.copy()
                if item['mtbrake']:
                    mtbrake = [bool(i) for i in self._arm_motor_brake_states]
                    ret['mtbrake'] = mtbrake.copy()
                if item['cmdnum']:
                    ret['cmdnum'] = self._cmd_num
                self._run_callback(callback, ret, name='report')

    def _report_thread_handle(self):
        main_socket_connected = self.connected
        report_socket_connected = self.reported
        protocol_identifier = 2
        last_send_time = 0
        max_reconnect_cnts = 10
        connect_failed_cnt = 0

        while self.connected:
            try:
                curr_time = time.monotonic()
                if self._keep_heart:
                    if protocol_identifier != 3 and self.version_is_ge(1, 8, 6) and self.arm_cmd.set_protocol_identifier(3) == 0:
                        protocol_identifier = 3
                    if protocol_identifier == 3 and curr_time - last_send_time > 10 and curr_time - self.arm_cmd.last_comm_time > 30:
                        code, _ = self.get_state()
                        # print('send heartbeat, code={}'.format(code))
                        if code >= 0:
                            last_send_time = curr_time
                        if curr_time - self.arm_cmd.last_comm_time > 90:
                            logger.error('client timeout over 90s, disconnect')
                            break
                if not self.reported:
                    # self.get_err_warn_code()
                    if report_socket_connected:
                        report_socket_connected = False
                        self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                    self._connect_report()
                    if not self.reported:
                        connect_failed_cnt += 1
                        if self.connected and (connect_failed_cnt <= max_reconnect_cnts or protocol_identifier == 3):
                            time.sleep(2)
                        elif not self.connected or protocol_identifier == 2:
                            logger.error('report thread is break, connected={}, failed_cnts={}'.format(self.connected, connect_failed_cnt))
                            break
                        continue
                    else:
                        connect_failed_cnt = 0
                connect_failed_cnt = 0
                if not report_socket_connected:
                    report_socket_connected = True
                    self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                recv_data = self._stream_report.read(1)
                if recv_data != -1:
                    size = convert.bytes_to_u32(recv_data)
                    if self._is_old_protocol and size > 256:
                        self._is_old_protocol = False
                    self._handle_report_data(recv_data)
                # else:
                #     if self.connected:
                #         code, err_warn = self.get_err_warn_code()
                #         if code == -1 or code == 3:
                #             break
                #     if not self.connected:
                #         break
            except Exception as e:
                logger.error(e)
                # if self.connected:
                #     code, _ = self.get_err_warn_code()
                #     if code == -1 or code == 3:
                #         break
                if not self.connected:
                    break
                if not self._stream_report or not self._stream_report.connected:
                    self._connect_report()
            time.sleep(0.001)
        if self._pause_cnts > 0:
            with self._pause_cond:
                self._pause_cond.notifyAll()
        self.disconnect()

    def _handle_report_data(self, data):
        def __handle_report_normal_old(rx_data):
            report_time = time.monotonic()
            interval = report_time - self._last_report_time
            self._max_report_interval = max(self._max_report_interval, interval)
            self._last_report_time = report_time
            # print('length:', convert.bytes_to_u32(rx_data[0:4]))
            state, mtbrake, mtable, error_code, warn_code = rx_data[4:9]
            angles = convert.bytes_to_fp32s(rx_data[9:7 * 4 + 9], 7)
            pose = convert.bytes_to_fp32s(rx_data[37:6 * 4 + 37], 6)
            cmd_num = convert.bytes_to_u16(rx_data[61:63])
            pose_offset = convert.bytes_to_fp32s(rx_data[63:6 * 4 + 63], 6)

            if error_code != self._error_code or warn_code != self._warn_code:
                if error_code != self._error_code:
                    self._error_code = error_code
                    if self._error_code != 0:
                        pretty_print('Error, code: {}'.format(self._error_code), color='red')
                    else:
                        pretty_print('Error had clean', color='blue')
                if warn_code != self._warn_code:
                    self._warn_code = warn_code
                    if self._warn_code != 0:
                        pretty_print('Warn, code: {}'.format(self._warn_code), color='yellow')
                    else:
                        pretty_print('Warnning had clean', color='blue')
                self._report_error_warn_changed_callback()
                logger.info('OnReport -> err={}, warn={}, state={}, cmdnum={}, mtbrake={}, mtable={}'.format(
                    error_code, warn_code, state, cmd_num, mtbrake, mtable
                ))
            elif not self._only_report_err_warn_changed:
                self._report_error_warn_changed_callback()

            if cmd_num != self._cmd_num:
                self._cmd_num = cmd_num
                self._report_cmdnum_changed_callback()

            if state != self._state:
                self._state = state
                self._report_state_changed_callback()

            mtbrake = [mtbrake & 0x01, mtbrake >> 1 & 0x01, mtbrake >> 2 & 0x01, mtbrake >> 3 & 0x01,
                       mtbrake >> 4 & 0x01, mtbrake >> 5 & 0x01, mtbrake >> 6 & 0x01, mtbrake >> 7 & 0x01]
            mtable = [mtable & 0x01, mtable >> 1 & 0x01, mtable >> 2 & 0x01, mtable >> 3 & 0x01,
                      mtable >> 4 & 0x01, mtable >> 5 & 0x01, mtable >> 6 & 0x01, mtable >> 7 & 0x01]

            if mtbrake != self._arm_motor_brake_states or mtable != self._arm_motor_enable_states:
                self._arm_motor_enable_states = mtable
                self._arm_motor_brake_states = mtbrake
                self._report_mtable_mtbrake_changed_callback()

            if not self._is_first_report:
                if state in [4, 5] or not all([bool(item[0] & item[1]) for item in zip(mtbrake, mtable)][:self.axis]):
                    # if self._is_ready:
                    #     pretty_print('[report], xArm is not ready to move', color='red')
                    self._is_ready = False
                else:
                    # if not self._is_ready:
                    #     pretty_print('[report], xArm is ready to move', color='green')
                    self._is_ready = True
            else:
                self._is_ready = False
            self._is_first_report = False
            if not self._is_ready:
                self._sleep_finish_time = 0

            reset_tgpio_params = False
            reset_linear_track_params = False
            if 0 < error_code <= 17:
                reset_tgpio_params = True
                reset_linear_track_params = True
            elif error_code in [19, 28]:
                reset_tgpio_params = True
            elif error_code == 111:
                reset_linear_track_params = True
            if reset_tgpio_params:
                self.modbus_baud = -1
                self.robotiq_is_activated = False
                self.gripper_is_enabled = False
                self.bio_gripper_is_enabled = False
                self.bio_gripper_speed = 0
                self.gripper_is_enabled = False
                self.gripper_speed = 0
                self.gripper_version_numbers = [-1, -1, -1]
            if reset_linear_track_params:
                self.linear_track_baud = -1
                self.linear_track_is_enabled = False
                self.linear_track_speed = 1

            # if error_code in [1, 10, 11, 12, 13, 14, 15, 16, 17, 19, 28]:
            #     self.modbus_baud = -1
            #     self.robotiq_is_activated = False
            #     self.gripper_is_enabled = False
            #     self.bio_gripper_is_enabled = False
            #     self.bio_gripper_speed = 0
            #     self.gripper_is_enabled = False
            #     self.gripper_speed = 0
            #     self.gripper_version_numbers = [-1, -1, -1]
            #     self.linear_track_is_enabled = False
            #     self.linear_track_speed = 0

            self._error_code = error_code
            self._warn_code = warn_code
            self.arm_cmd.has_err_warn = error_code != 0 or warn_code != 0
            _state = self._state
            self._state = state
            if self.state != 3 and (_state == 3 or self._pause_cnts > 0):
                with self._pause_cond:
                    self._pause_cond.notifyAll()
            self._cmd_num = cmd_num
            self._arm_motor_brake_states = mtbrake
            self._arm_motor_enable_states = mtable

            update_time = time.monotonic()
            self._last_update_cmdnum_time = update_time
            self._last_update_state_time = update_time
            self._last_update_err_time = update_time

            for i in range(len(pose)):
                pose[i] = filter_invaild_number(pose[i], 3 if i < 3 else 6, default=self._position[i])
            for i in range(len(angles)):
                angles[i] = filter_invaild_number(angles[i], 6, default=self._angles[i])
            for i in range(len(pose_offset)):
                pose_offset[i] = filter_invaild_number(pose_offset[i], 3 if i < 3 else 6, default=self._position_offset[i])

            if not (0 < self._error_code <= 17):
                self._position = pose
            if not (0 < self._error_code <= 17):
                self._angles = angles
            if not (0 < self._error_code <= 17):
                self._position_offset = pose_offset

            self._report_location_callback()

            self._report_callback()
            if not self._is_sync and self._error_code == 0 and self._state not in [4, 5, 6]:
                self._sync()
                self._is_sync = True

        def __handle_report_rich_old(rx_data):
            __handle_report_normal_old(rx_data)
            (self._arm_type,
             arm_axis,
             self._arm_master_id,
             self._arm_slave_id,
             self._arm_motor_tid,
             self._arm_motor_fid) = rx_data[87:93]

            if 7 >= arm_axis >= 5:
                self._arm_axis = arm_axis
            if self._arm_type == 5:
                self._arm_axis = 5
            elif self._arm_type == 6:
                self._arm_axis = 6
            elif self._arm_type == 3:
                self._arm_axis = 7

            ver_msg = rx_data[93:122]
            # self._version = str(ver_msg, 'utf-8')

            trs_msg = convert.bytes_to_fp32s(rx_data[123:143], 5)
            # trs_msg = [i[0] for i in trs_msg]
            (self._tcp_jerk,
             self._min_tcp_acc,
             self._max_tcp_acc,
             self._min_tcp_speed,
             self._max_tcp_speed) = trs_msg
            # print('tcp_jerk: {}, min_acc: {}, max_acc: {}, min_speed: {}, max_speed: {}'.format(
            #     self._tcp_jerk, self._min_tcp_acc, self._max_tcp_acc, self._min_tcp_speed, self._max_tcp_speed
            # ))

            p2p_msg = convert.bytes_to_fp32s(rx_data[143:163], 5)
            # p2p_msg = [i[0] for i in p2p_msg]
            (self._joint_jerk,
             self._min_joint_acc,
             self._max_joint_acc,
             self._min_joint_speed,
             self._max_joint_speed) = p2p_msg
            # print('joint_jerk: {}, min_acc: {}, max_acc: {}, min_speed: {}, max_speed: {}'.format(
            #     self._joint_jerk, self._min_joint_acc, self._max_joint_acc,
            #     self._min_joint_speed, self._max_joint_speed
            # ))

            rot_msg = convert.bytes_to_fp32s(rx_data[163:171], 2)
            # rot_msg = [i[0] for i in rot_msg]
            self._rot_jerk, self._max_rot_acc = rot_msg
            # print('rot_jerk: {}, mac_acc: {}'.format(self._rot_jerk, self._max_rot_acc))

            sv3_msg = convert.bytes_to_u16s(rx_data[171:187], 8)
            self._first_report_over = True

        def __handle_report_real(rx_data):
            state, mode = rx_data[4] & 0x0F, rx_data[4] >> 4
            cmd_num = convert.bytes_to_u16(rx_data[5:7])
            angles = convert.bytes_to_fp32s(rx_data[7:7 * 4 + 7], 7)
            pose = convert.bytes_to_fp32s(rx_data[35:6 * 4 + 35], 6)
            torque = convert.bytes_to_fp32s(rx_data[59:7 * 4 + 59], 7)
            if cmd_num != self._cmd_num:
                self._cmd_num = cmd_num
                self._report_cmdnum_changed_callback()
            if state != self._state:
                self._state = state
                self._report_state_changed_callback()
            if state in [4, 5]:
                self._is_ready = False
            else:
                self._is_ready = True

            if mode != self._mode:
                self._mode = mode
                self._report_mode_changed_callback()
            for i in range(len(pose)):
                pose[i] = filter_invaild_number(pose[i], 3 if i < 3 else 6, default=self._position[i])
            for i in range(len(angles)):
                angles[i] = filter_invaild_number(angles[i], 6, default=self._angles[i])

            if not (0 < self._error_code <= 17):
                self._position = pose
            if not (0 < self._error_code <= 17):
                self._angles = angles
            self._joints_torque = torque

            self._report_location_callback()

            self._report_callback()
            if not self._is_sync and self._state not in [4, 5]:
                self._sync()
                self._is_sync = True
            length = len(rx_data)
            if length >= 135:
                # FT_SENSOR
                self._ft_ext_force = convert.bytes_to_fp32s(rx_data[87:111], 6)
                self._ft_raw_force = convert.bytes_to_fp32s(rx_data[111:135], 6)

        def __handle_report_normal(rx_data):
            report_time = time.monotonic()
            interval = report_time - self._last_report_time
            self._max_report_interval = max(self._max_report_interval, interval)
            self._last_report_time = report_time
            # print('length:', convert.bytes_to_u32(rx_data[0:4]), len(rx_data))
            state, mode = rx_data[4] & 0x0F, rx_data[4] >> 4
            # if state != self._state or mode != self._mode:
            #     print('mode: {}, state={}, time={}'.format(mode, state, time.monotonic()))
            cmd_num = convert.bytes_to_u16(rx_data[5:7])
            angles = convert.bytes_to_fp32s(rx_data[7:7 * 4 + 7], 7)
            pose = convert.bytes_to_fp32s(rx_data[35:6 * 4 + 35], 6)
            torque = convert.bytes_to_fp32s(rx_data[59:7 * 4 + 59], 7)
            mtbrake, mtable, error_code, warn_code = rx_data[87:91]
            pose_offset = convert.bytes_to_fp32s(rx_data[91:6 * 4 + 91], 6)
            tcp_load = convert.bytes_to_fp32s(rx_data[115:4 * 4 + 115], 4)
            collis_sens, teach_sens = rx_data[131:133]
            # if (collis_sens not in list(range(6)) or teach_sens not in list(range(6))) \
            #         and ((error_code != 0 and error_code not in controller_error_keys) or (warn_code != 0 and warn_code not in controller_warn_keys)):
            #     self._stream_report.close()
            #     logger.warn('ReportDataException: data={}'.format(rx_data))
            #     return
            length = convert.bytes_to_u32(rx_data[0:4])
            data_len = len(rx_data)
            if (length != data_len and (length != 233 or data_len != 245)) or collis_sens not in list(range(6)) or teach_sens not in list(range(6)) \
                or mode not in list(range(12)) or state not in list(range(10)):
                self._stream_report.close()
                logger.warn('ReportDataException: length={}, data_len={}, '
                            'state={}, mode={}, collis_sens={}, teach_sens={}, '
                            'error_code={}, warn_code={}'.format(
                    length, data_len,
                    state, mode, collis_sens, teach_sens, error_code, warn_code
                ))
                return
            self._gravity_direction = convert.bytes_to_fp32s(rx_data[133:3*4 + 133], 3)

            reset_tgpio_params = False
            reset_linear_track_params = False
            if 0 < error_code <= 17:
                reset_tgpio_params = True
                reset_linear_track_params = True
            elif error_code in [19, 28]:
                reset_tgpio_params = True
            elif error_code == 111:
                reset_linear_track_params = True
            if reset_tgpio_params:
                self.modbus_baud = -1
                self.robotiq_is_activated = False
                self.gripper_is_enabled = False
                self.bio_gripper_is_enabled = False
                self.bio_gripper_speed = 0
                self.gripper_is_enabled = False
                self.gripper_speed = 0
                self.gripper_version_numbers = [-1, -1, -1]
            if reset_linear_track_params:
                self.linear_track_baud = -1
                self.linear_track_is_enabled = False
                self.linear_track_speed = 0

            # if error_code in [1, 10, 11, 12, 13, 14, 15, 16, 17, 19, 28]:
            #     self.modbus_baud = -1
            #     self.robotiq_is_activated = False
            #     self.gripper_is_enabled = False
            #     self.bio_gripper_is_enabled = False
            #     self.bio_gripper_speed = -1
            #     self.gripper_speed = -1
            #     self.gripper_version_numbers = [-1, -1, -1]
            #     self.linear_track_is_enabled = False
            #     self.linear_track_speed = -1

            # print('torque: {}'.format(torque))
            # print('tcp_load: {}'.format(tcp_load))
            # print('collis_sens: {}, teach_sens: {}'.format(collis_sens, teach_sens))

            if error_code != self._error_code or warn_code != self._warn_code:
                if error_code != self._error_code:
                    self._error_code = error_code
                    if self._error_code != 0:
                        pretty_print('ControllerError, code: {}'.format(self._error_code), color='red')
                    else:
                        pretty_print('ControllerError had clean', color='blue')
                if warn_code != self._warn_code:
                    self._warn_code = warn_code
                    if self._warn_code != 0:
                        pretty_print('ControllerWarning, code: {}'.format(self._warn_code), color='yellow')
                    else:
                        pretty_print('ControllerWarning had clean', color='blue')
                self._report_error_warn_changed_callback()
                logger.info('OnReport -> err={}, warn={}, state={}, cmdnum={}, mtbrake={}, mtable={}, mode={}'.format(
                    error_code, warn_code, state, cmd_num, mtbrake, mtable, mode
                ))
            elif not self._only_report_err_warn_changed:
                self._report_error_warn_changed_callback()

            if cmd_num != self._cmd_num:
                self._cmd_num = cmd_num
                self._report_cmdnum_changed_callback()

            if state != self._state:
                if not self._has_motion_cmd and self._state in [0, 1] and state not in [0, 1]:
                    self._need_sync = True
                if self._state in [0, 1] and state not in [0, 1]:
                    self._has_motion_cmd = False
                # print('old_state: {}, new_state: {}, has_motion_cmd={}, need_sync: {}'.format(self._state, state, self._has_motion_cmd, self._need_sync))
                self._state = state
                self._report_state_changed_callback()
            if mode != self._mode:
                self._mode = mode
                self._report_mode_changed_callback()

            mtbrake = [mtbrake & 0x01, mtbrake >> 1 & 0x01, mtbrake >> 2 & 0x01, mtbrake >> 3 & 0x01,
                       mtbrake >> 4 & 0x01, mtbrake >> 5 & 0x01, mtbrake >> 6 & 0x01, mtbrake >> 7 & 0x01]
            mtable = [mtable & 0x01, mtable >> 1 & 0x01, mtable >> 2 & 0x01, mtable >> 3 & 0x01,
                      mtable >> 4 & 0x01, mtable >> 5 & 0x01, mtable >> 6 & 0x01, mtable >> 7 & 0x01]

            if mtbrake != self._arm_motor_brake_states or mtable != self._arm_motor_enable_states:
                self._arm_motor_enable_states = mtable
                self._arm_motor_brake_states = mtbrake
                self._report_mtable_mtbrake_changed_callback()

            if not self._is_first_report:
                if state in [4, 5] or not all([bool(item[0] & item[1]) for item in zip(mtbrake, mtable)][:self.axis]):
                    # if self._is_ready:
                    #     pretty_print('[report], xArm is not ready to move', color='red')
                    self._is_ready = False
                else:
                    # if not self._is_ready:
                    #     pretty_print('[report], xArm is ready to move', color='green')
                    self._is_ready = True
            else:
                self._is_ready = False
            self._is_first_report = False
            if not self._is_ready:
                self._sleep_finish_time = 0

            self._error_code = error_code
            self._warn_code = warn_code
            self.arm_cmd.has_err_warn = error_code != 0 or warn_code != 0
            _state = self._state
            self._state = state
            if self.state != 3 and (_state == 3 or self._pause_cnts > 0):
                with self._pause_cond:
                    self._pause_cond.notifyAll()
            self._mode = mode
            self._cmd_num = cmd_num

            update_time = time.monotonic()
            self._last_update_cmdnum_time = update_time
            self._last_update_state_time = update_time
            self._last_update_err_time = update_time

            self._arm_motor_brake_states = mtbrake
            self._arm_motor_enable_states = mtable
            self._joints_torque = torque
            if compare_version(self.version_number, (0, 2, 0)):
                self._tcp_load = [float('{:.3f}'.format(tcp_load[0])), [float('{:.3f}'.format(i)) for i in tcp_load[1:]]]
            else:
                self._tcp_load = [float('{:.3f}'.format(tcp_load[0])), [float('{:.3f}'.format(i * 1000)) for i in tcp_load[1:]]]
            self._collision_sensitivity = collis_sens
            self._teach_sensitivity = teach_sens

            for i in range(len(pose)):
                pose[i] = filter_invaild_number(pose[i], 3 if i < 3 else 6, default=self._position[i])
            for i in range(len(angles)):
                angles[i] = filter_invaild_number(angles[i], 6, default=self._angles[i])
            for i in range(len(pose_offset)):
                pose_offset[i] = filter_invaild_number(pose_offset[i], 3 if i < 3 else 6, default=self._position_offset[i])

            if not (0 < self._error_code <= 17):
                self._position = pose
            if not (0 < self._error_code <= 17):
                self._angles = angles
            if not (0 < self._error_code <= 17):
                self._position_offset = pose_offset

            self._report_location_callback()

            self._report_callback()
            if not self._is_sync and self._error_code == 0 and self._state not in [4, 5]:
                self._sync()
                self._is_sync = True
            elif self._need_sync:
                self._need_sync = False
                self._sync()

        def __handle_report_rich(rx_data):
            # print('interval={}, max_interval={}'.format(interval, self._max_report_interval))
            __handle_report_normal(rx_data)
            (self._arm_type,
             arm_axis,
             self._arm_master_id,
             self._arm_slave_id,
             self._arm_motor_tid,
             self._arm_motor_fid) = rx_data[145:151]

            if 7 >= arm_axis >= 5:
                self._arm_axis = arm_axis

            # self._version = str(rx_data[151:180], 'utf-8')

            trs_msg = convert.bytes_to_fp32s(rx_data[181:201], 5)
            # trs_msg = [i[0] for i in trs_msg]
            (self._tcp_jerk,
             self._min_tcp_acc,
             self._max_tcp_acc,
             self._min_tcp_speed,
             self._max_tcp_speed) = trs_msg
            # print('tcp_jerk: {}, min_acc: {}, max_acc: {}, min_speed: {}, max_speed: {}'.format(
            #     self._tcp_jerk, self._min_tcp_acc, self._max_tcp_acc, self._min_tcp_speed, self._max_tcp_speed
            # ))

            p2p_msg = convert.bytes_to_fp32s(rx_data[201:221], 5)
            # p2p_msg = [i[0] for i in p2p_msg]
            (self._joint_jerk,
             self._min_joint_acc,
             self._max_joint_acc,
             self._min_joint_speed,
             self._max_joint_speed) = p2p_msg
            # print('joint_jerk: {}, min_acc: {}, max_acc: {}, min_speed: {}, max_speed: {}'.format(
            #     self._joint_jerk, self._min_joint_acc, self._max_joint_acc,
            #     self._min_joint_speed, self._max_joint_speed
            # ))

            rot_msg = convert.bytes_to_fp32s(rx_data[221:229], 2)
            # rot_msg = [i[0] for i in rot_msg]
            self._rot_jerk, self._max_rot_acc = rot_msg
            # print('rot_jerk: {}, mac_acc: {}'.format(self._rot_jerk, self._max_rot_acc))

            servo_codes = [val for val in rx_data[229:245]]
            for i in range(self.axis):
                if self._servo_codes[i][0] != servo_codes[i * 2] or self._servo_codes[i][1] != servo_codes[i * 2 + 1]:
                    print('servo_error_code, servo_id={}, status={}, code={}'.format(i + 1, servo_codes[i * 2], servo_codes[i * 2 + 1]))
                self._servo_codes[i][0] = servo_codes[i * 2]
                self._servo_codes[i][1] = servo_codes[i * 2 + 1]

            self._first_report_over = True

            # length = convert.bytes_to_u32(rx_data[0:4])
            length = len(rx_data)
            if length >= 252:
                temperatures = list(struct.unpack('>7b', struct.pack('>7B', *rx_data[245:252])))
                # temperatures = list(map(int, rx_data[245:252]))
                if temperatures != self.temperatures:
                    self._temperatures = temperatures
                    self._report_temperature_changed_callback()
            if length >= 284:
                speeds = convert.bytes_to_fp32s(rx_data[252:8 * 4 + 252], 8)
                self._realtime_tcp_speed = speeds[0]
                self._realtime_joint_speeds = speeds[1:]
                # print(speeds[0], speeds[1:])
            if length >= 288:
                count = convert.bytes_to_u32(rx_data[284:288])
                # print(count, rx_data[284:288])
                if self._count != -1 and count != self._count:
                    self._count = count
                    self._report_count_changed_callback()
                self._count = count
            if length >= 312:
                world_offset = convert.bytes_to_fp32s(rx_data[288:6 * 4 + 288], 6)
                for i in range(len(world_offset)):
                    if i < 3:
                        world_offset[i] = float('{:.3f}'.format(world_offset[i]))
                    else:
                        world_offset[i] = float('{:.6f}'.format(world_offset[i]))
                if math.inf not in world_offset and -math.inf not in world_offset and not (10 <= self._error_code <= 17):
                    self._world_offset = world_offset
            if length >= 314:
                self._cgpio_reset_enable, self._tgpio_reset_enable = rx_data[312:314]
            if length >= 417:
                self._is_simulation_robot = bool(rx_data[314])
                self._is_collision_detection, self._collision_tool_type = rx_data[315:317]
                self._collision_tool_params = convert.bytes_to_fp32s(rx_data[317:341], 6)

                voltages = convert.bytes_to_u16s(rx_data[341:355], 7)
                voltages = list(map(lambda x: x / 100, voltages))
                self._voltages = voltages

                currents = convert.bytes_to_fp32s(rx_data[355:383], 7)
                self._currents = currents

                cgpio_states = []
                cgpio_states.extend(rx_data[383:385])
                cgpio_states.extend(convert.bytes_to_u16s(rx_data[385:401], 8))
                cgpio_states[6:10] = list(map(lambda x: x / 4095.0 * 10.0, cgpio_states[6:10]))
                cgpio_states.append(list(map(int, rx_data[401:409])))
                cgpio_states.append(list(map(int, rx_data[409:417])))
                if self._control_box_type_is_1300 and length >= 433:
                    cgpio_states[-2].extend(list(map(int, rx_data[417:425])))
                    cgpio_states[-1].extend(list(map(int, rx_data[425:433])))
                self._cgpio_states = cgpio_states
            if length >= 481:
                # FT_SENSOR
                self._ft_ext_force = convert.bytes_to_fp32s(rx_data[433:457], 6)
                self._ft_raw_force = convert.bytes_to_fp32s(rx_data[457:481], 6)
            if length >= 482:
                iden_progress = rx_data[481]
                if iden_progress != self._iden_progress:
                    self._iden_progress = iden_progress
                    self._report_iden_progress_changed_callback()
            if length >= 494:
                pose_aa = convert.bytes_to_fp32s(rx_data[482:494], 3)
                for i in range(len(pose_aa)):
                    pose_aa[i] = filter_invaild_number(pose_aa[i], 6, default=self._pose_aa[i])
                self._pose_aa = self._position[:3] + pose_aa
            if length >= 495:
                self._is_reduced_mode = rx_data[494] & 0x01
                self._is_fence_mode = (rx_data[494] >> 1) & 0x01
                self._is_report_current = (rx_data[494] >> 2) & 0x01  # 针对get_report_tau_or_i的结果
                self._is_approx_motion = (rx_data[494] >> 3) & 0x01
                self._is_cart_continuous = (rx_data[494] >> 4) & 0x01
            if length >= 496:
                self._reduced_mode_is_on = rx_data[495]
                self._reduced_tcp_boundary = convert.bytes_to_16s(rx_data[496:508], 6)

        try:
            if self._report_type == 'real':
                __handle_report_real(data)
            elif self._report_type == 'rich':
                if self._is_old_protocol:
                    __handle_report_rich_old(data)
                else:
                    __handle_report_rich(data)
            else:
                if self._is_old_protocol:
                    __handle_report_normal_old(data)
                else:
                    __handle_report_normal(data)
        except Exception as e:
            logger.error(e)

    def _auto_get_report_thread(self):
        logger.debug('get report thread start')
        while self.connected:
            try:
                cmd_num = self._cmd_num
                state = self._state
                error_code = self._error_code
                warn_code = self._warn_code

                self.get_cmdnum()
                time.sleep(0.01)
                self.get_state()
                time.sleep(0.01)
                self.get_err_warn_code()
                time.sleep(0.01)
                self.get_servo_angle()
                time.sleep(0.01)
                self.get_position()

                if self.state != 3 and (state == 3 or self._pause_cnts > 0):
                    with self._pause_cond:
                        self._pause_cond.notifyAll()
                if cmd_num != self._cmd_num:
                    self._report_cmdnum_changed_callback()
                if state != self._state:
                    self._report_state_changed_callback()
                if state in [4, 5]:
                    # if self._is_ready:
                    #     pretty_print('[report], xArm is not ready to move', color='red')
                    self._sleep_finish_time = 0
                    self._is_ready = False
                else:
                    # if not self._is_ready:
                    #     pretty_print('[report], xArm is ready to move', color='green')
                    self._is_ready = True
                if error_code != self._error_code or warn_code != self._warn_code:
                    self._report_error_warn_changed_callback()
                elif not self._only_report_err_warn_changed and (self._error_code != 0 or self._warn_code != 0):
                    self._report_error_warn_changed_callback()

                self._report_location_callback()
                self._report_callback()

                if self._cmd_num >= self._max_cmd_num:
                    time.sleep(1)
                self._first_report_over = True
                time.sleep(0.1)
            except:
                pass
        self.disconnect()
        logger.debug('get report thread stopped')

    def _sync_tcp(self, index=None):
        if not self._stream_report or not self._stream_report.connected:
            self.get_position()
            self.get_servo_angle()
        self._last_angles = self._angles.copy()
        if index is None:
            self._last_position = self._position.copy()
        elif isinstance(index, int) and 0 <= index < 6:
            self._last_position[index] = self._position[index]
        # print('=============sync_tcp: index={}'.format(index))

    def _sync_joints(self, index=None):
        if not self._stream_report or not self._stream_report.connected:
            self.get_position()
            self.get_servo_angle()
        self._last_position = self._position.copy()
        if index is None:
            self._last_angles = self._angles.copy()
        elif isinstance(index, int) and 0 <= index < 7:
            self._last_angles[index] = self._angles[index]
        # print('=============sync_joint: index={}'.format(index))

    def _sync(self):
        if not self._stream_report or not self._stream_report.connected:
            self.get_position()
            self.get_servo_angle()
        self._last_position = self._position.copy()
        self._last_angles = self._angles.copy()
        # print('=============sync_all')

    def _set_params(self, **kwargs):
        is_radian = kwargs.get('is_radian', self._default_is_radian)
        if 'X' in kwargs and isinstance(kwargs['X'], (int, float)):
            self._last_position[0] = kwargs.get('X')
        if 'Y' in kwargs and isinstance(kwargs['Y'], (int, float)):
            self._last_position[1] = kwargs.get('Y')
        if 'Z' in kwargs and isinstance(kwargs['Z'], (int, float)):
            self._last_position[2] = kwargs.get('Z')
        if 'A' in kwargs and isinstance(kwargs['A'], (int, float)):
            self._last_position[3] = kwargs.get('A') if is_radian else math.radians(kwargs.get('A'))
        if 'B' in kwargs and isinstance(kwargs['B'], (int, float)):
            self._last_position[4] = kwargs.get('B') if is_radian else math.radians(kwargs.get('B'))
        if 'C' in kwargs and isinstance(kwargs['C'], (int, float)):
            self._last_position[5] = kwargs.get('C') if is_radian else math.radians(kwargs.get('C'))
        # if 'R' in kwargs and isinstance(kwargs['R'], (int, float)):
        #     self._last_position[6] = kwargs.get('R')
        if 'I' in kwargs and isinstance(kwargs['I'], (int, float)):
            self._last_angles[0] = kwargs.get('I') if is_radian else math.radians(kwargs.get('I'))
        if 'J' in kwargs and isinstance(kwargs['J'], (int, float)):
            self._last_angles[1] = kwargs.get('J') if is_radian else math.radians(kwargs.get('J'))
        if 'K' in kwargs and isinstance(kwargs['K'], (int, float)):
            self._last_angles[2] = kwargs.get('K') if is_radian else math.radians(kwargs.get('K'))
        if 'L' in kwargs and isinstance(kwargs['L'], (int, float)):
            self._last_angles[3] = kwargs.get('L') if is_radian else math.radians(kwargs.get('L'))
        if 'M' in kwargs and isinstance(kwargs['M'], (int, float)):
            self._last_angles[4] = kwargs.get('M') if is_radian else math.radians(kwargs.get('M'))
        if 'N' in kwargs and isinstance(kwargs['N'], (int, float)):
            self._last_angles[5] = kwargs.get('N') if is_radian else math.radians(kwargs.get('N'))
        if 'O' in kwargs and isinstance(kwargs['O'], (int, float)):
            self._last_angles[6] = kwargs.get('O') if is_radian else math.radians(kwargs.get('O'))

        if 'F' in kwargs and isinstance(kwargs['F'], (int, float)):
            self._last_tcp_speed = kwargs.get('F')
            self._last_tcp_speed = min(max(self._last_tcp_speed, self._min_tcp_speed), self._max_tcp_speed)
        if 'Q' in kwargs and isinstance(kwargs['Q'], (int, float)):
            self._last_tcp_acc = kwargs.get('Q')
            self._last_tcp_acc = min(max(self._last_tcp_acc, self._min_tcp_acc), self._max_tcp_acc)
        if 'F2' in kwargs and isinstance(kwargs['F2'], (int, float)):
            self._last_joint_speed = kwargs.get('F2')
            if not is_radian:
                self._last_joint_speed = math.radians(self._last_joint_speed)
            self._last_joint_speed = min(max(self._last_joint_speed, self._min_joint_speed), self._max_joint_speed)
        if 'Q2' in kwargs and isinstance(kwargs['Q2'], (int, float)):
            self._last_joint_acc = kwargs.get('Q2')
            if not is_radian:
                self._last_joint_acc = math.radians(self._last_joint_acc)
            self._last_joint_acc = min(max(self._last_joint_acc, self._min_joint_acc), self._max_joint_acc)
        if 'T' in kwargs and isinstance(kwargs['T'], (int, float)):
            self._mvtime = kwargs.get('T')
        if 'LIMIT_VELO' in kwargs and isinstance(kwargs['LIMIT_VELO'], (list, tuple)) \
                and len(kwargs['LIMIT_VELO']) == 2 and isinstance(kwargs['LIMIT_VELO'][0], (int, float)) \
                and isinstance(kwargs['LIMIT_VELO'][1], (int, float)) \
                and kwargs['LIMIT_VELO'][0] <= kwargs['LIMIT_VELO'][1]:
            self._min_tcp_speed, self._max_tcp_speed = kwargs.get('LIMIT_VELO')
        if 'LIMIT_ACC' in kwargs and isinstance(kwargs['LIMIT_ACC'], (list, tuple)) \
                and len(kwargs['LIMIT_ACC']) == 2 and isinstance(kwargs['LIMIT_ACC'][0], (int, float)) \
                and isinstance(kwargs['LIMIT_ACC'][1], (int, float)) \
                and kwargs['LIMIT_ACC'][0] <= kwargs['LIMIT_ACC'][1]:
            self._min_tcp_acc, self._max_tcp_acc = kwargs.get('LIMIT_ACC')

    def _get_params(self, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if is_radian:
            return {
                'lastPosition': self._last_position,
                'lastAngles': self._last_angles,
                'mvvelo': self._last_tcp_speed,
                'mvacc': self._last_tcp_acc,
                'tcpJerk': self._tcp_jerk,
                'jointJerk': self._joint_jerk,
                'angle_mvvelo': self._last_joint_speed,
                'angle_mvacc': self._last_joint_acc,
                'mvtime': self._mvtime,
                'LIMIT_VELO': [self._min_tcp_speed, self._max_tcp_speed],
                'LIMIT_ACC': [self._min_tcp_acc, self._max_tcp_acc],
                'LIMIT_ANGLE_VELO': [self._min_joint_speed, self._max_joint_speed],
                'LIMIT_ANGLE_ACC': [self._min_joint_acc, self._max_joint_acc],
            }
        else:
            return {
                'lastPosition': [math.degrees(self._last_position[i]) if 2 < i < 6 else self._last_position[i] for i in range(len(self._last_position))],
                'lastAngles': [math.degrees(angle) for angle in self._last_angles],
                'mvvelo': round(self._last_tcp_speed),
                'mvacc': round(self._last_tcp_acc),
                'tcpJerk': round(self._tcp_jerk),
                'jointJerk': round(math.degrees(self._joint_jerk)),
                'angle_mvvelo': round(math.degrees(self._last_joint_speed)),
                'angle_mvacc': round(math.degrees(self._last_joint_acc)),
                'mvtime': self._mvtime,
                'LIMIT_VELO': list(map(round, [self._min_tcp_speed, self._max_tcp_speed])),
                'LIMIT_ACC': list(map(round, [self._min_tcp_acc, self._max_tcp_acc])),
                'LIMIT_ANGLE_VELO': list(map(round, [math.degrees(self._min_joint_speed), math.degrees(self._max_joint_speed)])),
                'LIMIT_ANGLE_ACC': list(map(round, [math.degrees(self._min_joint_acc), math.degrees(self._max_joint_acc)])),
            }

    def _check_code(self, code, is_move_cmd=False, mode=-1):
        if is_move_cmd:
            if code in [0, XCONF.UxbusState.WAR_CODE]:
                if self.arm_cmd.state_is_ready:
                    if mode >= 0 and mode != self.mode:
                        logger.warn('The mode may be incorrect, just as a reminder, mode: {} ({})'.format(mode, self.mode))
                    return 0
                    # return 0 if mode < 0 or mode == self.mode else APIState.MODE_IS_NOT_CORRECT
                else:
                    return XCONF.UxbusState.STATE_NOT_READY
            else:
                return code
            # return 0 if code in [0, XCONF.UxbusState.WAR_CODE] and self.arm_cmd.state_is_ready else XCONF.UxbusState.STATE_NOT_READY if not self.arm_cmd.state_is_ready else code
        else:
            return 0 if code in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.STATE_NOT_READY] else code

    def _check_mode_is_correct(self, mode, timeout=1):
        if self._enable_report and self._stream_type == 'socket':
            cnt = int(10 * timeout)
            while cnt > 0 and self.mode != mode:
                time.sleep(0.1)
                cnt -= 1
            if self.mode != mode:
                return False
        return True

    @xarm_is_connected(_type='get')
    def get_version(self):
        ret = self.arm_cmd.get_version()
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            version = ''.join(list(map(chr, ret[1:])))
            self._version = version[:version.find('\0')]
        return ret[0], self._version

    @xarm_is_connected(_type='get')
    def get_robot_sn(self):
        ret = self.arm_cmd.get_robot_sn()
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            robot_sn = ''.join(list(map(chr, ret[1:])))
            split_inx = robot_sn.find('\0')
            self._robot_sn = robot_sn[:split_inx]
            control_box_sn = robot_sn[split_inx+1:]
            self._control_box_sn = control_box_sn[:control_box_sn.find('\0')].strip()
            self._arm_type_is_1300 = int(self._robot_sn[2:6]) >= 1300 if self._robot_sn[2:6].isdigit() else False
            self._control_box_type_is_1300 = int(self._control_box_sn[2:6]) >= 1300 if self._control_box_sn[2:6].isdigit() else False
        return ret[0], self._robot_sn

    @xarm_is_connected(_type='get')
    def check_verification(self):
        ret = self.arm_cmd.check_verification()
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1]

    @xarm_is_connected(_type='get')
    def get_position(self, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        ret = self.arm_cmd.get_tcp_pose()
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0 and len(ret) > 6:
            self._position = [filter_invaild_number(ret[i], 6, default=self._position[i-1]) for i in range(1, 7)]
        return ret[0], [float(
            '{:.6f}'.format(math.degrees(self._position[i]) if 2 < i < 6 and not is_radian else self._position[i])) for
                        i in range(len(self._position))]

    @xarm_is_connected(_type='get')
    def get_servo_angle(self, servo_id=None, is_radian=None, is_real=False):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if is_real and self.version_is_ge(1, 9, 110):
            ret = self.arm_cmd.get_joint_states(num=1)
        else:
            ret = self.arm_cmd.get_joint_pos()
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0 and len(ret) > 7:
            self._angles = [filter_invaild_number(ret[i], 6, default=self._angles[i-1]) for i in range(1, 8)]
        if servo_id is None or servo_id == 8 or len(self._angles) < servo_id:
            return ret[0], list(
                map(lambda x: float('{:.6f}'.format(x if is_radian else math.degrees(x))), self._angles))
        else:
            return ret[0], float(
                '{:.6f}'.format(self._angles[servo_id - 1] if is_radian else math.degrees(self._angles[servo_id - 1])))

    @xarm_is_connected(_type='get')
    def get_joint_states(self, is_radian=None, num=3):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        ret = self.arm_cmd.get_joint_states(num=num)
        ret[0] = self._check_code(ret[0])
        positon = ret[1:8]
        result = [positon]
        if num >= 2:
            velocity = ret[8:15]
            result.append(velocity)
        if num >= 3:
            effort = ret[15:22]
            result.append(effort)
        if ret[0] == 0:
            if not is_radian:
                for i in range(7):
                    positon[i] = math.degrees(positon[i])
                    if num >= 2:
                        velocity[i] = math.degrees(velocity[i])
        return ret[0], result

    @xarm_is_connected(_type='get')
    def get_position_aa(self, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        ret = self.arm_cmd.get_position_aa()
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0 and len(ret) > 6:
            self._pose_aa = [filter_invaild_number(ret[i], 6, default=self._pose_aa[i - 1]) for i in range(1, 7)]
        return ret[0], [float(
            '{:.6f}'.format(math.degrees(self._pose_aa[i]) if 2 < i < 6 and not is_radian else self._pose_aa[i]))
            for i in range(len(self._pose_aa))]

    @xarm_is_connected(_type='get')
    def get_pose_offset(self, pose1, pose2, orient_type_in=0, orient_type_out=0, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        _pose1 = [pose1[i] if i <= 2 or is_radian else math.radians(pose1[i]) for i in range(6)]
        _pose2 = [pose2[i] if i <= 2 or is_radian else math.radians(pose2[i]) for i in range(6)]
        ret = self.arm_cmd.get_pose_offset(_pose1, _pose2, orient_type_in, orient_type_out)
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0 and len(ret) > 6:
            pose = [float('{:.6f}'.format(ret[i] if i <= 3 or is_radian else math.degrees(ret[i]))) for i in
                    range(1, 7)]
            return ret[0], pose
        return ret[0], ret[1:7]

    def get_is_moving(self):
        self.get_state()
        return self._state == 1

    @xarm_is_connected(_type='get')
    def get_state(self):
        ret = self.arm_cmd.get_state()
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            # if ret[1] != self._state:
            #     self._state = ret[1]
            #     self._report_state_changed_callback()
            self._state = ret[1]
            self._last_update_state_time = time.monotonic()
        return ret[0], ret[1] if ret[0] == 0 else self._state

    @xarm_is_connected(_type='set')
    def set_state(self, state=0):
        _state = self._state
        ret = self.arm_cmd.set_state(state)
        ret[0] = self._check_code(ret[0])
        if state == 4 and ret[0] == 0:
            # self._last_position[:6] = self.position
            # self._last_angles = self.angles
            self._sleep_finish_time = 0
            # self._is_sync = False
        self.get_state()
        if _state != self._state:
            self._report_state_changed_callback()
        if self.state != 3 and (_state == 3 or self._pause_cnts > 0):
            with self._pause_cond:
                self._pause_cond.notifyAll()
        if self._state in [4, 5]:
            self._sleep_finish_time = 0
            if self._is_ready:
                pretty_print('[set_state], xArm is not ready to move', color='red')
            self._is_ready = False
        else:
            if not self._is_ready:
                pretty_print('[set_state], xArm is ready to move', color='green')
            self._is_ready = True
        self.log_api_info('API -> set_state({}) -> code={}, state={}'.format(state, ret[0], self._state), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_mode(self, mode=0, detection_param=0):
        if self.version_is_ge(1, 10, 0):
            detection_param = detection_param if detection_param >= 0 else 0
        else:
            detection_param = -1
        ret = self.arm_cmd.set_mode(mode, detection_param=detection_param)
        ret[0] = self._check_code(ret[0])
        self.log_api_info('API -> set_mode({}) -> code={}'.format(mode, ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_cmdnum(self):
        ret = self.arm_cmd.get_cmdnum()
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            if ret[1] != self._cmd_num:
                self._report_cmdnum_changed_callback()
            self._cmd_num = ret[1]
            self._last_update_cmdnum_time = time.monotonic()
        return ret[0], self._cmd_num

    @xarm_is_connected(_type='get')
    def get_err_warn_code(self, show=False, lang='en'):
        ret = self.arm_cmd.get_err_code()
        lang = lang if lang == 'cn' else 'en'
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            # if ret[1] != self._error_code or ret[2] != self._warn_code:
            #     self._error_code, self._warn_code = ret[1:3]
            #     self._report_error_warn_changed_callback()

            self._error_code, self._warn_code = ret[1:3]
            self._last_update_err_time = time.monotonic()
        if show:
            pretty_print('************* {}, {}: {} **************'.format(
                         '获取控制器错误警告码' if lang == 'cn' else 'GetErrorWarnCode',
                         '状态' if lang == 'cn' else 'Status',
                         ret[0]), color='light_blue')
            controller_error = ControllerError(self._error_code, status=0)
            controller_warn = ControllerWarn(self._warn_code, status=0)
            pretty_print('* {}: {}, {}: {}'.format(
                '错误码' if lang == 'cn' else 'ErrorCode',
                controller_error.code,
                '信息' if lang == 'cn' else 'Info',
                controller_error.title[lang]),
                         color='red' if self._error_code != 0 else 'white')
            pretty_print('* {}: {}, {}: {}'.format(
                '警告码' if lang == 'cn' else 'WarnCode',
                controller_warn.code,
                '信息' if lang == 'cn' else 'Info',
                controller_warn.title[lang]),
                         color='yellow' if self._warn_code != 0 else 'white')
            pretty_print('*' * 50, color='light_blue')
        return ret[0], ret[1:3] if ret[0] == 0 else [self._error_code, self._warn_code]

    @xarm_is_connected(_type='set')
    def clean_error(self):
        ret = self.arm_cmd.clean_err()
        self.get_state()
        if self._state in [4, 5]:
            self._sleep_finish_time = 0
            if self._is_ready:
                pretty_print('[clean_error], xArm is not ready to move', color='red')
            self._is_ready = False
        else:
            if not self._is_ready:
                pretty_print('[clean_error], xArm is ready to move', color='green')
            self._is_ready = True
        self.log_api_info('API -> clean_error -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def clean_warn(self):
        ret = self.arm_cmd.clean_war()
        self.log_api_info('API -> clean_warn -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    @xarm_is_not_simulation_mode(ret=0)
    def motion_enable(self, enable=True, servo_id=None):
        assert servo_id is None or (isinstance(servo_id, int) and 1 <= servo_id <= 8)
        if servo_id is None or servo_id == 8:
            ret = self.arm_cmd.motion_en(8, int(enable))
        else:
            ret = self.arm_cmd.motion_en(servo_id, int(enable))
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            self._is_ready = bool(enable)
        self.get_state()
        if self._state in [4, 5]:
            self._sleep_finish_time = 0
            if self._is_ready:
                pretty_print('[motion_enable], xArm is not ready to move', color='red')
            self._is_ready = False
        else:
            if not self._is_ready:
                pretty_print('[motion_enable], xArm is ready to move', color='green')
            self._is_ready = True
        self.log_api_info('API -> motion_enable -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]
    
    def _gen_feedback_key(self, wait, **kwargs):
        feedback_key = kwargs.get('feedback_key', '') if self._support_feedback and not wait else ''
        studio_wait = bool(feedback_key)
        feedback_key = str(uuid.uuid1()) if wait and self._support_feedback else feedback_key
        # feedback_key = str(uuid.uuid1()) if wait and self._support_feedback else ''
        self._fb_key_transid_map[feedback_key if feedback_key else 'no_use'] = -1
        return feedback_key, studio_wait
    
    def _get_feedback_transid(self, feedback_key, studio_wait=False, is_pop=True):
        return (self._fb_key_transid_map.pop(feedback_key, -1) if is_pop else self._fb_key_transid_map.get(feedback_key, -1)) if not studio_wait else -1
    
    def _set_feedback_key_tranid(self, feedback_key, trans_id, feedback_type=0):
        self._fb_key_transid_map[feedback_key] = trans_id
        self._fb_transid_type_map[trans_id] = feedback_type
        self._fb_transid_result_map.pop(trans_id, -1)
    
    def _wait_feedback(self, timeout=None, trans_id=-1, ignore_log=False):
        if timeout is not None:
            expired = time.monotonic() + timeout + (self._sleep_finish_time if self._sleep_finish_time > time.monotonic() else 0)
        else:
            expired = 0
        state5_cnt = 0
        while timeout is None or time.monotonic() < expired:
            if not self.connected:
                self._fb_transid_result_map.clear()
                if not ignore_log:
                    self.log_api_info('wait_feedback, xarm is disconnect', code=APIState.NOT_CONNECTED)
                return APIState.NOT_CONNECTED, -1
            if self.error_code != 0:
                self._fb_transid_result_map.clear()
                if not ignore_log:
                    self.log_api_info('wait_feedback, xarm has error, error={}'.format(self.error_code), code=APIState.HAS_ERROR)
                return APIState.HAS_ERROR, -1
            code, state = self.get_state()
            if code != 0:
                return code, -1
            if state >= 4:
                self._sleep_finish_time = 0
                if state == 5:
                    state5_cnt += 1
                if state != 5 or state5_cnt >= 20:
                    self._fb_transid_result_map.clear()
                    if not ignore_log:
                        self.log_api_info('wait_feedback, xarm is stop, state={}'.format(state), code=APIState.EMERGENCY_STOP)
                    return APIState.EMERGENCY_STOP, -1
            else:
                state5_cnt = 0
            if trans_id in self._fb_transid_result_map:
                return 0, self._fb_transid_result_map.pop(trans_id, -1)
            time.sleep(0.05)
        return APIState.WAIT_FINISH_TIMEOUT, -1
    
    def wait_move(self, timeout=None, trans_id=-1, set_cnt=2):
        if self._support_feedback and trans_id > 0:
            return self._wait_feedback(timeout, trans_id)[0]
        if timeout is not None:
            expired = time.monotonic() + timeout + (self._sleep_finish_time if self._sleep_finish_time > time.monotonic() else 0)
        else:
            expired = 0
        _, state = self.get_state()
        cnt = 0
        state5_cnt = 0
        max_cnt = set_cnt if _ == 0 and state == 1 else 10
        while timeout is None or time.monotonic() < expired:
            if not self.connected:
                self.log_api_info('wait_move, xarm is disconnect', code=APIState.NOT_CONNECTED)
                return APIState.NOT_CONNECTED
            if self.error_code != 0:
                self.log_api_info('wait_move, xarm has error, error={}'.format(self.error_code), code=APIState.HAS_ERROR)
                return APIState.HAS_ERROR
            if self.mode != 0 and self.mode != 11:
                return 0
            code, state = self.get_state()
            if code != 0:
                return code
            if state >= 4:
                self._sleep_finish_time = 0
                if state == 5:
                    state5_cnt += 1
                if state != 5 or state5_cnt >= 20:
                    self.log_api_info('wait_move, xarm is stop, state={}'.format(state), code=APIState.EMERGENCY_STOP)
                    return APIState.EMERGENCY_STOP
            else:
                state5_cnt = 0
            if time.monotonic() < self._sleep_finish_time or state == 3:
                cnt = 0
                max_cnt = 2 if state == 3 else max_cnt
                time.sleep(0.05)
                continue
            if state == 0 or state == 1:
                cnt = 0
                max_cnt = set_cnt
                time.sleep(0.05)
                continue
            else:
                cnt += 1
                if cnt >= max_cnt:
                    return 0
                time.sleep(0.05)
        return APIState.WAIT_FINISH_TIMEOUT

    @xarm_is_connected(_type='set')
    def _check_modbus_code(self, ret, length=2, only_check_code=False, host_id=XCONF.TGPIO_HOST_ID):
        code = ret[0]
        if self._check_code(code) == 0:
            if not only_check_code:
                if len(ret) < length:
                    return APIState.MODBUS_ERR_LENG
                if ret[1] != host_id:
                    return APIState.HOST_ID_ERR
            if code != 0:
                if host_id == XCONF.TGPIO_HOST_ID:
                    if self.error_code != 19 and self.error_code != 28:
                        self.get_err_warn_code()
                    if self.error_code != 19 and self.error_code != 28:
                        code = 0
                else:
                    if self.error_code != 100 + host_id:
                        self.get_err_warn_code()
                    if self.error_code != 100 + host_id:
                        code = 0
        return code

    @xarm_is_connected(_type='set')
    def checkset_modbus_baud(self, baudrate, check=True, host_id=XCONF.TGPIO_HOST_ID):
        if check and (not self._baud_checkset or baudrate <= 0):
            return 0
        if check and ((host_id == XCONF.TGPIO_HOST_ID and self.modbus_baud == baudrate) or (host_id == XCONF.LINEER_TRACK_HOST_ID and self.linear_track_baud == baudrate)):
            return 0
        if baudrate not in self.arm_cmd.BAUDRATES:
            return APIState.MODBUS_BAUD_NOT_SUPPORT
        ret, cur_baud_inx = self._get_modbus_baudrate_inx(host_id=host_id)
        if ret == 0:
            baud_inx = self.arm_cmd.BAUDRATES.index(baudrate)
            if cur_baud_inx != baud_inx:
                try:
                    self._ignore_error = True
                    self._ignore_state = True if self.state not in [4, 5] else False
                    state = self.state
                    # self.arm_cmd.tgpio_addr_w16(XCONF.ServoConf.MODBUS_BAUDRATE, baud_inx)
                    self.arm_cmd.tgpio_addr_w16(0x1A0B, baud_inx, bid=host_id)
                    time.sleep(0.3)
                    if host_id != XCONF.LINEER_TRACK_HOST_ID:
                        self.arm_cmd.tgpio_addr_w16(XCONF.ServoConf.SOFT_REBOOT, 1, bid=host_id)
                    if host_id == XCONF.TGPIO_HOST_ID:
                        if self.error_code != 19 and self.error_code != 28:
                            self.get_err_warn_code()
                        if self.error_code == 19 or self.error_code == 28:
                            self.clean_error()
                            if self._ignore_state:
                                self.set_state(state if state >= 3 else 0)
                        time.sleep(1)
                    else:
                        if self.error_code != 100 + host_id:
                            self.get_err_warn_code()
                        if self.error_code == 100 + host_id:
                            self.clean_error()
                            if self._ignore_state:
                                self.set_state(state if state >= 3 else 0)
                        time.sleep(1)
                except Exception as e:
                    self._ignore_error = False
                    self._ignore_state = False
                    logger.error('checkset_modbus_baud error: {}'.format(e))
                    return APIState.API_EXCEPTION
                self._ignore_error = False
                self._ignore_state = False
                ret, cur_baud_inx = self._get_modbus_baudrate_inx(host_id=host_id)
                self.log_api_info('API -> checkset_modbus_baud -> code={}, baud_inx={}'.format(ret, cur_baud_inx), code=ret)
            # if ret == 0 and cur_baud_inx < len(self.arm_cmd.BAUDRATES):
            #     self.modbus_baud = self.arm_cmd.BAUDRATES[cur_baud_inx]
        if host_id == XCONF.TGPIO_HOST_ID:
            return 0 if self.modbus_baud == baudrate else APIState.MODBUS_BAUD_NOT_CORRECT
        elif host_id == XCONF.LINEER_TRACK_HOST_ID:
            return 0 if self.linear_track_baud == baudrate else APIState.MODBUS_BAUD_NOT_CORRECT
        else:
            if ret == 0 and 0 <= cur_baud_inx < len(self.arm_cmd.BAUDRATES):
                return 0 if self.arm_cmd.BAUDRATES[cur_baud_inx] == baudrate else APIState.MODBUS_BAUD_NOT_CORRECT
            return APIState.MODBUS_BAUD_NOT_CORRECT

    @xarm_is_connected(_type='get')
    def _get_modbus_baudrate_inx(self, host_id=XCONF.TGPIO_HOST_ID):
        ret = self.arm_cmd.tgpio_addr_r16(XCONF.ServoConf.MODBUS_BAUDRATE & 0x0FFF, bid=host_id)
        if ret[0] in [XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            if host_id == XCONF.TGPIO_HOST_ID:
                if self.error_code != 19 and self.error_code != 28:
                    self.get_err_warn_code()
                if self.error_code != 19 and self.error_code != 28:
                    ret[0] = 0
            else:
                if self.error_code != 100 + host_id:
                    self.get_err_warn_code()
                if self.error_code != 100 + host_id:
                    ret[0] = 0
        if ret[0] == 0 and 0 <= ret[1] < len(self.arm_cmd.BAUDRATES):
            if host_id == XCONF.TGPIO_HOST_ID:
                self.modbus_baud = self.arm_cmd.BAUDRATES[ret[1]]
            elif host_id == XCONF.LINEER_TRACK_HOST_ID:
                self.linear_track_baud = self.arm_cmd.BAUDRATES[ret[1]]
        return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_tgpio_modbus_timeout(self, timeout, is_transparent_transmission=False, **kwargs):
        ret = self.arm_cmd.set_modbus_timeout(timeout, is_transparent_transmission=kwargs.get('is_tt', is_transparent_transmission))
        self.log_api_info('API -> set_tgpio_modbus_timeout -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_tgpio_modbus_baudrate(self, baud):
        code = self.checkset_modbus_baud(baud, check=False)
        self.log_api_info('API -> set_tgpio_modbus_baudrate -> code={}'.format(code), code=code)
        return code

    @xarm_is_connected(_type='get')
    def get_tgpio_modbus_baudrate(self):
        code, baud_inx = self._get_modbus_baudrate_inx()
        # if code == 0 and baud_inx < len(self.arm_cmd.BAUDRATES):
        #     self.modbus_baud = self.arm_cmd.BAUDRATES[baud_inx]
        return code, self.modbus_baud
    
    @xarm_is_connected(_type='set')
    def set_control_modbus_baudrate(self, baud):
        code = self.checkset_modbus_baud(baud, check=False, host_id=XCONF.LINEER_TRACK_HOST_ID)
        self.log_api_info('API -> set_control_modbus_baudrate -> code={}'.format(code), code=code)
        return code
    
    def getset_tgpio_modbus_data(self, datas, min_res_len=0, ignore_log=False, host_id=XCONF.TGPIO_HOST_ID, is_transparent_transmission=False, use_503_port=False, **kwargs):
        if not self.connected:
            return APIState.NOT_CONNECTED, []
        is_tt = kwargs.get('is_tt', is_transparent_transmission)
        if is_tt:
            if use_503_port:
                if not self.connected_503 and self.connect_503() != 0:
                    return APIState.NOT_CONNECTED, []
                ret = self.arm_cmd_503.tgpio_set_modbus(datas, len(datas), host_id=host_id, is_transparent_transmission=True)
            else:
                ret = self.arm_cmd.tgpio_set_modbus(datas, len(datas), host_id=host_id, is_transparent_transmission=True)
        else:
            ret = self.arm_cmd.tgpio_set_modbus(datas, len(datas), host_id=host_id)
        ret[0] = self._check_modbus_code(ret, min_res_len + 2, host_id=host_id)
        if not ignore_log:
            self.log_api_info('API -> getset_tgpio_modbus_data -> code={}, response={}'.format(ret[0], ret[2:]), code=ret[0])
        return ret[0], ret[2:]

    @xarm_is_connected(_type='set')
    def set_simulation_robot(self, on_off):
        ret = self.arm_cmd.set_simulation_robot(on_off)
        ret[0] = self._check_code(ret[0])
        self.log_api_info('API -> set_simulation_robot({}) -> code={}'.format(on_off, ret[0]), code=ret[0])
        return ret[0]
    
    @xarm_wait_until_not_pause
    @xarm_wait_until_cmdnum_lt_max
    @xarm_is_ready(_type='set')
    def set_tcp_load(self, weight, center_of_gravity, wait=False, **kwargs):
        if compare_version(self.version_number, (0, 2, 0)):
            _center_of_gravity = center_of_gravity
        else:
            _center_of_gravity = [item / 1000.0 for item in center_of_gravity]
        feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
        ret = self.arm_cmd.set_tcp_load(weight, _center_of_gravity, feedback_key=feedback_key)
        trans_id = self._get_feedback_transid(feedback_key, studio_wait)
        ret[0] = self._check_code(ret[0], is_move_cmd=True)
        self.log_api_info('API -> set_tcp_load -> code={}, weight={}, center={}'.format(ret[0], weight, _center_of_gravity), code=ret[0])
        if wait and ret[0] == 0:
            return self.wait_move(None, trans_id=trans_id)
        return ret[0]

    def set_only_check_type(self, only_check_type):
        self._only_check_type = only_check_type if only_check_type in [0, 1, 2, 3] else 0

    def get_dh_params(self):
        ret = self.arm_cmd.get_dh_params()
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1:]
    
    def set_dh_params(self, dh_params, flag=0):
        if len(dh_params) < 28:
            dh_params.extend([0] * (28 - len(dh_params)))
        ret = self.arm_cmd.set_dh_params(dh_params, flag)
        ret[0] = self._check_code(ret[0])
        return ret[0]
    
    def _feedback_thread_handle(self):
        while self.connected:
            try:
                data = self._feedback_que.get(timeout=1)
            except:
                continue
            self._feedback_callback(data)
    
    def _feedback_callback(self, data):
        trans_id = convert.bytes_to_u16(data[0:2])
        feedback_type = self._fb_transid_type_map.pop(trans_id, -1)
        if feedback_type != -1:
            self._fb_transid_result_map[trans_id] = data[12]  # feedback_code
        if feedback_type & data[8] == 0:
            return
        self.__report_callback(self.FEEDBACK_ID, data, name='feedback')
    
    def set_feedback_type(self, feedback_type):
        if not self._support_feedback:
            return APIState.CMD_NOT_EXIST
        ret = self.arm_cmd.set_feedback_type(feedback_type)
        ret[0] = self._check_code(ret[0])
        return ret[0]
    
    def set_common_param(self, param_type, param_val):
        ret = self.arm_cmd.set_common_param(param_type, param_val)
        ret[0] = self._check_code(ret[0])
        return ret[0]
    
    def get_common_param(self, param_type, return_val=True):
        ret = self.arm_cmd.get_common_param(param_type) 
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1] if return_val else ret[1:]
    
    def get_common_info(self, param_type, return_val=True):
        ret = self.arm_cmd.get_common_info(param_type) 
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1] if return_val else ret[1:]
        
    def get_traj_speeding(self, rate):
        ret = self.arm_cmd.get_traj_speeding(rate)
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1:]