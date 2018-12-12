#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import re
import math
import time
import threading
from collections import Iterable
from ..core.comm import SerialPort, SocketPort
from ..core.config.x_config import XCONF
from ..core.wrapper import UxbusCmdSer, UxbusCmdTcp
from ..core.utils import convert
from ..core.utils.log import logger, pretty_print
from ..core.config.x_code import ControllerWarn, ControllerError, ServoError
from .gripper import Gripper
from . import parse
from .code import APIState
from .utils import xarm_is_connected, xarm_is_ready

RAD_DEGREE = 57.295779513082320876798154814105
LIMIT_VELO = [1, 1000]  # mm/s
LIMIT_ACC = [1, 20000]  # mm/s^2
LIMIT_ANGLE_VELO = [1, 180]  # °/s
LIMIT_ANGLE_ACC = [1, 3600]  # °/s^2

LIMIT_TCP_ROTATE = [
    (-math.pi, math.pi),
    (-math.pi, math.pi),
    (-math.pi, math.pi)
]
LIMIT_JOINTS = [
    (-2 * math.pi, 2 * math.pi),
    (-2.18, 2.18),
    (-2 * math.pi, 2 * math.pi),
    (-4.01, 0.2),
    (-2 * math.pi, 2 * math.pi),
    (-1.79, math.pi),
    (-2 * math.pi, 2 * math.pi)
]
MAX_CMD_NUM = 256

REPORT_ID = 'REPORT'
REPORT_LOCATION_ID = 'LOCATION'

REPORT_CONNECT_CHANGED_ID = 'REPORT_CONNECT_CHANGED'
REPORT_STATE_CHANGED_ID = 'REPORT_STATE_CHANGED'
REPORT_MAABLE_MTBRAKE_CHANGED_ID = 'REPORT_MAABLE_MTBRAKE_CHANGED'
REPORT_ERROR_WARN_CHANGED_ID = 'REPORT_ERROR_WARN_CHANGED'
REPORT_CMDNUM_CHANGED_ID = 'REPORT_CMDNUM_CHANGED'


class XArm(Gripper):
    def __init__(self, port=None, baudrate=921600, timeout=None, filters=None,
                 enable_heartbeat=True, enable_report=True, report_type='normal', do_not_open=False,
                 limit_velo=None, limit_acc=None, limit_angle_velo=None, limit_angle_acc=None, is_radian=False):
        super(XArm, self).__init__()
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._filters = filters
        self._enable_heartbeat = enable_heartbeat
        self._enable_report = enable_report
        self._report_type = report_type

        self._min_velo, self._max_velo = limit_velo if limit_velo is not None and len(limit_velo) >= 2 else LIMIT_VELO
        self._min_acc, self._max_acc = limit_acc if limit_acc is not None and len(limit_acc) >= 2 else LIMIT_ACC

        self._min_angle_velo, self._max_angle_velo = list(map(lambda x: x / RAD_DEGREE, LIMIT_ANGLE_VELO))
        self._min_angle_acc, self._max_angle_acc = list(map(lambda x: x / RAD_DEGREE, LIMIT_ANGLE_ACC))
        if limit_angle_velo is not None and len(limit_angle_velo) >= 2:
            if not is_radian:
                limit_angle_velo = list(map(lambda x: x / RAD_DEGREE, limit_angle_velo))
            self._min_angle_velo, self._max_angle_velo = limit_angle_velo
        if limit_angle_acc is not None and len(limit_angle_acc) >= 2:
            if not is_radian:
                limit_angle_acc = list(map(lambda x: x / RAD_DEGREE, limit_angle_acc))
            self._min_angle_acc, self._max_angle_acc = limit_angle_acc

        self._stream_type = 'serial'
        self._stream = None
        self.arm_cmd = None
        self._stream_report = None
        self._report_thread = None
        self._only_report_err_warn_changed = True

        self._last_position = [201.5, 0, 140.5, -3.1415926, 0, 0]  # [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]
        self._last_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [servo_1(rad), servo_2(rad), servo_3(rad), servo_4(rad), servo_5(rad), servo_6(rad), servo_7(rad)]
        self._mvvelo = 100  # mm/s, rad/s
        self._mvacc = 2000  # mm/s^2, rad/s^2
        self._angle_mvvelo = 0.3490658503988659  # 20 °/s
        self._angle_mvacc = 8.726646259971648  # 500 °/s^2
        self._mvtime = 0

        self._version = None
        self._position = [0] * 6
        self._angles = [0] * 7
        self._position_offset = [0] * 6
        self._state = 4
        self._error_code = 0
        self._warn_code = 0
        self._mtbrake = [0, 0, 0, 0, 0, 0, 0, 0]  # [serve_1_brake, serve_2_brake, serve_3_brake, serve_4_brake, serve_5_brake, serve_6_brake, serve_7_brake, reserved]
        self._maable = [0, 0, 0, 0, 0, 0, 0, 0]  # [serve_1_enable, serve_2_enable, serve_3_enable, serve_4_enable, serve_5_enable, serve_6_enable, serve_7_enable, reserved]
        self._cmd_num = 0
        self._arm_type = 2
        self._arm_axis = 7
        self._arm_mid = 0
        self._arm_sid = 0
        self._arm_mttid = 0
        self._arm_mtfid = 0

        self._is_ready = False
        self._is_stop = False
        self._is_sync = False
        self._default_is_radian = is_radian

        self._sleep_finish_time = time.time()

        self._report_callbacks = {
            REPORT_ID: [],
            REPORT_LOCATION_ID: [],
            REPORT_CONNECT_CHANGED_ID: [],
            REPORT_ERROR_WARN_CHANGED_ID: [],
            REPORT_STATE_CHANGED_ID: [],
            REPORT_MAABLE_MTBRAKE_CHANGED_ID: [],
            REPORT_CMDNUM_CHANGED_ID: [],
        }

        if not do_not_open:
            self.connect()

    @property
    def connected(self):
        return self._stream and self._stream.connected

    @property
    def ready(self):
        return self._is_ready

    @property
    def default_is_radian(self):
        return self._default_is_radian

    @property
    def version(self):
        if not self._version:
            self.get_version()
        return self._version

    @property
    def position(self):
        if not self._enable_report:
            self.get_position()
        return [self._position[i] * RAD_DEGREE if 2 < i < 6 and not self._default_is_radian
                else self._position[i] for i in range(len(self._position))]

    @property
    def last_used_position(self):
        return [self._last_position[i] * RAD_DEGREE if 2 < i < 6 and not self._default_is_radian
                else self._last_position[i] for i in range(len(self._last_position))]

    @property
    def last_used_tcp_speed(self):
        return self._mvvelo

    @property
    def last_used_tcp_acc(self):
        return self._mvacc

    @property
    def angles(self):
        if not self._enable_report:
            self.get_servo_angle()
        return [angle if self._default_is_radian else angle * RAD_DEGREE for angle in self._angles]

    @property
    def last_used_angles(self):
        return [angle if self._default_is_radian else angle * RAD_DEGREE for angle in self._last_angles]

    @property
    def last_used_joint_speed(self):
        return self._angle_mvvelo if self._default_is_radian else self._angle_mvvelo * RAD_DEGREE

    @property
    def last_used_joint_acc(self):
        return self._angle_mvacc if self._default_is_radian else self._angle_mvacc * RAD_DEGREE

    @property
    def position_offset(self):
        return [self._position_offset[i] * RAD_DEGREE if 2 < i < 6 and not self._default_is_radian
                else self._position_offset[i] for i in range(len(self._position_offset))]

    @property
    def state(self):
        if not self._enable_report:
            self.get_state()
        return self._state

    @property
    def mtbrake(self):
        return self._mtbrake

    @property
    def maable(self):
        return self._maable

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
        return self.has_error or self.has_warn or (self.arm_cmd and self.arm_cmd.has_err_warn)

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
        return self._arm_mid

    @property
    def slave_id(self):
        return self._arm_sid

    @property
    def arm_mttid(self):
        return self._arm_mttid

    @property
    def arm_mtfid(self):
        return self._arm_mtfid

    def connect(self, port=None, baudrate=None, timeout=None):
        if self.connected:
            return
        self._is_ready = True
        self._port = port if port is not None else self._port
        self._baudrate = baudrate if baudrate is not None else self._baudrate
        self._timeout = timeout if timeout is not None else self._timeout
        if not self._port:
            raise Exception('can not connect to port/ip {}'.format(self._port))
        if isinstance(self._port, (str, bytes)):
            if self._port == 'localhost' or re.match(
                    r"^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$",
                    self._port):
                self._stream = SocketPort(self._port, XCONF.SocketConf.TCP_CONTROL_PORT,
                                          heartbeat=self._enable_heartbeat,
                                          buffer_size=XCONF.SocketConf.TCP_CONTROL_BUF_SIZE)
                if not self.connected:
                    raise Exception('connect socket failed')

                self._report_error_warn_changed_callback()

                self.arm_cmd = UxbusCmdTcp(self._stream)
                self._stream_type = 'socket'

                try:
                    self._connect_report()
                except:
                    self._stream_report = None

                if self._stream.connected:
                    self._report_thread = threading.Thread(target=self._report_thread_handle, daemon=True)
                    self._report_thread.start()
                self._report_connect_changed_callback()
            else:
                self._stream = SerialPort(self._port)
                if not self.connected:
                    raise Exception('connect serail failed')

                self._report_error_warn_changed_callback()

                self.arm_cmd = UxbusCmdSer(self._stream)
                self._stream_type = 'serial'
                if self._enable_report:
                    self._report_thread = threading.Thread(target=self._auto_get_report_thread, daemon=True)
                    self._report_thread.start()
                    self._report_connect_changed_callback(True, True)
                else:
                    self._report_connect_changed_callback(True, False)

    def _connect_report(self):
        if self._enable_report:
            if self._stream_report:
                try:
                    self._stream_report.close()
                except:
                    pass
                time.sleep(3)
            if self._report_type == 'real':
                self.__connect_report_real()
            elif self._report_type == 'rich':
                self.__connect_report_rich()
            else:
                self.__connect_report_normal()

    def __connect_report_normal(self):
        if self._stream_type == 'socket':
            self._stream_report = SocketPort(self._port,
                                             XCONF.SocketConf.TCP_REPORT_NORM_PORT,
                                             buffer_size=XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE)

    def __connect_report_rich(self):
        if self._stream_type == 'socket':
            self._stream_report = SocketPort(self._port,
                                             XCONF.SocketConf.TCP_REPORT_RICH_PORT,
                                             buffer_size=XCONF.SocketConf.TCP_REPORT_RICH_BUF_SIZE)

    def __connect_report_real(self):
        if self._stream_type == 'socket':
            self._stream_report = SocketPort(self._port,
                                             XCONF.SocketConf.TCP_REPORT_REAL_PORT,
                                             buffer_size=XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE)

    def _report_connect_changed_callback(self, main_connected=None, report_connected=None):
        if REPORT_CONNECT_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_CONNECT_CHANGED_ID]:
                try:
                    callback({
                        'connected': self._stream and self._stream.connected if main_connected is None else main_connected,
                        'reported': self._stream_report and self._stream_report.connected if report_connected is None else report_connected,
                    })
                except Exception as e:
                    logger.error('connect changed callback: {}'.format(e))

    def _report_state_changed_callback(self):
        if REPORT_STATE_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_STATE_CHANGED_ID]:
                try:
                    callback({
                        'state': self._state
                    })
                except Exception as e:
                    logger.error('state changed callback: {}'.format(e))

    def _report_maable_mtbrake_changed_callback(self):
        if REPORT_MAABLE_MTBRAKE_CHANGED_ID in self._report_callbacks.keys():
            maable = [bool(i) for i in self._maable]
            mtbrake = [bool(i) for i in self._mtbrake]
            for callback in self._report_callbacks[REPORT_MAABLE_MTBRAKE_CHANGED_ID]:
                try:
                    callback({
                        'maable': maable.copy(),
                        'mtbrake': mtbrake.copy()
                    })
                except Exception as e:
                    logger.error('maable/mtbrake changed callback: {}'.format(e))

    def _report_error_warn_changed_callback(self):
        if REPORT_ERROR_WARN_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_ERROR_WARN_CHANGED_ID]:
                try:
                    callback({
                        'warn_code': self._warn_code,
                        'error_code': self._error_code,
                    })
                except Exception as e:
                    logger.error('error warn changed callback: {}'.format(e))

    def _report_cmdnum_changed_callback(self):
        if REPORT_CMDNUM_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_CMDNUM_CHANGED_ID]:
                try:
                    callback({
                        'cmdnum': self._cmd_num,
                    })
                except Exception as e:
                    logger.error('cmdnum changed callback: {}'.format(e))

    def _report_location_callback(self):
        if REPORT_LOCATION_ID in self._report_callbacks.keys():
            for item in self._report_callbacks[REPORT_LOCATION_ID]:
                callback = item['callback']
                ret = {}
                if item['cartesian']:
                    ret['cartesian'] = self._position
                if item['joints']:
                    ret['joints'] = self._angles
                try:
                    callback(ret)
                except Exception as e:
                    logger.error('location callback: {}'.format(e))

    def _report_callback(self):
        if REPORT_ID in self._report_callbacks.keys():
            for item in self._report_callbacks[REPORT_ID]:
                callback = item['callback']
                ret = {}
                if item['cartesian']:
                    ret['cartesian'] = self._position
                if item['joints']:
                    ret['joints'] = self._angles
                if item['error_code']:
                    ret['error_code'] = self._error_code
                if item['warn_code']:
                    ret['warn_code'] = self._warn_code
                if item['state']:
                    ret['state'] = self._state
                if item['maable']:
                    maable = [bool(i) for i in self._maable]
                    ret['maable'] = maable.copy()
                if item['mtbrake']:
                    mtbrake = [bool(i) for i in self._mtbrake]
                    ret['mtbrake'] = mtbrake.copy()
                if item['cmdnum']:
                    ret['cmdnum'] = self._cmd_num
                try:
                    callback(ret)
                except Exception as e:
                    logger.error('report callback: {}'.format(e))

    def _report_thread_handle(self):
        def __handle_report_normal(rx_data):
            # print('length:', convert.bytes_to_u32(rx_data[0:4]))
            state, mtbrake, maable, error_code, warn_code = rx_data[4:9]
            angles = convert.bytes_to_fp32s(rx_data[9:7 * 4 + 9], 7)
            pose = convert.bytes_to_fp32s(rx_data[37:6 * 4 + 37], 6)
            cmd_num = convert.bytes_to_u16(rx_data[61:63])
            pose_offset = convert.bytes_to_fp32s(rx_data[63:6 * 4 + 63], 6)

            if error_code != self._error_code or warn_code != self._warn_code:
                self._warn_code = warn_code
                self._error_code = error_code
                self._report_error_warn_changed_callback()
                if self._error_code != 0:
                    pretty_print('Error, Code: {}'.format(self._error_code), color='red')
                else:
                    pretty_print('Error had clean', color='blue')
                if self._warn_code != 0:
                    pretty_print('WarnCode: {}'.format(self._error_code), color='yellow')
                else:
                    pretty_print('Warnning had clean', color='blue')
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
            maable = [maable & 0x01, maable >> 1 & 0x01, maable >> 2 & 0x01, maable >> 3 & 0x01,
                      maable >> 4 & 0x01, maable >> 5 & 0x01, maable >> 6 & 0x01, maable >> 7 & 0x01]

            if mtbrake != self._mtbrake or maable != self._maable:
                self._maable = maable
                self._mtbrake = mtbrake
                self._report_maable_mtbrake_changed_callback()

            _mtbrake = [bool(item[0] & item[1]) for item in zip(mtbrake, maable)]
            if state == 4 or not all(_mtbrake[:7]):
                self._is_ready = False
            else:
                self._is_ready = True

            self._error_code = error_code
            self._warn_code = warn_code
            self.arm_cmd.has_err_warn = error_code != 0 or warn_code != 0
            self._state = state
            self._cmd_num = cmd_num
            self._mtbrake = mtbrake
            self._maable = maable

            for i in range(len(pose)):
                if i < 3:
                    pose[i] = float('{:.3f}'.format(pose[i][0]))
                else:
                    pose[i] = float('{:.6f}'.format(pose[i][0]))
            for i in range(len(angles)):
                angles[i] = float('{:.6f}'.format(angles[i][0]))
            for i in range(len(pose_offset)):
                if i < 3:
                    pose_offset[i] = float('{:.3f}'.format(pose_offset[i][0]))
                else:
                    pose_offset[i] = float('{:.6f}'.format(pose_offset[i][0]))

            if math.inf not in pose:
                self._position = pose
            if math.inf not in angles:
                self._angles = angles
            if math.inf not in pose_offset:
                self._position_offset = pose_offset

            self._report_location_callback()

            self._report_callback()
            if not self._is_sync:
                self._sync()
                self._is_sync = True

        def __handle_report_rich(rx_data):
            __handle_report_normal(rx_data)
            (self._arm_type,
             self._arm_axis,
             self._arm_mid,
             self._arm_sid,
             self._arm_mtfid,
             self._arm_mttid) = rx_data[87:93]

            ver_msg = rx_data[93:112]
            trs_msg = convert.bytes_to_fp32s(rx_data[113:133], 5)
            p2p_msg = convert.bytes_to_fp32s(rx_data[133:153], 5)
            ros_msg = convert.bytes_to_fp32s(rx_data[153:161], 2)

            trs_msg = [i[0] for i in trs_msg]
            p2p_msg = [i[0] for i in p2p_msg]
            ros_msg = [i[0] for i in ros_msg]
            # ver_msg = str(ver_msg, 'utf-8')
            # print("arm_type: %d, arm_axis: %d, arm_mid: %d, arm_sid: %d, arm_mttid: %d, arm_mtfid: %d" % \
            #       (self.arm_type, self.arm_axis, self.arm_mid, self.arm_sid, self.arm_mttid, self.arm_mtfid))
            # print("trs_msg: {}".format(trs_msg))
            # print("p2p_msg: {}".format(p2p_msg))
            # print("ros_msg: {}".format(ros_msg))
            # print("ver_msg: {}".format(ver_msg))

            # version = ''.join(list(map(chr, ver_msg)))
            # version = version[:version.find('\0')]
            # (trs_jerk,
            #  trs_acc_min,
            #  trs_acc_max,
            #  trs_velo_min,
            #  trs_velo_max) = trs_msg
            # (p2p_jerk,
            #  p2p_acc_min,
            #  p2p_acc_max,
            #  p2p_velo_min,
            #  p2p_velo_max) = p2p_msg
            # rot_jerk, ros_acc_max = ros_msg

        main_socket_connected = self._stream and self._stream.connected
        report_socket_connected = self._stream_report and self._stream_report.connected
        while self._stream and self._stream.connected:
            try:
                if not self._stream_report or not self._stream_report.connected:
                    if report_socket_connected:
                        report_socket_connected = False
                        self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                    self._connect_report()
                    continue
                if not report_socket_connected:
                    report_socket_connected = True
                    self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                rx_data = self._stream_report.read()
                if rx_data != -1 and len(rx_data) >= XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE:
                    if len(rx_data) == XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE:
                        __handle_report_normal(rx_data)
                    elif len(rx_data) >= XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE:
                        __handle_report_rich(rx_data)
            except Exception as e:
                logger.error(e)
            time.sleep(0.001)
        self.disconnect()
        self._report_connect_changed_callback(False, False)

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

                if cmd_num != self._cmd_num:
                    self._report_cmdnum_changed_callback()
                if state != self._state:
                    self._report_state_changed_callback()
                if self._state == 4:
                    self._is_ready = False
                else:
                    self._is_ready = True
                if error_code != self._error_code or warn_code != self._warn_code:
                    self._report_error_warn_changed_callback()
                elif not self._only_report_err_warn_changed and (self._error_code != 0 or self._warn_code != 0):
                    self._report_error_warn_changed_callback()

                self._report_location_callback()
                self._report_callback()

                if self._cmd_num >= MAX_CMD_NUM:
                    time.sleep(1)

                time.sleep(0.1)
            except:
                pass
        self._report_connect_changed_callback(False, False)
        logger.debug('get report thread stopped')

    def disconnect(self):
        self._stream.close()
        if self._stream_report:
            self._stream_report.close()
        self._is_ready = False
        self._stream.join()
        if self._stream_report:
            self._stream_report.join()
        self._report_connect_changed_callback(False, False)

    def _sync(self):
        if not self._stream_report or not self._stream_report.connected:
            self.get_position()
            self.get_servo_angle()
        self._last_position = self._position
        self._last_angles = self._angles

    class _WaitMove:
        def __init__(self, owner, timeout):
            self.owner = owner
            self.timeout = timeout if timeout is not None else 10
            self.timer = None
            self.is_timeout = False

        def start(self):
            if self.timeout > 0:
                self.timer = threading.Timer(self.timeout, self.timeout_cb)
                self.timer.setDaemon(True)
                self.timer.start()
            self.check_stop_move()

        def check_stop_move(self):
            base_joint_pos = self.owner.angles.copy()
            time.sleep(0.1)
            count = 0
            while not self.is_timeout and not self.owner._is_stop and self.owner.connected and not self.owner.has_error:
                if time.time() < self.owner._sleep_finish_time:
                    time.sleep(0.01)
                    continue
                if self.owner.angles == base_joint_pos or self.owner.state != 1:
                    count += 1
                    if count >= 6:
                        break
                else:
                    base_joint_pos = self.owner.angles.copy()
                    count = 0
                time.sleep(0.05)
            # if not self.is_timeout:
            #     self.owner._sync()

        def timeout_cb(self):
            self.is_timeout = True

    @xarm_is_connected(_type='get')
    def get_position(self, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        ret = self.arm_cmd.get_tcp_pose()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) > 6:
            self._position = [float('{:.6f}'.format(ret[i][0])) for i in range(1, 7)]
            ret[0] = 0
        return ret[0], [self._position[i] * RAD_DEGREE if 2 < i < 6 and not is_radian else self._position[i] for i in
                        range(len(self._position))]

    @staticmethod
    def _is_out_of_tcp_rotate_range(angle, i):
        rotate_range = LIMIT_TCP_ROTATE[i]
        if angle < rotate_range[0] or angle > rotate_range[1]:
            return True
        return False

    def _wait_until_cmdnum_lt_max(self):
        self._is_stop = False
        while self.cmd_num >= MAX_CMD_NUM:
            if not self.connected:
                return APIState.NOT_CONNECTED
            elif self._is_stop:
                return 0
            elif self.has_error:
                return APIState.HAS_ERROR
            time.sleep(0.2)

    @xarm_is_ready(_type='set')
    def set_position(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None,
                     speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None,
                     wait=False, timeout=None, **kwargs):
        # self._is_stop = False
        # while self.cmd_num >= MAX_CMD_NUM:
        #     if not self.connected:
        #         return APIState.NOT_CONNECTED
        #     elif self._is_stop:
        #         return 0
        #     elif self.has_error:
        #         return APIState.HAS_ERROR
        #     time.sleep(0.1)
        ret = self._wait_until_cmdnum_lt_max()
        if ret is not None:
            return ret

        is_radian = self._default_is_radian if is_radian is None else is_radian
        tcp_pos = [x, y, z, roll, pitch, yaw]
        last_used_position = self._last_position.copy()
        last_used_speed = self._mvvelo
        last_used_acc = self._mvacc
        for i in range(6):
            value = tcp_pos[i]
            if value is None:
                continue
            elif isinstance(value, str):
                if value.isdigit():
                    value = float(value)
                else:
                    continue
            if relative:
                if 2 < i < 6:
                    if is_radian:
                        if self._is_out_of_tcp_rotate_range(self._last_position[i] + value, i - 3):
                            self._last_position = last_used_position
                            return APIState.OUT_OF_RANGE
                        self._last_position[i] += value
                    else:
                        if self._is_out_of_tcp_rotate_range(self._last_position[i] + value / RAD_DEGREE, i - 3):
                            self._last_position = last_used_position
                            return APIState.OUT_OF_RANGE
                        self._last_position[i] += value / RAD_DEGREE
                else:
                    self._last_position[i] += value
            else:
                if 2 < i < 6:
                    if is_radian:
                        if self._is_out_of_tcp_rotate_range(value, i - 3):
                            self._last_position = last_used_position
                            return APIState.OUT_OF_RANGE
                        self._last_position[i] = value
                    else:
                        if self._is_out_of_tcp_rotate_range(value / RAD_DEGREE, i - 3):
                            self._last_position = last_used_position
                            return APIState.OUT_OF_RANGE
                        self._last_position[i] = value / RAD_DEGREE
                else:
                    self._last_position[i] = value

        if speed is not None:
            if isinstance(speed, str):
                if speed.isdigit():
                    speed = float(speed)
                else:
                    speed = self._mvvelo
            self._mvvelo = min(max(speed, self._min_velo), self._max_velo)
        elif kwargs.get('mvvelo', None) is not None:
            mvvelo = kwargs.get('mvvelo')
            if isinstance(mvvelo, str):
                if mvvelo.isdigit():
                    mvvelo = float(mvvelo)
                else:
                    mvvelo = self._mvvelo
            self._mvvelo = min(max(mvvelo, self._min_velo), self._max_velo)
        if mvacc is not None:
            if isinstance(mvacc, str):
                if mvacc.isdigit():
                    mvacc = float(mvacc)
                else:
                    mvacc = self._mvacc
            self._mvacc = min(max(mvacc, self._min_acc), self._max_acc)
        if mvtime is not None:
            if isinstance(mvtime, str):
                if mvacc.isdigit():
                    mvtime = float(mvtime)
                else:
                    mvtime = self._mvtime
            self._mvtime = mvtime

        if kwargs.get('check', False):
            _, limit = self.is_tcp_limit(self._last_position)
            if _ == 0 and limit is True:
                self._last_position = last_used_position
                self._mvvelo = last_used_speed
                self._mvacc = last_used_acc
                return APIState.TCP_LIMIT
        if radius is not None and radius >= 0:
            ret = self.arm_cmd.move_lineb(self._last_position, self._mvvelo, self._mvacc, self._mvtime, radius)
            if ret[0] != 0:
                logger.debug('exception({}): move arc line: pos={}, mvvelo={}, mvacc={}, mvtime={}, mvradius={}'.format(
                    ret[0], self._last_position, self._mvvelo, self._mvacc, self._mvtime, radius
                ))
            else:
                logger.debug('move arc line: {}, mvvelo={}, mvacc={}, mvtime={}, mvradius={}'.format(
                    self._last_position, self._mvvelo, self._mvacc, self._mvtime, radius
                ))
        else:
            ret = self.arm_cmd.move_line(self._last_position, self._mvvelo, self._mvacc, self._mvtime)
            if ret[0] != 0:
                logger.debug('exception({}): move line: pos={}, mvvelo={}, mvacc={}, mvtime={}'.format(
                    ret[0], self._last_position, self._mvvelo, self._mvacc, self._mvtime
                ))
            else:
                logger.debug('move line: {}, mvvelo={}, mvacc={}, mvtime={}'.format(
                    self._last_position, self._mvvelo, self._mvacc, self._mvtime
                ))
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                logger.warn('if you want to wait, please enable report')
                return ret[0]
            self._is_stop = False
            self._WaitMove(self, timeout).start()
            self._is_stop = False
        if ret[0] < 0:
            self._last_position = last_used_position
            self._mvvelo = last_used_speed
            self._mvacc = last_used_acc
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_servo_angle(self, servo_id=None, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        ret = self.arm_cmd.get_joint_pos()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) > 7:
            self._angles = [float('{:.6f}'.format(ret[i][0])) for i in range(1, 8)]
            ret[0] = 0
        if servo_id is None or servo_id == 8 or len(self._angles) < servo_id:
            return ret[0], list(map(lambda x: x if is_radian else x * RAD_DEGREE, self._angles))
        else:
            return ret[0], self._angles[servo_id-1] if is_radian else self._angles[servo_id-1] * RAD_DEGREE

    @staticmethod
    def _is_out_of_angle_range(angle, i):
        angle_range = LIMIT_JOINTS[i]
        if angle <= angle_range[0] or angle >= angle_range[1]:
            return True
        return False

    @xarm_is_ready(_type='set')
    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None,
                        relative=False, is_radian=None, wait=False, timeout=None, **kwargs):
        assert ((servo_id is None or servo_id == 8) and isinstance(angle, Iterable)) \
            or (1 <= servo_id <= 7 and angle is not None and not isinstance(angle, Iterable)), \
            'param servo_id or angle error'
        # self._is_stop = False
        # while self.cmd_num >= MAX_CMD_NUM:
        #     if not self.connected:
        #         return APIState.NOT_CONNECTED
        #     elif self._is_stop:
        #         return 0
        #     elif self.has_error:
        #         return APIState.HAS_ERROR
        #     time.sleep(0.1)
        ret = self._wait_until_cmdnum_lt_max()
        if ret is not None:
            return ret

        last_used_angle = self._last_angles.copy()
        last_used_speed = self._angle_mvvelo
        last_used_acc = self._angle_mvacc
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if servo_id is None or servo_id == 8:
            for i in range(min(len(angle), len(self._last_angles))):
                value = angle[i]
                if value is None:
                    continue
                if isinstance(value, str):
                    if value.isdigit():
                        value = float(value)
                    else:
                        continue
                if relative:
                    if is_radian:
                        if self._is_out_of_angle_range(self._last_angles[i] + value, i):
                            self._last_angles = last_used_angle
                            return APIState.OUT_OF_RANGE
                        self._last_angles[i] += value
                    else:
                        if self._is_out_of_angle_range(self._last_angles[i] + value / RAD_DEGREE, i):
                            self._last_angles = last_used_angle
                            return APIState.OUT_OF_RANGE
                        self._last_angles[i] += value / RAD_DEGREE
                else:
                    if is_radian:
                        if self._is_out_of_angle_range(value, i):
                            self._last_angles = last_used_angle
                            return APIState.OUT_OF_RANGE
                        self._last_angles[i] = value
                    else:
                        if self._is_out_of_angle_range(value / RAD_DEGREE, i):
                            self._last_angles = last_used_angle
                            return APIState.OUT_OF_RANGE
                        self._last_angles[i] = value / RAD_DEGREE
        else:
            if isinstance(angle, str):
                if angle.isdigit():
                    angle = float(angle)
                else:
                    raise Exception('param angle error')
            if relative:
                if is_radian:
                    if self._is_out_of_angle_range(self._last_angles[servo_id - 1] + angle, servo_id - 1):
                        self._last_angles = last_used_angle
                        return APIState.OUT_OF_RANGE
                    self._last_angles[servo_id - 1] += angle
                else:
                    if self._is_out_of_angle_range(self._last_angles[servo_id - 1] + angle / RAD_DEGREE, servo_id - 1):
                        self._last_angles = last_used_angle
                        return APIState.OUT_OF_RANGE
                    self._last_angles[servo_id - 1] += angle / RAD_DEGREE
            else:
                if is_radian:
                    if self._is_out_of_angle_range(angle, servo_id - 1):
                        self._last_angles = last_used_angle
                        return APIState.OUT_OF_RANGE
                    self._last_angles[servo_id - 1] = angle
                else:
                    if self._is_out_of_angle_range(angle / RAD_DEGREE, servo_id - 1):
                        self._last_angles = last_used_angle
                        return APIState.OUT_OF_RANGE
                    self._last_angles[servo_id - 1] = angle / RAD_DEGREE

        if speed is not None:
            if isinstance(speed, str):
                if speed.isdigit():
                    speed = float(speed)
                else:
                    speed = self._angle_mvvelo if is_radian else self._angle_mvvelo * RAD_DEGREE
            if not is_radian:
                speed /= RAD_DEGREE
            self._angle_mvvelo = min(max(speed, self._min_angle_velo), self._max_angle_velo)
        elif kwargs.get('mvvelo', None) is not None:
            mvvelo = kwargs.get('mvvelo')
            if isinstance(mvvelo, str):
                if mvvelo.isdigit():
                    mvvelo = float(mvvelo)
                else:
                    mvvelo = self._angle_mvvelo if is_radian else self._angle_mvvelo * RAD_DEGREE
            if not is_radian:
                mvvelo /= RAD_DEGREE
            self._angle_mvvelo = min(max(mvvelo, self._min_angle_velo), self._max_angle_velo)
        if mvacc is not None:
            if isinstance(mvacc, str):
                if mvacc.isdigit():
                    mvacc = float(mvacc)
                else:
                    mvacc = self._angle_mvacc if is_radian else self._angle_mvacc * RAD_DEGREE
            if not is_radian:
                mvacc /= RAD_DEGREE
            self._angle_mvacc = min(max(mvacc, self._min_angle_acc), self._max_angle_acc)
        if mvtime is not None:
            if isinstance(mvtime, str):
                if mvacc.isdigit():
                    mvtime = float(mvtime)
                else:
                    mvtime = self._mvtime
            self._mvtime = mvtime

        if kwargs.get('check', False):
            _, limit = self.is_joint_limit(self._last_angles)
            if _ == 0 and limit is True:
                self._last_angles = last_used_angle
                self._angle_mvvelo = last_used_speed
                self._angle_mvacc = last_used_acc
                return APIState.JOINT_LIMIT

        ret = self.arm_cmd.move_joint(self._last_angles, self._angle_mvvelo, self._angle_mvacc, self._mvtime)
        if ret[0] != 0:
            logger.debug('exception({}): move joint: joint={}, mvvelo={}, mvacc={}, mvtime={}'.format(
                ret[0], self._last_angles, self._angle_mvvelo, self._angle_mvacc, self._mvtime
            ))
        else:
            logger.debug('move joint: {}, mvvelo={}, mvacc={}, mvtime={}'.format(
                self._last_angles, self._angle_mvvelo, self._angle_mvacc, self._mvtime
            ))
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                logger.warn('if you want to wait, please enable report')
                return ret[0]
            self._is_stop = False
            self._WaitMove(self, timeout).start()
            self._is_stop = False
        if ret[0] < 0:
            self._last_angles = last_used_angle
            self._angle_mvvelo = last_used_speed
            self._angle_mvacc = last_used_acc
        return ret[0]

    @xarm_is_ready(_type='set')
    def set_servo_angle_j(self, angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if not is_radian:
            angles = [angle / RAD_DEGREE for angle in angles]
        for i in range(7):
            if self._is_out_of_angle_range(angles[i], i):
                return APIState.OUT_OF_RANGE
        ret = self.arm_cmd.move_servoj(angles, self._angle_mvvelo, self._angle_mvacc, self._mvtime)
        return ret[0]

    @xarm_is_ready(_type='set')
    def move_gohome(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if speed is None:
            speed = 0.8726646259971648  # 50 °/s
        else:
            if not is_radian:
                speed /= RAD_DEGREE
        if mvacc is None:
            mvacc = 17.453292519943297  # 1000 °/s^2
        else:
            if not is_radian:
                mvacc /= RAD_DEGREE
        if mvtime is None:
            mvtime = 0

        ret = self.arm_cmd.move_gohome(speed, mvacc, mvtime)
        if ret[0] != 0:
            logger.debug('exception({}): move gohome: mvvelo={}, mvacc={}, mvtime={}'.format(
                ret[0], speed, mvacc, mvtime
            ))
        else:
            logger.debug('move gohome: mvvelo={}, mvacc={}, mvtime={}'.format(
                speed, mvacc, mvtime
            ))
        if ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            self._last_position = [201.5, 0, 140.5, -3.1415926, 0, 0]
            self._last_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                logger.warn('if you want to wait, please enable report')
                return ret[0]
            self._is_stop = False
            self._WaitMove(self, timeout).start()
            self._is_stop = False
        return ret[0]

    @xarm_is_ready(_type='set')
    def move_arc_lines(self, paths, is_radian=None, times=1, first_pause_time=0.1, repeat_pause_time=0,
                       automatic_calibration=True, speed=None, mvacc=None, mvtime=None, wait=False):
        assert len(paths) > 0
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if speed is None:
            speed = self._mvvelo
        if mvacc is None:
            mvacc = self._mvacc
        if mvtime is None:
            mvtime = 0

        if automatic_calibration:
            self.set_position(*paths[0], is_radian=is_radian, speed=speed, mvacc=mvacc, mvtime=mvtime, wait=True)
            _, angles = self.get_servo_angle(is_radian=True)
        self.set_pause_time(first_pause_time)
        self._is_stop = False
        last_used_angle_speed = self._angle_mvvelo

        def _move():
            if automatic_calibration:
                ret = self.set_servo_angle(angle=angles, is_radian=True, speed=0.8726646259971648, wait=False)
                if ret < 0:
                    logger.error('set_servo_angle, ret={}'.format(ret))
                    self._is_stop = True
                    return
                self._angle_mvvelo = last_used_angle_speed
            for path in paths:
                if len(path) > 6 and path[6] >= 0:
                    radius = path[6]
                else:
                    radius = 0
                # if self.has_error or self._is_stop:
                #     return
                # while self.cmd_num >= MAX_CMD_NUM:
                #     if self.has_error or self._is_stop:
                #         return
                #     time.sleep(0.1)
                ret = self.set_position(*path[:6], radius=radius, is_radian=is_radian, wait=False, speed=speed, mvacc=mvacc, mvtime=mvtime)
                if ret < 0:
                    logger.error('set_positon, ret={}'.format(ret))
                    self._is_stop = True
                    return
        count = 1
        if times == 0:
            while not self.has_error and not self._is_stop:
                _move()
                count += 1
                if not self._is_stop and self._error_code == 0:
                    self.set_pause_time(repeat_pause_time)
            if self._error_code != 0:
                logger.error('quit, controller error')
            elif self._is_stop:
                logger.error('quit, emergency_stop')
        else:
            for i in range(times):
                if self.has_error or self._is_stop:
                    break
                # print('第{}次开始'.format(count))
                _move()
                # print('第{}次结束'.format(count))
                count += 1
                if not self._is_stop and self._error_code == 0:
                    self.set_pause_time(repeat_pause_time)
            if self._error_code != 0:
                logger.error('quit, controller error')
            elif self._is_stop:
                logger.error('quit, emergency_stop')
        if wait:
            self._WaitMove(self, 0).start()
            self._is_stop = False

    @xarm_is_connected(_type='set')
    def set_servo_attach(self, servo_id=None):
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8
        ret = self.arm_cmd.set_brake(servo_id, 0)
        return ret

    @xarm_is_connected(_type='set')
    def set_servo_detach(self, servo_id=None):
        """
        :param servo_id: 1-7, 8
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8
        ret = self.arm_cmd.set_brake(servo_id, 1)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_version(self):
        ret = self.arm_cmd.get_version()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            version = ''.join(list(map(chr, ret[1:])))
            self._version = version[:version.find('\0')]
            ret[0] = 0
            return ret[0], self._version
        else:
            return ret[0], self._version

    def get_is_moving(self):
        self.get_state()
        return self._state == 1

    @xarm_is_connected(_type='get')
    def get_state(self):
        ret = self.arm_cmd.get_state()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._state = ret[1]
            ret[0] = 0
        return ret[0], self._state

    @xarm_is_connected(_type='set')
    def set_state(self, state=0):
        ret = self.arm_cmd.set_state(state)
        if state == 4 and ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            # self._last_position[:6] = self.position
            # self._last_angles = self.angles
            self._sleep_finish_time = 0
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_mode(self, mode=0):
        ret = self.arm_cmd.set_mode(mode)
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_cmdnum(self):
        ret = self.arm_cmd.get_cmdnum()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._cmd_num = ret[1]
            ret[0] = 0
        return ret[0], self._cmd_num

    @xarm_is_connected(_type='get')
    def get_err_warn_code(self, show=False):
        ret = self.arm_cmd.get_err_code()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._error_code, self._warn_code = ret[1:3]
            ret[0] = 0
        if show:
            pretty_print('*************获取错误警告码, 状态: {}**************'.format(ret[0]), color='light_blue')
            controller_error = ControllerError(self._error_code)
            controller_warn = ControllerWarn(self._warn_code)
            pretty_print('* 错误码: {}, 错误信息: {}'.format(self._error_code, controller_error.description), color='red' if self._error_code != 0 else 'white')
            pretty_print('* 警告码: {}, 警告信息: {}'.format(self._warn_code, controller_warn.description), color='yellow' if self._warn_code != 0 else 'white')
            pretty_print('*' * 50, color='light_blue')
        return ret[0], [self._error_code, self._warn_code]

    @xarm_is_connected(_type='set')
    def clean_error(self):
        ret = self.arm_cmd.clean_err()
        return ret[0]

    @xarm_is_connected(_type='set')
    def clean_warn(self):
        ret = self.arm_cmd.clean_war()
        return ret[0]

    @xarm_is_connected(_type='set')
    def motion_enable(self, enable=True, servo_id=None):
        assert servo_id is None or (isinstance(servo_id, int) and 1 <= servo_id <= 8)
        if servo_id is None or servo_id == 8:
            ret = self.arm_cmd.motion_en(8, int(enable))
        else:
            ret = self.arm_cmd.motion_en(servo_id, int(enable))
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._is_ready = bool(enable)
        return ret[0]

    def reset(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if not self._enable_report or self._stream_type != 'socket':
            self.get_err_warn_code()
            self.get_state()
        if self._warn_code != 0:
            self.clean_warn()
        if self._error_code != 0:
            self.clean_error()
            self.motion_enable(enable=True, servo_id=8)
            self.set_state(0)
        if not self._is_ready:
            self.motion_enable(enable=True, servo_id=8)
            self.set_state(state=0)
        self.move_gohome(speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, wait=wait, timeout=timeout)

    @xarm_is_connected(_type='set')
    def set_pause_time(self, sltime, wait=False):
        assert isinstance(sltime, (int, float))
        ret = self.arm_cmd.sleep_instruction(sltime)
        if wait:
            time.sleep(sltime)
        else:
            if time.time() >= self._sleep_finish_time:
                self._sleep_finish_time = time.time() + sltime
            else:
                self._sleep_finish_time += sltime
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_tcp_offset(self, offset, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        assert isinstance(offset, Iterable) and len(offset) >= 6
        tcp_offset = [0] * 6
        for i in range(len(offset)):
            if not offset[i]:
                continue
            if i < 3:
                tcp_offset[i] = offset[i]
            elif i < 6:
                if not is_radian:
                    tcp_offset[i] = offset[i] / RAD_DEGREE
                else:
                    tcp_offset[i] = offset[i]
        ret = self.arm_cmd.set_tcp_offset(tcp_offset)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_tcp_jerk(self, jerk):
        ret = self.arm_cmd.set_tcp_jerk(jerk)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_tcp_maxacc(self, acc):
        ret = self.arm_cmd.set_tcp_maxacc(acc)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_joint_jerk(self, jerk, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        _jerk = jerk
        if not is_radian:
            _jerk = _jerk / RAD_DEGREE
        ret = self.arm_cmd.set_joint_jerk(_jerk)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_joint_maxacc(self, acc, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        _acc = acc
        if not is_radian:
            _acc = acc / RAD_DEGREE
        ret = self.arm_cmd.set_joint_maxacc(_acc)
        return ret[0]

    @xarm_is_connected(_type='set')
    def clean_conf(self):
        ret = self.arm_cmd.clean_conf()
        return ret[0]

    @xarm_is_connected(_type='set')
    def save_conf(self):
        ret = self.arm_cmd.save_conf()
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_inverse_kinematics(self, pose, input_is_radian=None, return_is_radian=None):
        input_is_radian = self._default_is_radian if input_is_radian is None else input_is_radian
        return_is_radian = self._default_is_radian if return_is_radian is None else return_is_radian
        assert len(pose) >= 6
        if not input_is_radian:
            pose = [pose[i] if i < 3 else pose[i] / RAD_DEGREE for i in range(6)]
        ret = self.arm_cmd.get_ik(pose)
        angles = []
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            angles = [ret[i][0] for i in range(1, 8)]
            ret[0] = 0
            if not return_is_radian:
                angles = [angle * RAD_DEGREE for angle in angles]
        return ret[0], angles

    @xarm_is_connected(_type='get')
    def get_forward_kinematics(self, angles, input_is_radian=None, return_is_radian=None):
        input_is_radian = self._default_is_radian if input_is_radian is None else input_is_radian
        return_is_radian = self._default_is_radian if return_is_radian is None else return_is_radian
        assert len(angles) >= 7
        if not input_is_radian:
            angles = [angles[i] / RAD_DEGREE for i in range(7)]
        ret = self.arm_cmd.get_fk(angles)
        pose = []
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            pose = [ret[i][0] for i in range(1, 7)]
            ret[0] = 0
            if not return_is_radian:
                pose = [pose[i] if i < 3 else pose[i] * RAD_DEGREE for i in range(len(pose))]
        return ret[0], pose

    @xarm_is_connected(_type='get')
    def is_tcp_limit(self, pose, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        assert len(pose) >= 6
        for i in range(6):
            if isinstance(pose[i], str):
                pose[i] = float(pose[i])
            if pose[i] is None:
                pose[i] = self._last_position[i]
            elif i > 2 and not is_radian:
                pose[i] = pose[i] / RAD_DEGREE
        ret = self.arm_cmd.is_tcp_limit(pose)
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
            return ret[0], bool(ret[1])
        else:
            return ret[0], None

    @xarm_is_connected(_type='get')
    def is_joint_limit(self, joint, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        assert len(joint) >= 7
        for i in range(7):
            if isinstance(joint[i], str):
                joint[i] = float(joint[i])
            if joint[i] is None:
                joint[i] = self._last_angles[i]
            elif not is_radian:
                joint[i] = joint[i] / RAD_DEGREE
        ret = self.arm_cmd.is_joint_limit(joint)
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            ret[0] = 0
            return ret[0], bool(ret[1])
        else:
            return ret[0], None

    def _set_params(self, **kwargs):
        is_radian = kwargs.get('is_radian', self._default_is_radian)
        if 'X' in kwargs and isinstance(kwargs['X'], (int, float)):
            self._last_position[0] = kwargs.get('X')
        if 'Y' in kwargs and isinstance(kwargs['Y'], (int, float)):
            self._last_position[1] = kwargs.get('Y')
        if 'Z' in kwargs and isinstance(kwargs['Z'], (int, float)):
            self._last_position[2] = kwargs.get('Z')
        if 'A' in kwargs and isinstance(kwargs['A'], (int, float)):
            self._last_position[3] = kwargs.get('A') if is_radian else kwargs.get('A') / RAD_DEGREE
        if 'B' in kwargs and isinstance(kwargs['B'], (int, float)):
            self._last_position[4] = kwargs.get('B') if is_radian else kwargs.get('B') / RAD_DEGREE
        if 'C' in kwargs and isinstance(kwargs['C'], (int, float)):
            self._last_position[5] = kwargs.get('C') if is_radian else kwargs.get('C') / RAD_DEGREE
        # if 'R' in kwargs and isinstance(kwargs['R'], (int, float)):
        #     self._last_position[6] = kwargs.get('R')
        if 'I' in kwargs and isinstance(kwargs['I'], (int, float)):
            self._last_angles[0] = kwargs.get('I') if is_radian else kwargs.get('I') / RAD_DEGREE
        if 'J' in kwargs and isinstance(kwargs['J'], (int, float)):
            self._last_angles[1] = kwargs.get('J') if is_radian else kwargs.get('J') / RAD_DEGREE
        if 'K' in kwargs and isinstance(kwargs['K'], (int, float)):
            self._last_angles[2] = kwargs.get('K') if is_radian else kwargs.get('K') / RAD_DEGREE
        if 'L' in kwargs and isinstance(kwargs['L'], (int, float)):
            self._last_angles[3] = kwargs.get('L') if is_radian else kwargs.get('L') / RAD_DEGREE
        if 'M' in kwargs and isinstance(kwargs['M'], (int, float)):
            self._last_angles[4] = kwargs.get('M') if is_radian else kwargs.get('M') / RAD_DEGREE
        if 'N' in kwargs and isinstance(kwargs['N'], (int, float)):
            self._last_angles[5] = kwargs.get('N') if is_radian else kwargs.get('N') / RAD_DEGREE
        if 'O' in kwargs and isinstance(kwargs['O'], (int, float)):
            self._last_angles[6] = kwargs.get('O') if is_radian else kwargs.get('O') / RAD_DEGREE

        if 'F' in kwargs and isinstance(kwargs['F'], (int, float)):
            self._mvvelo = kwargs.get('F')
            self._mvvelo = min(max(self._mvvelo, self._min_velo), self._max_velo)
        if 'Q' in kwargs and isinstance(kwargs['Q'], (int, float)):
            self._mvacc = kwargs.get('Q')
            self._mvacc = min(max(self._mvacc, self._min_acc), self._max_acc)
        if 'F2' in kwargs and isinstance(kwargs['F2'], (int, float)):
            self._angle_mvvelo = kwargs.get('F2')
            if not is_radian:
                self._angle_mvvelo /= RAD_DEGREE
            self._angle_mvvelo = min(max(self._angle_mvvelo, self._min_angle_velo), self._max_angle_velo)
        if 'Q2' in kwargs and isinstance(kwargs['Q2'], (int, float)):
            self._angle_mvacc = kwargs.get('Q2')
            if not is_radian:
                self._angle_mvacc /= RAD_DEGREE
            self._angle_mvacc = min(max(self._angle_mvacc, self._min_angle_acc), self._max_angle_acc)
        if 'T' in kwargs and isinstance(kwargs['T'], (int, float)):
            self._mvtime = kwargs.get('T')
        if 'LIMIT_VELO' in kwargs and isinstance(kwargs['LIMIT_VELO'], (list, tuple)) \
                and len(kwargs['LIMIT_VELO']) == 2 and isinstance(kwargs['LIMIT_VELO'][0], (int, float)) \
                and isinstance(kwargs['LIMIT_VELO'][1], (int, float)) \
                and kwargs['LIMIT_VELO'][0] <= kwargs['LIMIT_VELO'][1]:
            self._min_velo, self._max_velo = kwargs.get('LIMIT_VELO')
        if 'LIMIT_ACC' in kwargs and isinstance(kwargs['LIMIT_ACC'], (list, tuple)) \
                and len(kwargs['LIMIT_ACC']) == 2 and isinstance(kwargs['LIMIT_ACC'][0], (int, float)) \
                and isinstance(kwargs['LIMIT_ACC'][1], (int, float)) \
                and kwargs['LIMIT_ACC'][0] <= kwargs['LIMIT_ACC'][1]:
            self._min_acc, self._max_acc = kwargs.get('LIMIT_ACC')

    def _get_params(self, is_radian=None):
        is_radian = self._default_is_radian if is_radian is None else is_radian
        if is_radian:
            return {
                'lastPosition': self._last_position,
                'lastAngles': self._last_angles,
                'mvvelo': self._mvvelo,
                'mvacc': self._mvacc,
                'angle_mvvelo': self._angle_mvvelo,
                'angle_mvacc': self._angle_mvacc,
                'mvtime': self._mvtime,
                'LIMIT_VELO': [self._min_velo, self._max_velo],
                'LIMIT_ACC': [self._min_acc, self._max_acc],
                'LIMIT_ANGLE_VELO': [self._min_angle_velo, self._max_angle_velo],
                'LIMIT_ANGLE_ACC': [self._min_angle_acc, self._max_angle_acc],
            }
        else:
            return {
                'lastPosition': [self._last_position[i] * RAD_DEGREE if 2 < i < 6 else self._last_position[i] for i in range(len(self._last_position))],
                'lastAngles': [angle * RAD_DEGREE for angle in self._last_angles],
                'mvvelo': self._mvvelo,
                'mvacc': self._mvacc,
                'angle_mvvelo': self._angle_mvvelo * RAD_DEGREE,
                'angle_mvacc': self._angle_mvacc * RAD_DEGREE,
                'mvtime': self._mvtime,
                'LIMIT_VELO': [self._min_velo, self._max_velo],
                'LIMIT_ACC': [self._min_acc, self._max_acc],
                'LIMIT_ANGLE_VELO': [self._min_angle_velo * RAD_DEGREE, self._max_angle_velo * RAD_DEGREE],
                'LIMIT_ANGLE_ACC': [self._min_angle_acc * RAD_DEGREE, self._max_angle_acc * RAD_DEGREE],
            }

    def emergency_stop(self):
        start_time = time.time()
        while self.state != 4 and time.time() - start_time < 3:
            self.set_state(4)
            time.sleep(0.1)
        self._is_stop = True
        start_time = time.time()
        self.motion_enable(enable=True)
        while self.state in [0, 3, 4] and time.time() - start_time < 3:
            self.set_state(0)
            time.sleep(0.1)
        self._sleep_finish_time = 0

    def send_cmd_async(self, command, timeout=None):
        pass

    def send_cmd_sync(self, command=None):
        if command is None:
            return 0
        if command.lower() == 'help':
            return 0, {
                'G1': 'set_position(MoveLine): G1 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)} F{speed(mm/s)} Q{acc(mm/s^2)} T{mvtime} W{wait}',
                'G4': 'set_pause_time: G4 V{sltime(second)}',
                'G7': 'set_servo_angle: G7 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)} F{speed(°/s or rad/s)} Q{acc(°/s^2 or rad/s^2)} T{mvtime} W{wait}',
                'G8': 'move_gohome: G8 F{speed(°/s or rad/s)} Q{acc(°/s^2 or rad/s^2)} T{mvtime} W{wait}',
                'G9': 'set_position(MoveArcLine): G9 X{x} Y{y} Z{z} A{roll} B{pitch(° or rad)} C{yaw(° or rad)} R{radius(mm)} F{speed(mm/s)} Q{acc(mm/s^2)} T{mvtime} W{wait}',
                'H1': 'get_version: H1',
                'H11': 'motion_enable: H11 S{servo_id} V{enable}',
                'H12': 'set_state: H12 V{state}',
                'H13': 'get_state: H13',
                'H14': 'get_cmdnum: H14',
                'H15': 'get_err_warn_code: H15',
                'H16': 'clean_error: H16',
                'H17': 'clean_warn: H17',
                'H18': 'set_servo_attach/set_servo_detach: H18 S{servo_id} V{1: enable(detach), 0: disable(attach)}',
                'H31': 'set_tcp_jerk: H31 V{jerk(mm/s^3)}',
                'H32': 'set_tcp_maxacc: H32 V{maxacc(mm/s^2)}',
                'H33': 'set_joint_jerk: H33 V{jerk(°/s^3 or rad/s^3)}',
                'H34': 'set_joint_maxacc: H34 {maxacc(°/s^2 or rad/s^2)}',
                'H39': 'clean_conf: H39',
                'H40': 'save_conf: H40',
                'H41': 'get_position: H41',
                'H42': 'get_servo_angle: H42',
                'H43': 'get_inverse_kinematics: H43 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)}',
                'H44': 'get_forward_kinematics: H44 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)}',
                'H45': 'is_joint_limit: H45 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)}',
                'H46': 'is_tcp_limit: H46 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)}',
                # 'H101': '(Danger, please do not use) set_servo_addr_16: H101 S{servo_id(1-7)} A{addr} V{value}',
                # 'H102': '(Danger, please do not use) get_servo_addr_16: H102 S{servo_id(1-7)} A{addr}',
                # 'H103': '(Danger, please do not use) set_servo_addr_32: H103 S{servo_id(1-7)} A{addr} V{value}',
                # 'H104': '(Danger, please do not use) get_servo_addr_32: H104 S{servo_id(1-7)} A{addr}',
                # 'H105': '(Danger, please do not use) set_servo_zero: H105 S{servo_id(1-7)}',
                'H106': 'get_servo_debug_msg: H106',
            }
        num = parse.gcode_get_chint(command, 'G')
        if num == 1:  # G1 xarm_move_line ex: G1 X300 Y0 Z100 A-180 B0 C0 F100 Q50 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            mvpose = parse.gcode_get_mvcarts(command)
            wait = parse.gcode_get_wait(command)
            ret = self.set_position(*mvpose, radius=-1, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, wait=wait)
            # ret = self.set_position(*mvpose, radius=-1, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, is_radian=False)
        elif num == 4:  # G4 xarm_sleep_cmd ex: G4 V1
            sltime = parse.gcode_get_mvtime(command)
            ret = self.set_pause_time(sltime)
        elif num == 7:  # G7 xarm_move_joint ex: G7 I11 J22 K33 L44 M-56 N67 O45 F50 Q30 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            mvjoint = parse.gcode_get_mvjoints(command)
            wait = parse.gcode_get_wait(command)
            ret = self.set_servo_angle(angle=mvjoint, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, wait=wait)
            # ret = self.set_servo_angle(angle=mvjoint, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, is_radian=False)
        elif num == 8:  # G8 xarm_move_gohome ex: G8 F100 Q40 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            wait = parse.gcode_get_wait(command)
            ret = self.move_gohome(speed=mvvelo, mvacc=mvacc, mvtime=mvtime, wait=wait)
            # ret = self.move_gohome(speed=mvvelo, mvacc=mvacc, mvtime=mvtime, is_radian=False)
        elif num == 9:  # G9 xarm_move_arc_line ex: G9 X300 Y0 Z100 A-180 B0 C0 R10 F100 Q50 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            mvpose = parse.gcode_get_mvcarts(command)
            mvradii = parse.gcode_get_mvradii(command)
            wait = parse.gcode_get_wait(command)
            if mvradii is None:
                mvradii = 0
            ret = self.set_position(*mvpose, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, radius=mvradii, wait=wait)
            # ret = self.set_position(*mvpose, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, radius=mvradii, is_radian=False)
        else:
            num = parse.gcode_get_chint(command, 'H')
            if num == 1:  # H0 H1 get_version ex: H0
                ret = self.get_version()
            elif num == 11:  # H11 motion_enable ex: H11 V1
                value = parse.gcode_get_chint(command, 'V')
                servo_id = parse.gcode_get_chint(command, 'S')
                if value == -1:
                    value = True
                if servo_id < 1:
                    servo_id = None
                ret = self.motion_enable(enable=value, servo_id=servo_id)
            elif num == 12:  # H12 set_state ex: H12 V0
                value = parse.gcode_get_chint(command, 'V')
                ret = self.set_state(value)
            elif num == 13:  # H13 get_state ex: H13
                ret = self.get_state()
            elif num == 14:  # H14 get_cmd_num ex: H14
                ret = self.get_cmdnum()
            elif num == 15:  # H15 get_error_warn_code ex: H15
                ret = self.get_err_warn_code()
            elif num == 16:  # H16 clean_error ex: H16
                ret = self.clean_error()
            elif num == 17:  # H17 clean_warn ex: H17
                ret = self.clean_warn()
            elif num == 18:  # H18 set_brake ex: H18 V0 S{servo_id}
                value = parse.gcode_get_chint(command, 'V')
                servo_id = parse.gcode_get_servo(command)
                if value == -1:
                    value = 0
                if value == 0:
                    ret = self.set_servo_attach(servo_id=servo_id)
                else:
                    ret = self.set_servo_detach(servo_id=servo_id)
            elif num == 31:  # H31 set_tcp_jerk ex: H31 V30
                value = parse.gcode_get_value(command)
                ret = self.set_tcp_jerk(value)
            elif num == 32:  # H32 set_tcp_maxacc ex: H32 V500
                value = parse.gcode_get_value(command)
                ret = self.set_tcp_maxacc(value)
            elif num == 33:  # H33 set_joint_jerk ex: H33 V30
                value = parse.gcode_get_value(command)
                ret = self.set_joint_jerk(value)
            elif num == 34:  # H34 set_joint_maxacc ex: H34 V100
                value = parse.gcode_get_value(command)
                ret = self.set_joint_maxacc(value)
            elif num == 39:  # H39 clean_conf ex: H39
                ret = self.clean_conf()
            elif num == 40:  # H40 save_conf ex: H40
                ret = self.save_conf()
            elif num == 41:  # H41 get_position ex: H41
                # value = parse.gcode_get_chint(command, 'V')
                # if value == -1:
                #     value = 0
                # ret = self.get_position(is_radian=value == 0)
                ret = self.get_position()
            elif num == 42:  # H42 get_servo_angle ex: H42
                # value = parse.gcode_get_chint(command, 'V')
                # if value == -1:
                #     value = 0
                # ret = self.get_servo_angle(is_radian=value == 0)
                ret = self.get_servo_angle()
            elif num == 43:  # H43 get_ik ex: H43 X100 Y0 Z100 A90 B90 C100
                pose = parse.gcode_get_mvcarts(command)
                ret = self.get_inverse_kinematics(pose, input_is_radian=False, return_is_radian=False)
            elif num == 44:  # H44 get_fk ex: H44 I11 J22 K33 L44 M-56 N67 O45
                joint = parse.gcode_get_mvjoints(command)
                ret = self.get_forward_kinematics(joint, input_is_radian=False, return_is_radian=False)
            elif num == 45:  # H45 is_joint_limit ex: H45 I11 J22 K33 L44 M-56 N67 O45
                joint = parse.gcode_get_mvjoints(command)
                ret = self.is_joint_limit(joint, is_radian=False)
            elif num == 46:  # H46 is_tcp_limit ex: H46 X100 Y0 Z100 A90 B90 C100
                pose = parse.gcode_get_mvcarts(command)
                ret = self.is_tcp_limit(pose, is_radian=False)
            elif num == 101:
                servo_id = parse.gcode_get_servo(command)
                addr = parse.gcode_get_chint(command, 'A')
                value = parse.gcode_get_value(command)
                ret = self.set_servo_addr_16(servo_id=servo_id, addr=addr, value=value)
            elif num == 102:
                servo_id = parse.gcode_get_servo(command)
                addr = parse.gcode_get_chint(command, 'A')
                ret = self.get_servo_addr_16(servo_id=servo_id, addr=addr)
            elif num == 103:
                servo_id = parse.gcode_get_servo(command)
                addr = parse.gcode_get_chint(command, 'A')
                value = parse.gcode_get_value(command)
                ret = self.set_servo_addr_32(servo_id=servo_id, addr=addr, value=value)
            elif num == 104:
                servo_id = parse.gcode_get_servo(command)
                addr = parse.gcode_get_chint(command, 'A')
                ret = self.get_servo_addr_32(servo_id=servo_id, addr=addr)
            elif num == 105:
                servo_id = parse.gcode_get_servo(command)
                ret = self.set_servo_zero(servo_id=servo_id)
            elif num == 106:
                ret = self.get_servo_debug_msg()
            else:
                logger.debug('command {} is not exist'.format(command))
                ret = APIState.CMD_NOT_EXIST, 'command {} is not exist'.format(command)
        return ret

    def _register_report_callback(self, report_id, callback):
        if report_id not in self._report_callbacks.keys():
            self._report_callbacks[report_id] = []
        if (callable(callback) or isinstance(callback, dict)) and callback not in self._report_callbacks[report_id]:
            self._report_callbacks[report_id].append(callback)
            return True
        elif not (callable(callback) or isinstance(callback, dict)):
            return False
        else:
            return True

    def _release_report_callback(self, report_id, callback):
        if report_id in self._report_callbacks.keys() and callback:
            if callback is None:
                self._report_callbacks[report_id].clear()
                return True
            elif callback:
                for cb in self._report_callbacks[report_id]:
                    if callback == cb:
                        self._report_callbacks[report_id].remove(callback)
                        return True
                    elif isinstance(cb, dict):
                        if cb['callback'] == callback:
                            self._report_callbacks[report_id].remove(cb)
                            return True
        return False

    def register_report_callback(self, callback=None, report_cartesian=True, report_joints=True,
                                 report_state=True, report_error_code=True, report_warn_code=True,
                                 report_maable=True, report_mtbrake=True, report_cmd_num=True):
        return self._register_report_callback(REPORT_ID, {
            'callback': callback,
            'cartesian': report_cartesian,
            'joints': report_joints,
            'error_code': report_error_code,
            'warn_code': report_warn_code,
            'state': report_state,
            'maable': report_maable,
            'mtbrake': report_mtbrake,
            'cmdnum': report_cmd_num
        })

    def register_report_location_callback(self, callback=None, report_cartesian=True, report_joints=False):
        ret = self._register_report_callback(REPORT_LOCATION_ID, {
            'callback': callback,
            'cartesian': report_cartesian,
            'joints': report_joints,
        })
        return ret

    def register_connect_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_CONNECT_CHANGED_ID, callback)

    def register_state_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_STATE_CHANGED_ID, callback)

    def register_maable_mtbrake_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_MAABLE_MTBRAKE_CHANGED_ID, callback)

    def register_error_warn_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_ERROR_WARN_CHANGED_ID, callback)

    def register_cmdnum_changed_callback(self, callback=None):
        return self._register_report_callback(REPORT_CMDNUM_CHANGED_ID, callback)

    def release_report_callback(self, callback=None):
        return self._release_report_callback(REPORT_ID, callback)

    def release_report_location_callback(self, callback=None):
        return self._release_report_callback(REPORT_LOCATION_ID, callback)

    def release_connect_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_CONNECT_CHANGED_ID, callback)

    def release_state_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_STATE_CHANGED_ID, callback)

    def release_maable_mtbrake_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_MAABLE_MTBRAKE_CHANGED_ID, callback)

    def release_error_warn_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_ERROR_WARN_CHANGED_ID, callback)

    def release_cmdnum_changed_callback(self, callback=None):
        return self._release_report_callback(REPORT_CMDNUM_CHANGED_ID, callback)

    @xarm_is_connected(_type='get')
    def get_servo_debug_msg(self, show=False):
        ret = self.arm_cmd.servo_get_dbmsg()
        dbmsg = []
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            for i in range(1, 9):
                servo_error = ServoError(ret[i * 2])
                dbmsg.append({
                    'name': '伺服{}'.format(i) if i < 8 else '机械爪',
                    'servo_id': i,
                    'status': ret[i * 2 - 1],
                    'error': {
                        'code': ret[i * 2],
                        'desc': servo_error.description if ret[i * 2 - 1] != 3 else '通信错误(Communication error)',
                        'handle': servo_error.handle if ret[i * 2 - 1] != 3 else ['检查连接，重新上电']
                    }
                })
        if show:
            pretty_print('************获取电机调试信息, 状态: {}*************'.format(ret[0]), color='light_blue')
            for servo_info in dbmsg:
                color = 'red' if servo_info['error']['code'] != 0 or servo_info['status'] != 0 else 'white'
                pretty_print('* {}, 状态: {}, 错误码: {}'.format(
                    servo_info['name'], servo_info['status'],
                    servo_info['error']['code']), color=color)
                if servo_info['error']['desc']:
                    pretty_print('*  错误信息: {}'.format(servo_info['error']['desc']), color=color)
                if servo_info['error']['handle']:
                    pretty_print('*  处理方法: {}'.format(servo_info['error']['handle']), color=color)
            pretty_print('*' * 50, color='light_blue')
        return ret[0], dbmsg

        # if show:
        #     if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
        #         print('=' * 50)
        #         for i in range(1, 8):
        #             if ret[i * 2 - 1] != 0:
        #                 servo_error = ServoError(ret[i * 2])
        #                 if ret[i * 2 - 1] == 3:
        #                     servo_error.description = '通信错误'
        #                 err_code = '{} ({})'.format(hex(ret[i * 2]), ret[i * 2])
        #                 print('伺服{}, 状态: {}, 错误码: {}, 错误信息: {}'.format(
        #                     i, ret[i * 2 - 1], err_code, servo_error.description))
        #                 print('处理方法: {}'.format(servo_error.handle))
        #             else:
        #                 print('伺服{}, 状态: {}, 错误码: 0'.format(i, ret[i * 2 - 1]))
        #         if ret[15] != 0:
        #             servo_error = ServoError(ret[16])
        #             if ret[15] == 3:
        #                 servo_error.description = '通信错误'
        #             err_code = '{} ({})'.format(hex(ret[16]), ret[16])
        #             print('机械爪, 状态: {}, 错误码: {}, 错误信息: {}'.format(
        #                 ret[15], err_code, servo_error.description))
        #             print('处理方法: {}'.format(servo_error.handle))
        #         else:
        #             print('机械爪, 状态: {}, 错误码: 0'.format(ret[15]))
        #         print('=' * 50)
        # return ret

    @xarm_is_connected(_type='set')
    def set_servo_zero(self, servo_id=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8
        ret = self.arm_cmd.servo_set_zero(servo_id)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_servo_addr_16(self, servo_id=None, addr=None, value=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :param value: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 7
        assert addr is not None and value is not None
        ret = self.arm_cmd.servo_addr_w16(servo_id, addr, value)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_servo_addr_16(self, servo_id=None, addr=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 7
        assert addr is not None
        ret = self.arm_cmd.servo_addr_r16(servo_id, addr)
        return ret[0], ret[1:]

    @xarm_is_connected(_type='set')
    def set_servo_addr_32(self, servo_id=None, addr=None, value=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :param value: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 7
        assert addr is not None and value is not None
        ret = self.arm_cmd.servo_addr_w32(servo_id, addr, value)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_servo_addr_32(self, servo_id=None, addr=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 7
        assert addr is not None
        ret = self.arm_cmd.servo_addr_r32(servo_id, addr)
        return ret[0], ret[1:]

    @xarm_is_connected(_type='set')
    def clean_servo_error(self, servo_id=None):
        """
        Danger, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8
        return self.set_servo_addr_16(servo_id, 0x0109, 1)


