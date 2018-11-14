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
from ..core.utils.log import logger
from ..core.config.x_code import ControllerWarn, ControllerError, ServoError
from .gripper import Gripper
from . import parse
from .code import APIState
from .utils import xarm_is_connected, xarm_is_ready

RAD_DEGREE = 57.295779513082320876798154814105
LIMIT_VELO = [1, 1000]  # mm/s
LIMIT_ACC = [1, 100000]  # mm/s^2
LIMIT_ANGLE_VELO = [1, 180]  # °/s
LIMIT_ANGLE_ACC = [1, 100000]  # °/s^2

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
                 limit_velo=None, limit_acc=None, limit_angle_velo=None, limit_angle_acc=None):
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

        self._min_angle_velo, self._max_angle_velo = limit_angle_velo if limit_angle_velo is not None and len(limit_angle_velo) >= 2 else LIMIT_ANGLE_VELO
        self._min_angle_acc, self._max_angle_acc = limit_angle_acc if limit_angle_acc is not None and len(limit_angle_acc) >= 2 else LIMIT_ANGLE_ACC

        self._stream_type = 'serial'
        self.stream = None
        self.arm_cmd = None
        self.stream_report = None
        self._report_thread = None
        self._only_report_err_warn_changed = True

        # self._last_position = [172, 0, 132, -3.14, 0, 0, 0]
        self._last_position = [201.5, 0, 140.5, -3.1415926, 0, 0, 0]  # [x, y, z, roll, yaw, pitch, radius]
        self._last_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [Axis-I, Axis-J, Axis-K, Axis-L, Axis-M, Axis-N, Axis-O]
        self._mvvelo = 100
        self._mvacc = 5000
        self._angle_mvvelo = 50
        self._angle_mvacc = 5000
        self._mvtime = 0

        self._version = None
        self._position = [0] * 6
        self._angles = [0] * 7
        self._position_offset = [0] * 6
        self._state = 4
        self._error_code = 0
        self._warn_code = 0
        self._mtbrake = [0, 0, 0, 0, 0, 0, 0, 0]  # [serve_1, serve_2, serve_3, serve_4, serve_5, serve_6, serve_7, 0]
        self._maable = [0, 0, 0, 0, 0, 0, 0, 0]  # [serve_1, serve_2, serve_3, serve_4, serve_5, serve_6, serve_7, 0]
        self._cmd_num = 0
        self._arm_type = 2
        self._arm_axis = 7
        self._arm_mid = 0
        self._arm_sid = 0
        self._arm_mttid = 0
        self._arm_mtfid = 0

        self._is_ready = False
        self.is_stop = False
        self._is_sync = False

        self.start_time = time.time()

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
        return self.stream and self.stream.connected

    @property
    def ready(self):
        return self._is_ready

    @property
    def version(self):
        if not self._version:
            self.get_version()
        return self._version

    @property
    def position(self):
        if not self._enable_report:
            self.get_position()
        return self._position

    @property
    def angles(self):
        if not self._enable_report:
            self.get_servo_angle()
        return self._angles

    @property
    def position_offset(self):
        return self._position_offset

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

    @property
    def has_err_warn(self):
        return self.arm_cmd and self.arm_cmd.has_err_warn

    def connect(self, port=None, baudrate=None, timeout=None):
        if self.connected:
            return
        # self._is_ready = False
        self._is_ready = True
        self._port = port if port is not None else self._port
        self._baudrate = baudrate if baudrate is not None else self._baudrate
        self._timeout = timeout if timeout is not None else self._timeout
        if isinstance(self._port, (str, bytes)):
            if self._port == 'localhost' or re.match(
                    r"^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$",
                    self._port):
                self.stream = SocketPort(self._port, XCONF.SocketConf.TCP_CONTROL_PORT,
                                         heartbeat=self._enable_heartbeat,
                                         buffer_size=XCONF.SocketConf.TCP_CONTROL_BUF_SIZE)
                if not self.connected:
                    raise Exception('connect socket failed')

                self._report_error_warn_changed_callback()

                self.arm_cmd = UxbusCmdTcp(self.stream)
                self._stream_type = 'socket'

                try:
                    self.connect_report()
                except:
                    self.stream_report = None

                if self.stream.connected:
                    self._report_thread = threading.Thread(target=self.report_thread_handle, daemon=True)
                    self._report_thread.start()
                self._report_connect_changed_callback()
            else:
                self.stream = SerialPort(self._port)
                if not self.connected:
                    raise Exception('connect serail failed')

                self._report_error_warn_changed_callback()

                self.arm_cmd = UxbusCmdSer(self.stream)
                self._stream_type = 'serial'
                if self._enable_report:
                    self._report_thread = threading.Thread(target=self.auto_get_report_thread, daemon=True)
                    self._report_thread.start()
                    self._report_connect_changed_callback(True, True)
                else:
                    self._report_connect_changed_callback(True, False)

    def connect_report(self):
        if self._enable_report:
            if self.stream_report:
                try:
                    self.stream_report.close()
                except:
                    pass
                time.sleep(3)
            if self._report_type == 'real':
                self._connect_report_real()
            elif self._report_type == 'rich':
                self._connect_report_rich()
            else:
                self._connect_report_normal()

    def _connect_report_normal(self):
        if self._stream_type == 'socket':
            self.stream_report = SocketPort(self._port,
                                            XCONF.SocketConf.TCP_REPORT_NORM_PORT,
                                            buffer_size=XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE)

    def _connect_report_rich(self):
        if self._stream_type == 'socket':
            self.stream_report = SocketPort(self._port,
                                            XCONF.SocketConf.TCP_REPORT_RICH_PORT,
                                            buffer_size=XCONF.SocketConf.TCP_REPORT_RICH_BUF_SIZE)

    def _connect_report_real(self):
        if self._stream_type == 'socket':
            self.stream_report = SocketPort(self._port,
                                            XCONF.SocketConf.TCP_REPORT_REAL_PORT,
                                            buffer_size=XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE)

    def _report_connect_changed_callback(self, main_connected=None, report_connected=None):
        if REPORT_CONNECT_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_CONNECT_CHANGED_ID]:
                try:
                    callback({
                        'connected': self.stream and self.stream.connected if main_connected is None else main_connected,
                        'reported': self.stream_report and self.stream_report.connected if report_connected is None else report_connected,
                    })
                except:
                    pass

    def _report_state_changed_callback(self):
        if REPORT_STATE_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_STATE_CHANGED_ID]:
                try:
                    callback({
                        'state': self.state
                    })
                except:
                    pass

    def _report_maable_mtbrake_changed_callback(self):
        if REPORT_MAABLE_MTBRAKE_CHANGED_ID in self._report_callbacks.keys():
            maable = [bool(i) for i in self.maable]
            mtbrake = [bool(i) for i in self.mtbrake]
            for callback in self._report_callbacks[REPORT_MAABLE_MTBRAKE_CHANGED_ID]:
                try:
                    callback({
                        'maable': maable.copy(),
                        'mtbrake': mtbrake.copy()
                    })
                except:
                    pass

    def _report_error_warn_changed_callback(self):
        if REPORT_ERROR_WARN_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_ERROR_WARN_CHANGED_ID]:
                try:
                    callback({
                        'warn_code': self.warn_code,
                        'error_code': self.error_code,
                    })
                except:
                    pass

    def _report_cmdnum_changed_callback(self):
        if REPORT_CMDNUM_CHANGED_ID in self._report_callbacks.keys():
            for callback in self._report_callbacks[REPORT_CMDNUM_CHANGED_ID]:
                try:
                    callback({
                        'cmdnum': self.cmd_num,
                    })
                except:
                    pass

    def _report_location_callback(self):
        if REPORT_LOCATION_ID in self._report_callbacks.keys():
            for item in self._report_callbacks[REPORT_LOCATION_ID]:
                callback = item['callback']
                ret = {}
                if item['cartesian']:
                    ret['cartesian'] = self.position
                if item['joints']:
                    ret['joints'] = self.angles
                try:
                    callback(ret)
                except:
                    pass

    def _report_callback(self):
        if REPORT_ID in self._report_callbacks.keys():
            for item in self._report_callbacks[REPORT_ID]:
                callback = item['callback']
                ret = {}
                if item['cartesian']:
                    ret['cartesian'] = self.position.copy()
                if item['joints']:
                    ret['joints'] = self.angles.copy()
                if item['error_code']:
                    ret['error_code'] = self.error_code
                if item['warn_code']:
                    ret['warn_code'] = self.warn_code
                if item['state']:
                    ret['state'] = self.state
                if item['maable']:
                    maable = [bool(i) for i in self.maable]
                    ret['maable'] = maable.copy()
                if item['mtbrake']:
                    mtbrake = [bool(i) for i in self.mtbrake]
                    ret['mtbrake'] = mtbrake.copy()
                if item['cmdnum']:
                    ret['cmdnum'] = self.cmd_num
                try:
                    callback(ret)
                except:
                    pass

    def report_thread_handle(self):
        def _handle_report_normal(rx_data):
            # print('length:', convert.bytes_to_u32(rx_data[0:4]))
            state, mtbrake, maable, error_code, warn_code = rx_data[4:9]
            angles = convert.bytes_to_fp32s(rx_data[9:7 * 4 + 9], 7)
            pose = convert.bytes_to_fp32s(rx_data[37:6 * 4 + 37], 6)
            cmd_num = convert.bytes_to_u16(rx_data[61:63])
            pose_offset = convert.bytes_to_fp32s(rx_data[63:6 * 4 + 63], 6)

            if error_code != self.error_code or warn_code != self.warn_code:
                self._warn_code = warn_code
                self._error_code = error_code
                if self._only_report_err_warn_changed:
                    self._report_error_warn_changed_callback()
            if not self._only_report_err_warn_changed:
                self._report_error_warn_changed_callback()

            if cmd_num != self.cmd_num:
                self._cmd_num = cmd_num
                self._report_cmdnum_changed_callback()

            if state != self._state:
                self._state = state
                self._report_state_changed_callback()
            if state == 4:
                self._is_ready = False
            else:
                self._is_ready = True

            mtbrake = [mtbrake & 0x01, mtbrake >> 1 & 0x01, mtbrake >> 2 & 0x01, mtbrake >> 3 & 0x01,
                       mtbrake >> 4 & 0x01, mtbrake >> 5 & 0x01, mtbrake >> 6 & 0x01, mtbrake >> 7 & 0x01]
            maable = [maable & 0x01, maable >> 1 & 0x01, maable >> 2 & 0x01, maable >> 3 & 0x01,
                      maable >> 4 & 0x01, maable >> 5 & 0x01, maable >> 6 & 0x01, maable >> 7 & 0x01]

            if mtbrake != self._mtbrake or maable != self._maable:
                self._maable = maable
                self._mtbrake = mtbrake
                self._report_maable_mtbrake_changed_callback()

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
                    # if abs(pose[i] - self._position[i]) < 0.005:
                    #     pose[i] = self._position[i]
                else:
                    pose[i] = float('{:.6f}'.format(pose[i][0]))
                    # if abs(pose[i] - self._position[i]) < 0.000005 or abs(pose[i] - self._position[i]) > 6.2:
                    #     pose[i] = self._position[i]
                    # if abs(float('{:.5f}'.format(pose[i]))) == abs(float('{:.5f}'.format(self._position[i]))) == 3.14159:
                    #     pose[i] = self._position[i]
            for i in range(len(angles)):
                angles[i] = float('{:.6f}'.format(angles[i][0]))
                # if abs(angles[i] - self._angles[i]) < 0.000005 or abs(angles[i] - self._angles[i]) > 6.2:
                #     angles[i] = self._angles[i]
                # if abs(float('{:.5f}'.format(angles[i]))) == abs(float('{:.5f}'.format(self._angles[i]))) == 3.14159:
                #     angles[i] = self._angles[i]
            for i in range(len(pose_offset)):
                if i < 3:
                    pose_offset[i] = float('{:.3f}'.format(pose_offset[i][0]))
                    # if abs(pose_offset[i] - self._position_offset[i]) < 0.005:
                    #     pose_offset[i] = self._position_offset[i]
                else:
                    pose_offset[i] = float('{:.6f}'.format(pose_offset[i][0]))
                    # if abs(pose_offset[i] - self._position_offset[i]) < 0.000005:
                    #     pose_offset[i] = self._position_offset[i]

            # if math.inf not in pose:
            #     pose[pose.index(math.inf)] = 0
            # if math.inf in angles:
            #     angles[angles.index(math.inf)] = 0
            # if math.inf in pose_offset:
            #     pose_offset[pose_offset.index(math.inf)] = 0

            if math.inf not in pose:
                self._position = pose
            if math.inf not in angles:
                self._angles = angles
            if math.inf not in pose_offset:
                self._position_offset = pose_offset

            self._report_location_callback()

            if self.state != 1:
                if time.time() - self.start_time > 3:
                    # self._last_position[:6] = self.position
                    # self._last_angles = angles
                    self.start_time = time.time()
            else:
                self.start_time = time.time()

            # print('state: {}, mtbrake: {}, maable: {}, err: {}, warn: {}, cmdnum: {}'.format(
            #     self.state, mtbrake, maable, self.error_code, self.warn_code, self.cmd_num
            # ))
            # print('angles: {}'.format(self.angles))
            # print('position: {}'.format(self.position))
            # print('position offset: {}'.format(self.position_offset))
            self._report_callback()
            if not self._is_sync:
                self.sync()
                self._is_sync = True

        def _handle_report_rich(rx_data):
            _handle_report_normal(rx_data)
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

        main_socket_connected = self.stream and self.stream.connected
        report_socket_connected = self.stream_report and self.stream_report.connected
        while self.stream and self.stream.connected:
            try:
                if not self.stream_report or not self.stream_report.connected:
                    if report_socket_connected:
                        report_socket_connected = False
                        self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                    self.connect_report()
                    continue
                if not report_socket_connected:
                    report_socket_connected = True
                    self._report_connect_changed_callback(main_socket_connected, report_socket_connected)
                rx_data = self.stream_report.read()
                if rx_data != -1 and len(rx_data) >= XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE:
                    if len(rx_data) == XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE:
                        _handle_report_normal(rx_data)
                    elif len(rx_data) >= XCONF.SocketConf.TCP_REPORT_NORMAL_BUF_SIZE:
                        _handle_report_rich(rx_data)
            except Exception as e:
                logger.error(e)
            time.sleep(0.001)
        self.disconnect()
        self._report_connect_changed_callback(False, False)

    def auto_get_report_thread(self):
        logger.debug('get report thread start')
        while self.connected:
            try:
                if self.cmd_num > 300:
                    time.sleep(0.02)
                self.get_position()
                time.sleep(0.02)
                self.get_servo_angle()
                time.sleep(0.02)
                state = self.state
                self.get_state()
                time.sleep(0.02)
                cmd_num = self.cmd_num
                self.get_cmdnum()
                self._report_location_callback()
                if cmd_num != self.cmd_num:
                    self._report_cmdnum_changed_callback()
                if self.state == 4:
                    self._is_ready = False
                else:
                    self._is_ready = True
                if state != self.state:
                    self._report_state_changed_callback()
                if self.arm_cmd.has_err_warn:
                    self.get_err_warn_code()
                    self._report_error_warn_changed_callback()
                self._report_callback()
                time.sleep(0.02)
            except:
                pass
        self._report_connect_changed_callback(False, False)
        logger.debug('get report thread stopped')

    def disconnect(self):
        self.stream.close()
        if self.stream_report:
            self.stream_report.close()
        self._is_ready = False
        self.stream.join()
        if self.stream_report:
            self.stream_report.join()
        self._report_connect_changed_callback(False, False)

    def sync(self):
        if not self.stream_report or not self.stream_report.connected:
            self.get_position()
            self.get_servo_angle()
        self._last_position[:6] = self.position
        self._last_angles = self.angles

    class WaitMove:
        def __init__(self, owner, timeout):
            self.owner = owner
            self.timeout = timeout if timeout is not None else 10
            self.timer = None
            self.is_timeout = False

        def start(self):
            self.timer = threading.Timer(self.timeout, self.timeout_cb)
            self.timer.start()
            self.check_stop_move()

        def check_stop_move(self):
            base_joint_pos = self.owner.angles.copy()
            time.sleep(0.1)
            count = 0
            while not self.is_timeout and not self.owner.is_stop and self.owner.connected:
                if self.owner.angles == base_joint_pos or self.owner.state != 1:
                    count += 1
                    if count >= 6:
                        break
                else:
                    base_joint_pos = self.owner.angles.copy()
                    count = 0
                time.sleep(0.05)
            if not self.is_timeout:
                self.owner.sync()

        def timeout_cb(self):
            self.is_timeout = True

    @xarm_is_connected
    def get_position(self, is_radian=True):
        ret = self.arm_cmd.get_tcp_pose()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) > 6:
            self._position = [float('{:.6f}'.format(ret[i][0])) for i in range(1, 7)]
            ret[0] = 0
        if is_radian:
            return ret[0], self.position
        else:
            return ret[0], [self.position[i] * RAD_DEGREE if 2 < i < 6 else self.position[i] for i in range(len(self.position))]

    @xarm_is_ready
    def set_position(self, x=None, y=None, z=None, roll=None, yaw=None, pitch=None, radius=None,
                     speed=None, mvacc=None, mvtime=None, relative=False, is_radian=True,
                     wait=False, timeout=None, **kwargs):
        tcp_pos = [x, y, z, roll, yaw, pitch, radius]
        for i in range(7):
            if tcp_pos[i] is None:
                continue
            elif isinstance(tcp_pos[i], str):
                tcp_pos[i] = float(tcp_pos[i])
            if relative:
                if 2 < i < 6:
                    if is_radian:
                        self._last_position[i] += tcp_pos[i]
                    else:
                        self._last_position[i] += tcp_pos[i] / RAD_DEGREE
                else:
                    self._last_position[i] += tcp_pos[i]
            else:
                if 2 < i < 6:
                    if is_radian:
                        self._last_position[i] = tcp_pos[i]
                    else:
                        self._last_position[i] = tcp_pos[i] / RAD_DEGREE
                else:
                    self._last_position[i] = tcp_pos[i]

        if speed is not None:
            if isinstance(speed, str):
                speed = float(speed)
            self._mvvelo = min(max(speed, self._min_velo), self._max_velo)
        elif kwargs.get('mvvelo', None) is not None:
            mvvelo = kwargs.get('mvvelo')
            if isinstance(mvvelo, str):
                mvvelo = float(mvvelo)
            self._mvvelo = min(max(mvvelo, self._min_velo), self._max_velo)
        if mvacc is not None:
            if isinstance(mvacc, str):
                mvacc = float(mvacc)
            self._mvacc = min(max(mvacc, self._min_acc), self._max_acc)
        if mvtime is not None:
            if isinstance(mvtime, str):
                mvtime = float(mvtime)
            self._mvtime = mvtime

        if kwargs.get('check', False):
            _, limit = self.is_tcp_limit(self._last_position[:6])
            if _ == 0 and limit is True:
                return APIState.TCP_LIMIT
            # ret = self.arm_cmd.is_tcp_limit(self._last_position[:6])
            # if ret[0] not in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] or bool(ret[1]) is not False:
            #     return APIState.TCP_LIMIT
        if radius is not None:
            ret = self.arm_cmd.move_lineb(self._last_position[:6], self._mvvelo, self._mvacc, self._mvtime, self._last_position[6])
            if ret[0] != 0:
                logger.debug('exception({}): move arc line: pos={}, mvvelo={}, mvacc={}, mvtime={}'.format(
                    ret[0], self._last_position, self._mvvelo, self._mvacc, self._mvtime
                ))
            else:
                logger.debug('move arc line: {}, mvvelo={}, mvacc={}, mvtime={}'.format(
                    self._last_position, self._mvvelo, self._mvacc, self._mvtime
                ))
        else:
            ret = self.arm_cmd.move_line(self._last_position[:6], self._mvvelo, self._mvacc, self._mvtime)
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
            self.is_stop = False
            self.WaitMove(self, timeout).start()
            self.is_stop = False
        return ret[0]

    @xarm_is_connected
    def get_servo_angle(self, servo_id=None, is_radian=True):
        """
        :param servo_id: 1-7, None(8)
        :param is_radian: if True return radian else return degree
        :return: 
        """
        ret = self.arm_cmd.get_joint_pos()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) > 7:
            self._angles = [float('{:.6f}'.format(ret[i][0])) for i in range(1, 8)]
            ret[0] = 0
        if servo_id is None or servo_id == 8 or len(self._angles) < servo_id:
            if is_radian:
                return ret[0], self._angles
            else:
                return ret[0], [self.angles[i] * RAD_DEGREE for i in range(len(self.angles))]
        else:
            if is_radian:
                return ret[0], self._angles[servo_id-1]
            else:
                return ret[0], self._angles[servo_id-1] * RAD_DEGREE

    @xarm_is_ready
    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None,
                        relative=False, is_radian=True, wait=False, timeout=None, **kwargs):
        """
        :param servo_id: 1-7, None(8)
        :param angle: 
        :param speed: 
        :param mvacc: 
        :param mvtime: 
        :param relative: 
        :param is_radian: 
        :param wait:
        :param timeout:
        :return: 
        """
        assert servo_id is None or (isinstance(servo_id, int) and 1 <= servo_id <= 8)
        if servo_id is None or servo_id == 8:
            if not isinstance(angle, Iterable):
                logger.error('If servo_id is None or 8, the argument angle must be an iterable object')
                return
            else:
                for i in range(min(len(angle), len(self._last_angles))):
                    if angle[i] is None:
                        continue
                    if relative:
                        try:
                            if is_radian:
                                self._last_angles[i] += float(angle[i])
                            else:
                                self._last_angles[i] += float(angle[i]) / RAD_DEGREE
                        except:
                            pass
                    else:
                        try:
                            if is_radian:
                                self._last_angles[i] = float(angle[i])
                            else:
                                self._last_angles[i] = float(angle[i]) / RAD_DEGREE
                        except:
                            pass
        else:
            if angle is None:
                logger.error('The argument angle must be an number')
                return 0
            elif isinstance(angle, str):
                angle = float(angle)
            if relative:
                if is_radian:
                    self._last_angles[servo_id - 1] += angle
                else:
                    self._last_angles[servo_id - 1] += angle / RAD_DEGREE
            else:
                if is_radian:
                    self._last_angles[servo_id - 1] = angle
                else:
                    self._last_angles[servo_id - 1] = angle / RAD_DEGREE

        if speed is not None:
            if isinstance(speed, str):
                speed = float(speed)
            if is_radian:
                speed *= RAD_DEGREE
            self._angle_mvvelo = min(max(speed, self._min_angle_velo), self._max_angle_velo)
        elif kwargs.get('mvvelo', None) is not None:
            mvvelo = kwargs.get('mvvelo')
            if isinstance(mvvelo, str):
                mvvelo = float(mvvelo)
            if is_radian:
                mvvelo *= RAD_DEGREE
            self._angle_mvvelo = min(max(mvvelo, self._min_angle_velo), self._max_angle_velo)
        if mvacc is not None:
            if isinstance(mvacc, str):
                mvacc = float(mvacc)
            if is_radian:
                mvacc *= RAD_DEGREE
            self._angle_mvacc = min(max(mvacc, self._min_angle_acc), self._max_angle_acc)
        if mvtime is not None:
            if isinstance(mvtime, str):
                mvtime = float(mvtime)
            self._mvtime = mvtime

        if kwargs.get('check', False):
            _, limit = self.is_joint_limit(self._last_angles)
            if _ == 0 and limit is True:
                return APIState.JOINT_LIMIT
            # ret = self.arm_cmd.is_joint_limit(self._last_angles)
            # if ret[0] not in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] or bool(ret[1]) is not False:
            #     return APIState.JOINT_LIMIT

        ret = self.arm_cmd.move_joint(self._last_angles, self._angle_mvvelo / RAD_DEGREE, self._angle_mvacc / RAD_DEGREE, self._mvtime)
        if ret[0] != 0:
            logger.debug('exception({}): move joint: joint={}, mvvelo={}, mvacc={}, mvtime={}'.format(
                ret[0], self._last_angles, self._angle_mvvelo / RAD_DEGREE, self._angle_mvacc / RAD_DEGREE, self._mvtime
            ))
        else:
            logger.debug('move joint: {}, mvvelo={}, mvacc={}, mvtime={}'.format(
                self._last_angles, self._angle_mvvelo / RAD_DEGREE, self._angle_mvacc / RAD_DEGREE, self._mvtime
            ))
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                logger.warn('if you want to wait, please enable report')
                return ret[0]
            self.is_stop = False
            self.WaitMove(self, timeout).start()
            self.is_stop = False
        return ret[0]

    @xarm_is_ready
    def move_gohome(self, speed=None, mvacc=None, mvtime=None, is_radian=True, wait=False, timeout=None, **kwargs):
        if speed is not None:
            if isinstance(speed, str):
                speed = float(speed)
            if is_radian:
                speed *= RAD_DEGREE
            self._angle_mvvelo = min(max(speed, self._min_angle_velo), self._max_angle_velo)
        elif kwargs.get('mvvelo', None) is not None:
            mvvelo = kwargs.get('mvvelo')
            if isinstance(mvvelo, str):
                mvvelo = float(mvvelo)
            if is_radian:
                mvvelo *= RAD_DEGREE
            self._angle_mvvelo = min(max(mvvelo, self._min_angle_velo), self._max_angle_velo)
        if mvacc is not None:
            if isinstance(mvacc, str):
                mvacc = float(mvacc)
            if is_radian:
                mvacc *= RAD_DEGREE
            self._angle_mvacc = min(max(mvacc, self._min_angle_acc), self._max_angle_acc)
        if mvtime is not None:
            if isinstance(mvtime, str):
                mvtime = float(mvtime)
            self._mvtime = mvtime

        ret = self.arm_cmd.move_gohome(self._angle_mvvelo / RAD_DEGREE, self._angle_mvacc / RAD_DEGREE, self._mvtime)
        if ret[0] != 0:
            logger.debug('exception({}): move gohome: mvvelo={}, mvacc={}, mvtime={}'.format(
                ret[0], self._angle_mvvelo / RAD_DEGREE, self._angle_mvacc / RAD_DEGREE, self._mvtime
            ))
        else:
            logger.debug('move gohome: mvvelo={}, mvacc={}, mvtime={}'.format(
                self._angle_mvvelo / RAD_DEGREE, self._angle_mvacc / RAD_DEGREE, self._mvtime
            ))
        if wait and ret[0] in [0, XCONF.UxbusState.WAR_CODE, XCONF.UxbusState.ERR_CODE]:
            if not self._enable_report:
                logger.warn('if you want to wait, please enable report')
                return ret[0]
            self.is_stop = False
            self.WaitMove(self, timeout).start()
            self.is_stop = False
        return ret[0]

    @xarm_is_connected
    def set_servo_attach(self, servo_id=None):
        """
        类型标注语法: version >= python3.5
        https://www.python.org/dev/peps/pep-0484/#type-aliases
        :param servo_id: 1-7, 8
        :return: 
        """
        # if servo_id is None or servo_id == 8:
        #     ret = self.arm_cmd.set_brake(8, 0)
        # else:
        #     ret = self.arm_cmd.set_brake(servo_id, 0)
        # # self.arm_cmd.set_state(0)
        # return ret[0]
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8
        if servo_id is None or servo_id == 8:
            ret = self.motion_enable(True)
        else:
            ret = self.motion_enable(servo_id=servo_id, enable=True)
        return ret

    @xarm_is_connected
    def set_servo_detach(self, servo_id=None):
        """
        :param servo_id: 1-7, 8
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8
        if servo_id == 8:
            ret = self.arm_cmd.set_brake(8, 1)
        else:
            ret = self.arm_cmd.set_brake(servo_id, 1)
        return ret[0]

    @xarm_is_connected
    def get_version(self):
        ret = self.arm_cmd.get_version()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            version = ''.join(list(map(chr, ret[1:])))
            self._version = version[:version.find('\0')]
            ret[0] = 0
            return ret[0], self._version
        else:
            return ret[0], self._version

    @xarm_is_connected
    def get_is_moving(self):
        self.get_state()
        return self.state == 1

    @xarm_is_connected
    def get_state(self):
        ret = self.arm_cmd.get_state()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._state = ret[1]
            ret[0] = 0
        return ret[0], self._state

    @xarm_is_connected
    def set_state(self, state=0):
        ret = self.arm_cmd.set_state(state)
        if state == 4 and ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._last_position[:6] = self.position
            self._last_angles = self.angles
        return ret[0]

    @xarm_is_connected
    def get_cmdnum(self):
        ret = self.arm_cmd.get_cmdnum()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._cmd_num = ret[1]
            ret[0] = 0
        return ret[0], self.cmd_num

    @xarm_is_connected
    def get_err_warn_code(self):
        ret = self.arm_cmd.get_err_code()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._error_code, self._warn_code = ret[1:3]
            ret[0] = 0
        return ret[0], [self._error_code, self._warn_code]

    @xarm_is_connected
    def clean_error(self):
        ret = self.arm_cmd.clean_err()
        return ret[0]

    @xarm_is_connected
    def clean_warn(self):
        ret = self.arm_cmd.clean_war()
        return ret[0]

    @xarm_is_connected
    def motion_enable(self, enable=True, servo_id=None):
        """
        :param enable: 
        :param servo_id: 1-7, None(8)
        :return: 
        """
        assert servo_id is None or (isinstance(servo_id, int) and 1 <= servo_id <= 8)
        if servo_id is None or servo_id == 8:
            ret = self.arm_cmd.motion_en(8, int(enable))
        else:
            ret = self.arm_cmd.motion_en(servo_id, int(enable))
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._is_ready = bool(enable)
        return ret[0]

    def reset(self, speed=None, is_radian=False, wait=False, timeout=None):
        self.motion_enable(enable=True, servo_id=8)
        # self.set_servo_attach()
        self.set_state(0)
        self.move_gohome(speed=speed, is_radian=is_radian, wait=wait, timeout=timeout)

    @xarm_is_connected
    def set_sleep_time(self, sltime, wait=False):
        ret = self.arm_cmd.sleep_instruction(sltime)
        if wait:
            time.sleep(sltime)
        return ret[0]

    @xarm_is_connected
    def set_tcp_offset(self, offset):
        ret = self.arm_cmd.set_tcp_offset(offset)
        return ret[0]

    @xarm_is_connected
    def set_tcp_jerk(self, jerk):
        ret = self.arm_cmd.set_tcp_jerk(jerk)
        return ret[0]

    @xarm_is_connected
    def set_tcp_maxacc(self, acc):
        ret = self.arm_cmd.set_tcp_maxacc(acc)
        return ret[0]

    @xarm_is_connected
    def set_joint_jerk(self, jerk):
        ret = self.arm_cmd.set_joint_jerk(jerk)
        return ret[0]

    @xarm_is_connected
    def set_joint_maxacc(self, acc):
        ret = self.arm_cmd.set_joint_maxacc(acc)
        return ret[0]

    @xarm_is_connected
    def clean_conf(self):
        ret = self.arm_cmd.clean_conf()
        return ret[0]

    @xarm_is_connected
    def save_conf(self):
        ret = self.arm_cmd.save_conf()
        return ret[0]

    @xarm_is_connected
    def get_ik(self, pose, is_radian=True):
        assert len(pose) >= 6
        if not is_radian:
            pose = [pose[i] if i < 3 else pose[i] / RAD_DEGREE for i in range(6)]
        ret = self.arm_cmd.get_ik(pose)
        angles = []
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            angles = [ret[i][0] for i in range(1, 8)]
            ret[0] = 0
            if not is_radian:
                angles = [angle * RAD_DEGREE for angle in angles]
        return ret[0], angles

    @xarm_is_connected
    def get_fk(self, angles, is_radian=True):
        assert len(angles) >= 7
        if not is_radian:
            angles = [angles[i] / RAD_DEGREE for i in range(7)]
        ret = self.arm_cmd.get_fk(angles)
        pose = []
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            pose = [ret[i][0] for i in range(1, 7)]
            ret[0] = 0
            if not is_radian:
                pose = [pose[i] if i < 3 else pose[i] * RAD_DEGREE for i in range(len(pose))]
        return ret[0], pose

    @xarm_is_connected
    def is_tcp_limit(self, pose, is_radian=True):
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

    @xarm_is_connected
    def is_joint_limit(self, joint, is_radian=True):
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

    def set_params(self, **kwargs):
        is_radian = kwargs.get('is_radian', False)
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
        if 'R' in kwargs and isinstance(kwargs['R'], (int, float)):
            self._last_position[6] = kwargs.get('R')
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
            self._angle_mvvelo = min(max(self._angle_mvvelo, self._min_angle_velo), self._max_angle_velo)
        if 'Q2' in kwargs and isinstance(kwargs['Q2'], (int, float)):
            self._angle_mvacc = kwargs.get('Q2')
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

    def get_params(self):
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

    def emergency_stop(self):
        start_time = time.time()
        while self.state != 4 and time.time() - start_time < 3:
            self.set_state(4)
            time.sleep(0.1)
        self.is_stop = True
        self.set_state(0)

    def send_cmd_async(self, command, timeout=None):
        pass

    def send_cmd_sync(self, command=None):
        if command is None:
            return 0
        if command.lower() == 'help':
            return 0, {
                'G1': 'move tcp, G1 X300 Y0 Z100 A-180 B0 C0 F100 Q5000',
                'G4': 'sleep, G4 V3',
                'G7': 'move joint, G7 I0 J0 K0 L0 M0 N0 O0 F30 Q5000',
                'G8': 'move home, G8 F50 Q5000',
                'G9': 'move tcp with blend, G9 X300 Y0 Z100 A-180 B0 C0 R10 F100 Q5000',
                'H1': 'get version',
                'H11': 'enable/disable motor, H11 V{1: enable, 0: disable} S{servo_id}',
                'H12': 'set state, H12 V{0: normal, 3: pause, 4: stop}',
                'H13': 'get state',
                'H14': 'get cmd num',
                'H15': 'get error warn code',
                'H16': 'clean error',
                'H17': 'clean warning',
                'H18': 'enable/disable brake, H18 V{1: enable(detach), 0: disable(attach)} S{servo_id}',
                'H31': 'set tcp jerk, H31 V{jerk}',
                'H32': 'set tcp maxacc, H32 V{maxacc}',
                'H33': 'set joint jerk, H33 V{jerk}',
                'H34': 'set joint maxacc, H34 V{maxacc}',
                'H41': 'get tcp pose, H41 V{0: is_radian, 1: not_radian}',
                'H42': 'get joint pose, H42 V{0: is_radian, 1: not_radian}',
                'H43': 'get ik, H43 X100 Y0 Z100 A90 B90 C100',
                'H44': 'get fk, H44 I11 J22 K33 L44 M-56 N67 O45',
                'H45': 'is joint limit, H45 I11 J22 K33 L44 M-56 N67 O45',
                'H46': 'is tcp limit, H46 X100 Y0 Z100 A90 B90 C100',
                'H101': 'warning, set servo addr w16, H101 S{servo_id} A{addr} V{value}',
                'H102': 'warning, get servo addr r16, H102 S{servo_id} A{addr}',
                'H103': 'warning, ser servo addr w32, H103 S{servo_id} A{addr} V{value}',
                'H104': 'warning, get servo addr r32, H104 S{servo_id} A{addr}',
                'H105': 'warning, set servo zero, H105 S{servo_id}',
                'H106': 'get servo debug msg',
            }
        num = parse.gcode_get_chint(command, 'G')
        if num == 1:  # G1 xarm_move_line ex: G1 X300 Y0 Z100 A-180 B0 C0 F100 Q50 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            mvpose = parse.gcode_get_mvcarts(command)
            ret = self.set_position(*mvpose, radius=None, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, is_radian=False)
        elif num == 4:  # G4 xarm_sleep_cmd ex: G4 V1
            sltime = parse.gcode_get_mvtime(command)
            ret = self.set_sleep_time(sltime)
        elif num == 7:  # G7 xarm_move_joint ex: G7 I11 J22 K33 L44 M-56 N67 O45 F50 Q30 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            mvjoint = parse.gcode_get_mvjoints(command)
            ret = self.set_servo_angle(angle=mvjoint, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, is_radian=False)
        elif num == 8:  # G8 xarm_move_gohome ex: G8 F100 Q40 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            ret = self.move_gohome(speed=mvvelo, mvacc=mvacc, mvtime=mvtime, is_radian=False)
        elif num == 9:  # G9 xarm_move_arc_line ex: G9 X300 Y0 Z100 A-180 B0 C0 R10 F100 Q50 T0
            mvvelo = parse.gcode_get_mvvelo(command)
            mvacc = parse.gcode_get_mvacc(command)
            mvtime = parse.gcode_get_mvtime(command)
            mvpose = parse.gcode_get_mvcarts(command)
            mvradii = parse.gcode_get_mvradii(command)
            if mvradii is None:
                mvradii = 0
            ret = self.set_position(*mvpose, speed=mvvelo, mvacc=mvacc, mvtime=mvtime, radius=mvradii, is_radian=False)
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
                servo_id = parse.gcode_get_chint(command, 'S')
                if value == -1:
                    value = 0
                if servo_id < 1:
                    servo_id = None
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
            elif num == 41:  # H41 get_position ex: H41 V0
                value = parse.gcode_get_chint(command, 'V')
                if value == -1:
                    value = 0
                ret = self.get_position(is_radian=value == 0)
            elif num == 42:  # H42 get_servo_angle ex: H42 V0
                value = parse.gcode_get_chint(command, 'V')
                if value == -1:
                    value = 0
                ret = self.get_servo_angle(is_radian=value == 0)
            elif num == 43:  # H43 get_ik ex: H43 X100 Y0 Z100 A90 B90 C100
                pose = parse.gcode_get_mvcarts(command)
                ret = self.get_ik(pose, is_radian=False)
            elif num == 44:  # H44 get_fk ex: H44 I11 J22 K33 L44 M-56 N67 O45
                joint = parse.gcode_get_mvjoints(command)
                ret = self.get_fk(joint, is_radian=False)
            elif num == 45:  # H45 is_joint_limit ex: H45 I11 J22 K33 L44 M-56 N67 O45
                joint = parse.gcode_get_mvjoints(command)
                ret = self.is_joint_limit(joint, is_radian=False)
            elif num == 46:  # H46 is_tcp_limit ex: H46 X100 Y0 Z100 A90 B90 C100
                pose = parse.gcode_get_mvcarts(command)
                ret = self.is_tcp_limit(pose, is_radian=False)
            elif num == 101:
                servo_id = parse.gcode_get_chint(command, 'S')
                if servo_id < 1:
                    servo_id = None
                addr = parse.gcode_get_chint(command, 'A')
                value = parse.gcode_get_value(command)
                ret = self.set_servo_addr_16(servo_id=servo_id, addr=addr, value=value)
            elif num == 102:
                servo_id = parse.gcode_get_chint(command, 'S')
                if servo_id < 1:
                    servo_id = None
                addr = parse.gcode_get_chint(command, 'A')
                ret = self.get_servo_addr_16(servo_id=servo_id, addr=addr)
            elif num == 103:
                servo_id = parse.gcode_get_chint(command, 'S')
                if servo_id < 1:
                    servo_id = None
                addr = parse.gcode_get_chint(command, 'A')
                value = parse.gcode_get_value(command)
                ret = self.set_servo_addr_32(servo_id=servo_id, addr=addr, value=value)
            elif num == 104:
                servo_id = parse.gcode_get_chint(command, 'S')
                if servo_id < 1:
                    servo_id = None
                addr = parse.gcode_get_chint(command, 'A')
                ret = self.get_servo_addr_32(servo_id=servo_id, addr=addr)
            elif num == 105:
                servo_id = parse.gcode_get_chint(command, 'S')
                if servo_id < 1:
                    servo_id = None
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
            elif callback in self._report_callbacks[report_id]:
                self._report_callbacks[report_id].remove(callback)
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

    @xarm_is_connected
    def set_servo_zero(self, servo_id=None):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8
        ret = self.arm_cmd.servo_set_zero(servo_id)
        return ret[0]

    @xarm_is_connected
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
            print('************获取电机调试信息, 状态: {}*************'.format(dbmsg['code']))
            for servo_info in dbmsg:
                print('* {}, 状态: {}, 错误码: {}'.format(
                    servo_info['name'], servo_info['status'],
                    servo_info['error']['code']))
                if servo_info['error']['desc']:
                    print('*  错误信息: {}'.format(servo_info['error']['desc']))
                if servo_info['error']['handle']:
                    print('*  处理方法: {}'.format(servo_info['error']['handle']))
            print('*' * 50)
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

    @xarm_is_connected
    def set_servo_addr_16(self, servo_id=None, addr=None, value=None):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :param value: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 7
        assert addr is not None and value is not None
        ret = self.arm_cmd.servo_addr_w16(servo_id, addr, value)
        return ret[0]

    @xarm_is_connected
    def get_servo_addr_16(self, servo_id=None, addr=None):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 7
        assert addr is not None
        return self.arm_cmd.servo_addr_r16(servo_id, addr)

    @xarm_is_connected
    def set_servo_addr_32(self, servo_id=None, addr=None, value=None):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :param value: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 7
        assert addr is not None and value is not None
        ret = self.arm_cmd.servo_addr_w32(servo_id, addr, value)
        return ret[0]

    @xarm_is_connected
    def get_servo_addr_32(self, servo_id=None, addr=None):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :param addr: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 7
        assert addr is not None
        return self.arm_cmd.servo_addr_r32(servo_id, addr)

    @xarm_is_connected
    def clean_servo_error(self, servo_id=None):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :param servo_id: 
        :return: 
        """
        assert isinstance(servo_id, int) and 1 <= servo_id <= 8
        return self.set_servo_addr_16(servo_id, 0x0109, 1)


