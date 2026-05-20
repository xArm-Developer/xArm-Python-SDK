#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2026, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
import math
import ctypes

C_INT8 = ctypes.c_int8
C_UINT8 = ctypes.c_uint8
C_CHAR = ctypes.c_char
C_INT16 = ctypes.c_int16
C_UINT16 = ctypes.c_uint16
C_UINT32 = ctypes.c_uint32
C_UINT64 = ctypes.c_uint64
C_FLOAT = ctypes.c_float
BigEndianStructure = ctypes.BigEndianStructure
LittleEndianStructure = ctypes.LittleEndianStructure

class _BaseValue(object):
    def get_value(self, *args, **kwargs):
        return self._value
    def __str__(self):
        return '{}({}: {})'.format(self.__class__.__name__, type(self._value).__name__, self._value)

class _BUInt16(_BaseValue, BigEndianStructure):
    _pack_ = 1
    _fields_ = [('_value', C_UINT16)]

class _BInt16(_BaseValue, BigEndianStructure):
    _pack_ = 1
    _fields_ = [('_value', C_INT16)]

class _BUInt32(_BaseValue, BigEndianStructure):
    _pack_ = 1
    _fields_ = [('_value', C_UINT32)]

class _BUInt64(_BaseValue, BigEndianStructure):
    _pack_ = 1
    _fields_ = [('_value', C_UINT64)]

class _Fp32(_BaseValue, LittleEndianStructure):
    _pack_ = 1
    _fields_ = [('_value', C_FLOAT)]

class _JointFp32(_Fp32):
    def get_value(self, is_radian=True):
        val = super().get_value()
        return val if is_radian else math.degrees(val)

class _BaseArray(ctypes.Array):
    _type_ = C_UINT8
    _length_ = 0
    def to_list(self, *args, **kwargs):
        return [self[i].get_value(*args, **kwargs) if isinstance(self[i], _BaseValue) else self[i] for i in range(self._length_)]
    def __str__(self):
        return '{}({}: {})'.format(self.__class__.__name__, self._type_.__name__, self.to_list())
class _UInt8Array(_BaseArray): _type_ = C_UINT8
class _UInt8Array2(_UInt8Array): _length_ = 2
class _UInt8Array8(_UInt8Array): _length_ = 8
class _UInt8Array14(_UInt8Array): _length_ = 14
class _UInt8Array17(_UInt8Array): _length_ = 17
class _UInt8Array40(_UInt8Array): _length_ = 40

class _Int8Array(_BaseArray): _type_ = C_INT8
class _Int8Array7(_Int8Array): _length_ = 7

class _Fp32Array(_BaseArray): _type_ = _Fp32
class _Fp32Array3(_Fp32Array): _length_ = 3
class _Fp32Array4(_Fp32Array): _length_ = 4
class _Fp32Array6(_Fp32Array): _length_ = 6
class _Fp32Array7(_Fp32Array): _length_ = 7
class _Fp32Array24(_Fp32Array): _length_ = 24
class _Fp32Array42(_Fp32Array): _length_ = 42

class _JointFp32Array(_Fp32Array):
    def to_list(self, is_radian=True):
        vals = super().to_list(is_radian=True)
        return [vals[i] if is_radian else math.degrees(vals[i]) for i in range(self._length_)]
class _JointFp32Array3(_JointFp32Array): _length_ = 3
class _JointFp32Array7(_JointFp32Array): _length_ = 7
class _JointFp32Array14(_JointFp32Array): _length_ = 14
class _TcpFp32Array6(_Fp32Array6):
    def to_list(self, is_radian=True):
        vals = super().to_list(is_radian=True)
        return [vals[i] if i < 3 or is_radian else math.degrees(vals[i]) for i in range(self._length_)]

class _BInt16Array(_BaseArray): _type_ = _BInt16
class _BInt16Array2(_BInt16Array): _length_ = 2
class _BInt16Array6(_BInt16Array): _length_ = 6

class _BUInt16Array(_BaseArray): _type_ = _BUInt16
class _BUInt16Array7(_BUInt16Array): _length_ = 7


class ReportDataStructure(ctypes.Structure):
    _pack_ = 1
    def __init__(self, *args, **kw):
        super().__init__(*args, **kw)
        self._is_radian = kw.get('is_radian', True)
        self._ndigits = kw.get('ndigits', None)

    def __len__(self):
        return ctypes.sizeof(self)

    def _to_round(self, val):
        if isinstance(val, float) and self._ndigits is not None:
            return round(val, self._ndigits)
        elif isinstance(val, list):
            return [self._to_round(v) for v in val]
        else:
            return val
    
    def _get_attr_val(self, attr):
        if isinstance(attr, _BaseArray):
            return attr.to_list(is_radian=self._is_radian)
        elif isinstance(attr, ctypes.Array):
            return [self._get_attr_val(item) for item in attr]
        else:
            if isinstance(attr, _BaseValue):
                attr = attr.get_value(is_radian=self._is_radian)
            return attr

    def __getattr__(self, name):
        if '_{}'.format(name) not in self.__dir__():
            raise AttributeError("'{0}' object has no attribute '{1}'".format(self.__class__.__name__, name))
        attr = getattr(self, '_{}'.format(name))
        return self._to_round(self._get_attr_val(attr))

    def __setattr__(self, name, value):
        if '_{}'.format(name) in self.__dir__():
            raise AttributeError("'{0}' object can't set attribute '{1}'".format(self.__class__.__name__, name))
        return super().__setattr__(name, value)
    
    def get(self, key, default=None):
        return getattr(self, key, default)
    
    @classmethod
    def create(cls, sock_port, **kwargs):
        if sock_port == 30000:
            return _Report30000DataStructure(**kwargs)
        elif sock_port == 30001:
            return _Report30001DataStructure(**kwargs)
        elif sock_port == 30002:
            return _Report30002DataStructure(**kwargs)
        elif sock_port == 30003:
            return _Report30003DataStructure(**kwargs)

    def update(self, data):
        ctypes.memmove(ctypes.byref(self), data, min(len(data), ctypes.sizeof(self)))

    def show(self):
        for item in self._fields_:
            name = item[0][1:] if item[0].startswith('_') else item[0]
            if name.endswith('reserved'):
                continue
            val = getattr(self, name)
            print('{}: {}'.format(name, val))
            # print('{}: {}, {}'.format(name, type(val), val))


class _Report30000DataStructure(ReportDataStructure):
    _fields_ = [
        ('_data_size', _BUInt32),
        ('_timestamp', _BUInt64),
        ('_state_mode', C_UINT8),
        ('_cmd_num', _BUInt16),
        ('_system_reserved', _UInt8Array17),
        ('_target_joint_angle', _JointFp32Array7),
        ('_target_joint_velocity', _JointFp32Array7),
        ('_target_joint_acceleration', _JointFp32Array7),
        ('_actual_joint_angle', _JointFp32Array7),
        ('_actual_joint_velocity', _JointFp32Array7),
        ('_actual_joint_acceleration', _JointFp32Array7),
        ('_actual_joint_current', _Fp32Array7),
        ('_estimated_joint_torque', _Fp32Array7),
        ('_joint_reserved', _Fp32Array42),
        ('_target_tcp_pose', _TcpFp32Array6),
        ('_target_tcp_speed', _TcpFp32Array6),
        ('_actual_tcp_pose', _TcpFp32Array6),
        ('_actual_tcp_speed', _TcpFp32Array6),
        ('_estimated_tcp_torque', _Fp32Array6),
        ('_target_tcp_acceleration', _TcpFp32Array6),
        ('_actual_tcp_acceleration', _TcpFp32Array6),
        ('_tcp_reserved', _Fp32Array24),
        ('_ft_raw_force', _Fp32Array6),
        ('_ft_ext_force', _Fp32Array6),
        ('_monitor_device_type', C_UINT8),
        ('_monitor_device_state', C_UINT8),
        ('_monitor_device_pos', _BInt16),
        ('_monitor_device_speed', _BInt16),
        ('_monitor_device_current', _BInt16),
        ('_monitor_reserved', _UInt8Array40),
        ('_cgpio_digital_input', _BInt16),
        ('_cgpio_digital_output', _BInt16),
        ('_cgpio_analog_input', _BInt16Array2), # mV
        ('_cgpio_analog_output', _BInt16Array2), # mV
        ('_tgpio_digital_input', _BInt16),
        ('_tgpio_digital_output', _BInt16),
        ('_tgpio_analog_input', _BInt16Array2), # mV
    ]
    # 以下仅仅为了提示, 对应属性xxx实际将从_xxx获取
    data_size: int
    timestamp: int
    # state_mode: int
    state: int
    mode: int
    cmd_num: int
    target_joint_angle: list
    target_joint_velocity: list
    target_joint_acceleration: list
    actual_joint_angle: list
    actual_joint_velocity: list
    actual_joint_acceleration: list
    actual_joint_current: list
    estimated_joint_torque: list
    target_tcp_pose: list   # [x, y, z, rx, ry, rz]
    target_tcp_speed: list
    actual_tcp_pose: list   # [z, y, z, rx, ry, rz]
    actual_tcp_speed: list
    estimated_tcp_torque: list
    target_tcp_acceleration: list
    actual_tcp_acceleration: list
    ft_raw_force: list
    ft_ext_force: list
    monitor_device_type: int
    monitor_device_state: int
    monitor_device_pos: int
    monitor_device_speed: int
    monitor_device_current: int
    cgpio_digital_input: int
    cgpio_digital_output: int
    cgpio_analog_input: list    # mV
    cgpio_analog_output: list   # mV
    tgpio_digital_input: int
    tgpio_digital_output: int
    tgpio_analog_input: list    # mV

    @property
    def state(self):
        return self._state_mode & 0x0F

    @property
    def mode(self):
        return self._state_mode >> 4
    

class _Report30001DataStructure(ReportDataStructure):
    _fields_ = [
        ('_data_size', _BUInt32),
        ('_state_mode', C_UINT8),
        ('_cmd_num', _BUInt16),
        ('_actual_joint_angle', _JointFp32Array7),
        ('_actual_tcp_pose', _TcpFp32Array6),
        ('_estimated_joint_torque', _Fp32Array7),
        ('_motor_brake_status', C_UINT8),
        ('_motor_enable_status', C_UINT8),
        ('_error_code', C_UINT8),
        ('_warn_code', C_UINT8),
        ('_tcp_offset', _TcpFp32Array6),
        ('_tcp_payload', _Fp32Array4),
        ('_collision_sens', C_UINT8),
        ('_teaching_sens', C_UINT8),
        ('_gravity_direction', _Fp32Array3),
    ]
    # 以下仅仅为了提示, 对应属性xxx实际将从_xxx获取
    data_size: int
    # state_mode: int
    state: int
    mode: int
    cmd_num: int
    actual_joint_angle: list
    actual_tcp_pose: list   # [x, y, z, roll, pitch, yaw]
    estimated_joint_torque: list
    motor_brake_status: int
    motor_enable_status: int
    error_code: int
    warn_code: int
    tcp_offset: list
    tcp_payload: list
    collision_sens: int
    teaching_sens: int
    gravity_direction: list

    @property
    def state(self):
        return self._state_mode & 0x0F

    @property
    def mode(self):
        return self._state_mode >> 4


class _Report30002DataStructure(ReportDataStructure):
    _fields_ = [
        ('_data_size', _BUInt32),
        ('_state_mode', C_UINT8),
        ('_cmd_num', _BUInt16),
        ('_actual_joint_angle', _JointFp32Array7),
        ('_actual_tcp_pose', _TcpFp32Array6),
        ('_estimated_joint_torque', _Fp32Array7),
        ('_motor_brake_status', C_UINT8),
        ('_motor_enable_status', C_UINT8),
        ('_error_code', C_UINT8),
        ('_warn_code', C_UINT8),
        ('_tcp_offset', _TcpFp32Array6),
        ('_tcp_payload', _Fp32Array4),
        ('_collision_sens', C_UINT8),
        ('_teaching_sens', C_UINT8),
        ('_gravity_direction', _Fp32Array3),
        ('_robot_type', C_UINT8),
        ('_robot_axis', C_UINT8),
        ('_master_id', C_UINT8),
        ('_slave_id', C_UINT8),
        ('_motor_tid', C_UINT8),
        ('_motor_fid', C_UINT8),
        ('_version', C_CHAR * 30),
        ('_tcp_jerk', _Fp32),
        ('_tcp_min_acc', _Fp32),
        ('_tcp_max_acc', _Fp32),
        ('_tcp_min_speed', _Fp32),
        ('_tcp_max_speed', _Fp32),
        ('_joint_jerk', _JointFp32),
        ('_joint_min_acc', _JointFp32),
        ('_joint_max_acc', _JointFp32),
        ('_joint_min_speed', _JointFp32),
        ('_joint_max_speed', _JointFp32),
        ('_rot_jerk', _JointFp32),
        ('_rot_max_acc', _JointFp32),
        ('_servo_status_codes', _UInt8Array14),
        ('_tgpio_status_codes', _UInt8Array2),
        ('_temperatures', _Int8Array7),
        ('_target_tcp_speed', _Fp32),
        ('_target_joint_speed', _JointFp32Array7),
        ('_counter', _BUInt32),
        ('_world_offset', _TcpFp32Array6),
        ('_cgpio_reset_enable', C_UINT8),
        ('_tgpio_reset_enable', C_UINT8),
        ('_is_simulation_robot', C_UINT8),
        ('_is_collision_detection', C_UINT8),
        ('_collision_tool_type', C_UINT8),
        ('_collision_tool_params', _Fp32Array6),
        ('_joint_voltages', _BUInt16Array7),
        ('_joint_currents', _Fp32Array7),
        ('_cgpio_module_status', C_UINT8),
        ('_cgpio_module_error_code', C_UINT8),
        ('_cgpio_digital_input_function_io_status', _BUInt16),
        ('_cgpio_digital_input_configuration_io_status', _BUInt16),
        ('_cgpio_digital_output_function_io_status', _BUInt16),
        ('_cgpio_digital_output_configuration_io_status', _BUInt16),
        ('_cgpio_analog_input_1', _BUInt16),
        ('_cgpio_analog_input_2', _BUInt16),
        ('_cgpio_analog_output_1', _BUInt16),
        ('_cgpio_analog_output_2', _BUInt16),
        ('_cgpio_digital_input_io_config_1', _UInt8Array8),
        ('_cgpio_digital_output_io_config_1', _UInt8Array8),
        ('_cgpio_digital_input_io_config_2', _UInt8Array8),
        ('_cgpio_digital_output_io_config_2', _UInt8Array8),
        ('_ft_ext_force', _Fp32Array6),
        ('_ft_raw_force', _Fp32Array6),
        ('_iden_progress', C_UINT8),
        ('_axis_angle', _JointFp32Array3),
        ('_config_status', C_UINT8),
        ('_is_reduced_mode', C_UINT8),
        ('_reduced_tcp_boundary', _BInt16Array6),
        ('_reduced_tcp_max_speed', _Fp32),
        ('_reduced_joint_max_speed', _JointFp32),
        ('_reduced_joint_limits', _JointFp32Array14),
        ('_is_fence_mode', C_UINT8),
        ('_is_collision_rebound', C_UINT8),
        ('_cgpio_alarm_code', _BUInt32),
        ('_switch_status', C_UINT8),
        ('_monitor_device_type', C_UINT8),
        ('_monitor_device_state', C_UINT8),
        ('_monitor_device_pos', _BInt16),
        ('_monitor_device_speed', _BInt16),
        ('_monitor_device_current', _BInt16),
    ]
    # 以下仅仅为了提示, 对应属性xxx实际将从_xxx获取
    data_size: int
    # state_mode: int
    state: int
    mode: int
    cmd_num: int
    actual_joint_angle: list
    actual_tcp_pose: list   # [x, y, z, roll, pitch, yaw]
    estimated_joint_torque: list
    motor_brake_status: int
    motor_enable_status: int
    error_code: int
    warn_code: int
    tcp_offset: list
    tcp_payload: list
    collision_sens: int
    teaching_sens: int
    gravity_direction: list
    robot_type: int
    robot_axis: int
    master_id: int
    slave_id: int
    motor_tid: int
    motor_fid: int
    version: str
    tcp_jerk: float
    tcp_min_acc: float
    tcp_max_acc: float
    tcp_min_speed: float
    tcp_max_speed: float
    joint_jerk: float
    joint_min_acc: float
    joint_max_acc: float
    joint_min_speed: float
    joint_max_speed: float
    rot_jerk: float
    rot_max_acc: float
    servo_status_codes: list
    tgpio_status_codes: list
    temperatures: list
    target_tcp_speed: float
    target_joint_speed: list
    counter: int
    world_offset: list
    cgpio_reset_enable: int
    tgpio_reset_enable: int
    collision_tool_type: int
    collision_tool_params: list
    joint_voltages: list
    joint_currents: list
    cgpio_module_status: int
    cgpio_module_error_code: int
    cgpio_digital_input_function_io_status: int
    cgpio_digital_input_configuration_io_status: int
    cgpio_digital_output_function_io_status: int
    cgpio_digital_output_configuration_io_status: int
    cgpio_analog_input: list    # [V, V]
    cgpio_analog_output: list   # [V, V]
    # cgpio_analog_input_1: float
    # cgpio_analog_input_2: float
    # cgpio_analog_output_1: float
    # cgpio_analog_output_2: float
    cgpio_digital_input_io_config: list
    cgpio_digital_output_io_config: list
    # cgpio_digital_input_io_config_1: list
    # cgpio_digital_output_io_config_1: list
    # cgpio_digital_input_io_config_2: list
    # cgpio_digital_output_io_config_2: list
    ft_ext_force: list
    ft_raw_force: list
    iden_progress: int
    axis_angle: list
    config_status: int
    reduced_tcp_boundary: list  # [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
    reduced_tcp_max_speed: float
    reduced_joint_max_speed: float
    reduced_joint_limits: list  # [[J1_min, J2_max], ..., [J7_min, J7_max]]
    is_simulation_robot: bool
    is_collision_detection: bool
    is_reduced_mode: bool
    is_fence_mode: bool
    is_collision_rebound: bool
    is_approx_motion: bool
    is_report_current: bool
    is_cart_continuous: bool
    ft_sensor_is_enable: bool
    cgpio_alarm_code: int
    switch_status: int
    monitor_device_type: int
    monitor_device_state: int
    monitor_device_pos: int
    monitor_device_speed: int
    monitor_device_current: int

    @property
    def state(self):
        return self._state_mode & 0x0F

    @property
    def mode(self):
        return self._state_mode >> 4

    @property
    def version(self):
        return self._version.decode('utf-8').rstrip('\x00')

    @property
    def servo_status_codes(self):
        return [(self._servo_status_codes[i * 2], self._servo_status_codes[i * 2 + 1]) for i in range(7)]
    
    @property
    def cgpio_analog_input(self):
        return [self._to_round(self._cgpio_analog_input_1.get_value() / 4095.0 * 10.0), self._to_round(self._cgpio_analog_input_2.get_value() / 4095.0 * 10.0)]

    @property
    def cgpio_analog_output(self):
        return [self._to_round(self._cgpio_analog_output_1.get_value() / 4095.0 * 10.0), self._to_round(self._cgpio_analog_output_2.get_value() / 4095.0 * 10.0)]
    
    @property
    def cgpio_digital_input_io_config(self):
        return [val for val in self._cgpio_digital_input_io_config_1] + [val for val in self._cgpio_digital_input_io_config_2]
    
    @property
    def cgpio_digital_output_io_config(self):
        return [val for val in self._cgpio_digital_output_io_config_1] + [val for val in self._cgpio_digital_output_io_config_2]

    @property
    def is_simulation_robot(self):
        return self._is_simulation_robot == 1
    
    @property
    def is_collision_detection(self):
        return self._is_collision_detection == 1
    
    @property
    def is_reduced_mode(self):
        return self._is_reduced_mode == 1
    
    @property
    def is_fence_mode(self):
        return self._is_fence_mode == 1
    
    @property
    def is_collision_rebound(self):
        return self._is_collision_rebound == 1
    
    @property
    def is_report_current(self):
        return (self._config_status >> 2) & 0x01 == 1

    @property
    def is_approx_motion(self):
        return (self._config_status >> 3) & 0x01 == 1

    @property
    def is_cart_continuous(self):
        return (self._config_status >> 4) & 0x01 == 1
    
    @property
    def ft_sensor_is_enable(self):
        return self._switch_status & 0x01 == 1

    @property
    def reduced_tcp_boundary(self):
        return [[self._reduced_tcp_boundary[i * 2 + 1].get_value(), self._reduced_tcp_boundary[i * 2].get_value()] for i in range(3)]

    @property
    def reduced_joint_limits(self):
        limits = self._to_round(self._reduced_joint_limits.to_list(is_radian=self._is_radian))
        return [[limits[i * 2], limits[i * 2 + 1]] for i in range(7)]


class _Report30003DataStructure(ReportDataStructure):
    _fields_ = [
        ('_data_size', _BUInt32),
        ('_state_mode', C_UINT8),
        ('_cmd_num', _BUInt16),
        ('_actual_joint_angle', _JointFp32Array7),
        ('_actual_tcp_pose', _TcpFp32Array6),
        ('_estimated_joint_torque', _Fp32Array7),
        ('_ft_ext_force', _Fp32Array6),
        ('_ft_raw_force', _Fp32Array6),
    ]
    # 以下仅仅为了提示, 对应属性xxx实际将从_xxx获取
    data_size: int
    # state_mode: int
    state: int
    mode: int
    cmd_num: int
    actual_joint_angle: list
    actual_tcp_pose: list   # [x, y, z, roll, pitch, yaw]
    estimated_joint_torque: list
    ft_ext_force: list
    ft_raw_force: list

    @property
    def state(self):
        return self._state_mode & 0x0F

    @property
    def mode(self):
        return self._state_mode >> 4


__all__ = ['ReportDataStructure']

if __name__ == '__main__':
    sock_port = 30000
    report_data = ReportDataStructure.create(sock_port, is_radian=True, ndigits=6)
    # read data from socket 
    # report_data.update(data)
