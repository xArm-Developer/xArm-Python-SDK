#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
from ..x3 import XArm
try:
    from ..tools.blockly_tool import BlocklyTool
except:
    print('import BlocklyTool module failed')
    BlocklyTool = None


class XArmAPI(object):
    def __init__(self, port=None, is_radian=False, do_not_open=False, **kwargs):
        """
        The API wrapper of xArm
        Note: Orientation of attitude angle
            roll: rotate around the X axis
            pitch: rotate around the Y axis
            yaw: rotate around the Z axis
        
        :param port: port name(such as 'COM5'/'/dev/ttyUSB0') or ip-address(such as '192.168.1.185')
            Note: this parameter is required if parameter do_not_open is False
        :param is_radian: set the default unit is radians or not, default is False
            Note: (aim of design)
                1. Default value for unified interface parameters
                2: Unification of the external unit system
                3. For compatibility with previous interfaces
            Note: the conversion of degree (°) and radians (rad)
                * 1 rad == 57.29577951308232 °
                * 1 ° == 0.017453292519943295 rad
                * 1 rad/s == 57.29577951308232 °/s
                * 1 °/s == 0.017453292519943295 rad/s
                * 1 rad/s^2 == 57.29577951308232 °/s^2
                * 1 °/s^2 == 0.017453292519943295 rad/s^2
                * 1 rad/s^3 == 57.29577951308232 °/s^3
                * 1 °/s^3 == 0.017453292519943295 rad/s^3
            Note: This parameter determines the value of the property self.default_is_radian 
            Note: This parameter determines the default value of the interface with the is_radian(input_is_radian/return_is_radian) parameter
               The list of affected interfaces is as follows:
                    1. method: get_position
                    2. method: set_position
                    3. method: get_servo_angle
                    4. method: set_servo_angle
                    5. method: set_servo_angle_j
                    6. method: move_gohome
                    7. method: reset
                    8. method: set_tcp_offset
                    9. method: set_joint_jerk
                    10. method: set_joint_maxacc
                    11. method: get_inverse_kinematics
                    12. method: get_forward_kinematics
                    13. method: is_tcp_limit
                    14. method: is_joint_limit
                    15. method: get_params
                    16: method: move_arc_lines
                    17: method: move_circle
            Note: This parameter determines the default return type for some interfaces (such as the position, velocity, and acceleration associated with the return angle arc).
                The affected attributes are as follows:
                    1. property: position
                    2. property: last_used_position
                    3. property: angles
                    4. property: last_used_angles
                    5. property: last_used_joint_speed
                    6. property: last_used_joint_acc
                    7. property: tcp_offset
        :param do_not_open: do not open, default is False, if true, you need to manually call the connect interface.
        :param kwargs: keyword parameters, generally do not need to set
            baudrate: baudrate, only available in serial way, default is 2000000
            timeout: timeout, only available in serial way, default is None
            filters: serial port filters, invalid, reserved.
            enable_report: whether to enable report, default is True
                Note: if enable_report is True, the self.last_used_position and self.last_used_angles value is the current position of robot
            report_type: report type('normal'/'rich'), only available in socket way, default is 'rich'
                Note:
                    'normal': Reported at a frequency of 10 Hz
                    'rich': Reported at a frequency of 10 Hz, more reported content than normal
            check_tcp_limit: check the tcp param value out of limit or not, default is True
                Note: only check the param roll/pitch/yaw of the interface `set_position`
            check_joint_limit: check the joint param value out of limit or not, default is True
                Note: only check the param angle of the interface `set_servo_angle` and the param angles of the interface `set_servo_angle_j`
            check_cmdnum_limit: check the cmdnum out of limit or not, default is True
                Note: only available in the interface `set_position` and `set_servo_angle`
        """
        self._arm = XArm(port=port,
                         is_radian=is_radian,
                         do_not_open=do_not_open,
                         **kwargs)
        self.__attr_alias_map = {
            'get_ik': self.get_inverse_kinematics,
            'get_fk': self.get_forward_kinematics,
            'set_sleep_time': self.set_pause_time,
            'register_maable_mtbrake_changed_callback': self.register_mtable_mtbrake_changed_callback,
            'release_maable_mtbrake_changed_callback': self.release_mtable_mtbrake_changed_callback,
            'position_offset': self.tcp_offset,
            'get_gpio_digital': self.get_tgpio_digital,
            'set_gpio_digital': self.set_tgpio_digital,
            'get_gpio_analog': self.get_tgpio_analog,
        }

    def __getattr__(self, item):
        if item in self.__attr_alias_map.keys():
            return self.__attr_alias_map[item]
        raise AttributeError('\'{}\' has not attribute \'{}\''.format(self.__class__.__name__, item))

    @property
    def core(self):
        """
        Core layer API, set only for advanced developers, please do not use
        Ex:
            self.core.move_line(...)
            self.core.move_lineb(...)
            self.core.move_joint(...)
            ...            
        """
        return self._arm.arm_cmd

    @property
    def version_number(self):
        """
        Frimware version number
        :return: (major_version_number, minor_version_number, revision_version_number)
        """
        return self._arm.version_number

    @property
    def connected(self):
        """
        Connection status
        """
        return self._arm.connected

    @property
    def default_is_radian(self):
        """
        The default unit is radians or not
        """
        return self._arm.default_is_radian

    @property
    def version(self):
        """
        xArm version
        """
        return self._arm.version

    @property
    def sn(self):
        """
        xArm sn
        """
        return self._arm.sn

    @property
    def position(self):
        """
        Cartesion position
        Note:
            1. If self.default_is_radian is True, the returned value (only roll/pitch/yaw) is in radians
        
        return: [x(mm), y(mm), z(mm), roll(° or rad), pitch(° or rad), yaw(° or rad)]
        """
        return self._arm.position

    @property
    def last_used_position(self):
        """
        The last used cartesion position, default value of parameter x/y/z/roll/pitch/yaw of interface set_position
        Note:
            1. If self.default_is_radian is True, the returned value (only roll/pitch/yaw) is in radians
            2. self.set_position(x=300) <==> self.set_position(x=300, *last_used_position[1:])
            2. self.set_position(roll=-180) <==> self.set_position(x=self.last_used_position[:3], roll=-180, *self.last_used_position[4:])
        
        :return: [x(mm), y(mm), z(mm), roll(° or rad), pitch(° or rad), yaw(° or rad)]
        """
        return self._arm.last_used_position

    @property
    def tcp_speed_limit(self):
        """
        Joint acceleration limit, only available in socket way and enable_report is True and report_type is 'rich' 
        
        :return: [min_tcp_acc(mm/s), max_tcp_acc(mm/s)]
        """
        return self._arm.tcp_speed_limit

    @property
    def tcp_acc_limit(self):
        """
        Joint acceleration limit, only available in socket way and enable_report is True and report_type is 'rich' 
        
        :return: [min_tcp_acc(mm/s^2), max_tcp_acc(mm/s^2)]
        """
        return self._arm.tcp_acc_limit

    @property
    def last_used_tcp_speed(self):
        """
        The last used cartesion speed, default value of parameter speed of interface set_position/move_circle
        
        :return: speed (mm/s)
        """
        return self._arm.last_used_tcp_speed

    @property
    def last_used_tcp_acc(self):
        """
        The last used cartesion acceleration, default value of parameter mvacc of interface set_position/move_circle
        
        :return: acceleration (mm/s^2)
        """
        return self._arm.last_used_tcp_acc

    @property
    def angles(self):
        """
        Servo angles
        Note:
            1. If self.default_is_radian is True, the returned value is in radians
        
        :return: [angle1(° or rad), angle2(° or rad), ..., anglen(° or rad)]
        """
        return self._arm.angles

    @property
    def joint_speed_limit(self):
        """
        Joint speed limit,  only available in socket way and enable_report is True and report_type is 'rich'
        Note:
            1. If self.default_is_radian is True, the returned value is in radians
            
        :return: [min_joint_speed(°/s or rad/s), max_joint_speed(°/s or rad/s)]
        """
        return self._arm.joint_speed_limit

    @property
    def joint_acc_limit(self):
        """
        Joint acceleration limit, only available in socket way and enable_report is True and report_type is 'rich' 
        Note:
            1. If self.default_is_radian is True, the returned value is in radians
        
        :return: [min_joint_acc(°/s^2 or rad/s^2), max_joint_acc(°/s^2 or rad/s^2)]
        """
        return self._arm.joint_acc_limit

    @property
    def last_used_angles(self):
        """
        The last used servo angles, default value of parameter angle of interface set_servo_angle
        Note:
            1. If self.default_is_radian is True, the returned value is in radians
            2. self.set_servo_angle(servo_id=1, angle=75) <==> self.set_servo_angle(angle=[75] + self.last_used_angles[1:])
            3. self.set_servo_angle(servo_id=5, angle=30) <==> self.set_servo_angle(angle=self.last_used_angles[:4] + [30] + self.last_used_angles[5:])
        
        :return: [angle1(° or rad), angle2(° or rad), ..., angle7(° or rad)]
        """
        return self._arm.last_used_angles

    @property
    def last_used_joint_speed(self):
        """
        The last used joint speed, default value of parameter speed of interface set_servo_angle
        Note:
            1. If self.default_is_radian is True, the returned value is in radians
        
        :return: speed (°/s or rad/s)
        """
        return self._arm.last_used_joint_speed

    @property
    def last_used_joint_acc(self):
        """
        The last used joint acceleration, default value of parameter mvacc of interface set_servo_angle
        Note:
            1. If self.default_is_radian is True, the returned value is in radians
        
        :return: acceleration (°/s^2 or rad/s^2)
        """
        return self._arm.last_used_joint_acc

    @property
    def tcp_offset(self):
        """
        Cartesion position offset, only available in socket way and enable_report is True 
        Note:
            1. If self.default_is_radian is True, the returned value(roll_offset/pitch_offset/yaw_offset) is in radians
        
        :return: [x_offset(mm), y_offset(mm), z_offset(mm), roll_offset(° or rad), pitch_offset(° or rad), yaw_offset(° or rad)]
        """
        return self._arm.position_offset

    @property
    def state(self):
        """
        xArm state
        
        :return: 
            1: in motion
            2: sleeping
            3: suspended
            4: stopping
        """
        return self._arm.state

    @property
    def mode(self):
        """
        xArm mode，only available in socket way and  enable_report is True
        
        :return: 
            0: position control mode
            1: servo motion mode
            2: joint teaching mode
            3: cartesian teaching mode (invalid)
        """
        return self._arm.mode

    @property
    def joints_torque(self):
        """
        Joints torque, only available in socket way and  enable_report is True and report_type is 'rich'
        
        :return: [joint-1, ....]
        """
        return self._arm.joints_torque

    @property
    def tcp_load(self):
        """
        xArm tcp load, only available in socket way and  enable_report is True and report_type is 'rich'
        
        :return: [weight, center of gravity] 
            such as: [weight(kg), [x(mm), y(mm), z(mm)]]
        """
        return self._arm.tcp_load

    @property
    def collision_sensitivity(self):
        """
        The sensitivity value of collision, only available in socket way and  enable_report is True and report_type is 'rich'
        
        :return: 0~5
        """
        return self._arm.collision_sensitivity

    @property
    def teach_sensitivity(self):
        """
        The sensitivity value of drag and teach, only available in socket way and  enable_report is True and report_type is 'rich'
        
        :return: 0~5
        """
        return self._arm.teach_sensitivity

    @property
    def motor_brake_states(self):
        """
        Motor brake state list, only available in socket way and  enable_report is True and report_type is 'rich'
        Note:
            For a robot with a number of axes n, only the first n states are valid, and the latter are reserved.
        
        :return: [motor-1-brake-state, motor-2-..., motor-3-..., motor-4-..., motor-5-..., motor-6-..., motor-7-..., reserved]
            motor-{i}-brake-state:
                0: enable
                1: disable
        """
        return self._arm.motor_brake_states

    @property
    def motor_enable_states(self):
        """
        Motor enable state list, only available in socket way and  enable_report is True and report_type is 'rich'
        Note:
            For a robot with a number of axes n, only the first n states are valid, and the latter are reserved.
            
        :return: [motor-1-enable-state, motor-2-..., motor-3-..., motor-4-..., motor-5-..., motor-6-..., motor-7-..., reserved]
            motor-{i}-enable-state:
                0: disable
                1: enable
        """
        return self._arm.motor_enable_states

    @property
    def has_err_warn(self):
        """
        Contorller have an error or warning or not
        
        :return: True/False
        """
        return self._arm.has_err_warn

    @property
    def has_error(self):
        """
        Controller have an error or not
        """
        return self._arm.has_error

    @property
    def has_warn(self):
        """
        Controller have an warnning or not
        """
        return self._arm.has_warn

    @property
    def error_code(self):
        """
        Controller error code. See the controller error code documentation for details.
        """
        return self._arm.error_code

    @property
    def warn_code(self):
        """
        Controller warn code. See the controller warn code documentation for details.
        """
        return self._arm.warn_code

    @property
    def cmd_num(self):
        """
        Number of command caches in the controller
        """
        return self._arm.cmd_num

    @property
    def device_type(self):
        """
        Device type, only available in socket way and  enable_report is True and report_type is 'rich'
        """
        return self._arm.device_type

    @property
    def axis(self):
        """
        Axis number, only available in socket way and enable_report is True and report_type is 'rich'
        """
        return self._arm.axis

    @property
    def master_id(self):
        """
        Master id, only available in socket way and enable_report is True and report_type is 'rich'
        """
        return self._arm.master_id

    @property
    def slave_id(self):
        """
        Slave id, only available in socket way and enable_report is True and report_type is 'rich'
        """
        return self._arm.slave_id

    @property
    def gravity_direction(self):
        """
        gravity direction, only available in socket way and enable_report is True and report_type is 'rich'
        :return: 
        """
        return self._arm.gravity_direction

    def connect(self, port=None, baudrate=None, timeout=None):
        """
        Connect to xArm
        
        :param port: port name or the ip address, default is the value when initializing an instance
        :param baudrate: baudrate, only available in serial way, default is the value when initializing an instance
        :param timeout: timeout, only available in serial way, default is the value when initializing an instance
        """
        self._arm.connect(port=port, baudrate=baudrate, timeout=timeout)

    def disconnect(self):
        """
        Disconnect
        """
        self._arm.disconnect()

    def send_cmd_sync(self, command=None):
        """
        Send cmd and wait (only waiting the cmd response, not waiting for the movement)
        Note:
            1. Some command depends on self.default_is_radian
        
        :param command: 
            'G1': 'set_position(MoveLine): G1 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)} F{speed(mm/s)} Q{acc(mm/s^2)} T{mvtime} W{wait}'
            'G4': 'set_pause_time: G4 V{sltime(second)}'
            'G7': 'set_servo_angle: G7 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)} F{speed(°/s or rad/s)} Q{acc(°/s^2 or rad/s^2)} T{mvtime} W{wait}'
            'G8': 'move_gohome: G8 F{speed(°/s or rad/s)} Q{acc(°/s^2 or rad/s^2)} T{mvtime} W{wait}'
            'G9': 'set_position(MoveArcLine): G9 X{x} Y{y} Z{z} A{roll} B{pitch(° or rad)} C{yaw(° or rad)} R{radius(mm)} F{speed(mm/s)} Q{acc(mm/s^2)} T{mvtime} W{wait}'
            'H1': 'get_version: H1'
            'H11': 'motion_enable: H11 S{servo_id} V{enable}'
            'H12': 'set_state: H12 V{state}'
            'H13': 'get_state: H13'
            'H14': 'get_cmdnum: H14'
            'H15': 'get_err_warn_code: H15'
            'H16': 'clean_error: H16'
            'H17': 'clean_warn: H17'
            'H18': 'set_servo_attach/set_servo_detach: H18 S{servo_id} V{1: enable(detach), 0: disable(attach)}'
            'H31': 'set_tcp_jerk: H31 V{jerk(mm/s^3)}'
            'H32': 'set_tcp_maxacc: H32 V{maxacc(mm/s^2)}'
            'H33': 'set_joint_jerk: H33 V{jerk(°/s^3 or rad/s^3)}'
            'H34': 'set_joint_maxacc: H34 {maxacc(°/s^2 or rad/s^2)}'
            'H39': 'clean_conf: H39'
            'H40': 'save_conf: H40'
            'H41': 'get_position: H41'
            'H42': 'get_servo_angle: H42'
            'H43': 'get_inverse_kinematics: H43 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)}'
            'H44': 'get_forward_kinematics: H44 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)}'
            'H45': 'is_joint_limit: H45 I{servo_1(° or rad)} J{servo_2(° or rad)} K{servo_3(° or rad)} L{servo_4(° or rad)} M{servo_5(° or rad)} N{servo_6(° or rad)} O{servo_7(° or rad)}'
            'H46': 'is_tcp_limit: H46 X{x(mm)} Y{y(mm)} Z{z(mm)} A{roll(° or rad)} B{pitch(° or rad)} C{yaw(° or rad)}'
            'H106': 'get_servo_debug_msg: H106'
        :return: code or tuple((code, ...))
            code: See the API code documentation for details.
        """
        return self._arm.send_cmd_sync(command=command)

    def get_position(self, is_radian=None):
        """
        Get the cartesian position
        Note:
            1. If the value(roll/pitch/yaw) you want to return is an radian unit, please set the parameter is_radian to True
                ex: code, pos = xarm.get_position(is_radian=True)
        
        :param is_radian: the returned value (only roll/pitch/yaw) is in radians or not, default is self.default_is_radian
        :return: tuple((code, [x, y, z, roll, pitch, yaw])), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
        """
        return self._arm.get_position(is_radian=is_radian)

    def set_position(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None,
                     speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None,
                     wait=False, timeout=None, **kwargs):
        """
        Set the cartesian position, the API will modify self.last_used_position value
        Note:
            1. If it is a 5-axis arm, ensure that the current robot arm has a roll value of 180° or π rad and has a roll value of 0 before calling this interface.
            2. If it is a 5-axis arm, roll must be set to 180° or π rad, pitch must be set to 0
            3. If the parameter(roll/pitch/yaw) you are passing is an radian unit, be sure to set the parameter is_radian to True.
                ex: code = xarm.set_position(x=300, y=0, z=200, roll=-3.14, pitch=0, yaw=0, is_radian=True)
            4. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
                ex: code = xarm.set_position(x=300, y=0, z=200, roll=180, pitch=0, yaw=0, is_radian=False, wait=True)
        
        :param x: cartesian position x, (unit: mm), default is self.last_used_position[0]
        :param y: cartesian position y, (unit: mm), default is self.last_used_position[1]
        :param z: cartesian position z, (unit: mm), default is self.last_used_position[2]
        :param roll: rotate around the X axis, (unit: rad if is_radian is True else °), default is self.last_used_position[3]
        :param pitch: rotate around the Y axis, (unit: rad if is_radian is True else °), default is self.last_used_position[4]
        :param yaw: rotate around the Z axis, (unit: rad if is_radian is True else °), default is self.last_used_position[5]
        :param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine
            MoveLine: Linear motion
                ex: code = xarm.set_position(..., radius=None)
            MoveArcLine: Linear arc motion with interpolation
                ex: code = xarm.set_position(..., radius=0)
        :param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
        :param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
        :param mvtime: 0, reserved
        :param relative: relative move or not
        :param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        :param kwargs: reserved
        :return: code
            code: See the API code documentation for details.
                code < 0: the last_used_position/last_used_tcp_speed/last_used_tcp_acc will not be modified
                code >= 0: the last_used_position/last_used_tcp_speed/last_used_tcp_acc will be modified
        """
        return self._arm.set_position(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, radius=radius,
                                      speed=speed, mvacc=mvacc, mvtime=mvtime, relative=relative,
                                      is_radian=is_radian, wait=wait, timeout=timeout, **kwargs)

    def get_servo_angle(self, servo_id=None, is_radian=None):
        """
        Get the servo angle
        Note:
            1. If the value you want to return is an radian unit, please set the parameter is_radian to True
                ex: code, angles = xarm.get_servo_angle(is_radian=True)
            2. If you want to return only the angle of a single joint, please set the parameter servo_id
                ex: code, angle = xarm.get_servo_angle(servo_id=2)
        
        :param servo_id: 1-(Number of axes), None(8), default is None
        :param is_radian: the returned value is in radians or not, default is self.default_is_radian
        :return: tuple((code, angle list if servo_id is None or 8 else angle)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
        """
        return self._arm.get_servo_angle(servo_id=servo_id, is_radian=is_radian)

    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None,
                        relative=False, is_radian=None, wait=False, timeout=None, **kwargs):
        """
        Set the servo angle, the API will modify self.last_used_angles value
        Note:
            1. If the parameter angle you are passing is an radian unit, be sure to set the parameter is_radian to True.
                ex: code = xarm.set_servo_angle(servo_id=1, angle=1.57, is_radian=True)
            2. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
                ex: code = xarm.set_servo_angle(servo_id=1, angle=45, is_radian=False,wait=True)
        
        :param servo_id: 1-(Number of axes), None(8)
            1. 1-(Number of axes) indicates the corresponding joint, the parameter angle should be a numeric value
                ex: code = xarm.set_servo_angle(servo_id=1, angle=45, is_radian=False)
            2. None(8) means all joints, default is None, the parameter angle should be a list of values whose length is the number of joints
                ex: code = xarm.set_servo_angle(angle=[30, -45, 0, 0, 0, 0, 0], is_radian=False)
        :param angle: angle or angle list, (unit: rad if is_radian is True else °)
            1. If servo_id is 1-(Number of axes), angle should be a numeric value
                ex: code = xarm.set_servo_angle(servo_id=1, angle=45, is_radian=False)
            2. If servo_id is None or 8, angle should be a list of values whose length is the number of joints
                like [axis-1, axis-2, axis-3, axis-3, axis-4, axis-5, axis-6, axis-7]
                ex: code = xarm.set_servo_angle(angle=[30, -45, 0, 0, 0, 0, 0], is_radian=False)
        :param speed: move speed (unit: rad/s if is_radian is True else °/s), default is self.last_used_joint_speed
        :param mvacc: move acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is self.last_used_joint_acc
        :param mvtime: 0, reserved
        :param relative: relative move or not
        :param is_radian: the angle in radians or not, default is self.default_is_radian
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        :param kwargs: reserved
        :return: code
            code: See the API code documentation for details.
                code < 0: the last_used_angles/last_used_joint_speed/last_used_joint_acc will not be modified
                code >= 0: the last_used_angles/last_used_joint_speed/last_used_joint_acc will be modified
        """
        return self._arm.set_servo_angle(servo_id=servo_id, angle=angle, speed=speed, mvacc=mvacc, mvtime=mvtime,
                                         relative=relative, is_radian=is_radian, wait=wait, timeout=timeout, **kwargs)

    def set_servo_angle_j(self, angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):
        """
        Set the servo angle, execute only the last instruction, need to be set to servo motion mode
        Note:
            1. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc
        
        :param angles: angle list, (unit: rad if is_radian is True else °)
        :param speed: speed, reserved
        :param mvacc: acceleration, reserved
        :param mvtime: 0, reserved
        :param is_radian: the angles in radians or not, defalut is self.default_is_radian
        :param kwargs: reserved
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_servo_angle_j(angles, speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, **kwargs)

    def move_circle(self, pose1, pose2, percent, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):
        """
        The motion calculates the trajectory of the space circle according to the three-point coordinates.
        The three-point coordinates are (current starting point, pose1, pose2).
        
        :param pose1: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
        :param pose2: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
        :param percent: the percentage of arc length and circumference of the movement
        :param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
        :param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
        :param mvtime: 0, reserved
        :param is_radian: roll/pitch/yaw value is radians or not, default is self.default_is_radian
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        :param kwargs: reserved
        :return: code
            code: See the API code documentation for details.
                code < 0: the last_used_tcp_speed/last_used_tcp_acc will not be modified
                code >= 0: the last_used_tcp_speed/last_used_tcp_acc will be modified
        """
        return self._arm.move_circle(pose1, pose2, percent, speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, wait=wait, **kwargs)

    def move_gohome(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):
        """
        Move to go home (Back to zero), the API will modify self.last_used_position and self.last_used_angles value
        Warnning: without limit detection
        Note:
            1. The API will change self.last_used_position value into [201.5, 0, 140.5, -180, 0, 0]
            2. The API will change self.last_used_angles value into [0, 0, 0, 0, 0, 0, 0]
            3. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
                ex: code = xarm.move_gohome(wait=True)
            4. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc
        
        :param speed: gohome speed (unit: rad/s if is_radian is True else °/s), default is 50 °/s
        :param mvacc: gohome acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is 5000 °/s^2
        :param mvtime: reserved
        :param is_radian: the speed and acceleration are in radians or not, default is self.default_is_radian
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.move_gohome(speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, wait=wait, timeout=timeout)

    def move_arc_lines(self, paths, is_radian=None, times=1, first_pause_time=0.1, repeat_pause_time=0,
                       automatic_calibration=True, speed=None, mvacc=None, mvtime=None, wait=False):
        """
        Continuous linear motion with interpolation
        Note:
            1. If an error occurs, it will return early
            2. If the emergency_stop interface is called actively, it will return early.
            3. The last_used_position/last_used_tcp_speed/last_used_tcp_acc will be modified
            4. The last_used_angles/last_used_joint_speed/last_used_joint_acc will not be modified
        
        :param paths: cartesian path list
            1. Specify arc radius： [[x, y, z, roll, pitch, yaw, radius], ....]
            1. Do not specify arc radius (radius=0)： [[x, y, z, roll, pitch, yaw], ....]
        :param is_radian: roll/pitch/yaw of paths are in radians or not, default is self.default_is_radian
        :param times: repeat times, 0 is infinite loop, default is 1
        :param first_pause_time: sleep time at first, purpose is to cache the instruction, default is 0.1s
        :param repeat_pause_time: interval between repeated movements, unit: second
        :param automatic_calibration: automatic calibration or not, default is True
        :param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
        :param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
        :param mvtime: 0, reserved 
        :param wait: whether to wait for the arm to complete, default is False
        """
        return self._arm.move_arc_lines(paths, is_radian=is_radian, times=times, first_pause_time=first_pause_time,
                                        repeat_pause_time=repeat_pause_time, automatic_calibration=automatic_calibration,
                                        speed=speed, mvacc=mvacc, mvtime=mvtime, wait=wait)

    def set_servo_attach(self, servo_id=None):
        """
        Attach the servo
        
        :param servo_id: 1-(Number of axes), 8, if servo_id is 8, will attach all servo
            1. 1-(Number of axes): attach only one joint
                ex: xarm.set_servo_attach(servo_id=1)
            2: 8: attach all joints
                ex: xarm.set_servo_attach(servo_id=8)
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_servo_attach(servo_id=servo_id)

    def set_servo_detach(self, servo_id=None):
        """
        Detach the servo, be sure to do protective work before unlocking to avoid injury or damage.
        
        :param servo_id: 1-(Number of axes), 8, if servo_id is 8, will detach all servo
            1. 1-(Number of axes): detach only one joint
                ex: xarm.set_servo_detach(servo_id=1)
            2: 8: detach all joints, please
                ex: xarm.set_servo_detach(servo_id=8)
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_servo_detach(servo_id=servo_id)

    def get_version(self):
        """
        Get the xArm version
        
        :return: tuple((code, version)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
        """
        return self._arm.get_version()

    def shutdown_system(self, value=1):
        """
        Shutdown the xArm controller system
        
        :param value: 1: remote shutdown
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.shutdown_system(value=value)

    def get_is_moving(self):
        """
        Check xArm is moving or not
        :return: True/False
        """
        return self._arm.get_is_moving()

    def get_state(self):
        """
        Get state
        
        :return: tuple((code, state)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
            state:
                1: in motion
                2: sleeping
                3: suspended
                4: stopping
        """
        return self._arm.get_state()

    def set_state(self, state=0):
        """
        Set the xArm state
        
        :param state: default is 0
            0: sport state
            3: pause state
            4: stop state
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_state(state=state)

    def set_mode(self, mode=0):
        """
        Set the xArm mode
        
        :param mode: default is 0
            0: position control mode
            1: servo motion mode
            2: joint teaching mode
            3: cartesian teaching mode (invalid)
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_mode(mode=mode)

    def get_cmdnum(self):
        """
        Get the cmd count in cache
        :return: tuple((code, cmd_num)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
        """
        return self._arm.get_cmdnum()

    def get_err_warn_code(self, show=False):
        """
        Get the controller error and warn code
        
        :param show: show the detail info if True
        :return: tuple((code, [error_code, warn_code])), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
            error_code: See the controller error code documentation for details.
            warn_code: See the controller warn code documentation for details.
        """
        return self._arm.get_err_warn_code(show=show)

    def clean_error(self):
        """
        Clean the error, need to be manually enabled motion and set state after clean error
        
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.clean_error()

    def clean_warn(self):
        """
        Clean the warn
        
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.clean_warn()

    def motion_enable(self, enable=True, servo_id=None):
        """
        Motion enable
        
        :param enable: 
        :param servo_id: 1-(Number of axes), None(8)
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.motion_enable(servo_id=servo_id, enable=enable)

    def reset(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):
        """
        Reset the xArm
        Warnning: without limit detection
        Note:
            1. The API will change self.last_used_position value into [201.5, 0, 140.5, -180, 0, 0]
            2. The API will change self.last_used_angles value into [0, 0, 0, 0, 0, 0, 0]
            3. If there are errors or warnings, this interface will clear the warnings and errors.
            4. If not ready, the api will auto enable motion and set state
            5. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc

        :param speed: reset speed (unit: rad/s if is_radian is True else °/s), default is 50 °/s
        :param mvacc: reset acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is 5000 °/s^2
        :param mvtime: reserved
        :param is_radian: the speed and acceleration are in radians or not, default is self.default_is_radian
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        """
        return self._arm.reset(speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, wait=wait, timeout=timeout)

    def set_pause_time(self, sltime, wait=False):
        """
        Set the arm pause time, xArm will pause sltime second
        
        :param sltime: sleep second
        :param wait: wait or not, default is False
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_pause_time(sltime, wait=wait)

    def set_tcp_offset(self, offset, is_radian=None):
        """
        Set the tool coordinate system offset at the end
        Note:
            1. Do not use if not required
            2. If not saved and you want to revert to the last saved value, please reset the offset by set_tcp_offset([0, 0, 0, 0, 0, 0])
            3. If not saved, it will be lost after reboot
            4. The save_conf interface can record the current settings and will not be lost after the restart.
            5. The clean_conf interface can restore system default settings
        
        :param offset: [x, y, z, roll, pitch, yaw]
        :param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_tcp_offset(offset, is_radian=is_radian)

    def set_tcp_jerk(self, jerk):
        """
        Set the translational jerk of Cartesian space
        Note:
            1. Do not use if not required
            2. If not saved, it will be lost after reboot
            3. The save_conf interface can record the current settings and will not be lost after the restart.
            4. The clean_conf interface can restore system default settings
        
        :param jerk: jerk (mm/s^3)
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_tcp_jerk(jerk)

    def set_tcp_maxacc(self, acc):
        """
        Set the max translational acceleration of Cartesian space
        Note:
            1. Do not use if not required
            2. If not saved, it will be lost after reboot
            3. The save_conf interface can record the current settings and will not be lost after the restart.
            4. The clean_conf interface can restore system default settings
        
        :param acc: max acceleration (mm/s^2)
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_tcp_maxacc(acc)

    def set_joint_jerk(self, jerk, is_radian=None):
        """
        Set the jerk of Joint space
        Note:
            1. Do not use if not required
            2. If not saved, it will be lost after reboot
            3. The save_conf interface can record the current settings and will not be lost after the restart.
            4. The clean_conf interface can restore system default settings
        
        :param jerk: jerk (°/s^3 or rad/s^3)
        :param is_radian: the jerk in radians or not, default is self.default_is_radian
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_joint_jerk(jerk, is_radian=is_radian)

    def set_joint_maxacc(self, acc, is_radian=None):
        """
        Set the max acceleration of Joint space
        
        Note:
            1. Do not use if not required
            2. If not saved, it will be lost after reboot
            3. The save_conf interface can record the current settings and will not be lost after the restart.
            4. The clean_conf interface can restore system default settings
        
        :param acc: max acceleration (°/s^2 or rad/s^2)
        :param is_radian: the jerk in radians or not, default is self.default_is_radian
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_joint_maxacc(acc, is_radian=is_radian)

    def set_tcp_load(self, weight, center_of_gravity):
        """
        Set the load
        
        Note:
            1. Do not use if not required
            2. If not saved, it will be lost after reboot
            3. The save_conf interface can record the current settings and will not be lost after the restart.
            4. The clean_conf interface can restore system default settings
        
        :param weight: load weight (unit: kg)
        :param center_of_gravity: load center of gravity, such as [x(mm), y(mm), z(mm)]
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_tcp_load(weight, center_of_gravity)

    def set_collision_sensitivity(self, value):
        """
        Set the sensitivity of collision
        
        Note:
            1. Do not use if not required
            2. If not saved, it will be lost after reboot
            3. The save_conf interface can record the current settings and will not be lost after the restart.
            4. The clean_conf interface can restore system default settings
        
        :param value: sensitivity value, 0~5
        :return: code
            code: See the API code documentation for details. 
        """
        return self._arm.set_collision_sensitivity(value)

    def set_teach_sensitivity(self, value):
        """
        Set the sensitivity of drag and teach
        
        Note:
            1. Do not use if not required
            2. If not saved, it will be lost after reboot
            3. The save_conf interface can record the current settings and will not be lost after the restart.
            4. The clean_conf interface can restore system default settings
        
        :param value: sensitivity value, 0~5
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_teach_sensitivity(value)

    def set_gravity_direction(self, direction):
        """
        Set the direction of gravity
        
        Note:
            1. Do not use if not required
            2. If not saved, it will be lost after reboot
            3. The save_conf interface can record the current settings and will not be lost after the restart.
            4. The clean_conf interface can restore system default settings
        
        :param direction: direction of gravity, such as [x(mm), y(mm), z(mm)]
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_gravity_direction(direction=direction)

    def set_mount_direction(self, base_tilt_deg, rotation_deg, is_radian=None):
        """
        Set the mount direction
        
        Note:
            1. Do not use if not required
            2. If not saved, it will be lost after reboot
            3. The save_conf interface can record the current settings and will not be lost after the restart.
            4. The clean_conf interface can restore system default settings
            
        :param base_tilt_deg: tilt degree
        :param rotation_deg: rotation degree
        :param is_radian: the jebase_tilt_deg/rotation_deg in radians or not, default is self.default_is_radian
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_mount_direction(base_tilt_deg, rotation_deg, is_radian=is_radian)

    def clean_conf(self):
        """
        Clean current config and restore system default settings
        Note:
            1. This interface will clear the current settings and restore to the original settings (system default settings)
        
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.clean_conf()

    def save_conf(self):
        """
        Save config
        Note:
            1. This interface can record the current settings and will not be lost after the restart.
            2. The clean_conf interface can restore system default settings
        
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.save_conf()

    def get_inverse_kinematics(self, pose, input_is_radian=None, return_is_radian=None):
        """
        Get inverse kinematics
        
        :param pose: [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
            Note: the roll/pitch/yaw unit is radian if input_is_radian is True, else °
        :param input_is_radian: the param pose value(only roll/pitch/yaw) is in radians or not, default is self.default_is_radian
        :param return_is_radian: the returned value is in radians or not, default is self.default_is_radian
        :return: tuple((code, angles)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
            angles: [angle-1(rad or °), angle-2, ..., angle-(Number of axes)] or []
                Note: the returned angle value is radians if return_is_radian is True, else °
        """
        return self._arm.get_inverse_kinematics(pose, input_is_radian=input_is_radian, return_is_radian=return_is_radian)

    def get_forward_kinematics(self, angles, input_is_radian=None, return_is_radian=None):
        """
        Get forward kinematics
        
        :param angles: [angle-1, angle-2, ..., angle-n], n is the number of axes of the arm
        :param input_is_radian: the param angles value is in radians or not, default is self.default_is_radian
        :param return_is_radian: the returned value is in radians or not, default is self.default_is_radian
        :return: tuple((code, pose)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
            pose: [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)] or []
                Note: the roll/pitch/yaw value is radians if return_is_radian is True, else °
        """
        return self._arm.get_forward_kinematics(angles, input_is_radian=input_is_radian, return_is_radian=return_is_radian)

    def is_tcp_limit(self, pose, is_radian=None):
        """
        Check the tcp pose is in limit
        
        :param pose: [x, y, z, roll, pitch, yaw]
        :param is_radian: roll/pitch/yaw value is radians or not, default is self.default_is_radian
        :return: tuple((code, limit)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
            limit: True/False/None, limit or not, or failed
        """
        return self._arm.is_tcp_limit(pose, is_radian=is_radian)

    def is_joint_limit(self, joint, is_radian=None):
        """
        Check the joint is in limit
        
        :param joint: [angle-1, angle-2, ..., angle-n], n is the number of axes of the arm
        :param is_radian: angle value is radians or not, default is self.default_is_radian
        :return: tuple((code, limit)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
            limit: True/False/None, limit or not, or failed
        """
        return self._arm.is_joint_limit(joint, is_radian=is_radian)

    def emergency_stop(self):
        """
        Emergency stop (set_state(4) -> motion_enable(True) -> set_state(0))
        Note:
            1. This interface does not automatically clear the error. If there is an error, you need to handle it according to the error code.
        """
        return self._arm.emergency_stop()

    def set_gripper_enable(self, enable):
        """
        Set the gripper enable
        
        :param enable: 
        :return: code
            code: See the Gripper code documentation for details.
        """
        return self._arm.set_gripper_enable(enable)

    def set_gripper_mode(self, mode):
        """
        Set the gripper mode
        
        :param mode: 1: location mode, 2: speed mode (no use), 3: torque mode (no use)
        :return: code
            code: See the Gripper code documentation for details.
        """
        return self._arm.set_gripper_mode(mode)

    def get_gripper_position(self):
        """
        Get the gripper position
        
        :return: tuple((code, pos)), only when code is 0, the returned result is correct.
            code: See the Gripper code documentation for details.
        """
        return self._arm.get_gripper_position()

    def set_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None):
        """
        Set the gripper position
        
        :param pos: pos
        :param wait: wait or not, default is False
        :param speed: speed
        :param auto_enable: auto enable or not, default is False
        :param timeout: second, default is 10s
        :return: code
            code: See the Gripper code documentation for details.
        """
        return self._arm.set_gripper_position(pos, wait=wait, speed=speed, auto_enable=auto_enable, timeout=timeout)

    def set_gripper_speed(self, speed):
        """
        Set the gripper speed
        
        :param speed: 
        :return: code
            code: See the Gripper code documentation for details.
        """
        return self._arm.set_gripper_speed(speed)

    def get_gripper_err_code(self):
        """
        Get the gripper error code
        
        :return: tuple((code, err_code)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
            err_code: See the Gripper code documentation for details.
        """
        return self._arm.get_gripper_err_code()

    def clean_gripper_error(self):
        """
        Clean the gripper error
        
        :return: code
            code: See the Gripper code documentation for details.
        """
        return self._arm.clean_gripper_error()

    def get_tgpio_digital(self, ionum=None):
        """
        Get the digital value of the specified Tool GPIO
        
        :param ionum: 0 or 1 or None(both 0 and 1), default is None
        :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
        """
        return self._arm.get_tgpio_digital(ionum)

    def set_tgpio_digital(self, ionum, value):
        """
        Set the digital value of the specified Tool GPIO
        
        :param ionum: 0 or 1
        :param value: value
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_tgpio_digital(ionum, value)

    def get_tgpio_analog(self, ionum=None):
        """
        Get the analog value of the specified Tool GPIO
        :param ionum: 0 or 1 or None(both 0 and 1), default is None
        :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
        """
        return self._arm.get_tgpio_analog(ionum)

    def get_cgpio_digital(self, ionum=None):
        """
        Get the digital value of the specified Controller GPIO

        :param ionum: 0~7 or None(both 0~7), default is None
        :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
        """
        return self._arm.get_cgpio_digital(ionum=ionum)

    def get_cgpio_analog(self, ionum=None):
        """
        Get the analog value of the specified Controller GPIO
        :param ionum: 0 or 1 or None(both 0 and 1), default is None
        :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
        """
        return self._arm.get_cgpio_analog(ionum=ionum)

    def set_cgpio_digital(self, ionum, value):
        """
        Set the digital value of the specified Controller GPIO

        :param ionum: 0~7
        :param value: value
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_cgpio_digital(ionum=ionum, value=value)

    def set_cgpio_analog(self, ionum, value):
        """
        Set the analog value of the specified Controller GPIO

        :param ionum: 0 or 1
        :param value: value
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_cgpio_analog(ionum=ionum, value=value)

    def set_cgpio_digital_input_function(self, ionum, fun):
        """
        Set the digital input functional mode of the Controller GPIO
        :param ionum: 0~7
        :param fun: functional mode
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_cgpio_digital_input_function(ionum=ionum, fun=fun)

    def set_cgpio_digital_output_function(self, ionum, fun):
        """
        Set the digital output functional mode of the specified Controller GPIO
        :param ionum: 0~7
        :param fun: functionnal mode
            0: system in stopping
            1: controller has error
            2: in motion
        :return: code
            code: See the API code documentation for details.
        """
        return self._arm.set_cgpio_digital_output_function(ionum=ionum, fun=fun)

    def get_cgpio_state(self):
        """
        Get the state of the Controller GPIO
        :return: code, states
            code: See the API code documentation for details.
            states: [...]
                states[0]: contorller gpio module state
                    states[0] == 0: normal
                    states[0] == 1：wrong
                    states[0] == 6：communication failure
                states[1]: controller gpio module error code
                    states[1] == 0: normal
                    states[1] != 0：error code
                states[2]: digital input functional gpio state
                    Note: digital-i-input functional gpio state = states[2] >> i & 0x01
                states[3]: digital input configuring gpio state
                    Note: digital-i-input configuring gpio state = states[3] >> i & 0x01
                states[4]: digital output functional gpio state
                    Note: digital-i-output functional gpio state = states[4] >> i & 0x01
                states[5]: digital output configuring gpio state
                    Note: digital-i-output configuring gpio state = states[5] >> i & 0x01
                states[6]: analog-0 input value
                states[7]: analog-1 input value
                states[8]: analog-0 output value
                states[9]: analog-1 output value
                states[10]: digital input functional info, [digital-0-input-functional-mode, ... digital-7-input-functional-mode]
                states[11]: digital output functional info, [digital-0-output-functional-mode, ... digital-7-output-functional-mode]
        """
        return self._arm.get_cgpio_state()

    def register_report_callback(self, callback=None, report_cartesian=True, report_joints=True,
                                 report_state=True, report_error_code=True, report_warn_code=True,
                                 report_mtable=True, report_mtbrake=True, report_cmd_num=True):
        """
        Register the report callback, only available if enable_report is True
        
        :param callback: 
            callback data:
            {
                'cartesian': [], # if report_cartesian is True
                'joints': [], # if report_joints is True
                'error_code': 0, # if report_error_code is True
                'warn_code': 0, # if report_warn_code is True
                'state': state, # if report_state is True
                'mtbrake': mtbrake, # if report_mtbrake is True, and available if enable_report is True and the connect way is socket
                'mtable': mtable, # if report_mtable is True, and available if enable_report is True and the connect way is socket
                'cmdnum': cmdnum, # if report_cmd_num is True
            }
        :param report_cartesian: report cartesian or not, default is True
        :param report_joints: report joints or not, default is True
        :param report_state: report state or not, default is True
        :param report_error_code: report error or not, default is True
        :param report_warn_code: report warn or not, default is True
        :param report_mtable: report motor enable states or not, default is True
        :param report_mtbrake: report motor brake states or not, default is True
        :param report_cmd_num: report cmdnum or not, default is True
        :return: True/False
        """
        return self._arm.register_report_callback(callback=callback,
                                                  report_cartesian=report_cartesian,
                                                  report_joints=report_joints,
                                                  report_state=report_state,
                                                  report_error_code=report_error_code,
                                                  report_warn_code=report_warn_code,
                                                  report_mtable=report_mtable,
                                                  report_mtbrake=report_mtbrake,
                                                  report_cmd_num=report_cmd_num)

    def register_report_location_callback(self, callback=None, report_cartesian=True, report_joints=True):
        """
        Register the report location callback, only available if enable_report is True
        
        :param callback: 
            callback data:
            {
                "cartesian": [x, y, z, roll, pitch, yaw], ## if report_cartesian is True
                "joints": [angle-1, angle-2, angle-3, angle-4, angle-5, angle-6, angle-7], ## if report_joints is True
            }
        :param report_cartesian: report or not, True/False, default is True
        :param report_joints: report or not, True/False, default is True
        :return: True/False
        """
        return self._arm.register_report_location_callback(callback=callback,
                                                           report_cartesian=report_cartesian,
                                                           report_joints=report_joints)

    def register_connect_changed_callback(self, callback=None):
        """
        Register the connect status changed callback
        
        :param callback: 
            callback data:
            {
                "connected": connected,
                "reported": reported,
            }
        :return: True/False
        """
        return self._arm.register_connect_changed_callback(callback=callback)

    def register_state_changed_callback(self, callback=None):
        """
        Register the state status changed callback, only available if enable_report is True
        
        :param callback:
            callback data:
            {
                "state": state,
            }
        :return: True/False
        """
        return self._arm.register_state_changed_callback(callback=callback)

    def register_mode_changed_callback(self, callback=None):
        """
        Register the mode changed callback, only available if enable_report is True and the connect way is socket

        :param callback:
            callback data:
            {
                "mode": mode,
            }
        :return: True/False
        """
        return self._arm.register_mode_changed_callback(callback=callback)

    def register_mtable_mtbrake_changed_callback(self, callback=None):
        """
        Register the motor enable states or motor brake states changed callback, only available if enable_report is True and the connect way is socket
        
        :param callback: 
            callback data:
            {
                "mtable": [motor-1-motion-enable, motor-2-motion-enable, ...],
                "mtbrake": [motor-1-brake-enable, motor-1-brake-enable,...],
            }
        :return: True/False
        """
        return self._arm.register_mtable_mtbrake_changed_callback(callback=callback)

    def register_error_warn_changed_callback(self, callback=None):
        """
        Register the error code or warn code changed callback, only available if enable_report is True
        
        :param callback: 
            callback data:
            {
                "error_code": error_code,
                "warn_code": warn_code,
            }
        :return: True/False
        """
        return self._arm.register_error_warn_changed_callback(callback=callback)

    def register_cmdnum_changed_callback(self, callback=None):
        """
        Register the cmdnum changed callback, only available if enable_report is True
        
        :param callback: 
            callback data:
            {
                "cmdnum": cmdnum
            }
        :return: True/False
        """
        return self._arm.register_cmdnum_changed_callback(callback=callback)

    def release_report_callback(self, callback=None):
        """
        Release the report callback
        
        :param callback: 
        :return: True/False
        """
        return self._arm.release_report_callback(callback)

    def release_report_location_callback(self, callback=None):
        """
        Release the location report callback
        
        :param callback: 
        :return: True/False
        """
        return self._arm.release_report_location_callback(callback)

    def release_connect_changed_callback(self, callback=None):
        """
        Release the connect changed callback
        
        :param callback: 
        :return: True/False
        """
        return self._arm.release_connect_changed_callback(callback)

    def release_state_changed_callback(self, callback=None):
        """
        Release the state changed callback
        
        :param callback: 
        :return: True/False
        """
        return self._arm.release_state_changed_callback(callback)

    def release_mode_changed_callback(self, callback=None):
        """
        Release the mode changed callback

        :param callback: 
        :return: True/False
        """
        return self._arm.release_mode_changed_callback(callback)

    def release_mtable_mtbrake_changed_callback(self, callback=None):
        """
        Release the motor enable states or motor brake states changed callback
        
        :param callback: 
        :return: True/False
        """
        return self._arm.release_mtable_mtbrake_changed_callback(callback)

    def release_error_warn_changed_callback(self, callback=None):
        """
        Release the error warn changed callback
        
        :param callback: 
        :return: True/False
        """
        return self._arm.release_error_warn_changed_callback(callback)

    def release_cmdnum_changed_callback(self, callback=None):
        """
        Release the cmdnum changed callback
        
        :param callback: 
        :return: True/False
        """
        return self._arm.release_cmdnum_changed_callback(callback)

    def get_servo_debug_msg(self, show=False):
        """
        Get the servo debug msg, used only for debugging
        
        :param show: show the detail info if True
        :return: tuple((code, servo_info_list)), only when code is 0, the returned result is correct.
            code: See the API code documentation for details.
        """
        return self._arm.get_servo_debug_msg(show=show)

    # def clean_servo_error(self, servo_id=None):
    #     """
    #     Danger, do not use, used only for debugging
    #     :param servo_id:
    #     :return:
    #     """
    #     return self._arm.clean_servo_error(servo_id)
    #
    # def set_gripper_zero(self):
    #     """
    #     Warning: do not use, used only for debugging
    #     :return:
    #     """
    #     return self._arm.set_gripper_zero()
    #
    # def set_servo_zero(self, servo_id=None):
    #     """
    #     Danger, do not use, used only for debugging
    #     :param servo_id: 1~7, 8
    #     :return:
    #     """
    #     return self._arm.set_servo_zero(servo_id=servo_id)
    #
    # def set_servo_addr_16(self, servo_id=None, addr=None, value=None):
    #     """
    #     Danger, do not use, used only for debugging
    #     :param servo_id:
    #     :param addr:
    #     :param value:
    #     :return:
    #     """
    #     return self._arm.set_servo_addr_16(servo_id=servo_id, addr=addr, value=value)
    #
    # def get_servo_addr_16(self, servo_id=None, addr=None):
    #     """
    #     Danger, do not use, used only for debugging
    #     :param servo_id:
    #     :param addr:
    #     :return:
    #     """
    #     return self._arm.get_servo_addr_16(servo_id=servo_id, addr=addr)
    #
    # def set_servo_addr_32(self, servo_id=None, addr=None, value=None):
    #     """
    #     Danger, do not use, used only for debugging
    #     :param servo_id:
    #     :param addr:
    #     :param value:
    #     :return:
    #     """
    #     return self._arm.set_servo_addr_32(servo_id=servo_id, addr=addr, value=value)
    #
    # def get_servo_addr_32(self, servo_id=None, addr=None):
    #     """
    #     Danger, do not use, used only for debugging
    #     :param servo_id:
    #     :param addr:
    #     :return:
    #     """
    #     return self._arm.get_servo_addr_32(servo_id=servo_id, addr=addr)

    def run_blockly_app(self, path):
        """
        Run the app generated by xArmStudio software
        :param path: app path
        """
        try:
            if not os.path.exists(path):
                path = os.path.join(os.path.expanduser('~'), '.UFACTORY', 'projects', 'test', 'xarm{}'.format(self.axis), 'app', 'myapp', path)
            if os.path.isdir(path):
                path = os.path.join(path, 'app.xml')
            if not os.path.exists(path):
                raise FileNotFoundError
            blockly_tool = BlocklyTool(path)
            succeed = blockly_tool.to_python(arm=self)
            if succeed:
                exec(blockly_tool.codes, {'arm': self})
            else:
                print('The conversion is incomplete and some blocks are not yet supported.')
        except Exception as e:
            print(e)
