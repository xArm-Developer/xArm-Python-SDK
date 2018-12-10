#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


from ..x3 import XArm


class XArmAPI(object):
    def __init__(self, port=None, baudrate=921600, timeout=None, filters=None, enable_heartbeat=True,
                 enable_report=True, report_type='normal', do_not_open=False,
                 limit_velo=None, limit_acc=None, limit_angle_velo=None, limit_angle_acc=None, is_radian=False):
        """
        The API wrapper of xArm
        :param port: port name(such as 'COM5'/'/dev/ttyUSB0') or ip-address(such as '192.168.1.185')
            Note: this parameter is required if parameter do_not_open is False
        :param baudrate: baudrate, only available in serial way, default is 921600
        :param timeout: timeout, only available in serial way, default is None
        :param filters: serial port filters, invalid, reserved.
        :param enable_heartbeat: whether to enable heartbeat, default is True, only available in socket way
        :param enable_report: whether to enable report, default is True
            Note: if enable_report is True, the self.last_used_position and self.last_used_angles value is the current position of robot
        :param report_type: report type('normal'/'real'/'rich'), only available in socket way, default is 'normal'
            Note:
                'normal': Reported at a frequency of 10 Hz
                'real': Reported at a frequency of 10 Hz (used only for debugging)
                'rich': Reported at a frequency of 100 Hz
        :param do_not_open: do not open, default is False
        :param limit_velo: limit velo, default is [0, 1000] mm/s
        :param limit_acc: limit acc, default is [0, 100000] mm/s^2
        :param limit_angle_velo: limit angle velo, default is [1°/s, 180°/s] (unit: °/s or radian/s)
            Note: If the parameter is_radian is True then use radian/s, otherwise use °/s
        :param limit_angle_acc: limit angle acc, default is [1°/s^2, 100000°/s^2] (unit: °/s^2 or radian/s^2)
            Note: If the parameter is_radian is True then use radian/s^2, otherwise use °/s^2
        :param is_radian: set the default is radian or not, default is False
            Note: (aim of design)
                1. Default value for unified interface parameters
                2: Unification of the external unit system
                3. For compatibility with previous interfaces
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
                    16: method: continuous_move_arc_line
            Note: This parameter determines the default return type for some interfaces (such as the position, velocity, and acceleration associated with the return angle arc).
                The affected attributes are as follows:
                    1. property: position
                    2. property: last_used_position
                    3. property: angles
                    4. property: last_used_angles
                    5. property: last_used_joint_speed
                    6. property: last_used_joint_acc
                    7. property: position_offset
        """
        self._arm = XArm(port=port,
                         baudrate=baudrate,
                         timeout=timeout,
                         filters=filters,
                         enable_heartbeat=enable_heartbeat,
                         enable_report=enable_report,
                         report_type=report_type,
                         do_not_open=do_not_open,
                         limit_velo=limit_velo,
                         limit_acc=limit_acc,
                         limit_angle_velo=limit_angle_velo,
                         limit_angle_acc=limit_angle_acc,
                         is_radian=is_radian)
        self.__attr_alias_map = {
            'get_ik': self.get_inverse_kinematics,
            'get_fk': self.get_forward_kinematics,
            'set_sleep_time': self.set_pause_time,
        }

    def __getattr__(self, item):
        if item in self.__attr_alias_map.keys():
            return self.__attr_alias_map[item]
        raise AttributeError('\'{}\' has not attribute \'{}\''.format(self.__class__.__name__, item))

    @property
    def core(self):
        """
        Core layer API, set only for advanced developers, do not provide documentation, please do not use
        :return: 
        """
        return self._arm.arm_cmd

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
        :return: 
        """
        return self._arm.default_is_radian

    @property
    def version(self):
        """
        xArm version
        """
        return self._arm.version

    @property
    def position(self):
        """
        Cartesion position
        Note:
            1. If self.default_is_radian is True, the returned value (only roll/yaw/pitch) is in radians
        return: [x(mm), y(mm), z(mm), roll(° or radian), yaw(° or radian), pitch(° or radian)]
        """
        return self._arm.position

    @property
    def last_used_position(self):
        """
        The last used cartesion position, default value of parameter x/y/z/roll/yaw/pitch of interface set_position
        Note:
            1. If self.default_is_radian is True, the returned value (only roll/yaw/pitch) is in radians
            2. self.set_position(x=300) <==> self.set_position(x=300, *last_used_position[1:])
            2. self.set_position(roll=-180) <==> self.set_position(x=self.last_used_position[:3], roll=-180, *self.last_used_position[4:])
        :return: [x(mm), y(mm), z(mm), roll(° or radian), yaw(° or radian), pitch(° or radian)]
        """
        return self._arm.last_used_position

    @property
    def last_used_tcp_speed(self):
        """
        The last used cartesion speed, default value of parameter speed of interface set_position
        :return: speed (mm/s)
        """
        return self._arm.last_used_tcp_speed

    @property
    def last_used_tcp_acc(self):
        """
        The last used cartesion acceleration, default value of parameter mvacc of interface set_position
        :return: acceleration (mm/s^2)
        """
        return self._arm.last_used_tcp_acc

    @property
    def angles(self):
        """
        Servo angles
        Note:
            1. If self.default_is_radian is True, the returned value is in radians
        :return: [angle1(° or radian), angle2(° or radian), ..., angle7(° or radian)]
        """
        return self._arm.angles

    @property
    def last_used_angles(self):
        """
        The last used servo angles, default value of parameter angle of interface set_servo_angle
        Note:
            1. If self.default_is_radian is True, the returned value is in radians
            2. self.set_servo_angle(servo_id=1, angle=75) <==> self.set_servo_angle(angle=[75] + self.last_used_angles[1:])
            3. self.set_servo_angle(servo_id=5, angle=30) <==> self.set_servo_angle(angle=self.last_used_angles[:4] + [30] + self.last_used_angles[5:])
        :return: [angle1(° or radian), angle2(° or radian), ..., angle7(° or radian)]
        """
        return self._arm.last_used_angles

    @property
    def last_used_joint_speed(self):
        """
        The last used joint speed, default value of parameter speed of interface set_servo_angle
        Note:
            1. If self.default_is_radian is True, the returned value is in radians
        :return: speed (°/s or radian/s)
        """
        return self._arm.last_used_joint_speed

    @property
    def last_used_joint_acc(self):
        """
        The last used joint acceleration, default value of parameter mvacc of interface set_servo_angle
        Note:
            1. If self.default_is_radian is True, the returned value is in radians
        :return: acceleration (°/s^2 or radian/s^2)
        """
        return self._arm.last_used_joint_acc

    @property
    def position_offset(self):
        """
        Cartesion position offset, only available in socket way and enable_report is True 
        Note:
            1. If self.default_is_radian is True, the returned value(roll_offset/yaw_offset/pitch_offset) is in radians
        :return: [x_offset(mm), y_offset(mm), z_offset(mm), roll_offset(° or radian), yaw_offset(° or radian), pitch_offset(° or radian)]
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
    def mtbrake(self):
        """
        Servo brake state list, only available in socket way and enable_report is True
        :return: [servo-1-brake-state, servo-2-..., servo-3-..., servo-4-..., servo-5-..., servo-6-..., servo-7-..., reserved]
            servo-{i}-brake-state:
                0: enable
                1: disable
        """
        return self._arm.mtbrake

    @property
    def maable(self):
        """
        Servo enable state list, only available in socket way and enable_report is True
        :return: [servo-1-enable-state, servo-2-..., servo-3-..., servo-4-..., servo-5-..., servo-6-..., servo-7-..., reserved]
            servo-{i}-enable-state:
                0: disable
                1: enable
        """
        return self._arm.maable

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
        Controller have an error or not
        """
        return self._arm.has_warn

    @property
    def error_code(self):
        """
        Controller error code. See the error code documentation for details.
        """
        return self._arm.error_code

    @property
    def warn_code(self):
        """
        Controller warn code. See the warn code documentation for details.
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
        :param command: 
        :return: code or tuple((code, ...))
            code: See the return code documentation for details.
        """
        return self._arm.send_cmd_sync(command=command)

    def get_position(self, is_radian=None):
        """
        Get the cartesian position
        Note:
            1. If the value(roll/yaw/pitch) you want to return is an radian unit, please set the parameter is_radian to True
                ex: code, pos = xarm.get_position(is_radian=True)
        :param is_radian: the returned value (only roll/yaw/pitch) is in radians or not, default is self.default_is_radian
        :return: tuple((code, [x, y, z, roll, yaw, pitch])), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
        """
        return self._arm.get_position(is_radian=is_radian)

    def set_position(self, x=None, y=None, z=None, roll=None, yaw=None, pitch=None, radius=None,
                     speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None,
                     wait=False, timeout=None, **kwargs):
        """
        Set the cartesian position, the API will modify self.last_used_position value
        Note:
            1. If the parameter(roll/yaw/pitch) you are passing is an radian unit, be sure to set the parameter is_radian to True.
                ex: code = xarm.set_position(x=300, y=0, z=200, roll=-3.14, yaw=0, pitch=0, is_radian=True)
            2. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
                ex: code = xarm.set_position(x=300, y=0, z=200, roll=180, yaw=0, pitch=0, is_radian=False, wait=True)
        :param x: cartesian position x, (unit: mm), default is self.last_used_position[0]
        :param y: cartesian position y, (unit: mm), default is self.last_used_position[1]
        :param z: cartesian position z, (unit: mm), default is self.last_used_position[2]
        :param roll: cartesian roll, (unit: radian if is_radian is True else °), default is self.last_used_position[3]
        :param yaw: cartesian yaw, (unit: radian if is_radian is True else °), default is self.last_used_position[4]
        :param pitch: cartesian pitch, (unit: radian if is_radian is True else °), default is self.last_used_position[5]
        :param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine
            MoveLine: Linear motion
                ex: code = xarm.set_position(..., radius=None)
            MoveArcLine: Linear arc motion with interpolation
                ex: code = xarm.set_position(..., radius=0)
        :param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
        :param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
        :param mvtime: 0, reserved
        :param relative: relative move or not
        :param is_radian: the roll/yaw/pitch in radians or not, default is self.default_is_radian
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        :param kwargs: reserved
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.set_position(x=x, y=y, z=z, pitch=pitch, yaw=yaw, roll=roll, radius=radius,
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
        :param servo_id: 1-7, None(8), default is None
        :param is_radian: the returned value is in radians or not, default is self.default_is_radian
        :return: tuple((code, angle list if servo_id is None or 8 else angle)), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
        """
        return self._arm.get_servo_angle(servo_id=servo_id, is_radian=is_radian)

    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None,
                        relative=False, is_radian=None, wait=False, timeout=None, **kwargs):
        """
        Set the servo angle, the API will modify self.last_used_angles value
        Note:
            1. If the parameter angle you are passing is an radian unit, be sure to set the parameter is_radian to True.
                ex: code = xarm.set_servo_angle(servo_id=3, angle=45, is_radian=True)
            2. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
                ex: code = xarm.set_servo_angle(servo_id=1, angle=45, is_radian=False,wait=True)
        :param servo_id: 1-7, None(8)
            1. 1-7 indicates the corresponding joint, the parameter angle should be a numeric value
                ex: code = xarm.set_servo_angle(servo_id=1, angle=45, is_radian=False)
            2. None(8) means all joints, default is None, the parameter angle should be a list of values whose length is the number of joints
                ex: code = xarm.set_servo_angle(angle=[30, -45, 0, 0, 0, 0, 80], is_radian=False)
        :param angle: angle or angle list, (unit: radian if is_radian is True else °)
            1. If servo_id is 1-7, angle should be a numeric value
                ex: code = xarm.set_servo_angle(servo_id=1, angle=45, is_radian=False)
            2. If servo_id is None or 8, angle should be a list of values whose length is the number of joints
                like [axis-1, axis-2, axis-3, axis-3, axis-4, axis-5, axis-6, axis-7]
                ex: code = xarm.set_servo_angle(angle=[30, -45, 0, 0, 0, 0, 80], is_radian=False)
        :param speed: move speed (unit: rad/s if is_radian is True else °/s), default is self.last_used_joint_speed
        :param mvacc: move acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is self.last_used_joint_acc
        :param mvtime: 0, reserved
        :param relative: relative move or not
        :param is_radian: the angle in radians or not, default is self.default_is_radian
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        :param kwargs: reserved
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.set_servo_angle(servo_id=servo_id, angle=angle, speed=speed, mvacc=mvacc, mvtime=mvtime,
                                         relative=relative, is_radian=is_radian, wait=wait, timeout=timeout, **kwargs)

    def set_servo_angle_j(self, angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):
        """
        Set the servo angle, execute only the last instruction, need to be set to servo motion mode
        :param angles: angle list, (unit: radian if is_radian is True else °)
        :param speed: speed, reserved
        :param mvacc: acceleration, reserved
        :param mvtime: 0, reserved
        :param is_radian: the angles in radians or not, defalut is self.default_is_radian
        :param kwargs: reserved
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.set_servo_angle_j(angles, speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, **kwargs)

    def move_gohome(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):
        """
        Move to go home (Back to zero), the API will modify self.last_used_position and self.last_used_angles value
        Warnning: without limit detection
        Note:
            1. The API will change self.last_used_position value into [201.5, 0, 140.5, -180, 0, 0]
            2. The API will change self.last_used_angles value into [0, 0, 0, 0, 0, 0, 0]
            3. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
                ex: code = xarm.move_gohome(wait=True)
        :param speed: gohome speed (unit: rad/s if is_radian is True else °/s), default is 50 °/s
        :param mvacc: gohome acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is 5000 °/s^2
        :param mvtime: reserved
        :param is_radian: the speed and acceleration are in radians or not, default is self.default_is_radian
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.move_gohome(speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, wait=wait, timeout=timeout)

    def continuous_move_arc_line(self, paths, is_radian=None, times=1, first_pause_time=0.1, repeat_pause_time=0,
                                 automatic_calibration=True, speed=None, mvacc=None, mvtime=None, wait=False):
        """
        Continuous linear motion with interpolation
        Note:
            1. If an error occurs, it will return early
            2. If the emergency_stop interface is called actively, it will return early.
        :param paths: cartesian path list
            1. Specify arc radius： [[x, y, z, roll, yaw, pitch, radius], ....]
            1. Do not specify arc radius (radius=0)： [[x, y, z, roll, yaw, pitch], ....]
        :param is_radian: roll/yaw/pitch of paths are in radians or not, default is self.default_is_radian
        :param times: repeat times, 0 is infinite loop, default is 1
        :param first_pause_time: sleep time at first, purpose is to cache the instruction, default is 0.1s
        :param repeat_pause_time: interval between repeated movements, unit: second
        :param automatic_calibration: automatic calibration or not, default is True
        :param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
        :param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
        :param mvtime: 0, reserved 
        :param wait: whether to wait for the arm to complete, default is False
        """
        return self._arm.continuous_move_arc_line(paths, is_radian=is_radian, times=times, first_pause_time=first_pause_time,
                                                  repeat_pause_time=repeat_pause_time, automatic_calibration=automatic_calibration,
                                                  speed=speed, mvacc=mvacc, mvtime=mvtime, wait=wait)

    def set_servo_attach(self, servo_id=None):
        """
        Attach the servo
        :param servo_id: 1-7, 8, if servo_id is 8, will attach all servo
            1. 1-7: attach only one joint
                ex: xarm.set_servo_attach(servo_id=1)
            2: 8: attach all joints
                ex: xarm.set_servo_attach(servo_id=8)
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.set_servo_attach(servo_id=servo_id)

    def set_servo_detach(self, servo_id=None):
        """
        Detach the servo, be sure to do protective work before unlocking to avoid injury or damage.
        :param servo_id: 1-7, 8, if servo_id is 8, will detach all servo
            1. 1-7: detach only one joint
                ex: xarm.set_servo_detach(servo_id=1)
            2: 8: detach all joints, please
                ex: xarm.set_servo_detach(servo_id=8)
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.set_servo_detach(servo_id=servo_id)

    def get_version(self):
        """
        Get the xArm version
        :return: tuple((code, version)), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
        """
        return self._arm.get_version()

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
            code: See the return code documentation for details.
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
            code: See the return code documentation for details.
        """
        return self._arm.set_state(state=state)

    def set_mode(self, mode=0):
        """
        Set the xArm mode
        :param mode: default is 0
            0: position control mode
            1: servo motion mode
            2: joint teaching mode (invalid)
            3: cartesian teaching mode (invalid)
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.set_mode(mode=mode)

    def get_cmdnum(self):
        """
        Get the cmd count in cache
        :return: tuple((code, cmd_num)), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
        """
        return self._arm.get_cmdnum()

    def get_err_warn_code(self, show=False):
        """
        Get the controller error and warn code
        :param show: show the detail info if True
        :return: tuple((code, [error_code, warn_code])), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
            error_code: See the controller error code documentation for details.
            warn_code: See the controller warn code documentation for details.
        """
        return self._arm.get_err_warn_code(show=show)

    def clean_error(self):
        """
        Clean the error, need to be manually enabled motion and set state after clean error
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.clean_error()

    def clean_warn(self):
        """
        Clean the warn
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.clean_warn()

    def motion_enable(self, enable=True, servo_id=None):
        """
        Motion enable
        :param enable: 
        :param servo_id: 1-7, None(8)
        :return: code
            code: See the return code documentation for details.
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
            code: See the return code documentation for details.
        """
        return self._arm.set_pause_time(sltime, wait=wait)

    def set_tcp_offset(self, offset, is_radian=None):
        """
        Set the tool coordinate system offset at the end
        Note:
            1. Do not use if not required
            2. If not saved, it will be lost after reboot
            3. The save_conf interface can record the current settings and will not be lost after the restart.
            4. The clean_conf interface can restore system default settings
        :param offset: [x, y, z, roll, yaw, pitch]
        :param is_radian: the roll/yaw/pitch in radians or not, default is self.default_is_radian
        :return: code
            code: See the return code documentation for details.
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
            code: See the return code documentation for details.
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
        :param acc: acceleration (mm/s^2)
        :return: code
            code: See the return code documentation for details.
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
        :param jerk: jerk (°/s^3 or radian/s^3)
        :param is_radian: the jerk in radians or not, default is self.default_is_radian
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.set_joint_jerk(jerk, is_radian=is_radian)

    def set_joint_maxacc(self, acc, is_radian=None):
        """
        Set the max acceleration of Joint space
        :param acc: mac acceleration (°/s^2 or radian/s^2)
        :param is_radian: the jerk in radians or not, default is self.default_is_radian
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.set_joint_maxacc(acc, is_radian=is_radian)

    def clean_conf(self):
        """
        Clean current config and restore system default settings
        Note:
            1. This interface will clear the current settings and restore to the original settings (system default settings)
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.clean_conf()

    def save_conf(self):
        """
        Save config
        Note:
            1. This interface can record the current settings and will not be lost after the restart.
            2. The clean_conf interface can restore system default settings
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.save_conf()

    def get_inverse_kinematics(self, pose, input_is_radian=None, return_is_radian=None):
        """
        Get inverse kinematics
        :param pose: [x(mm), y(mm), z(mm), roll(radian or radian or °), yaw(radian or radian or °), pitch(radian or radian or °)]
            Note: the roll/yaw/pitch unit is radian if input_is_radian is True, else °
        :param input_is_radian: the param pose value(only roll/yaw/pitch) is in radians or not, default is self.default_is_radian
        :param return_is_radian: the returned value is in radians or not, default is self.default_is_radian
        :return: tuple((code, angles)), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
            angles: [angle-1(radian or radian or °), angle-2, ..., angle-7] or []
                Note: the returned angle value unit is radian if return_is_radian is True, else °
        """
        return self._arm.get_inverse_kinematics(pose, input_is_radian=input_is_radian, return_is_radian=return_is_radian)

    def get_forward_kinematics(self, angles, input_is_radian=None, return_is_radian=None):
        """
        get forward kinematics
        :param angles: [angle-1, angle-2, ..., angle-7]
        :param input_is_radian: the param angles value is in radians or not, default is self.default_is_radian
        :param return_is_radian: the returned value is in radians or not, default is self.default_is_radian
        :return: tuple((code, pose)), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
            pose: [x(mm), y(mm), z(mm), roll(radian or °), yaw(radian or °), pitch(radian or °)] or []
                Note: the roll/yaw/pitch unit is radian if return_is_radian is True, else °
        """
        return self._arm.get_forward_kinematics(angles, input_is_radian=input_is_radian, return_is_radian=return_is_radian)

    def is_tcp_limit(self, pose, is_radian=None):
        """
        Check the tcp pose is in limit
        :param pose: [x, y, z, roll, yaw, pitch]
        :param is_radian: roll/yaw/pitch value is radian or not, default is self.default_is_radian
        :return: tuple((code, limit)), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
            limit: True/False/None, limit or not, or failed
        """
        return self._arm.is_tcp_limit(pose, is_radian=is_radian)

    def is_joint_limit(self, joint, is_radian=None):
        """
        Check the joint is in limit
        :param joint: angle list
        :param is_radian: angle value is radian or not, default is self.default_is_radian
        :return: tuple((code, limit)), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
            limit: True/False/None, limit or not, or failed
        """
        return self._arm.is_joint_limit(joint, is_radian=is_radian)

    def set_params(self, **kwargs):
        """
        internal use
        :param kwargs: 
        :return: 
        """
        return self._arm.set_params(**kwargs)

    def get_params(self, is_radian=None):
        """
        internal use
        :return: 
        """
        return self._arm.get_params(is_radian=is_radian)

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
            code: See the return code documentation for details.
        """
        return self._arm.gripper_enable(enable)

    def set_gripper_mode(self, mode):
        """
        Set the gripper mode
        :param mode: 1: location mode, 2: speed mode (no use), 3: torque mode (no use)
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.set_gripper_mode(mode)

    def get_gripper_position(self):
        """
        Get the gripper position
        :return: tuple((code, pos)), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
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
            code: See the return code documentation for details.
        """
        return self._arm.set_gripper_position(pos, wait=wait, speed=speed, auto_enable=auto_enable, timeout=timeout)

    def set_gripper_speed(self, speed):
        """
        Set the gripper speed
        :param speed: 
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.set_gripper_speed(speed)

    def get_gripper_err_code(self):
        """
        Get the gripper error code
        :return: tuple((code, err_code)), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
        """
        return self._arm.get_gripper_err_code()

    def clean_gripper_error(self):
        """
        Clean the gripper error
        :return: code
            code: See the return code documentation for details.
        """
        return self._arm.clean_gripper_error()

    def register_report_callback(self, callback=None, report_cartesian=True, report_joints=True,
                                 report_state=True, report_error_code=True, report_warn_code=True,
                                 report_maable=True, report_mtbrake=True, report_cmd_num=True):
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
                'maable': maable, # if report_maable is True, and available if enable_report is True and the connect way is socket
                'cmdnum': cmdnum, # if report_cmd_num is True
            }
        :param report_cartesian: report cartesian or not, default is True
        :param report_joints: report joints or not, default is True
        :param report_state: report state or not, default is True
        :param report_error_code: report error or not, default is True
        :param report_warn_code: report warn or not, default is True
        :param report_maable: report maable or not, default is True
        :param report_mtbrake: report mtbrake or not, default is True
        :param report_cmd_num: report cmdnum or not, default is True
        :return: 
        """
        return self._arm.register_report_callback(callback=callback,
                                                  report_cartesian=report_cartesian,
                                                  report_joints=report_joints,
                                                  report_state=report_state,
                                                  report_error_code=report_error_code,
                                                  report_warn_code=report_warn_code,
                                                  report_maable=report_maable,
                                                  report_mtbrake=report_mtbrake,
                                                  report_cmd_num=report_cmd_num)

    def register_report_location_callback(self, callback=None, report_cartesian=True, report_joints=True):
        """
        Register the report location callback, only available if enable_report is True
        :param callback: 
            callback data:
            {
                "cartesian": [x, y, z, roll, yaw, pitch], ## if report_cartesian is True
                "joints": [angle-1, angle-2, angle-3, angle-4, angle-5, angle-6, angle-7], ## if report_joints is True
            }
        :param report_cartesian: report or not, True/False, default is True
        :param report_joints: report or not, True/False, default is True
        :return: 
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
        :return: 
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
        :return: 
        """
        return self._arm.register_state_changed_callback(callback=callback)

    def register_maable_mtbrake_changed_callback(self, callback=None):
        """
        Register the maable or mtbrake status changed callback, only available if enable_report is True and the connect way is socket
        :param callback: 
            callback data:
            {
                "maable": [axis-1-motion-enable, axis-2-motion-enable, ...],
                "mtbrake": [axis-1-brake-enable, axis-1-brake-enable,...],
            }
        :return: 
        """
        return self._arm.register_maable_mtbrake_changed_callback(callback=callback)

    def register_error_warn_changed_callback(self, callback=None):
        """
        Register the error code or warn code changed callback, only available if enable_report is True
        :param callback: 
            callback data:
            {
                "error_code": error_code,
                "warn_code": warn_code,
            }
        :return: 
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
        :return: 
        """
        return self._arm.register_cmdnum_changed_callback(callback=callback)

    def release_report_callback(self, callback=None):
        """
        Release the report callback
        :param callback: 
        :return: 
        """
        return self._arm.release_report_callback(callback)

    def release_report_location_callback(self, callback=None):
        """
        Release the location report callback
        :param callback: 
        :return: 
        """
        return self._arm.release_report_location_callback(callback)

    def release_connect_changed_callback(self, callback=None):
        """
        Release the connect changed callback
        :param callback: 
        :return: 
        """
        return self._arm.release_connect_changed_callback(callback)

    def release_state_changed_callback(self, callback=None):
        """
        Release the state changed callback
        :param callback: 
        :return: 
        """
        return self._arm.release_state_changed_callback(callback)

    def release_maable_mtbrake_changed_callback(self, callback=None):
        """
        Release the maable or mtbrake changed callback
        :param callback: 
        :return: 
        """
        return self._arm.release_maable_mtbrake_changed_callback(callback)

    def release_error_warn_changed_callback(self, callback=None):
        """
        Release the error warn changed callback
        :param callback: 
        :return: 
        """
        return self._arm.release_error_warn_changed_callback(callback)

    def release_cmdnum_changed_callback(self, callback=None):
        """
        Release the cmdnum changed callback
        :param callback: 
        :return: 
        """
        return self._arm.release_cmdnum_changed_callback(callback)

    def get_servo_debug_msg(self, show=False):
        """
        Get the servo debug msg, used only for debugging
        :param show: show the detail info if True
        :return: tuple((code, servo_info_list)), only when code is 0, the returned result is correct.
            code: See the return code documentation for details.
        """
        return self._arm.get_servo_debug_msg(show=show)

    # def clean_servo_error(self, servo_id=None):
    #     """
    #     Warnning, do not use, used only for debugging
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
    #     Warnning, do not use, used only for debugging
    #     :param servo_id: 1~7, 8
    #     :return:
    #     """
    #     return self._arm.set_servo_zero(servo_id=servo_id)
    #
    # def set_servo_addr_16(self, servo_id=None, addr=None, value=None):
    #     """
    #     Warnning, do not use, used only for debugging
    #     :param servo_id:
    #     :param addr:
    #     :param value:
    #     :return:
    #     """
    #     return self._arm.set_servo_addr_16(servo_id=servo_id, addr=addr, value=value)
    #
    # def get_servo_addr_16(self, servo_id=None, addr=None):
    #     """
    #     Warnning, do not use, used only for debugging
    #     :param servo_id:
    #     :param addr:
    #     :return:
    #     """
    #     return self._arm.get_servo_addr_16(servo_id=servo_id, addr=addr)
    #
    # def set_servo_addr_32(self, servo_id=None, addr=None, value=None):
    #     """
    #     Warnning, do not use, used only for debugging
    #     :param servo_id:
    #     :param addr:
    #     :param value:
    #     :return:
    #     """
    #     return self._arm.set_servo_addr_32(servo_id=servo_id, addr=addr, value=value)
    #
    # def get_servo_addr_32(self, servo_id=None, addr=None):
    #     """
    #     Warnning, do not use, used only for debugging
    #     :param servo_id:
    #     :param addr:
    #     :return:
    #     """
    #     return self._arm.get_servo_addr_32(servo_id=servo_id, addr=addr)


