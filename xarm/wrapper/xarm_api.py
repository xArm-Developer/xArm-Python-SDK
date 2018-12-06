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
                 limit_velo=None, limit_acc=None, limit_angle_velo=None, limit_angle_acc=None):
        """
        The API wrapper of xArm
        :param port: port name(such as 'COM5'/'/dev/ttyUSB0') or ip-address(such as '192.168.1.185')
        :param baudrate: baudrate, only available in serial way
        :param timeout: timeout, only available in serial way
        :param filters: filters, reserved.
        :param enable_heartbeat: whether to enable heartbeat, default is True, only available in socket way
        :param enable_report: whether to enable report, default is True
        :param report_type: report type('normal'/'real'/'rich'), only available in socket way
        :param do_not_open: do not open, default is False
        :param limit_velo: limit velo, default is [0, 1000] mm/s
        :param limit_acc: limit acc, default is [0, 100000] mm/s^2
        :param limit_angle_velo: limit angle velo, default is [1, 180] °/s
        :param limit_angle_acc: limit angle acc, default is [1, 100000] °/s^2
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
                         limit_angle_acc=limit_angle_acc)

    @property
    def connected(self):
        """
        Connection status
        """
        return self._arm.connected

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
        return: [x(mm), y(mm), z(mm), roll(radian), yaw(radian), pitch(radian)]
        """
        return self._arm.position

    @property
    def angles(self):
        """
        Servo angles
        :return: [angle1(radian), angle2(radian), angle3(radian), angle4(radian), angle5(radian), angle6(radian), angle7(radian)]
        """
        return self._arm.angles

    @property
    def position_offset(self):
        """
        Cartesion position offset, only available in socket way and enable_report is True 
        :return: [x_offset(mm), y_offset(mm), z_offset(mm), roll_offset(radian), yaw_offset(radian), pitch_offset(radian)]
        """
        return self._arm.position_offset

    @property
    def state(self):
        """
        xArm state
        :return: 0: in motion, 1: sleeping, 2: suspended, 3: stopping
        """
        return self._arm.state

    @property
    def mtbrake(self):
        """
        Servo brake state, only available in socket way and enable_report is True
        :return: [servo-1, servo-2, servo-3, servo-4, servo-5, servo-6, servo-7, reserved]
        """
        return self._arm.mtbrake

    @property
    def maable(self):
        """
        Servo enable state, only available in socket way and enable_report is True
        :return: [servo-1, servo-2, servo-3, servo-4, servo-5, servo-6, servo-7, reserved]
        """
        return self._arm.maable

    @property
    def has_err_warn(self):
        """
        xArm has error warn or not
        :return: True/False
        """
        return self._arm.has_err_warn

    @property
    def error_code(self):
        """
        Error code
        """
        return self._arm.error_code

    @property
    def warn_code(self):
        """
        Warn code
        """
        return self._arm.warn_code

    @property
    def cmd_num(self):
        """
        Number of command caches
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
        :param port: port name or the ip address
        :param baudrate: baudrate, only available in serial way
        :param timeout: timeout, only available in serial way
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
        """
        return self._arm.send_cmd_sync(command=command)

    def get_position(self, is_radian=True):
        """
        Get the cartesian position
        Note:
            1. If the value(roll/yaw/pitch) you want to return is an angle unit, please set the parameter is_radian to False
                ex: code, pos = xarm.get_position(is_radian=False)
        :param is_radian: the returned value (only roll/yaw/pitch) is in radians, default is True
        :return: tuple((code, [x, y, z, roll, yaw, pitch])), only when code is 0, the returned result is correct.
        """
        return self._arm.get_position(is_radian=is_radian)

    def set_position(self, x=None, y=None, z=None, roll=None, yaw=None, pitch=None, radius=None,
                     speed=None, mvacc=None, mvtime=None, relative=False, is_radian=True,
                     wait=False, timeout=None, **kwargs):
        """
        Set the cartesian position
        Note:
            1. If the parameter(roll/yaw/pitch) you are passing is an angle unit, be sure to set the parameter is_radian to False.
                ex: code = xarm.set_position(x=300, y=0, z=200, roll=180, yaw=0, pitch=0, is_radian=False)
            2. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
                ex: code = xarm.set_position(x=300, y=0, z=200, roll=180, yaw=0, pitch=0, is_radian=False, wait=True)
        :param x: cartesian position x, (unit: mm)
        :param y: cartesian position y, (unit: mm)
        :param z: cartesian position z, (unit: mm)
        :param roll: cartesian roll, (unit: radian if is_radian is True else °)
        :param yaw: cartesian yaw, (unit: radian if is_radian is True else °)
        :param pitch: cartesian pitch, (unit: radian if is_radian is True else °)
        :param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine
            MoveLine: Linear motion
                ex: code = xarm.set_position(..., radius=None)
            MoveArcLine: Linear arc motion with interpolation
                ex: code = xarm.set_position(..., radius=0)
        :param speed: move speed (mm/s, rad/s)
        :param mvacc: move acceleration (mm/s^2, rad/s^2)
        :param mvtime: 0, reserved
        :param relative: relative move or not
        :param is_radian: the roll/yaw/pitch in radians or not, default is True
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        :param kwargs: reserved
        :return: code
        """
        return self._arm.set_position(x=x, y=y, z=z, pitch=pitch, yaw=yaw, roll=roll, radius=radius,
                                      speed=speed, mvacc=mvacc, mvtime=mvtime, relative=relative,
                                      is_radian=is_radian, wait=wait, timeout=timeout, **kwargs)

    def get_servo_angle(self, servo_id=None, is_radian=True):
        """
        Get the servo angle
        Note:
            1. If the value you want to return is an angle unit, please set the parameter is_radian to False
                ex: code, angles = xarm.get_servo_angle(is_radian=False)
            2. If you want to return only the angle of a single joint, please set the parameter servo_id
                ex: code, angle = xarm.get_servo_angle(servo_id=2)
        :param servo_id: 1-7, None(8), default is None
        :param is_radian: the returned value is in radians or not, default is True
        :return: tuple((code, angle list if servo_id is None or 8 else angle)), only when code is 0, the returned result is correct.
        """
        return self._arm.get_servo_angle(servo_id=servo_id, is_radian=is_radian)

    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None,
                        relative=False, is_radian=True, wait=False, timeout=None, **kwargs):
        """
        Set the servo angle
        Note:
            1. If the parameter angle you are passing is an angle unit, be sure to set the parameter is_radian to False.
                ex: code = xarm.set_servo_angle(servo_id=3, angle=45, is_radian=False)
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
        :param speed: move speed (unit: rad/s if is_radian is True else °/s)
        :param mvacc: move acceleration (unit: rad/s^2 if is_radian is True else °/s^2)
        :param mvtime: 0, reserved
        :param relative: relative move or not
        :param is_radian: the angle in radians or not, default is True
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        :param kwargs: reserved
        :return: code
        """
        return self._arm.set_servo_angle(servo_id=servo_id, angle=angle, speed=speed, mvacc=mvacc, mvtime=mvtime,
                                         relative=relative, is_radian=is_radian, wait=wait, timeout=timeout, **kwargs)

    def set_servo_angle_j(self, angles, speed=None, mvacc=None, mvtime=None, is_radian=True, **kwargs):
        """
        Set the servo angle, execute only the last instruction
        :param angles: angle list, (unit: radian if is_radian is True else °)
        :param speed: speed, reserved
        :param mvacc: acceleration, reserved
        :param mvtime: 0, reserved
        :param is_radian: the angles in radians or not, defalut is True
        :param kwargs: reserved
        :return: 
        """
        return self._arm.set_servo_angle_j(angles, speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, **kwargs)

    def move_gohome(self, speed=None, mvacc=None, mvtime=None, is_radian=True, wait=False, timeout=None):
        """
        Move to go home (Back to zero)
        Note:
            1. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
                ex: code = xarm.move_gohome(wait=True)
        :param speed: gohome speed (unit: rad/s if is_radian is True else °/s)
        :param mvacc: gohome acceleration (unit: rad/s^2 if is_radian is True else °/s^2)
        :param mvtime: 0, reserved
        :param is_radian: the speed and acceleration are in radians or not, default is True
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        :return: code
        """
        return self._arm.move_gohome(speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, wait=wait, timeout=timeout)

    def set_servo_attach(self, servo_id=None):
        """
        Attach the servo
        :param servo_id: 1-7, 8, if servo_id is 8, will attach all servo
            1. 1-7: attach only one joint
                ex: xarm.set_servo_attach(servo_id=1)
            2: 8: attach all joints
                ex: xarm.set_servo_attach(servo_id=8)
        :return: code
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
        """
        return self._arm.set_servo_detach(servo_id=servo_id)

    def get_version(self):
        """
        Get the xArm version
        :return: tuple((code, version)), only when code is 0, the returned result is correct.
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
            state:
                1: moving
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
        """
        return self._arm.set_state(state=state)

    def set_mode(self, mode=0):
        """
        Set the xArm mode
        :param mode: default is 0
        :return: code
        """
        return self._arm.set_mode(mode=mode)

    def get_cmdnum(self):
        """
        Get the cmd count in cache
        :return: tuple((code, cmd num)), only when code is 0, the returned result is correct.
        """
        return self._arm.get_cmdnum()

    def get_err_warn_code(self, show=False):
        """
        Get the error and warn code
        :param show: show the detail info if True
        :return: tuple((code, [error_code, warn_code])), only when code is 0, the returned result is correct.
        """
        return self._arm.get_err_warn_code(show=show)

    def clean_error(self):
        """
        Clean the error, need to be manually enabled motion and set state after clean error
        :return: code
        """
        return self._arm.clean_error()

    def clean_warn(self):
        """
        Clean the warn
        :return: code
        """
        return self._arm.clean_warn()

    def motion_enable(self, enable=True, servo_id=None):
        """
        Motion enable
        :param enable: 
        :param servo_id: 1-7, None(8)
        :return: code
        """
        return self._arm.motion_enable(servo_id=servo_id, enable=enable)

    def reset(self, speed=None, mvacc=None, mvtime=None, is_radian=True, wait=False, timeout=None):
        """
        Reset the xArm (motion enable -> set state -> back to zero)
        Note:
            1. If there are errors or warnings, this interface will clear the warnings and errors.
        :param speed: reset speed (unit: rad/s if is_radian is True else °/s)
        :param mvacc: reset acceleration (unit: rad/s^2 if is_radian is True else °/s^2)
        :param mvtime: reserved
        :param is_radian: the speed and acceleration are in radians or not, default is True
        :param wait: whether to wait for the arm to complete, default is False
        :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True
        """
        return self._arm.reset(speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, wait=wait, timeout=timeout)

    def set_sleep_time(self, sltime, wait=False):
        """
        Set the arm pause time, xArm will pause sltime second
        :param sltime: sleep second
        :param wait: wait or not, default is False
        :return: code
        """
        return self._arm.set_sleep_time(sltime, wait=wait)

    def set_tcp_offset(self, offset):
        """
        Set tcp offset, do not use, just for debugging
        :param offset: 
        :return: code
        """
        return self._arm.set_tcp_offset(offset)

    def set_tcp_jerk(self, jerk):
        """
        Set tcp jerk, do not use, just for debugging
        :param jerk: 
        :return: code
        """
        return self._arm.set_tcp_jerk(jerk)

    def set_tcp_maxacc(self, acc):
        """
        Set tcp maxacc, do not use, just for debugging
        :param acc: 
        :return: code
        """
        return self._arm.set_tcp_maxacc(acc)

    def set_joint_jerk(self, jerk):
        """
        Set joint jerk, do not use, just for debugging
        :param jerk: 
        :return: code
        """
        return self._arm.set_joint_jerk(jerk)

    def set_joint_maxacc(self, acc):
        """
        Set joint maxacc, do not use, just for debugging
        :param acc: 
        :return: code
        """
        return self._arm.set_joint_maxacc(acc)

    def clean_conf(self):
        """
        Clean config
        :return: code
        """
        return self._arm.clean_conf()

    def save_conf(self):
        """
        Save config
        :return: code
        """
        return self._arm.save_conf()

    def get_ik(self, pose, is_radian=True):
        """
        Inverse kinematics, do not use, just for debugging
        :param pose: 
        :param is_radian:
        :return: tuple((code, angles)), only when code is 0, the returned result is correct.
        """
        return self._arm.get_ik(pose, is_radian=is_radian)

    def get_fk(self, angles, is_radian=True):
        """
        Positive kinematics, do not use, just for debugging
        :param angles: 
        :param is_radian: 
        :return: tuple((code, pose)), only when code is 0, the returned result is correct.
        """
        return self._arm.get_fk(angles, is_radian=is_radian)

    def is_tcp_limit(self, pose, is_radian=True):
        """
        Check the tcp pose is in limit
        :param pose: [x, y, z, roll, yaw, pitch]
        :param is_radian: roll/yaw/pitch value is radian or not, default is True
        :return: tuple((code, limit)), only when code is 0, the returned result is correct.
        """
        return self._arm.is_tcp_limit(pose, is_radian=is_radian)

    def is_joint_limit(self, joint, is_radian=True):
        """
        Check the joint is in limit
        :param joint: angle list
        :param is_radian: angle value is radian or not, default is True
        :return: tuple((code, limit)), only when code is 0, the returned result is correct.
        """
        return self._arm.is_joint_limit(joint, is_radian=is_radian)

    def set_params(self, **kwargs):
        return self._arm.set_params(**kwargs)

    def get_params(self):
        return self._arm.get_params()

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
        """
        return self._arm.gripper_enable(enable)

    def set_gripper_mode(self, mode):
        """
        Set the gripper mode
        :param mode: 1: location mode, 2: speed mode (no use), 3: torque mode (no use)
        :return: code
        """
        return self._arm.set_gripper_mode(mode)

    def get_gripper_position(self):
        """
        Get the gripper position
        :return: tuple((code, pos)), only when code is 0, the returned result is correct.
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
        """
        return self._arm.set_gripper_position(pos, wait=wait, speed=speed, auto_enable=auto_enable, timeout=timeout)

    def set_gripper_speed(self, speed):
        """
        Set the gripper speed
        :param speed: 
        :return: code
        """
        return self._arm.set_gripper_speed(speed)

    def get_gripper_err_code(self):
        """
        Get the gripper error code
        :return: tuple((code, err_code)), only when code is 0, the returned result is correct.
        """
        return self._arm.get_gripper_err_code()

    def clean_gripper_error(self):
        """
        Clean the gripper error
        :return: code
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
        Get the servo debug msg, just for debugging
        :param show: show the detail info if True
        :return: tuple((code, servo_info_list)), only when code is 0, the returned result is correct.
        """
        return self._arm.get_servo_debug_msg(show=show)

    # def clean_servo_error(self, servo_id=None):
    #     """
    #     Warnning, do not use, just for debugging
    #     :param servo_id:
    #     :return:
    #     """
    #     return self._arm.clean_servo_error(servo_id)
    #
    # def set_gripper_zero(self):
    #     """
    #     Warning: do not use, just for debugging
    #     :return:
    #     """
    #     return self._arm.set_gripper_zero()
    #
    # def set_servo_zero(self, servo_id=None):
    #     """
    #     Warnning, do not use, just for debugging
    #     :param servo_id: 1~7, 8
    #     :return:
    #     """
    #     return self._arm.set_servo_zero(servo_id=servo_id)
    #
    # def set_servo_addr_16(self, servo_id=None, addr=None, value=None):
    #     """
    #     Warnning, do not use, just for debugging
    #     :param servo_id:
    #     :param addr:
    #     :param value:
    #     :return:
    #     """
    #     return self._arm.set_servo_addr_16(servo_id=servo_id, addr=addr, value=value)
    #
    # def get_servo_addr_16(self, servo_id=None, addr=None):
    #     """
    #     Warnning, do not use, just for debugging
    #     :param servo_id:
    #     :param addr:
    #     :return:
    #     """
    #     return self._arm.get_servo_addr_16(servo_id=servo_id, addr=addr)
    #
    # def set_servo_addr_32(self, servo_id=None, addr=None, value=None):
    #     """
    #     Warnning, do not use, just for debugging
    #     :param servo_id:
    #     :param addr:
    #     :param value:
    #     :return:
    #     """
    #     return self._arm.set_servo_addr_32(servo_id=servo_id, addr=addr, value=value)
    #
    # def get_servo_addr_32(self, servo_id=None, addr=None):
    #     """
    #     Warnning, do not use, just for debugging
    #     :param servo_id:
    #     :param addr:
    #     :return:
    #     """
    #     return self._arm.get_servo_addr_32(servo_id=servo_id, addr=addr)


