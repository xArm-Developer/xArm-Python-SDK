#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


from ..x3 import XArm
from .. import x3


class XArmAPI(object):
    def __init__(self, port=None, baudrate=921600, timeout=None, filters=None, enable_heartbeat=False,
                 enable_report=False, report_type='normal', do_not_open=False,
                 limit_velo=None, limit_acc=None, limit_angle_velo=None, limit_angle_acc=None):
        """
        The API wrapper of xArm
        :param port: port name or ip-address
        :param baudrate: baudrate, only available in serial way
        :param timeout: timeout, only available in serial way
        :param filters: filters, no use
        :param enable_heartbeat: default is False, only available in socket way
        :param enable_report: default is False
        :param report_type: 'normal' or 'real' or 'rich', only available in socket way
        :param do_not_open: do not open, default is False
        :param limit_velo: limit velo, default is [0, 10000]
        :param limit_acc: limit acc, default is [0, 1000000]
        :param limit_angle_velo: limit angle velo
        :param limit_angle_acc: limit angle acc
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
        return self._arm.connected

    @property
    def version(self):
        return self._arm.version

    @property
    def position(self):
        return self._arm.position

    @property
    def angles(self):
        return self._arm.angles

    @property
    def position_offset(self):
        return self._arm.position_offset

    @property
    def state(self):
        return self._arm.state

    @property
    def mtbrake(self):
        return self._arm.mtbrake

    @property
    def maable(self):
        return self._arm.maable

    @property
    def error_code(self):
        return self._arm.error_code

    @property
    def warn_code(self):
        return self._arm.warn_code

    @property
    def cmd_num(self):
        return self._arm.cmd_num

    @property
    def device_type(self):
        return self._arm.device_type

    @property
    def axis(self):
        return self._arm.axis

    @property
    def master_id(self):
        return self._arm.master_id

    @property
    def slave_id(self):
        return self._arm.slave_id

    def connect(self, port=None, baudrate=None, timeout=None):
        self._arm.connect(port=port, baudrate=baudrate, timeout=timeout)

    def disconnect(self):
        self._arm.disconnect()

    def sync(self):
        self._arm.sync()

    def send_cmd_sync(self, command=None):
        return self._arm.send_cmd_sync(command=command)

    def get_position(self, is_radian=True):
        """
        Get the position
        :param is_radian: return radian or not
        :return: pos list, [x, y, z, roll, yaw, pitch]
        """
        return self._arm.get_position(is_radian=is_radian)

    def set_position(self, x=None, y=None, z=None, roll=None, yaw=None, pitch=None, radius=None,
                     speed=None, mvacc=None, mvtime=None, relative=False, is_radian=True,
                     wait=False, timeout=None, **kwargs):
        """
        Set position
        :param x:
        :param y:
        :param z:
        :param roll: A 
        :param yaw: B
        :param pitch: C 
        :param radius: move radius, if radius is 0, will move line
        :param speed: move speed
        :param mvacc: move acc
        :param mvtime: 0
        :param relative: relative move or not
        :param is_radian: roll/yaw/pitch value is radian or not 
        :param wait: if True will wait the robot stop
        :param timeout: second，default is 10s
        :param kwargs: 
        :return: 
        """
        return self._arm.set_position(x=x, y=y, z=z, pitch=pitch, yaw=yaw, roll=roll, radius=radius,
                                      speed=speed, mvacc=mvacc, mvtime=mvtime, relative=relative,
                                      is_radian=is_radian, wait=wait, timeout=timeout, **kwargs)

    def get_servo_angle(self, servo_id=None, is_radian=True):
        """
        Get the servo angle
        :param servo_id: 1-7, default is None
        :param is_radian: return radian or not
        :return: angle list if servo_id is None or 0 else angle
        """
        return self._arm.get_servo_angle(servo_id=servo_id, is_radian=is_radian)

    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None,
                        relative=False, is_radian=True, wait=False, timeout=None, **kwargs):
        """
        Set the servo angle
        :param servo_id: 1-7, None(0)
        :param angle: angle or angle list
        :param speed: move speed
        :param mvacc: move acc
        :param mvtime: 0
        :param relative: relative move or not
        :param is_radian: angle value is radian or not
        :param wait: if True will wait the robot stop
        :param timeout: second，default is 10s
        :param kwargs: 
        :return: 
        """
        return self._arm.set_servo_angle(servo_id=servo_id, angle=angle, speed=speed, mvacc=mvacc, mvtime=mvtime,
                                         relative=relative, is_radian=is_radian, wait=wait, timeout=timeout, **kwargs)

    def move_gohome(self, speed=None, mvacc=None, mvtime=None, is_radian=True, wait=False, timeout=None):
        """
        Move to go home
        :param speed: 
        :param mvacc: 
        :param mvtime: 
        :param is_radian:
        :param wait: if True will wait the robot stop
        :param timeout: second，default is 10s
        :return: 
        """
        return self._arm.move_gohome(speed=speed, mvacc=mvacc, mvtime=mvtime, is_radian=is_radian, wait=wait, timeout=timeout)

    def set_servo_attach(self, servo_id=None):
        """
        Set servo attach
        :param servo_id: 1-7, None(8), if servo_id is None or 0, will attach all servo
        :return: 
        """
        return self._arm.set_servo_attach(servo_id=servo_id)

    def set_servo_detach(self, servo_id=None):
        """
        Set servo detach
        :param servo_id: 1-7, None(8), if servo_id is None or 0, will detach all servo
        :return: 
        """
        return self._arm.set_servo_detach(servo_id=servo_id)

    def get_version(self):
        return self._arm.get_version()

    def get_is_moving(self):
        return self._arm.get_is_moving()

    def get_state(self):
        """
        Get state
        :return: 
            1: moving
            2: sleeping
            3: suspended
            4: stopping
        """
        return self._arm.get_state()

    def set_state(self, state=0):
        """
        Set state
        :param state: default is 0
            0: sport state
            3: pause state
            4: stop state
        :return: 
        """
        return self._arm.set_state(state=state)

    def get_cmdnum(self):
        """
        Get cmd count in cache
        :return: 
        """
        return self._arm.get_cmdnum()

    def get_err_warn_code(self):
        """
        Get error and warn code
        :return: [error_code, warn_code]
        """
        return self._arm.get_err_warn_code()

    def clean_error(self):
        """
        Clean error
        :return: 
        """
        return self._arm.clean_error()

    def clean_warn(self):
        """
        Clean warn
        :return: 
        """
        return self._arm.clean_warn()

    def motion_enable(self, enable=True, servo_id=None):
        """
        Motion enable
        :param enable: 
        :param servo_id: 1-7, None(8)
        :return: 
        """
        return self._arm.motion_enable(servo_id=servo_id, enable=enable)

    def reset(self, speed=None, is_radian=False):
        return self._arm.reset(speed=speed, is_radian=is_radian)

    def set_sleep_time(self, sltime, wait=False):
        """
        Set sleep time, xArm will sleep sltime second
        :param sltime:
        :param wait:
        :return: 
        """
        return self._arm.set_sleep_time(sltime, wait=wait)

    def set_tcp_offset(self, offset):
        return self._arm.set_tcp_offset(offset)

    def set_tcp_jerk(self, jerk):
        return self._arm.set_tcp_jerk(jerk)

    def set_tcp_maxacc(self, acc):
        return self._arm.set_tcp_maxacc(acc)

    def set_joint_jerk(self, jerk):
        return self._arm.set_joint_jerk(jerk)

    def set_joint_maxacc(self, acc):
        return self._arm.set_joint_maxacc(acc)

    def clean_conf(self):
        return self._arm.clean_conf()

    def save_conf(self):
        return self._arm.save_conf()

    def get_ik(self, pose, is_radian=True):
        return self._arm.get_ik(pose, is_radian=is_radian)

    def get_fk(self, angles, is_radian=True):
        return self._arm.get_fk(angles, is_radian=is_radian)

    def is_tcp_limit(self, pose, is_radian=True):
        """
        Check tcp pose is in lint
        :param pose: [x, y, z, roll, yaw, pitch]
        :param is_radian: roll/yaw/pitch value is radian or not
        :return: True/False/None
        """
        return self._arm.is_tcp_limit(pose, is_radian=is_radian)

    def is_joint_limit(self, joint, is_radian=True):
        """
        Check joint is in limit
        :param joint: angle list
        :param is_radian: angle value is radian or not
        :return: True/False/None
        """
        return self._arm.is_joint_limit(joint, is_radian=is_radian)

    def set_params(self, **kwargs):
        return self._arm.set_params(**kwargs)

    def get_params(self):
        return self._arm.get_params()

    def urgent_stop(self):
        return self._arm.urgent_stop()

    def set_gripper_enable(self, enable):
        return self._arm.gripper_enable(enable)

    def set_gripper_mode(self, mode):
        return self._arm.set_gripper_mode(mode)

    def get_gripper_position(self):
        return self._arm.get_gripper_position()

    def set_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None):
        return self._arm.set_gripper_position(pos, wait=wait, speed=speed, auto_enable=auto_enable, timeout=timeout)

    def set_gripper_speed(self, speed):
        return self._arm.set_gripper_speed(speed)

    def get_gripper_err_code(self):
        return self._arm.get_gripper_err_code()

    def clean_gripper_error(self):
        return self._arm.clean_gripper_error()

    def set_gripper_zero(self):
        return self._arm.set_gripper_zero()

    def register_report_callback(self, callback=None, report_cartesian=True, report_joints=True,
                                 report_state=True, report_error_code=True, report_warn_code=True,
                                 report_maable=True, report_mtbrake=True, report_cmd_num=True):
        """
        Register report callback, only available if enable_report is True
        :param callback: 
            callback data:
            {
                'cartesian': [], # if report_cartesian is True
                'joints': [], # if report_joints is True
                'errorCode': 0, # if report_error_code is True
                'warnCode': 0, # if report_warn_code is True
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
        Register report location callback, only available if enable_report is True
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
        Register connect status changed callback
        :param callback: 
            callback data:
            {
                "mainConnected": mainConnected,
                "reportConnected": reportConnected,
            }
        :return: 
        """
        return self._arm.register_connect_changed_callback(callback=callback)

    def register_state_changed_callback(self, callback=None):
        """
        Register state status changed callback, only available if enable_report is True
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
        Register maable or mtbrake status changed callback, only available if enable_report is True and the connect way is socket
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
        Register error code or warn code changed callback, only available if enable_report is True
        :param callback: 
            callback data:
            {
                "errorCode": errorCode,
                "warnCode": warnCode,
            }
        :return: 
        """
        return self._arm.register_error_warn_changed_callback(callback=callback)

    def register_cmdnum_changed_callback(self, callback=None):
        """
        Register cmdnum changed callback, only available if enable_report is True
        :param callback: 
            callback data:
            {
                "cmdnum": cmdnum
            }
        :return: 
        """
        return self._arm.register_cmdnum_changed_callback(callback=callback)

    def set_servo_zero(self, servo_id=None):
        """
        Set servo zero, warnning
        :param servo_id: 1~7, 8
        :return: 
        """
        return self._arm.set_servo_zero(servo_id=servo_id)

    def get_servo_debug_msg(self):
        return self._arm.get_servo_debug_msg()

    def set_servo_addr_16(self, servo_id=None, addr=None, value=None):
        return self._arm.set_servo_addr_16(servo_id=servo_id, addr=addr, value=value)

    def get_servo_addr_16(self, servo_id=None, addr=None):
        return self._arm.get_servo_addr_16(servo_id=servo_id, addr=addr)

    def set_servo_addr_32(self, servo_id=None, addr=None, value=None):
        return self._arm.set_servo_addr_32(servo_id=servo_id, addr=addr, value=value)

    def get_servo_addr_32(self, servo_id=None, addr=None):
        return self._arm.get_servo_addr_32(servo_id=servo_id, addr=addr)

    def clean_servo_error(self, servo_id=None):
        return self._arm.clean_servo_error(servo_id)

