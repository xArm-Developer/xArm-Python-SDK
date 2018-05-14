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
                 limit_velo=None, limit_acc=None):
        self._arm = XArm(port=port, baudrate=baudrate, timeout=timeout, filters=filters,
                         enable_heartbeat=enable_heartbeat,
                         enable_report=enable_report, report_type=report_type,
                         do_not_open=do_not_open,
                         limit_velo=limit_velo, limit_acc=limit_acc)

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
    def error_code(self):
        return self._arm.error_code

    @property
    def warn_code(self):
        return self._arm.warn_code

    @property
    def cmd_num(self):
        return self._arm.cmd_num

    @property
    def arm_type(self):
        return self._arm.arm_type

    @property
    def arm_axis(self):
        return self._arm.arm_axis

    def connect(self, port=None, baudrate=None, timeout=None):
        self._arm.connect(port=port, baudrate=baudrate, timeout=timeout)

    def disconnect(self):
        self._arm.disconnect()

    def send_cmd_sync(self, command):
        return self._arm.send_cmd_sync(command)

    def get_position(self, is_radian=True):
        return self._arm.get_position(is_radian=is_radian)

    def set_position(self, x=None, y=None, z=None, pitch=None, yaw=None, roll=None, radius=None, speed=None, mvacc=None,
                     mvtime=None, relative=False, is_radian=True):
        return self._arm.set_position(x=x, y=y, z=z, pitch=pitch, yaw=yaw, roll=roll, radius=radius,
                                      speed=speed, mvacc=mvacc, mvtime=mvtime, relative=relative, is_radian=is_radian)

    def get_servo_angle(self, servo_id=None, is_radian=True):
        """
        Get the servo angle
        :param servo_id: 1-7, None(0)
        :param is_radian: return radian or not
        :return: 
        """
        return self._arm.get_servo_angle(servo_id=servo_id, is_radian=is_radian)

    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None, relative=False, is_radian=True):
        """
        Set the servo angle
        :param servo_id: 1-7, None(0)
        :param angle: angle or angle list
        :param speed: 
        :param mvacc: 
        :param mvtime: 
        :param relative: relative or not
        :param is_radian: angle is radian or not
        :return: 
        """
        return self._arm.set_servo_angle(servo_id=servo_id, angle=angle, speed=speed, mvacc=mvacc, mvtime=mvtime,
                                         relative=relative, is_radian=is_radian)

    def move_gohome(self, speed=None, mvacc=None, mvtime=None):
        return self._arm.move_gohome(speed=speed, mvacc=mvacc, mvtime=mvtime)

    def set_servo_attach(self, servo_id=None):
        """
        Set servo attach
        :param servo_id: 1-7, None(0)
        :return: 
        """
        return self._arm.set_servo_attach(servo_id=servo_id)

    def set_servo_detach(self, servo_id=None):
        """
        Set servo detach
        :param servo_id: 1-7, None(0)
        :return: 
        """
        return self._arm.set_servo_detach(servo_id=servo_id)

    def get_version(self):
        return self._arm.get_version()

    def get_state(self):
        return self._arm.get_state()

    def set_state(self, state=0):
        return self._arm.set_state(state=state)

    def get_cmdnum(self):
        return self._arm.get_cmdnum()

    def get_err_warn_code(self):
        return self._arm.get_err_warn_code()

    def clean_error(self):
        return self._arm.clean_error()

    def clean_warn(self):
        return self._arm.clean_warn()

    def motion_enable(self, servo_id=None, enable=True):
        """
        Motion enable
        :param servo_id: 1-7, None(0)
        :param enable: 
        :return: 
        """
        return self._arm.motion_enable(servo_id=servo_id, enable=enable)

    def reset(self):
        return self._arm.reset()

    def set_sleep_time(self, sltime):
        return self._arm.set_sleep_time(sltime)

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
        return self._arm.is_tcp_limit(pose, is_radian=is_radian)

    def is_joint_limit(self, joint, is_radian=True):
        return self._arm.is_joint_limit(joint, is_radian=is_radian)

    def set_params(self, **kwargs):
        return self._arm.set_params(**kwargs)

    def get_params(self):
        return self._arm.get_params()

    def urgent_stop(self):
        return self._arm.urgent_stop()

    def gripper_enable(self, enable):
        return self._arm.gripper_enable(enable)

    def get_gripper_position(self):
        return self._arm.get_gripper_position()

    def set_gripper_position(self, pos):
        return self._arm.set_gripper_position(pos)

    def set_gripper_speed(self, speed):
        return self._arm.set_gripper_speed(speed)

    def get_gripper_err_code(self):
        return self._arm.get_gripper_err_code()

    def clean_gripper_error(self):
        return self._arm.clean_gripper_error()

