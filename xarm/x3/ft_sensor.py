#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from .utils import xarm_is_connected
from ..core.utils.log import logger
from ..core.utils import convert
from .base import Base
from .code import APIState


class FtSensor(Base):
    def __init__(self):
        super(FtSensor, self).__init__()

    @xarm_is_connected(_type='set')
    def set_impedance(self, coord, c_axis, M, K, B):
        if len(c_axis) < 6 or len(M) < 6 or len(K) < 6 or len(B) < 6:
            logger.error('set_impedance: parameters error')
            return APIState.API_EXCEPTION
        ret = self.arm_cmd.set_impedance(coord, c_axis, M, K, B)
        self.log_api_info('API -> set_impedance -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='set')
    def set_impedance_mbk(self, M, K, B):
        if len(M) < 6 or len(K) < 6 or len(B) < 6:
            logger.error('set_impedance_mbk: parameters error')
            return APIState.API_EXCEPTION
        ret = self.arm_cmd.set_impedance_mbk(M, K, B)
        self.log_api_info('API -> set_impedance_mbk -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='set')
    def set_impedance_config(self, coord, c_axis):
        if len(c_axis) < 6:
            logger.error('set_impedance_config: parameters error')
            return APIState.API_EXCEPTION
        ret = self.arm_cmd.set_impedance_config(coord, c_axis)
        self.log_api_info('API -> set_impedance_config -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='set')
    def config_force_control(self, coord, c_axis, f_ref, limits):
        if len(c_axis) < 6 or len(f_ref) < 6 or len(limits) < 6:
            logger.error('config_force_control: parameters error')
            return APIState.API_EXCEPTION
        ret = self.arm_cmd.config_force_control(coord, c_axis, f_ref, limits)
        self.log_api_info('API -> config_force_control -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='set')
    def set_force_control_pid(self, kp, ki, kd, xe_limit):
        if len(kp) < 6 or len(ki) < 6 or len(kd) < 6 or len(xe_limit) < 6:
            logger.error('set_force_control_pid: parameters error')
            return APIState.API_EXCEPTION
        ret = self.arm_cmd.set_force_control_pid(kp, ki, kd, xe_limit)
        self.log_api_info('API -> set_force_control_pid -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='set')
    def ft_sensor_set_zero(self):
        ret = self.arm_cmd.ft_sensor_set_zero()
        self.log_api_info('API -> ft_sensor_set_zero -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='get')
    def ft_sensor_iden_load(self):
        ret = self.arm_cmd.ft_sensor_iden_load()
        self.log_api_info('API -> ft_sensor_iden_load -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0]), ret[1:11]

    @xarm_is_connected(_type='set')
    def ft_sensor_cali_load(self, iden_result_list):
        ret = self.arm_cmd.ft_sensor_cali_load(iden_result_list)
        self.log_api_info('API -> ft_sensor_cali_load -> code={}, iden_result_list={}'.format(ret[0], iden_result_list), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='set')
    def ft_sensor_enable(self, on_off):
        ret = self.arm_cmd.ft_sensor_enable(on_off)
        self.log_api_info('API -> ft_sensor_enable -> code={}, on_off={}'.format(ret[0], on_off), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='get')
    def ft_sensor_app_set(self, app_code):
        ret = self.arm_cmd.ft_sensor_app_set(app_code)
        self.log_api_info('API -> ft_sensor_app_set -> code={}, app_code={}'.format(ret[0], app_code), code=ret[0])
        return self._check_code(ret[0]), ret[1]

    @xarm_is_connected(_type='get')
    def ft_sensor_app_get(self):
        ret = self.arm_cmd.ft_sensor_app_get()
        return self._check_code(ret[0]), ret[1]

    @xarm_is_connected(_type='get')
    def get_ft_sensor_data(self):
        ret = self.arm_cmd.ft_sensor_get_data(self.version_is_ge(1, 8, 3))
        return self._check_code(ret[0]), ret[1:7]

    @xarm_is_connected(_type='get')
    def get_ft_senfor_config(self):
        ret = self.arm_cmd.ft_senfor_get_config()
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1:]
