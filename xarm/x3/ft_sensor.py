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
    def set_impedance(self, coord, c_axis, M, K, B, **kwargs):
        if len(c_axis) < 6 or len(M) < 6 or len(K) < 6 or len(B) < 6:
            logger.error('set_impedance: parameters error')
            return APIState.API_EXCEPTION
        params_limit = kwargs.get('params_limit', True)
        if params_limit:
            for i in range(6):
                if i < 3:
                    if M[i] < 0.02 or M[i] > 1.0:
                        logger.error('set_impedance, M[{}] over range, range=[0.02, 1.0]'.format(i))
                        return APIState.API_EXCEPTION
                    if K[i] < 0 or K[i] > 2000:
                        logger.error('set_impedance, K[{}] over range, range=[0, 2000]'.format(i))
                        return APIState.API_EXCEPTION
                else:
                    if M[i] < 0.0001 or M[i] > 0.01:
                        logger.error('set_impedance, M[{}] over range, range=[0.0001, 0.01]'.format(i))
                        return APIState.API_EXCEPTION
                    if K[i] < 0 or K[i] > 20:
                        logger.error('set_impedance, K[{}] over range, range=[0, 20]'.format(i))
                        return APIState.API_EXCEPTION
                if B[i] < 0:
                    logger.error('set_impedance, the value of B[{}] must be greater than or equal to 0'.format(i))
                    return APIState.API_EXCEPTION
        ret = self.arm_cmd.set_impedance(coord, c_axis, M, K, B)
        self.log_api_info('API -> set_impedance -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='set')
    def set_impedance_mbk(self, M, K, B, **kwargs):
        if len(M) < 6 or len(K) < 6 or len(B) < 6:
            logger.error('set_impedance_mbk: parameters error')
            return APIState.API_EXCEPTION
        params_limit = kwargs.get('params_limit', True)
        if params_limit:
            for i in range(6):
                if i < 3:
                    if M[i] < 0.02 or M[i] > 1.0:
                        logger.error('set_impedance_mbk, M[{}] over range, range=[0.02, 1.0]'.format(i))
                        return APIState.API_EXCEPTION
                    if K[i] < 0 or K[i] > 2000:
                        logger.error('set_impedance_mbk, K[{}] over range, range=[0, 2000]'.format(i))
                        return APIState.API_EXCEPTION
                else:
                    if M[i] < 0.0001 or M[i] > 0.01:
                        logger.error('set_impedance_mbk, M[{}] over range, range=[0.0001, 0.01]'.format(i))
                        return APIState.API_EXCEPTION
                    if K[i] < 0 or K[i] > 20:
                        logger.error('set_impedance_mbk, K[{}] over range, range=[0, 20]'.format(i))
                        return APIState.API_EXCEPTION
                if B[i] < 0:
                    logger.error('set_impedance_mbk, the value of B[{}] must be greater than or equal to 0'.format(i))
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
    def config_force_control(self, coord, c_axis, f_ref, limits, **kwargs):
        if len(c_axis) < 6 or len(f_ref) < 6 or len(limits) < 6:
            logger.error('config_force_control: parameters error')
            return APIState.API_EXCEPTION
        params_limit = kwargs.get('params_limit', True)
        if params_limit:
            max_f_ref = [150, 150, 200, 4, 4, 4]
            for i in range(6):
                if f_ref[i] < -max_f_ref[i] or f_ref[i] > max_f_ref[i]:
                    logger.error('config_force_control, f_ref[{}] over range, range=[{}, {}]'.format(i, -max_f_ref[i], max_f_ref[i]))
                    return APIState.API_EXCEPTION
        ret = self.arm_cmd.config_force_control(coord, c_axis, f_ref, limits)
        self.log_api_info('API -> config_force_control -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='set')
    def set_force_control_pid(self, kp, ki, kd, xe_limit, **kwargs):
        if len(kp) < 6 or len(ki) < 6 or len(kd) < 6 or len(xe_limit) < 6:
            logger.error('set_force_control_pid: parameters error')
            return APIState.API_EXCEPTION
        params_limit = kwargs.get('params_limit', True)
        if params_limit:
            for i in range(6):
                if kp[i] < 0 or kp[i] > 0.05:
                    logger.error('set_force_control_pid, kp[{}] over range, range=[0, 0.05]'.format(i))
                    return APIState.API_EXCEPTION
                if ki[i] < 0 or ki[i] > 0.0005:
                    logger.error('set_force_control_pid, ki[{}] over range, range=[0, 0.0005]'.format(i))
                    return APIState.API_EXCEPTION
                if kd[i] < 0 or kd[i] > 0.05:
                    logger.error('set_force_control_pid, kd[{}] over range, range=[0, 0.05]'.format(i))
                    return APIState.API_EXCEPTION
                if xe_limit[i] < 0 or xe_limit[i] > 200:
                    logger.error('set_force_control_pid, xe_limit[{}] over range, range=[0, 200]'.format(i))
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
    def ft_sensor_cali_load(self, iden_result_list, association_setting_tcp_load=False, **kwargs):
        ret = self.arm_cmd.ft_sensor_cali_load(iden_result_list)
        self.log_api_info('API -> ft_sensor_cali_load -> code={}, iden_result_list={}'.format(ret[0], iden_result_list), code=ret[0])
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0 and association_setting_tcp_load:
            m = kwargs.get('m', 0.325)
            x = kwargs.get('x', -17)
            y = kwargs.get('y', 9)
            z = kwargs.get('z', 11.8)
            weight = iden_result_list[0] + m
            center_of_gravity = [
                (m * x + iden_result_list[0] * iden_result_list[1]) / weight,
                (m * y + iden_result_list[0] * iden_result_list[2]) / weight,
                (m * z + iden_result_list[0] * (32 + iden_result_list[3])) / weight
            ]
            return self.set_tcp_load(weight, center_of_gravity)
        return ret[0]

    @xarm_is_connected(_type='set')
    def ft_sensor_enable(self, on_off):
        ret = self.arm_cmd.ft_sensor_enable(on_off)
        self.log_api_info('API -> ft_sensor_enable -> code={}, on_off={}'.format(ret[0], on_off), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='get')
    def ft_sensor_app_set(self, app_code):
        ret = self.arm_cmd.ft_sensor_app_set(app_code)
        self.log_api_info('API -> ft_sensor_app_set -> code={}, app_code={}'.format(ret[0], app_code), code=ret[0])
        return self._check_code(ret[0])

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

    @xarm_is_connected(_type='get')
    def get_ft_sensor_error(self):
        ret = self.arm_cmd.ft_sensor_get_error()
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1]
