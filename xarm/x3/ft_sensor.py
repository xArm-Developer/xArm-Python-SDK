#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
import time
from ..core.utils.log import logger
from ..core.utils import convert
from .base import Base
from .code import APIState
from .decorator import xarm_is_connected


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
        protocol_identifier = self.arm_cmd.get_protocol_identifier()
        self.arm_cmd.set_protocol_identifier(2)
        self._keep_heart = False
        ret = self.arm_cmd.ft_sensor_iden_load()
        self.arm_cmd.set_protocol_identifier(protocol_identifier)
        self._keep_heart = True
        self.log_api_info('API -> ft_sensor_iden_load -> code={}'.format(ret[0]), code=ret[0])
        code = self._check_code(ret[0])
        if code == 0 or len(ret) > 5:
            ret[2] = ret[2] * 1000  # x_centroid, 从m转成mm
            ret[3] = ret[3] * 1000  # y_centroid, 从m转成mm
            ret[4] = ret[4] * 1000  # z_centroid, 从m转成mm
        return self._check_code(ret[0]), ret[1:11]

    @xarm_is_connected(_type='set')
    def ft_sensor_cali_load(self, iden_result_list, association_setting_tcp_load=False, **kwargs):
        if len(iden_result_list) < 10:
            return APIState.PARAM_ERROR
        params = iden_result_list[:]
        params[1] = params[1] / 1000.0  # x_centroid, 从mm转成m
        params[2] = params[2] / 1000.0  # y_centroid, 从mm转成m
        params[3] = params[3] / 1000.0  # z_centroid, 从mm转成m
        ret = self.arm_cmd.ft_sensor_cali_load(params)
        self.log_api_info('API -> ft_sensor_cali_load -> code={}, iden_result_list={}'.format(ret[0], iden_result_list), code=ret[0])
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0 and association_setting_tcp_load:
            m = kwargs.get('m', 0.325)
            x = kwargs.get('x', -17)
            y = kwargs.get('y', 9)
            z = kwargs.get('z', 11.8)
            weight = params[0] + m
            center_of_gravity = [
                (m * x + params[0] * params[1]) / weight,
                (m * y + params[0] * params[2]) / weight,
                (m * z + params[0] * (32 + params[3])) / weight
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
    def get_ft_sensor_config(self):
        ret = self.arm_cmd.ft_sensor_get_config()
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1:]

    @xarm_is_connected(_type='get')
    def get_ft_sensor_error(self):
        ret = self.arm_cmd.ft_sensor_get_error()
        ret[0] = self._check_code(ret[0])
        return ret[0], ret[1]

    def set_ft_sensor_sn(self, sn):
        assert len(sn) >= 14, 'The length of SN is wrong'
        ret = [0]
        if len(sn) == 14:
            for i in range(0, 14):
                value = ord(sn[i])
                if i < 8:
                    ret = self.arm_cmd.servo_addr_w16(8, 0x1300+i, value)
                    ret[0] = self._check_code(ret[0])
                else:
                    ret = self.arm_cmd.servo_addr_w16(8, 0x1400+(i-8), value)
                    ret[0] = self._check_code(ret[0])
                if ret[0] != 0:
                    break
                time.sleep(0.05)
        self.log_api_info('API -> set_ft_sensor_sn -> code={}, sn={}'.format(ret[0], sn), code=ret[0])
        return ret[0]

    def get_ft_sensor_sn(self):
        rd_sn = ''
        ret = [0, '']
        for i in range(0, 14):
            if i < 8:
                ret = self.arm_cmd.servo_addr_r16(8, 0x0300+i)
                ret[0] = self._check_code(ret[0])
            else:
                ret = self.arm_cmd.servo_addr_r16(8, 0x0400+(i-8))
                ret[0] = self._check_code(ret[0])
            if i < 2 and ret[-1] not in [65, 73]:
                return 1, "********"

            if chr(ret[-1]).isalnum():
                rd_sn = ''.join([rd_sn, chr(ret[-1])])
            else:
                rd_sn = ''.join([rd_sn, '*'])
            time.sleep(0.05)
        self.log_api_info('API -> get_ft_sensor_sn -> code={}, sn={}'.format(ret[0], rd_sn), code=ret[0])
        return ret[0], rd_sn

    def get_ft_sensor_version(self):
        versions = ['*', '*', '*']
        ret1 = self.arm_cmd.servo_addr_r16(8, 0x0801)
        ret1[0] = self._check_code(ret1[0])
        ret2 = self.arm_cmd.servo_addr_r16(8, 0x0802)
        ret2[0] = self._check_code(ret2[0])
        ret3 = self.arm_cmd.servo_addr_r16(8, 0x0803)
        ret3[0] = self._check_code(ret3[0])

        if ret1[0] == 0 and ret1[1] < 10:
            versions[0] = ret1[1]
        if ret2[0] == 0 and ret1[1] < 100:
            versions[1] = ret2[1]
        if ret3[0] == 0 and ret1[1] < 1000:
            versions[2] = ret3[1]

        return ret1[0] or ret2[0] or ret3[0], '.'.join(map(str, versions))
