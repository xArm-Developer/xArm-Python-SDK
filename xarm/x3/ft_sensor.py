#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
import time
from collections.abc import Iterable
from ..core.utils.log import logger
from ..core.utils import convert
from .base import Base
from .code import APIState
from .decorator import xarm_is_connected


class FtSensor(Base):
    def __init__(self):
        super(FtSensor, self).__init__()

    def set_ft_sensor_admittance_parameters(self, coord=None, c_axis=None, M=None, K=None, B=None, **kwargs):
        if isinstance(coord, int):
            # 当第一个参数为整型时, 参数顺序为 coord/c_axis/M/K/B
            #  coord: 必须, 整型
            #  c_axis: 必须, 大小为6的数组
            #  M/B/K: 可选, 要么都为None(不设置), 要么都为大小为6的数组
            if not isinstance(c_axis, Iterable) or len(c_axis) < 6:
                logger.error('set_ft_sensor_admittance_parameters: the second parameter can only be an array of size 6 when the first parameter is an integer.')
                return APIState.API_EXCEPTION
            if M is not None or B is not None or K is not None:
                if not isinstance(M, Iterable) or len(M) < 6 \
                    or not isinstance(B, Iterable) or len(B) < 6 \
                    or not isinstance(K, Iterable) or len(K) < 6:
                    logger.error('set_ft_sensor_admittance_parameters: the 3rd/4th/5th parameters are either None or arrays of size 6 when the first parameter is an integer.')
                    return APIState.API_EXCEPTION
                if kwargs.get('params_limit', True):
                    for i in range(6):
                        M_RANGGE = [0.02, 1.0] if i < 3 else [0.0001, 0.01]
                        K_RANGGE = [0, 2000] if i < 3 else [0, 20]
                        if M[i] < M_RANGGE[0] or M[i] > M_RANGGE[1]:
                            logger.error('set_ft_sensor_admittance_parameters, (the third parameter) M[{}] over range, range={}'.format(i, M_RANGGE))
                            return APIState.API_EXCEPTION
                        if K[i] < K_RANGGE[0] or K[i] > K_RANGGE[1]:
                            logger.error('set_ft_sensor_admittance_parameters, (the 4th parameter) K[{}] over range, range={}'.format(i, K_RANGGE))
                            return APIState.API_EXCEPTION
                        if B[i] < 0:
                            logger.error('set_ft_sensor_admittance_parameters, (the 5th parameter) B[{}] must be greater than or equal to 0'.format(i))
                            return APIState.API_EXCEPTION
                ret = self.arm_cmd.set_admittance(coord, c_axis, M, K, B)
                self.log_api_info('API -> set_ft_sensor_admittance_parameters(coord, c_axis, M, K, B) -> code={}'.format(ret[0]), code=ret[0])
                return self._check_code(ret[0])
            else:
                ret = self.arm_cmd.set_admittance_config(coord, c_axis)
                self.log_api_info('API -> set_ft_sensor_admittance_parameters(coord, c_axis) -> code={}'.format(ret[0]), code=ret[0])
                return self._check_code(ret[0])
        elif isinstance(coord, Iterable):
            # 当第一个参数为大小为6的数组时, 参数顺序为 M/K/B/coord/c_axis
            #  M/B/K: 必须, 都为大小为6的数组
            #  coord: 可选, 要么None, 要么整型, 为整型时c_axis必须为大小为6的数组
            #  c_axis: 可选, 要么None, 要么大小为6的数组, 为数组时coord必须为整型
            coord, c_axis, M, K, B = K, B, coord, c_axis, M  # 交换参数顺序
            if not isinstance(M, Iterable) or len(M) < 6 \
                or not isinstance(B, Iterable) or len(B) < 6 \
                or not isinstance(K, Iterable) or len(K) < 6:
                logger.error('set_ft_sensor_admittance_parameters: the first/second/third parameters can only be arrays of size 6 when the first parameter is an array.')
                return APIState.API_EXCEPTION
            if kwargs.get('params_limit', True):
                for i in range(6):
                    M_RANGGE = [0.02, 1.0] if i < 3 else [0.0001, 0.01]
                    K_RANGGE = [0, 2000] if i < 3 else [0, 20]
                    if M[i] < M_RANGGE[0] or M[i] > M_RANGGE[1]:
                        logger.error('set_ft_sensor_admittance_parameters, (the first parameter) M[{}] over range, range={}'.format(i, M_RANGGE))
                        return APIState.API_EXCEPTION
                    if K[i] < K_RANGGE[0] or K[i] > K_RANGGE[1]:
                        logger.error('set_ft_sensor_admittance_parameters, (the second parameter) K[{}] over range, range={}'.format(i, K_RANGGE))
                        return APIState.API_EXCEPTION
                    if B[i] < 0:
                        logger.error('set_ft_sensor_admittance_parameters, (the third parameter) B[{}] must be greater than or equal to 0'.format(i))
                        return APIState.API_EXCEPTION
            if coord is not None or c_axis is not None:
                if not isinstance(coord, int) or not isinstance(c_axis, Iterable) or len(c_axis) < 6:
                    logger.error('set_ft_sensor_admittance_parameters: the 4th and 5th parameters are either None or an integer and an array of size 6 respectively when the first parameter is an array.')
                    return APIState.API_EXCEPTION
                ret = self.arm_cmd.set_admittance(coord, c_axis, M, K, B)
                self.log_api_info('API -> set_ft_sensor_admittance_parameters(coord, c_axis, M, K, B) -> code={}'.format(ret[0]), code=ret[0])
                return self._check_code(ret[0])
            else:
                ret = self.arm_cmd.set_admittance_mbk(M, K, B)
                self.log_api_info('API -> set_ft_sensor_admittance_parameters(M, K, B) -> code={}'.format(ret[0]), code=ret[0])
                return self._check_code(ret[0])
        elif coord is None and c_axis is None:
            if not isinstance(M, Iterable) or len(M) < 6 \
                or not isinstance(B, Iterable) or len(B) < 6 \
                or not isinstance(K, Iterable) or len(K) < 6:
                logger.error('set_ft_sensor_admittance_parameters: the 3rd/4th/5th parameters can only be an array of size 6 when the first/second parameters is None.')
                return APIState.API_EXCEPTION
            if kwargs.get('params_limit', True):
                for i in range(6):
                    M_RANGGE = [0.02, 1.0] if i < 3 else [0.0001, 0.01]
                    K_RANGGE = [0, 2000] if i < 3 else [0, 20]
                    if M[i] < M_RANGGE[0] or M[i] > M_RANGGE[1]:
                        logger.error('set_ft_sensor_admittance_parameters, (the third parameter) M[{}] over range, range={}'.format(i, M_RANGGE))
                        return APIState.API_EXCEPTION
                    if K[i] < K_RANGGE[0] or K[i] > K_RANGGE[1]:
                        logger.error('set_ft_sensor_admittance_parameters, (the 4th parameter) K[{}] over range, range={}'.format(i, K_RANGGE))
                        return APIState.API_EXCEPTION
                    if B[i] < 0:
                        logger.error('set_ft_sensor_admittance_parameters, (the 5th parameter) B[{}] must be greater than or equal to 0'.format(i))
                        return APIState.API_EXCEPTION
            ret = self.arm_cmd.set_admittance_mbk(M, K, B)
            self.log_api_info('API -> set_ft_sensor_admittance_parameters(M, K, B) -> code={}'.format(ret[0]), code=ret[0])
            return self._check_code(ret[0])
        else:
            logger.error('set_ft_sensor_admittance_parameters: parameters error')
            return APIState.API_EXCEPTION

    def set_ft_sensor_force_parameters(self, coord=None, c_axis=None, f_ref=None, limits=None, kp=None, ki=None, kd=None, xe_limit=None, **kwargs):
        if isinstance(coord, int):
            # 当第一个参数为整型时, 参数顺序为 coord/c_axis/f_ref/limits/kp/ki/kd/xe_limit
            #  coord: 必须, 整型
            #  c_axis/f_ref/limits: 必须, 大小为6的数组
            #  kp/ki/kd/xe_limit: 可选, 要么都为None(不设置), 要么都为大小为6的数组
            if not isinstance(c_axis, Iterable) or len(c_axis) < 6 \
                or not isinstance(f_ref, Iterable) or len(f_ref) < 6 \
                or not isinstance(limits, Iterable) or len(limits) < 6:
                logger.error('set_ft_sensor_force_parameters: the second/third/4th parameters can only be an array of size 6 when the first parameter is an integer.')
                return APIState.API_EXCEPTION
            if kwargs.get('params_limit', True):
                max_f_ref = [150, 150, 200, 4, 4, 4]
                for i in range(6):
                    if f_ref[i] < -max_f_ref[i] or f_ref[i] > max_f_ref[i]:
                        logger.error('set_ft_sensor_force_parameters, (the third parameter)  f_ref[{}] over range, range=[{}, {}]'.format(i, -max_f_ref[i], max_f_ref[i]))
                        return APIState.API_EXCEPTION
            code = 0
            if kp is not None or ki is not None or kd is not None or xe_limit is not None:
                if not isinstance(kp, Iterable) or len(kp) < 6 \
                    or not isinstance(ki, Iterable) or len(ki) < 6 \
                    or not isinstance(kd, Iterable) or len(kd) < 6 \
                    or not isinstance(xe_limit, Iterable) or len(xe_limit) < 6:
                    logger.error('set_ft_sensor_force_parameters: the 5th/6th/7th/8th parameters are either None or arrays of size 6 when the first parameter is an integer.')
                    return APIState.API_EXCEPTION
                if kwargs.get('params_limit', True):
                    kp_RANGGE = [0, 0.05]
                    ki_RANGGE = [0, 0.0005]
                    kd_RANGGE = [0, 0.05]
                    xe_limit_RANGGE = [0, 200]
                    for i in range(6):
                        if kp[i] < kp_RANGGE[0] or kp[i] > kp_RANGGE[1]:
                            logger.error('set_ft_sensor_force_parameters, (the 5th parameter) kp[{}] over range, range={}'.format(i, kp_RANGGE))
                            return APIState.API_EXCEPTION
                        if ki[i] < ki_RANGGE[0] or ki[i] > ki_RANGGE[1]:
                            logger.error('set_ft_sensor_force_parameters, (the 6th parameter) ki[{}] over range, range={}'.format(i, ki_RANGGE))
                            return APIState.API_EXCEPTION
                        if kd[i] < kd_RANGGE[0] or kd[i] > kd_RANGGE[1]:
                            logger.error('set_ft_sensor_force_parameters, (the 7th parameter) kd[{}] over range, range={}'.format(i, kd_RANGGE))
                            return APIState.API_EXCEPTION
                        if xe_limit[i] < xe_limit_RANGGE[0] or xe_limit[i] > xe_limit_RANGGE[1]:
                            logger.error('set_ft_sensor_force_parameters, (the 8th parameter) xe_limit[{}] over range, range={}'.format(i, xe_limit_RANGGE))
                            return APIState.API_EXCEPTION
                ret = self.arm_cmd.set_force_control_pid(kp, ki, kd, xe_limit)
                self.log_api_info('API -> set_ft_sensor_force_parameters(kp, ki, kd, xe_limit) -> code={}'.format(ret[0]), code=ret[0])
                code = self._check_code(ret[0])
            ret = self.arm_cmd.config_force_control(coord, c_axis, f_ref, limits)
            self.log_api_info('API -> set_ft_sensor_force_parameters(coord, c_axis, f_ref, limits) -> code={}'.format(ret[0]), code=ret[0])
            code1 = self._check_code(ret[0])
            return code1 if code1 else code
        elif isinstance(coord, Iterable):
            # 当第一个参数为大小为6的数组时, 参数顺序为 kp/ki/kd/xe_limit/coord/c_axis/f_ref/limits
            #  kp/ki/kd/xe_limit: 必须, 都为大小为6的数组
            #  coord: 可选, 要么None, 要么整型, 为整型时c_axis/f_ref/limits必须为大小为6的数组
            #  c_axis/f_ref/limits: 可选, 要么都为None, 要么都为大小为6的数组, 为数组时coord必须为整型
            coord, c_axis, f_ref, limits, kp, ki, kd, xe_limit = kp, ki, kd, xe_limit, coord, c_axis, f_ref, limits  # 交换参数顺序
            if not isinstance(kp, Iterable) or len(kp) < 6 \
                or not isinstance(ki, Iterable) or len(ki) < 6 \
                or not isinstance(kd, Iterable) or len(kd) < 6 \
                or not isinstance(xe_limit, Iterable) or len(xe_limit) < 6:
                logger.error('set_ft_sensor_force_parameters: the first/second/third/4th parameters are either None or arrays of size 6 when the first parameter is an array.')
                return APIState.API_EXCEPTION
            if kwargs.get('params_limit', True):
                kp_RANGGE = [0, 0.05]
                ki_RANGGE = [0, 0.0005]
                kd_RANGGE = [0, 0.05]
                xe_limit_RANGGE = [0, 200]
                for i in range(6):
                    if kp[i] < kp_RANGGE[0] or kp[i] > kp_RANGGE[1]:
                        logger.error('set_ft_sensor_force_parameters, (the first parameter) kp[{}] over range, range={}'.format(i, kp_RANGGE))
                        return APIState.API_EXCEPTION
                    if ki[i] < ki_RANGGE[0] or ki[i] > ki_RANGGE[1]:
                        logger.error('set_ft_sensor_force_parameters, (the second parameter) ki[{}] over range, range={}'.format(i, ki_RANGGE))
                        return APIState.API_EXCEPTION
                    if kd[i] < kd_RANGGE[0] or kd[i] > kd_RANGGE[1]:
                        logger.error('set_ft_sensor_force_parameters, (the third parameter) kd[{}] over range, range={}'.format(i, kd_RANGGE))
                        return APIState.API_EXCEPTION
                    if xe_limit[i] < xe_limit_RANGGE[0] or xe_limit[i] > xe_limit_RANGGE[1]:
                        logger.error('set_ft_sensor_force_parameters, (the 4th parameter) xe_limit[{}] over range, range={}'.format(i, xe_limit_RANGGE))
                        return APIState.API_EXCEPTION
            code = 0
            if coord is not None or c_axis is not None or f_ref is not None or limits is not None:
                if not isinstance(coord, int) or not isinstance(c_axis, Iterable) or len(c_axis) < 6 or not isinstance(f_ref, Iterable) or len(f_ref) < 6 or not isinstance(limits, Iterable) or len(limits) < 6:
                    logger.error('set_ft_sensor_admittance_parameters: the 5th and 6th/7th/8th parameters are either None or an integer and an array of size 6 respectively when the first parameter is an array.')
                    return APIState.API_EXCEPTION
                if kwargs.get('params_limit', True):
                    max_f_ref = [150, 150, 200, 4, 4, 4]
                    for i in range(6):
                        if f_ref[i] < -max_f_ref[i] or f_ref[i] > max_f_ref[i]:
                            logger.error('set_ft_sensor_force_parameters, (the 7th parameter)  f_ref[{}] over range, range=[{}, {}]'.format(i, -max_f_ref[i], max_f_ref[i]))
                            return APIState.API_EXCEPTION
                ret = self.arm_cmd.config_force_control(coord, c_axis, f_ref, limits)
                self.log_api_info('API -> set_ft_sensor_force_parameters(coord, c_axis, f_ref, limits) -> code={}'.format(ret[0]), code=ret[0])
                code = self._check_code(ret[0])
            ret = self.arm_cmd.set_force_control_pid(kp, ki, kd, xe_limit)
            self.log_api_info('API -> set_ft_sensor_force_parameters(kp, ki, kd, xe_limit) -> code={}'.format(ret[0]), code=ret[0])
            code1 = self._check_code(ret[0])
            return code1 if code1 else code
        elif coord is None and c_axis is None and f_ref is None and limits is None:
            if not isinstance(kp, Iterable) or len(kp) < 6 \
                or not isinstance(ki, Iterable) or len(ki) < 6 \
                or not isinstance(kd, Iterable) or len(kd) < 6 \
                or not isinstance(xe_limit, Iterable) or len(xe_limit) < 6:
                logger.error('set_ft_sensor_force_parameters: the 5th/6th/7th/8th parameters can only be an array of size 6 when the first/second/third/4th parameters is None.')
                return APIState.API_EXCEPTION
            if kwargs.get('params_limit', True):
                kp_RANGGE = [0, 0.05]
                ki_RANGGE = [0, 0.0005]
                kd_RANGGE = [0, 0.05]
                xe_limit_RANGGE = [0, 200]
                for i in range(6):
                    if kp[i] < kp_RANGGE[0] or kp[i] > kp_RANGGE[1]:
                        logger.error('set_ft_sensor_force_parameters, (the 5th parameter) kp[{}] over range, range={}'.format(i, kp_RANGGE))
                        return APIState.API_EXCEPTION
                    if ki[i] < ki_RANGGE[0] or ki[i] > ki_RANGGE[1]:
                        logger.error('set_ft_sensor_force_parameters, (the 6th parameter) ki[{}] over range, range={}'.format(i, ki_RANGGE))
                        return APIState.API_EXCEPTION
                    if kd[i] < kd_RANGGE[0] or kd[i] > kd_RANGGE[1]:
                        logger.error('set_ft_sensor_force_parameters, (the 7th parameter) kd[{}] over range, range={}'.format(i, kd_RANGGE))
                        return APIState.API_EXCEPTION
                    if xe_limit[i] < xe_limit_RANGGE[0] or xe_limit[i] > xe_limit_RANGGE[1]:
                        logger.error('set_ft_sensor_force_parameters, (the 8th parameter) xe_limit[{}] over range, range={}'.format(i, xe_limit_RANGGE))
                        return APIState.API_EXCEPTION
            ret = self.arm_cmd.set_force_control_pid(kp, ki, kd, xe_limit)
            self.log_api_info('API -> set_ft_sensor_force_parameters(kp, ki, kd, xe_limit) -> code={}'.format(ret[0]), code=ret[0])
            return self._check_code(ret[0])
        else:
            logger.error('set_ft_sensor_force_parameters: parameters error')
            return APIState.API_EXCEPTION

    # @xarm_is_connected(_type='set')
    # def set_impedance(self, coord, c_axis, M, K, B, **kwargs):
    #     return self.set_ft_sensor_admittance_parameters(coord, c_axis, M, K, B, **kwargs)
    #     # if len(c_axis) < 6 or len(M) < 6 or len(K) < 6 or len(B) < 6:
    #     #     logger.error('set_admittance: parameters error')
    #     #     return APIState.API_EXCEPTION
    #     # params_limit = kwargs.get('params_limit', True)
    #     # if params_limit:
    #     #     for i in range(6):
    #     #         if i < 3:
    #     #             if M[i] < 0.02 or M[i] > 1.0:
    #     #                 logger.error('set_admittance, M[{}] over range, range=[0.02, 1.0]'.format(i))
    #     #                 return APIState.API_EXCEPTION
    #     #             if K[i] < 0 or K[i] > 2000:
    #     #                 logger.error('set_admittance, K[{}] over range, range=[0, 2000]'.format(i))
    #     #                 return APIState.API_EXCEPTION
    #     #         else:
    #     #             if M[i] < 0.0001 or M[i] > 0.01:
    #     #                 logger.error('set_admittance, M[{}] over range, range=[0.0001, 0.01]'.format(i))
    #     #                 return APIState.API_EXCEPTION
    #     #             if K[i] < 0 or K[i] > 20:
    #     #                 logger.error('set_admittance, K[{}] over range, range=[0, 20]'.format(i))
    #     #                 return APIState.API_EXCEPTION
    #     #         if B[i] < 0:
    #     #             logger.error('set_admittance, the value of B[{}] must be greater than or equal to 0'.format(i))
    #     #             return APIState.API_EXCEPTION
    #     # ret = self.arm_cmd.set_admittance(coord, c_axis, M, K, B)
    #     # self.log_api_info('API -> set_admittance -> code={}'.format(ret[0]), code=ret[0])
    #     # return self._check_code(ret[0])

    # @xarm_is_connected(_type='set')
    # def set_impedance_mbk(self, M, K, B, **kwargs):
    #     return self.set_ft_sensor_admittance_parameters(M, K, B, **kwargs)
    #     # if len(M) < 6 or len(K) < 6 or len(B) < 6:
    #     #     logger.error('set_admittance_mbk: parameters error')
    #     #     return APIState.API_EXCEPTION
    #     # params_limit = kwargs.get('params_limit', True)
    #     # if params_limit:
    #     #     for i in range(6):
    #     #         if i < 3:
    #     #             if M[i] < 0.02 or M[i] > 1.0:
    #     #                 logger.error('set_admittance_mbk, M[{}] over range, range=[0.02, 1.0]'.format(i))
    #     #                 return APIState.API_EXCEPTION
    #     #             if K[i] < 0 or K[i] > 2000:
    #     #                 logger.error('set_admittance_mbk, K[{}] over range, range=[0, 2000]'.format(i))
    #     #                 return APIState.API_EXCEPTION
    #     #         else:
    #     #             if M[i] < 0.0001 or M[i] > 0.01:
    #     #                 logger.error('set_admittance_mbk, M[{}] over range, range=[0.0001, 0.01]'.format(i))
    #     #                 return APIState.API_EXCEPTION
    #     #             if K[i] < 0 or K[i] > 20:
    #     #                 logger.error('set_admittance_mbk, K[{}] over range, range=[0, 20]'.format(i))
    #     #                 return APIState.API_EXCEPTION
    #     #         if B[i] < 0:
    #     #             logger.error('set_admittance_mbk, the value of B[{}] must be greater than or equal to 0'.format(i))
    #     #             return APIState.API_EXCEPTION
    #     # ret = self.arm_cmd.set_admittance_mbk(M, K, B)
    #     # self.log_api_info('API -> set_admittance_mbk -> code={}'.format(ret[0]), code=ret[0])
    #     # return self._check_code(ret[0])

    # @xarm_is_connected(_type='set')
    # def set_impedance_config(self, coord, c_axis):
    #     return self.set_ft_sensor_admittance_parameters(coord, c_axis)
    #     # if len(c_axis) < 6:
    #     #     logger.error('set_admittance_config: parameters error')
    #     #     return APIState.API_EXCEPTION
    #     # ret = self.arm_cmd.set_admittance_config(coord, c_axis)
    #     # self.log_api_info('API -> set_admittance_config -> code={}'.format(ret[0]), code=ret[0])
    #     # return self._check_code(ret[0])

    # @xarm_is_connected(_type='set')
    # def config_force_control(self, coord, c_axis, f_ref, limits, **kwargs):
    #     return self.set_ft_sensor_force_parameters(coord, c_axis, f_ref, limits, **kwargs)
    #     # if len(c_axis) < 6 or len(f_ref) < 6 or len(limits) < 6:
    #     #     logger.error('config_force_control: parameters error')
    #     #     return APIState.API_EXCEPTION
    #     # params_limit = kwargs.get('params_limit', True)
    #     # if params_limit:
    #     #     max_f_ref = [150, 150, 200, 4, 4, 4]
    #     #     for i in range(6):
    #     #         if f_ref[i] < -max_f_ref[i] or f_ref[i] > max_f_ref[i]:
    #     #             logger.error('config_force_control, f_ref[{}] over range, range=[{}, {}]'.format(i, -max_f_ref[i], max_f_ref[i]))
    #     #             return APIState.API_EXCEPTION
    #     # ret = self.arm_cmd.config_force_control(coord, c_axis, f_ref, limits)
    #     # self.log_api_info('API -> config_force_control -> code={}'.format(ret[0]), code=ret[0])
    #     # return self._check_code(ret[0])

    # @xarm_is_connected(_type='set')
    # def set_force_control_pid(self, kp, ki, kd, xe_limit, **kwargs):
    #     return self.set_ft_sensor_force_parameters(kp, ki, kd, xe_limit, **kwargs)
    #     # if len(kp) < 6 or len(ki) < 6 or len(kd) < 6 or len(xe_limit) < 6:
    #     #     logger.error('set_force_control_pid: parameters error')
    #     #     return APIState.API_EXCEPTION
    #     # params_limit = kwargs.get('params_limit', True)
    #     # if params_limit:
    #     #     for i in range(6):
    #     #         if kp[i] < 0 or kp[i] > 0.05:
    #     #             logger.error('set_force_control_pid, kp[{}] over range, range=[0, 0.05]'.format(i))
    #     #             return APIState.API_EXCEPTION
    #     #         if ki[i] < 0 or ki[i] > 0.0005:
    #     #             logger.error('set_force_control_pid, ki[{}] over range, range=[0, 0.0005]'.format(i))
    #     #             return APIState.API_EXCEPTION
    #     #         if kd[i] < 0 or kd[i] > 0.05:
    #     #             logger.error('set_force_control_pid, kd[{}] over range, range=[0, 0.05]'.format(i))
    #     #             return APIState.API_EXCEPTION
    #     #         if xe_limit[i] < 0 or xe_limit[i] > 200:
    #     #             logger.error('set_force_control_pid, xe_limit[{}] over range, range=[0, 200]'.format(i))
    #     #             return APIState.API_EXCEPTION
    #     # ret = self.arm_cmd.set_force_control_pid(kp, ki, kd, xe_limit)
    #     # self.log_api_info('API -> set_force_control_pid -> code={}'.format(ret[0]), code=ret[0])
    #     # return self._check_code(ret[0])

    @xarm_is_connected(_type='set')
    def set_ft_sensor_zero(self):
        ret = self.arm_cmd.ft_sensor_set_zero()
        self.log_api_info('API -> set_ft_sensor_zero -> code={}'.format(ret[0]), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='get')
    def iden_ft_sensor_load_offset(self):
        protocol_identifier = self.arm_cmd.get_protocol_identifier()
        self.arm_cmd.set_protocol_identifier(2)
        self._keep_heart = False
        ret = self.arm_cmd.ft_sensor_iden_load()
        self.arm_cmd.set_protocol_identifier(protocol_identifier)
        self._keep_heart = True
        self.log_api_info('API -> iden_ft_sensor_load_offset -> code={}'.format(ret[0]), code=ret[0])
        code = self._check_code(ret[0])
        if code == 0 or len(ret) > 5:
            ret[2] = ret[2] * 1000  # x_centroid, 从m转成mm
            ret[3] = ret[3] * 1000  # y_centroid, 从m转成mm
            ret[4] = ret[4] * 1000  # z_centroid, 从m转成mm
        return self._check_code(ret[0]), ret[1:11]

    @xarm_is_connected(_type='set')
    def set_ft_sensor_load_offset(self, iden_result_list, association_setting_tcp_load=False, **kwargs):
        if len(iden_result_list) < 10:
            return APIState.PARAM_ERROR
        params = iden_result_list[:]
        params[1] = params[1] / 1000.0  # x_centroid, 从mm转成m
        params[2] = params[2] / 1000.0  # y_centroid, 从mm转成m
        params[3] = params[3] / 1000.0  # z_centroid, 从mm转成m
        ret = self.arm_cmd.ft_sensor_cali_load(params)
        self.log_api_info('API -> set_ft_sensor_load_offset -> code={}, iden_result_list={}'.format(ret[0], iden_result_list), code=ret[0])
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0 and association_setting_tcp_load:
            m = kwargs.get('m', 0.270)  # 0.325
            x = kwargs.get('x', -17)
            y = kwargs.get('y', 9)
            z = kwargs.get('z', 11.8)
            weight = params[0] + m
            center_of_gravity = [
                (m * x + params[0] * params[1]) / weight,
                (m * y + params[0] * params[2]) / weight,
                (m * z + params[0] * (32 + params[3])) / weight
            ]
            self.set_state(0)
            return self.set_tcp_load(weight, center_of_gravity)
        return ret[0]

    @xarm_is_connected(_type='set')
    def set_ft_sensor_enable(self, on_off):
        ret = self.arm_cmd.ft_sensor_enable(on_off)
        self.log_api_info('API -> set_ft_sensor_enable -> code={}, on_off={}'.format(ret[0], on_off), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='get')
    def set_ft_sensor_mode(self, mode, **kwargs):
        mode = kwargs.get('app_code', mode)
        ret = self.arm_cmd.ft_sensor_app_set(mode)
        self.log_api_info('API -> set_ft_sensor_mode -> code={}, app_code={}'.format(ret[0], mode), code=ret[0])
        return self._check_code(ret[0])

    @xarm_is_connected(_type='get')
    def get_ft_sensor_mode(self):
        ret = self.arm_cmd.ft_sensor_app_get()
        return self._check_code(ret[0]), ret[1]

    @xarm_is_connected(_type='get')
    def get_ft_sensor_data(self, is_raw=False):
        is_raw = is_raw if self.version_is_ge(2, 6, 109) else False
        ret = self.arm_cmd.ft_sensor_get_data(self.version_is_ge(1, 8, 3), is_raw)
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
