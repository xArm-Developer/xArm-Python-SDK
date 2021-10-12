#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import json
import time
from urllib import request
from .utils import xarm_is_connected
from .code import APIState
from ..core.config.x_config import XCONF
from ..core.utils.log import logger
from .base import Base


class Record(Base):
    def __init__(self):
        super(Record, self).__init__()

    @xarm_is_connected(_type='get')
    def get_trajectories(self, ip=None):
        if ip is None:
            url = 'http://{}:18333/cmd'.format(self._port)
        else:
            url = 'http://{}:18333/cmd'.format(ip)
        try:
            data = {'cmd': 'xarm_list_trajs'}
            req = request.Request(url, headers={'Content-Type': 'application/json'}, data=json.dumps(data).encode('utf-8'))
            res = request.urlopen(req)
            if res.code == 200:
                result = json.loads(res.read().decode('utf-8'))
                return result['res'][0], [{'name': item['name'], 'duration': item['count'] / 100} for item in result['res'][1]]
            else:
                return APIState.API_EXCEPTION, []
        except Exception as e:
            return APIState.API_EXCEPTION, []

    @xarm_is_connected(_type='set')
    def start_record_trajectory(self):
        ret = self.arm_cmd.set_record_traj(1)
        self.log_api_info('API -> start_record_trajectory -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def stop_record_trajectory(self, filename=None):
        ret = self.arm_cmd.set_record_traj(0)
        if isinstance(filename, str) and filename.strip():
            ret2 = self.save_record_trajectory(filename, wait=True, timeout=10)
            if ret2 != 0:
                return ret2
        self.log_api_info('API -> stop_record_trajectory -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def save_record_trajectory(self, filename, wait=True, timeout=2):
        assert isinstance(filename, str) and filename.strip()
        filename = filename.strip()
        if not filename.endswith('.traj'):
            full_filename = '{}.traj'.format(filename)
        else:
            full_filename = filename
        ret = self.arm_cmd.save_traj(full_filename, wait_time=0)
        self.log_api_info('API -> save_record_trajectory -> code={}'.format(ret[0]), code=ret[0])
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0:
            if wait:
                start_time = time.time()
                while time.time() - start_time < timeout:
                    code, status = self.get_trajectory_rw_status()
                    if self._check_code(code) == 0:
                        if status == XCONF.TrajState.IDLE:
                            logger.error('Save {} failed, idle'.format(filename))
                            return APIState.TRAJ_RW_FAILED
                        elif status == XCONF.TrajState.SAVE_SUCCESS:
                            logger.info('Save {} success'.format(filename))
                            return 0
                        elif status == XCONF.TrajState.SAVE_FAIL:
                            logger.error('Save {} failed'.format(filename))
                            return APIState.TRAJ_RW_FAILED
                    time.sleep(0.1)
                logger.warning('Save {} timeout'.format(filename))
                return APIState.TRAJ_RW_TOUT
            else:
                return ret[0]
        logger.error('Save {} failed, ret={}'.format(filename, ret))
        return ret[0]

    @xarm_is_connected(_type='set')
    def load_trajectory(self, filename, wait=True, timeout=10):
        assert isinstance(filename, str) and filename.strip()
        filename = filename.strip()
        if not filename.endswith('.traj'):
            full_filename = '{}.traj'.format(filename)
        else:
            full_filename = filename
        ret = self.arm_cmd.load_traj(full_filename, wait_time=0)
        self.log_api_info('API -> load_trajectory -> code={}'.format(ret[0]), code=ret[0])
        if ret[0] == 0:
            if wait:
                start_time = time.time()
                while time.time() - start_time < timeout:
                    code, status = self.get_trajectory_rw_status()
                    if code == 0:
                        if status == XCONF.TrajState.IDLE:
                            logger.info('Load {} failed, idle'.format(filename))
                            return APIState.TRAJ_RW_FAILED
                        elif status == XCONF.TrajState.LOAD_SUCCESS:
                            logger.info('Load {} success'.format(filename))
                            return 0
                        elif status == XCONF.TrajState.LOAD_FAIL:
                            logger.error('Load {} failed'.format(filename))
                            return APIState.TRAJ_RW_FAILED
                    time.sleep(0.1)
                logger.warning('Load {} timeout'.format(filename))
                return APIState.TRAJ_RW_TOUT
            else:
                return ret[0]
        logger.error('Load {} failed, ret={}'.format(filename, ret))
        return ret[0]

    @xarm_is_connected(_type='set')
    def playback_trajectory(self, times=1, filename=None, wait=False, double_speed=1):
        assert isinstance(times, int)
        mode = self.mode
        times = times if times > 0 else -1
        if isinstance(filename, str) and filename.strip():
            ret = self.load_trajectory(filename, wait=True, timeout=10)
            if ret != 0:
                return ret
        if self.state in [4]:
            return APIState.NOT_READY
        if self.version_is_ge(1, 2, 11):
            ret = self.arm_cmd.playback_traj(times, double_speed)
        else:
            ret = self.arm_cmd.playback_traj_old(times)
        self.log_api_info('API -> playback_trajectory -> code={}'.format(ret[0]), code=ret[0])
        if ret[0] == 0 and wait:
            start_time = time.time()
            while self.state != 1:
                if self.state in [4]:
                    return APIState.NOT_READY
                if time.time() - start_time > 5:
                    return APIState.TRAJ_PLAYBACK_TOUT
                time.sleep(0.1)
            max_count = int((time.time() - start_time) / 0.1)
            max_count = max_count if max_count > 10 else 10
            start_time = time.time()
            while self.mode != 11:
                if self.state == 1:
                    start_time = time.time()
                    time.sleep(0.1)
                    continue
                if self.state in [4]:
                    return APIState.NOT_READY
                if time.time() - start_time > 5:
                    return APIState.TRAJ_PLAYBACK_TOUT
                time.sleep(0.1)
            time.sleep(0.1)
            count = 0
            while self.state not in [4]:
                if self.state == 2:
                    if times == 1:
                        break
                    count += 1
                else:
                    count = 0
                if count > max_count:
                    break
                time.sleep(0.1)
            # while self.state != 4 and self.state != 2:
            #     time.sleep(0.1)
            if self.state not in [4]:
                # self.set_mode(mode)
                self.set_mode(0)
                self.set_state(0)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_trajectory_rw_status(self):
        ret = self.arm_cmd.get_traj_rw_status()
        return ret[0], ret[1]
