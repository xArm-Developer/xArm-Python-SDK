#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import json
import time
import uuid
from urllib import request
from .code import APIState
from ..core.config.x_config import XCONF
from ..core.utils.log import logger
from .base import Base
from .decorator import xarm_is_connected


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
    def stop_record_trajectory(self, filename=None, **kwargs):
        ret = self.arm_cmd.set_record_traj(0)
        if isinstance(filename, str) and filename.strip():
            ret2 = self.save_record_trajectory(filename, wait=True, timeout=10, **kwargs)
            if ret2 != 0:
                return ret2
        self.log_api_info('API -> stop_record_trajectory -> code={}'.format(ret[0]), code=ret[0])
        return ret[0]

    @xarm_is_connected(_type='set')
    def save_record_trajectory(self, filename, wait=True, timeout=2, **kwargs):
        assert isinstance(filename, str) and filename.strip()
        filename = filename.strip()
        if not filename.endswith('.traj'):
            full_filename = '{}.traj'.format(filename)
        else:
            full_filename = filename
        self.get_trajectory_rw_status()
        feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
        ret = self.arm_cmd.save_traj(full_filename, wait_time=0, feedback_key=feedback_key)
        trans_id = self._get_feedback_transid(feedback_key, studio_wait)
        self.log_api_info('API -> save_record_trajectory -> code={}'.format(ret[0]), code=ret[0])
        ret[0] = self._check_code(ret[0])
        if ret[0] == 0 and wait:
            return self.__wait_save_traj(timeout, trans_id, filename)
        if ret[0] != 0:
            logger.error('Save {} failed, ret={}'.format(filename, ret))
        return ret[0]
    
    def __check_traj_status(self, status, filename='unknown'):
        if status == XCONF.TrajState.LOAD_SUCCESS:
            logger.info('Load {} success'.format(filename))
            return 0
        elif status == XCONF.TrajState.LOAD_FAIL:
            logger.error('Load {} failed'.format(filename))
            return APIState.TRAJ_RW_FAILED
        elif status == XCONF.TrajState.SAVE_SUCCESS:
            logger.info('Save {} success'.format(filename))
            return 0
        elif status == XCONF.TrajState.SAVE_FAIL:
            logger.error('Save {} failed'.format(filename))
            return APIState.TRAJ_RW_FAILED
        return -1
    
    def __wait_traj_op(self, timeout, trans_id, filename='unknown', op='Load'):
        if self._support_feedback and trans_id > 0:
            code, feedback_code = self._wait_feedback(timeout, trans_id)
            _, status = self.get_trajectory_rw_status()
            if code == 0:
                success_status = XCONF.TrajState.LOAD_SUCCESS if op == 'Load' else XCONF.TrajState.SAVE_SUCCESS
                failure_status = XCONF.TrajState.LOAD_FAIL if op == 'Load' else XCONF.TrajState.SAVE_FAIL
                status = success_status if feedback_code == XCONF.FeedbackCode.SUCCESS else failure_status if feedback_code == XCONF.FeedbackCode.FAILURE else status            
            return self.__check_traj_status(status, filename)
        else:
            expired = (time.monotonic() + timeout) if timeout is not None else 0
            idle_cnts = 0
            while timeout is None or time.monotonic() < expired:
                time.sleep(0.1)
                code, status = self.get_trajectory_rw_status()
                if self._check_code(code) == 0:
                    if status == XCONF.TrajState.IDLE:
                        idle_cnts += 1
                        if idle_cnts >= 5:
                            logger.info('{} {} failed, idle'.format(op, filename))
                            return APIState.TRAJ_RW_FAILED
                    else:
                        code = self.__check_traj_status(status, filename)
                        if code >= 0:
                            return code
            logger.warning('{} {} timeout'.format(op, filename))
            return APIState.TRAJ_RW_TOUT
    
    def __wait_load_traj(self, timeout, trans_id, filename='unknown'):
        return self.__wait_traj_op(timeout, trans_id, filename, 'Load')

    def __wait_save_traj(self, timeout, trans_id, filename='unknown'):
        return self.__wait_traj_op(timeout, trans_id, filename, 'Save')
    
    def __wait_play_traj(self, timeout, trans_id, times=1):
        if self._support_feedback and trans_id > 0:
            code, feedback_code = self._wait_feedback(timeout, trans_id)
            if feedback_code == XCONF.FeedbackCode.FAILURE:
                code = APIState.TRAJ_PLAYBACK_FAILED
        else:
            start_time = time.monotonic()
            while self.state != 1:
                if self.state in [4]:
                    return APIState.STATE_NOT_READY
                if time.monotonic() - start_time > 5:
                    return APIState.TRAJ_PLAYBACK_TOUT
                time.sleep(0.1)
            max_count = int((time.monotonic() - start_time) / 0.1)
            max_count = max_count if max_count > 10 else 10
            start_time = time.monotonic()
            while self.mode != 11:
                if self.state == 1:
                    start_time = time.monotonic()
                    time.sleep(0.1)
                    continue
                if self.state in [4]:
                    return APIState.STATE_NOT_READY
                if time.monotonic() - start_time > 5:
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
            code = 0 if self.state != 4 else APIState.STATE_NOT_READY

        if self.state not in [4]:
            self.set_mode(0)
            self.set_state(0)
        self._sync()
        return code

    @xarm_is_connected(_type='set')
    def load_trajectory(self, filename, wait=True, timeout=None, **kwargs):
        assert isinstance(filename, str) and filename.strip()
        filename = filename.strip()
        if not filename.endswith('.traj'):
            full_filename = '{}.traj'.format(filename)
        else:
            full_filename = filename
        self.get_trajectory_rw_status()
        feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
        ret = self.arm_cmd.load_traj(full_filename, wait_time=0, feedback_key=feedback_key)
        trans_id = self._get_feedback_transid(feedback_key, studio_wait)
        self.log_api_info('API -> load_trajectory -> code={}'.format(ret[0]), code=ret[0])
        if ret[0] == 0 and wait:
            return self.__wait_load_traj(timeout, trans_id, filename)
        if ret[0] != 0:
            logger.error('Load {} failed, ret={}'.format(filename, ret))
        return ret[0]

    @xarm_is_connected(_type='set')
    def playback_trajectory(self, times=1, filename=None, wait=False, double_speed=1, **kwargs):
        assert isinstance(times, int)
        times = times if times > 0 else -1
        if isinstance(filename, str) and filename.strip():
            ret = self.load_trajectory(filename, wait=True, timeout=None)
            if ret != 0:
                return ret
        if self.state in [4]:
            return APIState.NOT_READY
        feedback_key, studio_wait = self._gen_feedback_key(wait, **kwargs)
        if self.version_is_ge(1, 2, 11):
            ret = self.arm_cmd.playback_traj(times, double_speed, feedback_key=feedback_key)
        else:
            ret = self.arm_cmd.playback_traj_old(times)
        trans_id = self._get_feedback_transid(feedback_key, studio_wait)
        self.log_api_info('API -> playback_trajectory -> code={}'.format(ret[0]), code=ret[0])
        if ret[0] == 0 and wait:
            return self.__wait_play_traj(None, trans_id, times)
        return ret[0]

    @xarm_is_connected(_type='get')
    def get_trajectory_rw_status(self):
        ret = self.arm_cmd.get_traj_rw_status()
        return ret[0], ret[1]
