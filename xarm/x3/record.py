#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import json
from urllib import request
from .utils import xarm_is_connected
from .code import APIState


class Record(object):
    def __init__(self):
        pass

    @xarm_is_connected(_type='get')
    def list_trajectories(self, ip=None):
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
        print('record start: ret={}'.format(ret))
        return ret[0]

    @xarm_is_connected(_type='set')
    def stop_record_trajectory(self, filename=None, wait_time=2):
        ret = self.arm_cmd.set_record_traj(0)
        print('record stop: ret={}'.format(ret))
        if isinstance(filename, str) and filename.strip():
            self.save_record_trajectory(filename, wait_time=wait_time)
        return ret[0]

    @xarm_is_connected(_type='set')
    def save_record_trajectory(self, filename, wait_time=2):
        assert isinstance(filename, str) and filename.strip()
        filename = filename.strip()
        if not filename.endswith('.traj'):
            filename = '{}.traj'.format(filename)
        ret = self.arm_cmd.save_traj(filename.strip(), wait_time=wait_time)
        print('record save: ret={}'.format(ret))
        return ret[0]

    @xarm_is_connected(_type='set')
    def load_trajectory(self, filename, wait_time=2):
        assert isinstance(filename, str) and filename.strip()
        filename = filename.strip()
        if not filename.endswith('.traj'):
            filename = '{}.traj'.format(filename)
        ret = self.arm_cmd.load_traj(filename, wait_time=wait_time)
        print('record load: ret={}'.format(ret))
        return ret[0]

    @xarm_is_connected(_type='set')
    def playback_trajectory(self, times=1, filename=None, wait_time=2):
        assert isinstance(times, int)
        times = times if times > 0 else -1
        if isinstance(filename, str) and filename.strip():
            self.load_trajectory(filename, wait_time=wait_time)
        ret = self.arm_cmd.playback_traj(times)
        print('record play: ret={}'.format(ret))
        return ret[0]
