#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import json
import warnings
from ..core.utils.log import logger
from .code import APIState

try:
    from requests import Session
except:
    import urllib.request

    class Session(object):
        class Request:
            def __init__(self, url, data, **kwargs):
                req = urllib.request.Request(url, data.encode('utf-8'))
                self.r = urllib.request.urlopen(req)
                self._data = self.r.read()

            @property
            def status_code(self):
                return self.r.code

            def json(self):
                return json.loads(self._data.decode('utf-8'))

        def post(self, url, data=None, **kwargs):
            return self.Request(url, data)

        def close(self):
            pass


class Studio(object):
    def __init__(self, ip, ignore_warnning=False):
        super(Studio, self).__init__()
        if not ignore_warnning:
            warnings.warn("don't use it for now, just for debugging")
        self.__ip = ip
        self.__session = Session()

    def __del__(self):
        self.__session.close()

    def run_blockly_app(self, name, **kwargs):
        try:
            self.call_studio_api({}, api_name='Core.command.xarm_set_blockly_init', show_fail_log=False)
        except:
            pass
        kwargs['appName'] = name
        ret = self.call_studio_api(kwargs, api_name='Core.command.run_blockly')
        if ret:
            return ret['code']
        return APIState.API_EXCEPTION

    def delete_blockly_app(self, name):
        ret = self.call_studio_api({
            'parentPath': name,
            'selectNode': {
                'type': 'file'
            }
        }, api_name='Core.command.app_delete_item')
        if ret:
            return ret['code']
        return APIState.API_EXCEPTION

    def playback_trajectory(self, filename, times=1, wait=False, double_speed=1):
        ret = self.call_studio_api({
            'filename': filename,
            'times': times,
            'speed': double_speed,
            'wait': wait,
        }, api_name='Core.command.xarm_playback_traj')
        if ret:
            return ret['code']
        return APIState.API_EXCEPTION

    def delete_trajectory(self, filename):
        ret = self.call_studio_api({
            'filename': filename,
        }, api_name='Core.command.xarm_delete_traj')
        if ret:
            return ret['code']
        return APIState.API_EXCEPTION

    def set_initial_point(self, point):
        ret = self.call_studio_api({'point': point}, api_name='Core.command.xarm_set_initial_point')
        if ret:
            return ret['code']
        return APIState.API_EXCEPTION

    def get_initial_point(self):
        ret = self.call_studio_api(api_name='XArm.xarm_initial_point')
        if ret:
            return ret['code'], ret['data']
        return APIState.API_EXCEPTION, [0] * 7

    def call_sdk_api(self, *args, **kwargs):
        kwargs['api_name'] = 'XArm.xarm.{}'.format(kwargs['api_name'])
        return self.call_studio_api(*args, **kwargs)

    def call_studio_api(self, *args, **kwargs):
        kwargs['path'] = 'v2/api'
        return self.__call_remote_api(*args, **kwargs)

    def __call_remote_api(self, *args, **kwargs):
        api_name = kwargs.pop('api_name', None)
        show_fail_log = kwargs.pop('show_fail_log', True)
        path = kwargs.pop('path')
        if self.__ip and api_name:
            r = self.__session.post('http://{}:18333/{}'.format(self.__ip, path), data=json.dumps({
                'cmd': api_name, 'args': args, 'kwargs': kwargs
            }), timeout=(5, None))
            if r.status_code == 200:
                res = r.json()
                if res['code'] != 0:
                    if show_fail_log:
                        logger.error(res['data'])
                return res
            else:
                if show_fail_log:
                    logger.error('request failed, http_status_code={}'.format(r.status_code))
        else:
            if show_fail_log:
                logger.error('ip or api_name is empty, ip={}, api_name={}'.format(self.__ip, api_name))


