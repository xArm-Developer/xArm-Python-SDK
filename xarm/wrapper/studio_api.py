#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
import json
import functools
from .xarm_api import XArmAPI

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


class XArmStudioAPI(object):
    def __init__(self, ip, ignore_warnning=False):
        if not ignore_warnning:
            import warnings
            warnings.warn("don't use it for now, just for debugging")
        self.__ip = ip
        self.__session = Session()
        self.arm = self.__RemoteXArmAPI(self._call_sdk_api)

    def __del__(self):
        self.__session.close()

    def run_blockly_app(self, name, **kwargs):
        try:
            self._call_studio_api({}, api_name='xarm_set_blockly_init', show_fail_log=False)
        except:
            pass
        kwargs['appName'] = name
        return self._call_studio_api(kwargs, api_name='run_blockly')

    def delete_blockly_app(self, name):
        return self._call_studio_api({
            'parentPath': name,
            'selectNode': {
                'type': 'file'
            }
        }, api_name='app_delete_item')

    def playback_trajectory(self, filename, times=1, wait=False, double_speed=1):
        return self._call_studio_api({
            'filename': filename,
            'times': times,
            'speed': double_speed,
            'wait': wait,
        }, api_name='xarm_playback_traj')

    def delete_trajectory(self, filename):
        return self._call_studio_api({
            'filename': filename,
        }, api_name='xarm_delete_traj')

    def _call_studio_api(self, *args, **kwargs):
        kwargs['path'] = 'cmd'
        return self.__call_remote_api(None, 0, *args, **kwargs)

    def _call_sdk_api(self, *args, **kwargs):
        kwargs['path'] = 'api'
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
                if 'info' in res:
                    if show_fail_log:
                        print(res['info'])
                else:
                    return res['res']
            else:
                if show_fail_log:
                    print('request failed, http_status_code={}'.format(r.status_code))
        else:
            if show_fail_log:
                print('ip or api_name is empty, ip={}, api_name={}'.format(self.__ip, api_name))

    class __RemoteXArmAPI(XArmAPI):
        def __init__(self, call_sdk_func, **kwargs):
            XArmAPI.__init__(self, do_not_open=True)
            self._arm = self.__RemoteXArm(call_sdk_func, self._arm)

        class __RemoteXArm:
            def __init__(self, call_sdk_func, _arm):
                self.__call_sdk_func = call_sdk_func
                self.__arm = _arm

            def __getattr__(self, item):
                if item.startswith(('register', 'release', 'arm_cmd')):
                    raise Exception('Cannot remotely call interfaces that cannot serialize parameters or results')
                attr = getattr(self.__arm, item)
                remote_api = functools.partial(self.__call_sdk_func, api_name=item)
                return remote_api if callable(attr) else remote_api()
