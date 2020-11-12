#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
import json
import requests
import functools
from .xarm_api import XArmAPI


class RemoteXArmAPI(XArmAPI):
    class __RemoteXArm:
        def __init__(self, _ip, _path, _arm):
            self._ip = _ip
            self._path = _path
            self._arm = _arm

        def __getattr__(self, item):
            attr = getattr(self._arm, item)
            remote_api = functools.partial(self.__call_studio_api, api_name=item)
            return remote_api if callable(attr) else remote_api()

        def __call_studio_api(self, *args, **kwargs):
            api_name = kwargs.pop('api_name', None)
            ip = kwargs.pop('ip', self._ip)
            path = kwargs.pop('path', self._path)
            if ip and api_name:
                r = requests.post('http://{}:18333/{}'.format(ip, path), data=json.dumps({
                    'cmd': api_name, 'args': args, 'kwargs': kwargs
                }), timeout=(5, None))
                if r.status_code == 200:
                    res = r.json()
                    if 'info' in res:
                        print(res['info'])
                    else:
                        return res['res']
                else:
                    print('request failed, http_status_code={}'.format(r.status_code))
            else:
                print('ip or api_name is empty, ip={}, api_name={}'.format(ip, api_name))

        def delete_blockly_app(self, name):
            return self.__call_studio_api(None, 0, {
                'parentPath': name,
                'selectNode': {
                    'type': 'file'
                }
            }, api_name='app_delete_item', path='cmd')

    def __init__(self, ip):
        # import warnings
        # warnings.warn("don't use it for now, just for debugging")
        XArmAPI.__init__(self, do_not_open=True)
        self._arm = self.__RemoteXArm(ip, 'api', self._arm)

    def delete_blockly_app(self, name):
        return self._arm.delete_blockly_app(name)
