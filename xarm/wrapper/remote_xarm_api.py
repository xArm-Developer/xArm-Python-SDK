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


def _call_studio_api(*args, **kwargs):
    api_name = kwargs.pop('api_name', None)
    ip = kwargs.pop('ip', None)
    path = kwargs.pop('path', 'api')
    if ip and api_name:
        r = requests.post('http://{}:18333/{}'.format(ip, path), data=json.dumps({
            'cmd': api_name, 'args': args, 'kwargs': kwargs
        }), timeout=(5, None))
        if r.status_code == 200:
            res = r.json()
            if 'info' in res:
                return -1, res['info']
            return res['res']
        else:
            print('request failed, http_status_code={}'.format(r.status_code))
    else:
        print('ip or api_name is empty, ip={}, api_name={}'.format(ip, api_name))


class _RemoteXArm(object):
    def __init__(self, ip, _arm):
        self.ip = ip
        self._arm = _arm

    def __getattr__(self, item, *args, **kwargs):
        kwargs['ip'] = self.ip
        kwargs['api_name'] = item
        attr = getattr(self._arm, item)
        remote_api = functools.partial(_call_studio_api, *args, **kwargs)
        return remote_api if callable(attr) else remote_api()


class RemoteXArmAPI(XArmAPI):
    def __init__(self, ip):
        import warnings
        warnings.warn("don't use it for now, just for debugging")
        XArmAPI.__init__(self, do_not_open=True)
        self.__ip = ip
        self.__old_arm = self._arm
        self._arm = _RemoteXArm(ip, self.__old_arm)

    def delete_blockly_app(self, name):
        return _call_studio_api(None, 0, {
            'parentPath': name,
            'selectNode': {
                'type': 'file'
            }
        }, api_name='app_delete_item', path='cmd', ip=self.__ip)
