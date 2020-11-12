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


class RemoteXArmAPI(XArmAPI):
    class __RemoteXArm:
        def __init__(self, _ip, _path, _arm):
            self._ip = _ip
            self._path = _path
            self._arm = _arm
            self.s = Session()

        def __del__(self):
            self.s.close()

        def __getattr__(self, item):
            if item.startswith(('register', 'release')):
                raise Exception('Cannot call an interface with callback parameters remotely')
            attr = getattr(self._arm, item)
            remote_api = functools.partial(self.__call_studio_api, api_name=item)
            return remote_api if callable(attr) else remote_api()

        def __call_studio_api(self, *args, **kwargs):
            api_name = kwargs.pop('api_name', None)
            ip = kwargs.pop('ip', self._ip)
            path = kwargs.pop('path', self._path)
            if ip and api_name:
                r = self.s.post('http://{}:18333/{}'.format(ip, path), data=json.dumps({
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

    def __init__(self, ip, ignore_warnning=False):
        if not ignore_warnning:
            import warnings
            warnings.warn("don't use it for now, just for debugging")
        XArmAPI.__init__(self, do_not_open=True)
        self._arm = self.__RemoteXArm(ip, 'api', self._arm)

    def delete_blockly_app(self, name):
        return self._arm.delete_blockly_app(name)
