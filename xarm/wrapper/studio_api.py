#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import functools
from ..x3.studio import Studio
from .xarm_api import XArmAPI


class XArmStudioAPI(Studio):
    def __init__(self, ip, ignore_warnning=False):
        super(XArmStudioAPI, self).__init__(ip, ignore_warnning=ignore_warnning)
        self.arm = self.__RemoteXArmAPI(self.call_sdk_api)

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
