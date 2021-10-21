#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFactory, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


class ThreadManage(object):
    def __init__(self):
        self.threads = []

    def append(self, thread):
        self.threads.append(thread)

    def remove(self, thread):
        if thread in self.threads:
            self.threads.remove(thread)

    def join(self, timeout=None):
        for t in self.threads:
            try:
                t.join(timeout=timeout)
            except:
                pass
        self.threads.clear()

    def count(self):
        return len(self.threads)
