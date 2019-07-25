#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


class Record(object):
    def __init__(self):
        pass

    def list_record_files(self):
        pass

    def start_record_trajectory(self):
        ret = self.arm_cmd.set_record_traj(1)
        print(ret)
        return ret[0]

    def stop_record_trajectory(self):
        ret = self.arm_cmd.set_record_traj(0)
        print(ret)
        return ret[0]

    def save_record_trajectory(self, filename, wait_time=2):
        ret = self.arm_cmd.save_traj(filename.strip(), wait_time=wait_time)
        print(ret)
        return ret[0]

    def load_record_trajectory(self, filename, wait_time=2):
        ret = self.arm_cmd.load_traj(filename.strip(), wait_time=wait_time)
        print(ret)
        return ret[0]

    def play_record_trajectory(self, times=1):
        ret = self.arm_cmd.playback_traj(times)
        print(ret)
        return ret[0]
