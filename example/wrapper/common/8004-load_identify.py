#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: identify load with the end force sensor
"""

import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.clean_error()
arm.set_mode(0)
arm.set_state(0)
time.sleep(0.1)


def progress(item):
    print('progress: {}'.format(item['progress']))


arm.register_iden_progress_changed_callback(progress)
start_time = time.monotonic()
arm.ft_sensor_enable(1)
code, result = arm.ft_sensor_iden_load()
end_time = time.monotonic()
print('code={}, result={}, cost_time={}'.format(code, result, end_time - start_time))
arm.release_iden_progress_changed_callback(progress)
arm.ft_sensor_app_set(0)
arm.ft_sensor_enable(0)
arm.disconnect()

# arm.ft_sensor_enable(1)
# # Load identification ?
# response = input("\nDo you need Load Calibration?  y(es) / n(o):\n")
# if response == 'y':
#     print("Identification process start, please wait for finish..")
#     result = arm.ft_sensor_iden_load()[1]
#
#     print("\nIdentification result: ")
#     print(result)
#     response = input("\nContinue?  y(es) / n(o):\n")
#     if response == 'n':
#         print("Bye~")
#         exit()
#     arm.ft_sensor_cali_load(result)
# else:
#     # no need to identify and write load parameters：
#     # [load mass : (kg)，Centroid x : (m)，Centroid y : (m)，Centroid z :(m)，offset_Fx (N)，offset_Fx (N)，offset_Fx(N)，offset_Mx(Nm)，offset_My (Nm)，offset_Mz (Nm)]
#     result = [0.7131531834602356, -0.0005494913784787059, -0.0026768327224999666, 0.06637067347764969, -17.749963760375977, 2.7701117992401123, -30.62084197998047, 0.13900014758110046, -0.37988412380218506, -0.14760535955429077]
#     arm.ft_sensor_cali_load(result)
#
# # waiting to take effect
# time.sleep(1)
# arm.ft_sensor_app_set(0)
# arm.ft_sensor_enable(0)
# arm.disconnect()
