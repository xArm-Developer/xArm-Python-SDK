from vishal import get_locations
import cv2
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

# from xarm.wrapper import XArmAPI
from xarm.wrapper import XArmAPI

# cap = cv2.VideoCapture(0)

# if not cap.isOpened():
#     print("Camera not found or not accessible")
#     exit()

arm = XArmAPI(port='192.168.1.166')
# arm.set_tcp_load(1, [200, 0, 0])
# # arm.set_tcp_offset([160, 0, 0, 0, 0, 0])
# arm.set_collision_sensitivity(3)
# arm.clean_warn()
# arm.clean_error()
# arm.motion_enable(enable=True)
# arm.set_mode(0)
# arm.set_state(0)
# arm.connect()

# start = (172.3, 45.1, 250.2, 73.6, -84.2, 105.1)
# april_tag = (179, -151.7, 201, 73.6, -84.2, 105.1)

# arm.set_position(start[0], start[1], start[2], start[3], start[4], start[5], None, 50, 50, wait=True)

# locations = get_locations(cap, 2, 50)

# print(locations)

# arm.set_position(april_tag[0], april_tag[1], april_tag[2], april_tag[3], april_tag[4], april_tag[5], None, 50, 50, wait=True)
# arm.set_position(april_tag[0], april_tag[1], april_tag[2] + 100, april_tag[3], april_tag[4], april_tag[5], None, 50, 50, wait=True)

# for i in locations:
#     arm.set_position(april_tag[0] + (1000 * i[1]), april_tag[1] + (1000 * i[0]), april_tag[2] + 100,  april_tag[3], april_tag[4], april_tag[5], None, 50, 50, wait=True)
#     arm.set_position(april_tag[0] + (1000 * i[1]), april_tag[1] + (1000 * i[0]), april_tag[2], april_tag[3], april_tag[4], april_tag[5], None, 50, 50, wait=True)
#     arm.set_position(april_tag[0] + (1000 * i[1]), april_tag[1] + (1000 * i[0]), april_tag[2] + 100,  april_tag[3], april_tag[4], april_tag[5], None, 50, 50, wait=True)