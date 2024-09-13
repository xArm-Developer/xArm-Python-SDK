# import sys
# import os
# import time
# import numpy as np
# from xarm.wrapper import xarm_api
from circle import findBeaker
from scaling import getScalar
import time

# sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# class Arm_constants:
#     # units in mm:
#     SIZE_SQUARE : float = 27.49
#     SIZE_BOARD : float = 250
#     POS_Z_HIGHEST_PIECE : float = 53.71
#     POS_Z_BOARD : float = 0
#     OFFSET_X1 : float = 0
#     OFFSET_Y1 : float = 0
#     OFFSET_X2 : float = 0
#     OFFSET_Y2 : float = 0
#     SQUARE_LOCATIONS = np.empty((8, 8), dtype=tuple)
#     RESERVE_LOCATIONS = np.empty((4,1), dtype=tuple)
#     ROBOT_COLOR: bool = False # true if robot playing as white
#     ARM: XArmAPI
#     SQUARES_IN_ROW: int = 8
#     # POS_Z_BASE_BOARD : int = 135

# def instantiateArm():
#     Arm_constants.ARM = XArmAPI('192.168.1.166')
#     Arm_constants.ARM.motion_enable(enable=True)
#     Arm_constants.ARM.set_mode(0)
#     Arm_constants.ARM.connect()
#     # Arm_constants.ARM.clean_error()
#     # Arm_constants.ARM.clean_warn()

# def deinstantiateArm():
#     Arm_constants.ARM.motion_enable(enable=False)
#     Arm_constants.ARM.disconnect()

# def moveToSquare(x, y):
#     # print(square)
#     # print(indices)
#     Arm_constants.ARM.set_position(x, y, 100, 180, 0, 0, None, 100, 50, wait=True)

# instantiateArm()
# moveToSquare()
# # deinstantiateArm()
scalar = getScalar() 
time.sleep(3)
beaker = findBeaker()

scalar = (scalar[0], scalar[0] * scalar[1], scalar[0] * scalar[2]) 
beaker = (beaker[0] * scalar[0], beaker[1] * scalar[0])

x

print(scalar)
print(beaker)

