# -*- coding: utf-8 -*-
# __author: rock
# @time: 2021-04-02

import time
from .utils import xarm_is_connected, xarm_is_pause, xarm_is_not_simulation_mode, xarm_wait_until_cmdnum_lt_max
from ..core.utils.log import logger
from ..core.config.x_config import XCONF
from .code import APIState
from .base import Base


class BaseBoard(Base):
    BASE_BOARD_ID = 10

    def __init__(self):
        super(BaseBoard, self).__init__()

    def get_base_board_version(self, board_id=10):
        versions = ['*', '*', '*']

        ret1 = self.arm_cmd.base_tool_addr_r16(board_id, 0x0801)
        ret2 = self.arm_cmd.base_tool_addr_r16(board_id, 0x0802)
        ret3 = self.arm_cmd.base_tool_addr_r16(board_id, 0x0803)

        code = 0
        if ret1[0] == 0 and len(ret1) == 2:
            versions[0] = ret1[1]
        else:
            code = ret1[0]
        if ret2[0] == 0 and len(ret2) == 2:
            versions[1] = ret2[1]
        else:
            code = ret2[0]
        if ret3[0] == 0 and len(ret3) == 2:
            versions[2] = ret3[1]
        else:
            code = ret3[0]

        return code, '.'.join(map(str, versions))

    def check_place_mode(self):
        acc_x = self.arm_cmd.base_tool_addr_r32(self.BASE_BOARD_ID, addr=0x0C00)
        acc_y = self.arm_cmd.base_tool_addr_r32(self.BASE_BOARD_ID, addr=0x0C02)
        acc_z = self.arm_cmd.base_tool_addr_r32(self.BASE_BOARD_ID, addr=0x0C04)
        print("ax: %f ,ay: %f az: %f\r\n" % (acc_x[1], acc_y[1], acc_z[1]))

        if acc_z[1] > -1.3 and acc_z[1] < -0.7:
            print("吊装")
            return 0, 2
        if acc_z[1] < 1.3 and acc_z[1] > 0.7:
            print("水平")
            return 0, 1
        if acc_x[1] < 1.3 and acc_x[1] > 0.9:
            print("壁装向下")
            return 0, 4
        if acc_x[1] > -1.3 and acc_x[1] < -0.9:
            print("壁装向上")
            return 0, 3
        return 0, 0
