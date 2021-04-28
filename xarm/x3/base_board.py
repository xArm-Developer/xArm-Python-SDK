# -*- coding: utf-8 -*-
# __author: rock
# @time: 2021-04-02

from ..core.utils.log import logger
from .base import Base
import math


class BaseBoard(Base):
    BASE_BOARD_ID = 10
    init_angle = 0

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

    def get_current_angle(self):
        ret1 = self.arm_cmd.base_tool_addr_r32(self.BASE_BOARD_ID, addr=0x0C00)
        ret2 = self.arm_cmd.base_tool_addr_r32(self.BASE_BOARD_ID, addr=0x0C02)
        ret3 = self.arm_cmd.base_tool_addr_r32(self.BASE_BOARD_ID, addr=0x0C04)
        code = 0
        if ret1[0] == 0 and ret2[0] == 0 and ret3[0] == 0:
            acc_x = ret1[1]
            acc_y = ret2[1]
            acc_z = ret3[1]
            logger.info("ax: {} ,ay: {} az: {}".format(acc_x, acc_y, acc_z))
            angle = self.__get_z_axios_offset_angle(acc_x, acc_y, acc_z)
            return code, angle

        else:
            code = ret1[0] or ret2[0] or ret3[0]
            return code, 0

    def get_init_angle(self):
        cmds = [0x0C00, 0x0C02, 0x0C04]
        times = 10
        offset_angle_li = []
        axyz = []
        code = 0
        while times:
            times = times - 1
            for cmd in cmds:
                ret = self.arm_cmd.base_tool_addr_r32(self.BASE_BOARD_ID, cmd)
                code = ret[0]
                if code == 0:
                    axyz.append(ret[1])
                else:
                    raise Exception('Get value failed')
            angle = self.__get_z_axios_offset_angle(*axyz)
            axyz = []
            offset_angle_li.append(angle)
        self.init_angle = (sum(offset_angle_li))/len(offset_angle_li)
        # print('init_angle:', self.init_angle)
        return code, self.init_angle

    def __get_z_axios_offset_angle(self, x=1, y=1, z=1):

        angle = math.degrees(math.atan(z / (math.sqrt(abs((x ** 2) + (y ** 2))))))
        angle = 90 - angle
        return angle


