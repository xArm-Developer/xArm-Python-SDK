# -*- coding: utf-8 -*-
# __author: rock
# @time: 2021-04-02

from ..core.utils.log import logger
from .base import Base
import math


class BaseBoard(Base):

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

    def get_current_angle(self, board_id=10):
        ret1 = self.arm_cmd.base_tool_addr_r32(board_id, addr=0x0C00)
        ret2 = self.arm_cmd.base_tool_addr_r32(board_id, addr=0x0C02)
        ret3 = self.arm_cmd.base_tool_addr_r32(board_id, addr=0x0C04)
        code = 0
        if ret1[0] == 0 and ret2[0] == 0 and ret3[0] == 0:
            acc_x = ret1[1]
            acc_y = ret2[1]
            acc_z = ret3[1]
            angle = self.__get_z_axios_offset_angle(acc_x, acc_y, acc_z)
            return code, angle

        else:
            code = ret1[0] or ret2[0] or ret3[0]
            return code, 0

    @staticmethod
    def __get_z_axios_offset_angle(x=1, y=1, z=1):
        angle = math.degrees(math.atan(z / (math.sqrt(abs((x ** 2) + (y ** 2))))))
        angle = 90 - angle
        return angle

    def write_sn(self, sn=''):
        code = 0
        if len(sn) == 14:
            for i in range(0, 14, 2):
                ret = self.arm_cmd.core.base_tool_addr_w16(servo_id=id, addr=0x1900+(int(i/2)),
                                                           value=ord(sn[i]) | ord(sn[i+1]) << 8)
                code = ret[0]
            return code
        else:
            return 1


