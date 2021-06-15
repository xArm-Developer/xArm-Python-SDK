# -*- coding: utf-8 -*-
# __author: rock
# @time: 2021-04-02

from ..core.utils.log import logger
from .base import Base
import math
import time


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
        code, acc_x, acc_y, acc_z = self.get_imu_data(board_id)
        ret = self.arm_cmd.base_tool_addr_w16(board_id, addr=0x0606, value=1)
        if code == 0:
            angle = self.__get_z_axios_offset_angle(acc_x, acc_y, acc_z)
            return code, angle
        else:
            return ret[0], 0

    @staticmethod
    def __get_z_axios_offset_angle(x=1, y=1, z=1):
        angle = math.degrees(math.atan(z / (math.sqrt(abs((x ** 2) + (y ** 2))))))
        angle = 90 - angle
        return angle

    def write_sn(self, sn='', servo_id=10):
        code = 0
        if len(sn) == 14:
            for i in range(0, 14, 2):
                ret = self.arm_cmd.base_tool_addr_w16(servo_id, addr=0x1900 + (int(i / 2)),
                                                      value=ord(sn[i]) | ord(sn[i + 1]) << 8)
                time.sleep(0.1)
                if ret[0] != 0:
                    return 1
                code = ret[0]

        # logger.info("write_sn: {}".format(sn))
        return code

    def get_sn(self, servo_id=10):
        rd_sn = ''
        ret = [0, '']
        for i in range(0, 14, 2):
            ret = self.arm_cmd.base_tool_addr_r16(servo_id, addr=0x0900 + (int(i / 2)))
            time.sleep(0.1)
            rd_sn = ''.join([rd_sn, chr(ret[1] & 0x00FF)])
            rd_sn = ''.join([rd_sn, chr((ret[1] >> 8) & 0x00FF)])
            if ret[0] != 0:
                return 1, ''
        return ret[0], rd_sn

    def write_iden_to_base(self, idens, servo_id=10):
        cmds = [0x0D00, 0x0D0C, 0x0D18, 0x0D24, 0x0D30, 0x0D3C, 0x0D48]
        code = 0
        if idens:
            for i, data in enumerate(idens):
                # print(i, data)
                for j, d in enumerate(data):
                    ret = self.arm_cmd.base_tool_addr_w32(servo_id, addr=(cmds[i] + (2 * j)) | 0x1000, value=d)
                    # print("%x, %f, ret:%d" % (cmds[i] + (2 * j), d, ret[0]))

                    time.sleep(0.1)
                    code = ret[0]
                    if code != 0:
                        return code
            return code
        else:
            return 1

    def get_imu_data(self, board_id=10):
        ret = self.arm_cmd.base_tool_addr_w16(board_id, addr=0x0606, value=1)
        if ret[0] == 0:
            ret1 = self.arm_cmd.base_tool_addr_r32(board_id, addr=0x0C00)
            ret2 = self.arm_cmd.base_tool_addr_r32(board_id, addr=0x0C02)
            ret3 = self.arm_cmd.base_tool_addr_r32(board_id, addr=0x0C04)
            if ret1[0] == 0 and ret2[0] == 0 and ret3[0] == 0:
                acc_x = ret1[1]
                acc_y = ret2[1]
                acc_z = ret3[1]
                if acc_x != 0 and acc_y != 0 and acc_z != 0:
                    return 0, acc_x, acc_y, acc_z

        return [1, 1, 1, 1]
