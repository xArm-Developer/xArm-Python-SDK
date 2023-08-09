# -*- coding: utf-8 -*-
# __author: rock
# @time: 2021-04-02

import math
import time
from ..core.utils.log import logger
from .base import Base
from .decorator import xarm_is_connected


class BaseBoard(Base):

    def __init__(self):
        super(BaseBoard, self).__init__()

    @xarm_is_connected(_type='get')
    def get_base_board_version(self, board_id=10):
        versions = ['*', '*', '*']

        ret1 = self.arm_cmd.tgpio_addr_r16(0x0801, bid=board_id)
        ret2 = self.arm_cmd.tgpio_addr_r16(0x0802, bid=board_id)
        ret3 = self.arm_cmd.tgpio_addr_r16(0x0803, bid=board_id)

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

    @xarm_is_connected(_type='get')
    def get_current_angle(self, board_id=10):
        code, acc_xyz = self.get_imu_data(board_id)
        self.arm_cmd.tgpio_addr_w16(addr=0x0606, value=1, bid=board_id)
        if code == 0 and acc_xyz[0] != 0 and acc_xyz[1] != 0 and acc_xyz[2] != 0:
            angle = self.__get_z_axios_offset_angle(acc_xyz[0], acc_xyz[1], acc_xyz[2])
            return code, angle
        else:
            return code, 0

    @staticmethod
    def __get_z_axios_offset_angle(x=1, y=1, z=1):
        angle = math.degrees(math.atan(z / (math.sqrt(abs((x ** 2) + (y ** 2))))))
        angle = 90 - angle
        return angle

    @xarm_is_connected(_type='set')
    def write_sn(self, sn='', servo_id=10):
        code = 0
        if len(sn) == 14:
            for i in range(0, 14, 2):
                ret = self.arm_cmd.tgpio_addr_w16(addr=0x1900 + (int(i / 2)), value=ord(sn[i]) | ord(sn[i + 1]) << 8, bid=servo_id)
                code = self._check_code(ret[0])
                time.sleep(0.1)
                if code != 0:
                    self.log_api_info('API -> write_sn -> code={}, sn={}'.format(code, sn), code=code)
                    return code
        self.log_api_info('API -> write_sn -> code={}, sn={}'.format(code, sn), code=code)
        return code

    @xarm_is_connected(_type='get')
    def get_sn(self, servo_id=10):
        rd_sn = ''
        ret = [0, '']
        for i in range(0, 14, 2):
            ret = self.arm_cmd.tgpio_addr_r16(addr=0x0900 + (int(i / 2)), bid=servo_id)
            time.sleep(0.1)
            rd_sn = ''.join([rd_sn, chr(ret[1] & 0x00FF)])
            rd_sn = ''.join([rd_sn, chr((ret[1] >> 8) & 0x00FF)])
            ret[0] = self._check_code(ret[0])
            if ret[0] != 0:
                self.log_api_info('API -> get_sn -> code={}, sn={}'.format(ret[0], rd_sn), code=ret[0])
                return ret[0], ''
        self.log_api_info('API -> get_sn -> code={}, sn={}'.format(ret[0], rd_sn), code=ret[0])
        return ret[0], rd_sn

    @xarm_is_connected(_type='set')
    def write_iden_to_base(self, idens, servo_id=10):
        cmds = [0x0D00, 0x0D0C, 0x0D18, 0x0D24, 0x0D30, 0x0D3C, 0x0D48]
        code = 0
        if idens:
            for i, data in enumerate(idens):
                # print(i, data)
                for j, d in enumerate(data):
                    ret = self.arm_cmd.tgpio_addr_w32(addr=(cmds[i] + (2 * j)) | 0x1000, value=d, bid=servo_id)
                    # print("%x, %f, ret:%d" % (cmds[i] + (2 * j), d, ret[0]))

                    time.sleep(0.1)
                    code = ret[0]
                    if code != 0:
                        return code
            return code
        else:
            return 1

    @xarm_is_connected(_type='get')
    def get_imu_data(self, board_id=10):
        code = 0
        if board_id == 9:
            self.arm_cmd.tgpio_addr_w16(addr=0x0606, value=1, bid=board_id)

        ret1 = self.arm_cmd.tgpio_addr_r32(addr=0x0C00, bid=board_id, fmt='>f')
        ret2 = self.arm_cmd.tgpio_addr_r32(addr=0x0C02, bid=board_id, fmt='>f')
        ret3 = self.arm_cmd.tgpio_addr_r32(addr=0x0C04, bid=board_id, fmt='>f')
        code = 0 if ret1[0] == 0 else ret1[0]
        code = code if ret2[0] == 0 else ret2[0]
        code = code if ret3[0] == 0 else ret3[0]

        if board_id == 9:
            self.arm_cmd.tgpio_addr_w16(addr=0x0606, value=0, bid=board_id)
        if code != 0:
            return code, [1, 1, 1]
        else:
            return code, [ret1[1], ret2[1], ret3[1]]

    @xarm_is_connected(_type='get')
    def read_iden_from_base(self, servo_id=10):
        cmds = [0x0D00, 0x0D0C, 0x0D18, 0x0D24, 0x0D30, 0x0D3C, 0x0D48]
        code = 0
        conf = []
        for i in range(7):
            vl = []
            for j in range(6):
                ret = self.arm_cmd.tgpio_addr_r32((cmds[i] + (2 * j)), servo_id, fmt='>f')
                time.sleep(0.01)
                # print("%x, %f, ret:%d" % (cmds[i] + (2 * j), ret[1], ret[0]))
                vl.append(ret[1])
                code = ret[0]
                if code != 0:
                    return code, None
            conf.append(vl)
        return code, conf
        
    @xarm_is_connected(_type='set')
    def write_poe_to_end(self, datas, servo_id=9):
        cmds1 = [0x0E00, 0x0E0C, 0x0E18, 0x0E24, 0x0E30, 0x0E3C, 0x0E48]
        cmds2 = 0x0F00
        code = 0
        if datas:
            joints, homes = datas
            for i, data in enumerate(joints):
                for j, d in enumerate(data):
                    ret = self.arm_cmd.tgpio_addr_w32(addr=(cmds1[i] + (2 * j)) | 0x1000, value=d, bid=servo_id)
                    time.sleep(0.1)
                    code = ret[0]
                    if code != 0:
                        return code
            for i, data in enumerate(homes):
                for j, d in enumerate(data):
                    ret = self.arm_cmd.tgpio_addr_w32(addr=(cmds2 + (2 * (i * len(data) + j))) | 0x1000, value=d, bid=servo_id)
                    time.sleep(0.1)
                    code = ret[0]
                    if code != 0:
                        return code
            return code
        else:
            return 1

    @xarm_is_connected(_type='get')
    def read_poe_from_end(self, servo_id=9):
        cmds = [0x0E00, 0x0E0C, 0x0E18, 0x0E24, 0x0E30, 0x0E3C, 0x0E48]
        cmds2 = 0x0F00
        code = 0
        conf = []
        joints_conf = []
        homes_conf = []
        for i in range(7):
            vl = []
            for j in range(6):
                ret = self.arm_cmd.tgpio_addr_r32(cmds[i] + (2 * j), servo_id, fmt='>f')
                time.sleep(0.01)
                vl.append(ret[1])
                code = ret[0]
                if code != 0:
                    return code, None
            joints_conf.append(vl)
        for i in range(4):
            vl = []
            for j in range(4):
                ret = self.arm_cmd.tgpio_addr_r32(cmds2 + 2 * (i * 4 + j), servo_id, fmt='>f')
                time.sleep(0.01)
                vl.append(ret[1])
                code = ret[0]
                if code != 0:
                    return code, None
            homes_conf.append(vl)
        conf = [joints_conf, homes_conf]
        return code, conf
