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



