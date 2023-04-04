#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2023, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
from .base import Base
from .decorator import xarm_is_connected


class ModbusTcp(Base):
    def __init__(self):
        super(ModbusTcp, self).__init__()
    
    @xarm_is_connected(_type='get')
    def read_coil_bits(self, addr, quantity):
        """
        func_code: 0x01
        """
        return self.arm_cmd.read_coil_bits(addr, quantity)

    @xarm_is_connected(_type='get')
    def read_input_bits(self, addr, quantity):
        """
        func_code: 0x02
        """
        return self.arm_cmd.read_input_bits(addr, quantity)
    
    @xarm_is_connected(_type='get')
    def read_holding_registers(self, addr, quantity, is_signed=False):
        """
        func_code: 0x03
        """
        return self.arm_cmd.read_holding_registers(addr, quantity, is_signed)
    
    @xarm_is_connected(_type='get')
    def read_input_registers(self, addr, quantity, is_signed=False):
        """
        func_code: 0x04
        """
        return self.arm_cmd.read_input_registers(addr, quantity, is_signed)
    
    @xarm_is_connected(_type='set')
    def write_single_coil_bit(self, addr, bit_val):
        """
        func_code: 0x05
        """
        return self.arm_cmd.write_single_coil_bit(addr, bit_val)
    
    @xarm_is_connected(_type='set')
    def write_single_holding_register(self, addr, reg_val):
        """
        func_code: 0x06
        """
        return self.arm_cmd.write_single_holding_register(addr, reg_val)

    @xarm_is_connected(_type='set')
    def write_multiple_coil_bits(self, addr, bits):
        """
        func_code: 0x0F
        """
        return self.arm_cmd.write_multiple_coil_bits(addr, bits)

    @xarm_is_connected(_type='set')
    def write_multiple_holding_registers(self, addr, regs):
        """
        func_code: 0x10
        """
        return self.arm_cmd.write_multiple_holding_registers(addr, regs)

    @xarm_is_connected(_type='set')
    def mask_write_holding_register(self, addr, and_mask, or_mask):
        """
        func_code: 0x16
        """
        return self.arm_cmd.mask_write_holding_register(addr, and_mask, or_mask)

    @xarm_is_connected(_type='get')
    def write_and_read_holding_registers(self, r_addr, r_quantity, w_addr, w_regs, is_signed=False):
        return self.arm_cmd.mask_write_holding_register(r_addr, r_quantity, w_addr, w_regs, is_signed)
