#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import re
from ._blockly_node import _BlocklyNode

OPS_MAP = {
    'EQ': '==',
    'NEQ': '!=',
    'LT': '<',
    'LTE': '<=',
    'GT': '>',
    'GTE': '>=',
    '===': '==',
    '!==': '!=',
    '>=': '>=',
    '>': '>',
    '<=': '<=',
    '<': '<',
}


class _BlocklyBase(_BlocklyNode):
    def __init__(self, xml_path):
        super(_BlocklyBase, self).__init__(xml_path)
        self._funcs = {}
        self._define_is_prime_func = False
        self._define_bin_matchs_func = False
    
    def _get_field_value(self, block):
        field = self._get_node('field', root=block)
        if field is not None:
            return field.text
        else:
            return self._get_nodes('field', root=self._get_node('value', root=block), descendant=True)[0].text

    def _get_block_val(self, block, arg_map=None):
        block_v = self._get_node('block', root=block)
        if block_v is not None:
            val = self._get_condition_expression(block, arg_map=arg_map)
        else:
            shadow = self._get_node('shadow', root=block)
            val = self._get_node('field', root=shadow).text
        return val

    def _get_condition_expression(self, value_block, arg_map=None):
        block = self._get_node('block', value_block)
        if block is None:
            shadow = self._get_node('shadow', root=value_block)
            return self._get_node('field', root=shadow).text
        if block.attrib['type'] == 'logic_boolean':
            return str(self._get_node('field', block).text == 'TRUE')
        elif block.attrib['type'] == 'logic_compare':
            return self.__get_logic_compare(block, arg_map=arg_map)
        elif block.attrib['type'] == 'logic_operation':
            return self.__get_logic_operation(block, arg_map=arg_map)
        elif block.attrib['type'] == 'logic_negate':
            value = self._get_node('value', root=block)
            return 'not ({})'.format(self._get_condition_expression(value, arg_map=arg_map))
        elif block.attrib['type'] == 'gpio_get_digital':
            io = self._get_node('field', block).text
            return 'self._arm.get_tgpio_digital({})[{}]'.format(io, 1)
        elif block.attrib['type'] == 'gpio_get_analog':
            io = self._get_node('field', block).text
            return 'self._arm.get_tgpio_analog({})[{}]'.format(io, 1)
        elif block.attrib['type'] == 'gpio_get_controller_digital':
            io = self._get_node('field', block).text
            return 'self._arm.get_cgpio_digital({})[{}]'.format(io, 1)
        elif block.attrib['type'] == 'gpio_get_controller_digital_di':
            io = self._get_node('field', block).text
            return 'self._arm.get_cgpio_digital({})[{}]'.format(io, 1)
        elif block.attrib['type'] == 'gpio_get_controller_analog':
            io = self._get_node('field', block).text
            return 'self._arm.get_cgpio_analog({})[{}]'.format(io, 1)
        elif block.attrib['type'] == 'gpio_get_ci':
            io = self._get_node('field', block).text
            return '1 if self._cgpio_state is None else self._cgpio_state[3] >> {} & 0x0001'.format(io)
        elif block.attrib['type'] == 'gpio_get_co':
            io = self._get_node('field', block).text
            return '0 if self._cgpio_state is None else self._cgpio_state[5] >> {} & 0x0001'.format(io)
        elif block.attrib['type'] == 'gpio_get_ai':
            io = self._get_node('field', block).text
            return '0 if self._cgpio_state is None else self._cgpio_state[6 + {}]'.format(io)
        elif block.attrib['type'] == 'gpio_get_ao':
            io = self._get_node('field', block).text
            return '0 if self._cgpio_state is None else self._cgpio_state[8 + {}]'.format(io)
        elif block.attrib['type'] == 'gpio_match_controller_digitals_bin':
            bin_val = self._get_node('field', block).text
            self._define_bin_matchs_func = True
            return 'self._cgpio_digitals_is_matchs_bin(\'{}\')'.format(bin_val)
        elif block.attrib['type'] == 'get_suction_cup':
            return 'self._arm.get_suction_cup()[{}]'.format(1)
        elif block.attrib['type'] == 'check_air_pump_state':
            fields = self._get_nodes('field', root=block)
            state = 1 if fields[0].text == 'ON' else 0
            timeout = float(fields[1].text)
            return 'self._arm.arm.check_air_pump_state({}, timeout={})'.format(state, timeout)
        elif block.attrib['type'] == 'check_bio_gripper_is_catch':
            fields = self._get_nodes('field', root=block)
            timeout = float(fields[0].text)
            return 'self._arm.arm.check_bio_gripper_is_catch(timeout={})'.format(timeout)
        elif block.attrib['type'] == 'check_robotiq_is_catch':
            fields = self._get_nodes('field', root=block)
            timeout = float(fields[0].text)
            return 'self._arm.arm.check_robotiq_is_catch(timeout={})'.format(timeout)
        elif block.attrib['type'] == 'math_number':
            val = self._get_node('field', block).text
            return val
        elif block.attrib['type'] == 'math_arithmetic':
            return self.__get_math_arithmetic(block, arg_map=arg_map)
        elif block.attrib['type'] == 'math_number_property':
            return self.__get_math_number_property(block, arg_map=arg_map)
        elif block.attrib['type'] == 'math_random_int':
            return self.__get_math_random_int(block, arg_map=arg_map)
        elif block.attrib['type'] == 'math_round':
            return self.__get_math_round(block, arg_map=arg_map)
        elif block.attrib['type'] == 'math_single':
            # 算术函数
            return self.__get_math_single(block, arg_map=arg_map)
        elif block.attrib['type'] == 'math_trig':
            # 三角函数
            return self.__get_math_trig(block, arg_map=arg_map)
        elif block.attrib['type'] == 'math_constant':
            # 常量
            return self.__get_math_constant(block, arg_map=arg_map)
        elif block.attrib['type'] == 'math_modulo':
            return self.__get_math_modulo(block, arg_map=arg_map)
        elif block.attrib['type'] == 'math_constrain':
            return self.__get_math_constrain(block, arg_map=arg_map)
        elif block.attrib['type'] == 'variables_get':
            field = self._get_node('field', block).text
            if arg_map and field in arg_map:
                return '{}'.format(arg_map[field])
            else:
                return 'self._vars.get(\'{}\', 0)'.format(field)
        elif block.attrib['type'] == 'move_var':
            val = self._get_node('field', block).text
            return val
        elif block.attrib['type'] == 'tool_get_date':
            return 'datetime.datetime.now()'
        elif block.attrib['type'] == 'tool_combination':
            field = self._get_node('field', block).text
            values = self._get_nodes('value', block)
            var1 = self._get_condition_expression(values[0], arg_map=arg_map)
            var2 = self._get_condition_expression(values[1], arg_map=arg_map)
            return '\'{{}}{{}}{{}}\'.format({}, \'{}\', {})'.format(var1, field, var2)
        elif block.attrib['type'] == 'procedures_callreturn':
            mutation = self._get_node('mutation', block).attrib['name']
            if not mutation:
                mutation = '1'
            if mutation in self._funcs:
                name = self._funcs[mutation]
            else:
                name = 'function_{}'.format(len(self._funcs) + 1)

            args = self._get_nodes('arg', root=self._get_node('mutation', block))
            values = self._get_nodes('value', root=block)
            if args and values and len(args) == len(values):
                return 'self.{}({})'.format(name, ','.join(
                    [self._get_condition_expression(val, arg_map=arg_map) for val in values]))
            else:
                return 'self.{}()'.format(name)
        elif block.attrib['type'] == 'python_expression':
            ret = self._get_node('field', block).text
            return ret
        elif block.attrib['type'] == 'gpio_get_controller_ci_li':
            fields = self._get_nodes('field', root=block)
            values = []
            for field in fields[:-1]:
                values.append(int(field.text))
            timeout = float(fields[-1].text)
            return 'self._arm.arm.get_cgpio_li_state({}, timeout={}, is_ci=True)'.format(values, timeout)
        elif block.attrib['type'] == 'gpio_get_controller_di_li':
            fields = self._get_nodes('field', root=block)
            values = []
            for field in fields[:-1]:
                values.append(int(field.text))
            timeout = float(fields[-1].text)
            return 'self._arm.arm.get_cgpio_li_state({}, timeout={}, is_ci=False)'.format(values, timeout)
        elif block.attrib['type'] == 'gpio_get_tgpio_li':
            fields = self._get_nodes('field', root=block)
            values = []
            for field in fields[:-1]:
                values.append(int(field.text))
            timeout = float(fields[-1].text)
            return 'self._arm.arm.get_tgpio_li_state({}, timeout={})'.format(values, timeout)
        elif block.attrib['type'] == 'get_modbus_rtu':
            fields = self._get_nodes('field', root=block)
            host_id = fields[0].text
            cmd = fields[1].text
            cmd_li = re.sub(',', ' ', cmd)
            is_run_cmd = ''.join(cmd_li.split()).isalnum()
            if not is_run_cmd:
                self._append_main_code('-1', indent + 2)
            cmd_li = re.sub(' +', ' ', cmd_li)
            cmd_li = re.sub('\xa0', ' ', cmd_li)
            cmd_li = re.sub('\s+', ' ', cmd_li)
            cmd_li = cmd_li.strip().split(' ')
            int_li = [int(da, 16) for da in cmd_li]
            return '[" ".join([hex(da).split("0x")[1].upper().zfill(2) for da in data]) if isinstance(data, list) else data for ' \
                   'data in self._arm.getset_tgpio_modbus_data({}, host_id={})][-1]'.format(int_li, host_id)
        elif block.attrib['type'] == 'get_gripper_status':
            fields = self._get_nodes('field', root=block)
            timeout = fields[0].text
            return '0 == self._arm.arm.check_catch_gripper_status({})'.format(timeout)
        elif block.attrib['type'] == 'get_ft_sensor':
            direction_li = ['Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz']
            fields = self._get_nodes('field', root=block)
            direction = fields[0].text
            return '[self._arm.ft_sensor_enable(1), self._arm.set_state(0), self._arm.get_ft_sensor_data()[1][{}]][2] if' \
                   ' self._arm.get_ft_sensor_config()[1][1] == 0 else self._arm.get_ft_sensor_data()[1][{}]'.\
                format(direction_li.index(direction), direction_li.index(direction))
        elif block.attrib['type'] == 'get_counter':
            return 'self._arm.count'


    def __get_logic_compare(self, block, arg_map=None):
        op = OPS_MAP.get(self._get_node('field', block).text)
        cond_a = 0
        cond_b = 0
        values = self._get_nodes('value', block)
        if len(values) > 0:
            cond_a = self._get_condition_expression(values[0], arg_map=arg_map)
            if len(values) > 1:
                cond_b = self._get_condition_expression(values[1], arg_map=arg_map)
        return '{} {} {}'.format(cond_a, op, cond_b)

    def __get_logic_operation(self, block, arg_map=None):
        op = self._get_node('field', block).text.lower()
        cond_a = False
        cond_b = False
        values = self._get_nodes('value', block)
        if len(values) > 0:
            cond_a = self._get_condition_expression(values[0], arg_map=arg_map)
            if len(values) > 1:
                cond_b = self._get_condition_expression(values[1], arg_map=arg_map)
        return '{} {} {}'.format(cond_a, op, cond_b)

    def __get_math_arithmetic(self, block, arg_map=None):
        field = self._get_node('field', block).text
        values = self._get_nodes('value', block)
        if len(values) > 1:
            val_a = self._get_block_val(values[0], arg_map=arg_map)
            val_b = self._get_block_val(values[1], arg_map=arg_map)
            if field == 'ADD':
                return '({} + {})'.format(val_a, val_b)
            elif field == 'MINUS':
                return '({} - {})'.format(val_a, val_b)
            elif field == 'MULTIPLY':
                return '({} * {})'.format(val_a, val_b)
            elif field == 'DIVIDE':
                return '({} / {})'.format(val_a, val_b)
            elif field == 'POWER':
                return 'pow({}, {})'.format(val_a, val_b)

    def __get_math_number_property(self, block, arg_map=None):
        field = self._get_node('field', block).text
        values = self._get_nodes('value', block)
        if len(values) >= 1:
            val_a = self._get_block_val(values[0], arg_map=arg_map)

            if field == 'EVEN':
                # 偶数
                return '{} % 2 == 0'.format(val_a)
            elif field == 'ODD':
                # 奇数
                return '{} % 2 == 1'.format(val_a)
            elif field == 'PRIME':
                # 质数
                self._define_is_prime_func = True
                return 'self.is_prime({})'.format(val_a)
            elif field == 'WHOLE':
                # 整数
                return '{} % 1 == 0'.format(val_a)
            elif field == 'POSITIVE':
                # 正数
                return '{} > 0'.format(val_a)
            elif field == 'NEGATIVE':
                # 负数
                return '{} < 0'.format(val_a)
            elif field == 'DIVISIBLE_BY':
                # 可被整除
                if len(values) > 1:
                    val_b = self._get_block_val(values[1], arg_map=arg_map)
                else:
                    val_b = 0
                return '{} % {} == 0'.format(val_a, val_b)

    def __get_math_random_int(self, block, arg_map=None):
        values = self._get_nodes('value', block)
        if len(values) > 1:
            val_a = self._get_block_val(values[0], arg_map=arg_map)
            val_b = self._get_block_val(values[1], arg_map=arg_map)
            return 'random.randint({}, {})'.format(val_a, val_b)

    def __get_math_round(self, block, arg_map=None):
        field = self._get_node('field', block).text
        values = self._get_nodes('value', block)
        if len(values) >= 1:
            val_a = self._get_block_val(values[0], arg_map=arg_map)
            if field == 'ROUND':
                # 四舍五入
                return 'round({})'.format(val_a)
            elif field == 'ROUNDUP':
                # 上舍入
                return 'math.ceil({})'.format(val_a)
            elif field == 'ROUNDDOWN':
                # 下舍入
                return 'math.floor({})'.format(val_a)

    def __get_math_single(self, block, arg_map=None):
        # 算术函数
        field = self._get_node('field', block).text
        values = self._get_nodes('value', block)
        if len(values) >= 1:
            val_a = self._get_block_val(values[0], arg_map=arg_map)
            if field == 'ROOT':
                # 平方根
                return 'math.sqrt({})'.format(val_a)
            elif field == 'ABS':
                # 绝对值
                return 'abs({})'.format(val_a)
            elif field == 'NEG':
                # 相反数
                return '-{}'.format(val_a)
            elif field == 'LN':
                # ln
                return 'math.log({})'.format(val_a)
            elif field == 'LOG10':
                # log10
                return '(math.log({}) / math.log(10))'.format(val_a)
            elif field == 'EXP':
                # exp
                return 'math.exp({})'.format(val_a)
            elif field == 'POW10':
                # 10的多少次方
                return 'math.pow(10, {})'.format(val_a)

    def __get_math_trig(self, block, arg_map=None):
        # 三角函数
        field = self._get_node('field', block).text
        values = self._get_nodes('value', block)
        if len(values) >= 1:
            val_a = self._get_block_val(values[0], arg_map=arg_map)
            if field == 'SIN':
                return 'math.sin({})'.format(val_a)
            elif field == 'COS':
                return 'math.cos({})'.format(val_a)
            elif field == 'TAN':
                return 'math.tan({})'.format(val_a)
            elif field == 'ASIN':
                return 'math.asin({})'.format(val_a)
            elif field == 'ACOS':
                return 'math.acos({})'.format(val_a)
            elif field == 'ATAN':
                return 'math.atan({})'.format(val_a)

    def __get_math_constant(self, block, arg_map=None):
        field = self._get_node('field', block).text
        if field == 'PI':
            return 'math.pi'
        elif field == 'E':
            return 'math.e'
        elif field == 'GOLDEN_RATIO':
            return '(1 + math.sqrt(5)) / 2'
        elif field == 'SQRT2':
            return 'math.sqrt(2)'
        elif field == 'SQRT1_2':
            return 'math.sqrt(0.5)'
        elif field == 'INFINITY':
            return 'math.inf'

    def __get_math_modulo(self, block, arg_map=None):
        values = self._get_nodes('value', block)
        if len(values) > 1:
            val_a = self._get_block_val(values[0], arg_map=arg_map)
            val_b = self._get_block_val(values[1], arg_map=arg_map)
            return '{} % {}'.format(val_a, val_b)

    def __get_math_constrain(self, block, arg_map=None):
        values = self._get_nodes('value', block)
        if len(values) > 2:
            val_a = self._get_block_val(values[0], arg_map=arg_map)
            val_b = self._get_block_val(values[1], arg_map=arg_map)
            val_c = self._get_block_val(values[2], arg_map=arg_map)
            return 'min(max({}, {}), {})'.format(val_a, val_b, val_c)
