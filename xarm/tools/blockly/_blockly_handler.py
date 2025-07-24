#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import json
import re
import math
from ._blockly_base import _BlocklyBase, OPS_MAP
from ._blockly_highlight import HIGHLIGHT_BLOCKS


class _BlocklyHandler(_BlocklyBase):
    def __init__(self, xml_path):
        super(_BlocklyHandler, self).__init__(xml_path)
        self._succeed = True
        self._highlight_callback = None
        self._loop_interval_sec = -1
        self._init_code_list = []
        self._main_init_code_list = []
        self._main_func_code_list = []
        self._main_run_code_list = []
        self._is_main_run_code = True
        self._is_exec = False
        self._is_ide = False

        self._listen_tgpio_digital = False
        self._listen_tgpio_analog = False
        self._listen_cgpio_state = False
        self._listen_count = False
        self._listen_holding = False
        self._tgpio_digital_callbacks = []
        self._tgpio_analog_callbacks = []
        self._cgpio_digital_callbacks = []
        self._cgpio_analog_callbacks = []
        self._count_callbacks= []
        self._holding_callbacks= []

        self.is_run_linear_track = False
        self._is_run_blockly = False
        self._listen_counter = False
        self._vacuum_version = '1'
    
    def _append_init_code(self, code):
        self._init_code_list.append(code)

    def _append_main_init_code(self, code, indent=0):
        self._main_init_code_list.append('{}{}'.format('    ' * indent, code))
    
    def _append_main_code(self, code, indent=0):
        if self._is_main_run_code:
            self._main_run_code_list.append('{}{}'.format('    ' * (indent + 1), code))
        else:
            self._main_func_code_list.append('{}{}'.format('    ' * indent, code))
    
    def _parse_variables(self):
        variables = self._get_nodes('variables')
        var_list = []
        for var in variables:
            for variable in self._get_nodes('variable', root=var):
                var_list.append(variable.text)
        return var_list
    
    def _parse_block(self, root=None, indent=0, arg_map=None, **kwargs):
        blocks = self._get_nodes('block', root=root)
        for block in blocks:
            # if not self._succeed:
            #     return
            is_statement = root is None or root.tag == self._ns + 'statement'
            while block is not None:
                if not is_statement:
                    block = self._get_node('next', root=block)
                    if not block:
                        break
                    block = self._get_node('block', root=block)
                else:
                    is_statement = False
                if block.attrib.get('disabled', False):
                    continue
                func = getattr(self, '_handle_{}'.format(block.attrib['type']), None)
                if func and callable(func):
                    if self._highlight_callback is not None:
                        if block.attrib['type'] in HIGHLIGHT_BLOCKS:
                            self._append_main_code('highlight_callback(\'{}\')'.format(block.attrib['id']), indent=indent+2)
                    try:
                        func(block, indent, arg_map=arg_map, **kwargs)
                    except Exception as e:
                        self._succeed = False
                        print('convert {} failed, {}'.format(block.attrib['type'], e))
                else:
                    self._succeed = False
                    print('block {} can\'t convert to python code'.format(block.attrib['type']))
    
    def _handle_set_speed(self, block, indent=0, arg_map=None):
        self._append_main_code('self._tcp_speed = {}'.format(self._get_field_value(block)), indent + 2)

    def _handle_set_acceleration(self, block, indent=0, arg_map=None):
        self._append_main_code('self._tcp_acc = {}'.format(self._get_field_value(block)), indent + 2)

    def _handle_set_angle_speed(self, block, indent=0, arg_map=None):
        self._append_main_code('self._angle_speed = {}'.format(self._get_field_value(block)), indent + 2)

    def _handle_set_angle_acceleration(self, block, indent=0, arg_map=None):
        self._append_main_code('self._angle_acc = {}'.format(self._get_field_value(block)), indent + 2)

    def _handle_set_counter_increase(self, block, indent=0, arg_map=None):
        self._listen_counter = True
        self._append_main_code('code = self._arm.set_counter_increase()', indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_counter_increase\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_counter_reset(self, block, indent=0, arg_map=None):
        self._listen_counter = True
        self._append_main_code('code = self._arm.set_counter_reset()', indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_counter_reset\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_reset(self, block, indent=0, arg_map=None):
        self._append_main_code('self._arm.move_gohome()', indent + 2)

    def _handle_sleep(self, block, indent=0, arg_map=None):
        value = self._get_node('value', root=block)
        value = self._get_block_val(value, arg_map=arg_map)
        self._append_main_code('code = self._arm.set_pause_time({})'.format(value), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_pause_time\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_move(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        orientation = fields[0].text
        value = fields[1].text if orientation == 'forward' or orientation == 'left' or orientation == 'up' else '-{}'.format(fields[1].text)
        wait = fields[2].text == 'TRUE'
        param = 'x' if orientation == 'forward' or orientation == 'backward' else 'y' if orientation == 'left' or orientation == 'right' else 'z' if orientation == 'up' or orientation == 'down' else None
        if param is None:
            return
        radius = -1 if wait else 0
        self._append_main_code('code = self._arm.set_position({}={}, radius={}, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True, wait={})'.format(param, value, radius, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_position\'):', indent + 2)
        self._append_main_code('    return', indent + 2)
    
    def _handle_move_variable(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        orientation = fields[0].text
        wait = fields[1].text == 'TRUE'
        value = self._get_node('value', root=block)
        value = self._get_block_val(value, arg_map=arg_map)
        value = value if orientation == 'forward' or orientation == 'left' or orientation == 'up' else '-{}'.format(
            value)
        param = 'x' if orientation == 'forward' or orientation == 'backward' else 'y' if orientation == 'left' or orientation == 'right' else 'z' if orientation == 'up' or orientation == 'down' else None
        if param is None:
            return
        radius = -1 if wait else 0
        self._append_main_code(
            'code = self._arm.set_position({}={}, radius={}, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True, wait={})'.format(
                param, value, radius, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_position\'):', indent + 2)
        self._append_main_code('    return', indent + 2)
    
    def _handle_move_arc_to(self, block, indent=0, arg_map=None):
        value = self._get_node('value', root=block)
        p_block = self._get_node('block', root=value)
        fields = self._get_nodes('field', root=p_block)
        values = []
        for field in fields[:-2]:
            values.append(float(field.text))
        radius = float(fields[-2].text)
        wait = fields[-1].text == 'TRUE'
        self._append_main_code('code = self._arm.set_position(*{}, speed=self._tcp_speed, mvacc=self._tcp_acc, radius={}, wait={})'.format(values, radius, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_position\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_move_circle(self, block, indent=0, arg_map=None):
        values = self._get_nodes('value', root=block)
        percent = self._get_block_val(values[2], arg_map=arg_map)
        wait = self._get_block_val(values[3], arg_map=arg_map)
        if wait == 'TRUE' or wait == 'FALSE':
            wait = wait == 'TRUE'
        p1_block = self._get_node('block', root=values[0])
        fields = self._get_nodes('field', root=p1_block)
        pose1 = []
        for field in fields:
            pose1.append(float(field.text))
        p2_block = self._get_node('block', root=values[1])
        fields = self._get_nodes('field', root=p2_block)
        pose2 = []
        for field in fields:
            pose2.append(float(field.text))
        self._append_main_code('code = self._arm.move_circle({}, {}, float({}) / 360 * 100, speed=self._tcp_speed, mvacc=self._tcp_acc, wait={})'.format(pose1, pose2, percent, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'move_circle\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_move_7(self, block, indent=0, arg_map=None):
        value = self._get_node('value', root=block)
        p_block = self._get_node('block', root=value)
        fields = self._get_nodes('field', root=p_block)
        values = []
        for field in fields[:-1]:
            values.append(float(field.text))
        wait = fields[-1].text == 'TRUE'
        self._append_main_code('code = self._arm.set_servo_angle(angle={}, speed=self._angle_speed, mvacc=self._angle_acc, wait={})'.format(values, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_servo_angle\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_move_joints(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        values = []
        for field in fields[:-1]:
            values.append(float(field.text))
        radius_fields = self._get_nodes('field', root=block, name='r')
        if len(radius_fields) > 0:
            radius = values[-1]
            values = values[:-1]
        else:
            radius = None
        wait = fields[-1].text == 'TRUE'
        self._append_main_code('code = self._arm.set_servo_angle(angle={}, speed=self._angle_speed, mvacc=self._angle_acc, wait={}, radius={})'.format(values, wait, radius), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_servo_angle\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_move_cartesian(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        values = []
        for field in fields[:-2]:
            values.append(float(field.text))
        radius = float(fields[-2].text)
        wait = fields[-1].text == 'TRUE'
        self._append_main_code('code = self._arm.set_position(*{}, speed=self._tcp_speed, mvacc=self._tcp_acc, radius={}, wait={})'.format(values, radius, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_position\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_move_tool_line(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        values = []
        for field in fields[:-1]:
            values.append(float(field.text))
        wait = fields[-1].text == 'TRUE'
        self._append_main_code('code = self._arm.set_tool_position(*{}, speed=self._tcp_speed, mvacc=self._tcp_acc, wait={})'.format(values, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_position\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_move_joints_var(self, block, indent=0, arg_map=None):
        field = self._get_node('field', root=block)
        wait = field.text == 'TRUE'
        value_nodes = self._get_nodes('value', root=block)
        values = []
        for val_node in value_nodes:
            val = self._get_condition_expression(val_node, arg_map=arg_map)
            values.append(val)
        radius_fields = self._get_nodes('value', root=block, name='r')
        if len(radius_fields) > 0:
            radius = values[-1]
            values = values[:-1]
        else:
            radius = None
        values = '[{}]'.format(','.join(values))
        self._append_main_code('code = self._arm.set_servo_angle(angle={}, speed=self._angle_speed, mvacc=self._angle_acc, wait={}, radius={})'.format(values, wait, radius), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_servo_angle\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_move_cartesian_var(self, block, indent=0, arg_map=None):
        field = self._get_node('field', root=block)
        wait = field.text == 'TRUE'
        value_nodes = self._get_nodes('value', root=block)
        values = []
        for val_node in value_nodes:
            val = self._get_condition_expression(val_node, arg_map=arg_map)
            values.append(val)
        radius = values.pop()
        values = '[{}]'.format(','.join(values))
        self._append_main_code('code = self._arm.set_position(*{}, speed=self._tcp_speed, mvacc=self._tcp_acc, radius={}, wait={})'.format(values, radius, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_position\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_move_circle_var(self, block, indent=0, arg_map=None):
        values = self._get_nodes('value', root=block)
        percent = self._get_block_val(values[2], arg_map=arg_map)
        wait = self._get_block_val(values[3], arg_map=arg_map)
        if wait == 'TRUE' or wait == 'FALSE':
            wait = wait == 'TRUE'
        
        pose1_block= self._get_node('block', root=values[0])
        pose1_nodes = self._get_nodes('value', root=pose1_block)
        pose1 = []
        for val_node in pose1_nodes:
            val = self._get_condition_expression(val_node, arg_map=arg_map)
            pose1.append(val)

        pose2_block = self._get_node('block', root=values[1])
        pose2_nodes = self._get_nodes('value', root=pose2_block)
        pose2 = []
        for val_node in pose2_nodes:
            val = self._get_condition_expression(val_node, arg_map=arg_map)
            pose2.append(val)

        pose1 = '[{}]'.format(', '.join(pose1))
        pose2 = '[{}]'.format(', '.join(pose2))
        self._append_main_code('code = self._arm.move_circle({}, {}, float({}) / 360 * 100, speed=self._tcp_speed, mvacc=self._tcp_acc, wait={})'.format(pose1, pose2, percent, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'move_circle\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_move_axis_angle(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        values = []
        for field in fields[:-2]:
            values.append(float(field.text))
        radius = float(fields[-2].text)
        wait = fields[-1].text == 'TRUE'
        self._append_main_code(
            'code = self._arm.set_position_aa({}, speed=self._tcp_speed, mvacc=self._tcp_acc, radius={}, wait={})'.format(
                values, radius, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_position_aa\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_move_axis_angle_variable(self, block, indent=0, arg_map=None):
        field = self._get_node('field', root=block)
        wait = field.text == 'TRUE'
        value_nodes = self._get_nodes('value', root=block)
        values = []
        for val_node in value_nodes:
            val = self._get_condition_expression(val_node, arg_map=arg_map)
            values.append(val)
        radius = values.pop()
        values = '[{}]'.format(','.join(values))
        self._append_main_code(
            'code = self._arm.set_position_aa({}, speed=self._tcp_speed, mvacc=self._tcp_acc, radius={}, wait={})'.format(
                values, radius, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_position_aa\'):', indent + 2)
        self._append_main_code('    return', indent + 2)


    def _handle_motion_set_state(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        state = fields[0].text
        self._append_main_code('code = self._arm.set_state({})'.format(state), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_state\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_motion_stop(self, block, indent=0, arg_map=None):
        self._append_main_code('self._arm.emergency_stop()', indent + 2)

    def _handle_studio_run_traj(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        filename = fields[0].text
        speed = fields[1].text
        times = fields[2].text
        if filename:
            self._append_main_code('code = self._arm.playback_trajectory(times={}, filename=\'{}\', wait=True, double_speed={})'.format(times, filename, speed), indent + 2)
            self._append_main_code('if not self._check_code(code, \'playback_trajectory\'):', indent + 2)
            self._append_main_code('    return', indent + 2)
        else:
            self._append_main_code('pass', indent + 2)

    def _handle_app_studio_traj(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        filename = fields[0].text
        speed = fields[1].text
        value = self._get_node('value', root=block)
        times = self._get_nodes('field', root=value, descendant=True)[0].text
        self._append_main_code('code = self._arm.playback_trajectory(times={}, filename=\'{}\', wait=True, double_speed={})'.format(times, filename, speed), indent + 2)
        self._append_main_code('if not self._check_code(code, \'playback_trajectory\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_tool_message(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', block)
        color = json.dumps(fields[0].text, ensure_ascii=False) if len(fields) > 1 else 'white'
        msg = json.dumps(fields[2].text if fields[-1].text is not None else '', ensure_ascii=False)
        if self._highlight_callback is not None:
            self._append_main_code('print({}, color={})'.format(msg, color), indent + 2)
        else:
            self._append_main_code('print({})'.format(msg), indent + 2)

    def _handle_tool_console(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', block)
        color = json.dumps(fields[0].text, ensure_ascii=False) if len(fields) > 1 else 'white'
        msg = json.dumps(fields[1].text if fields[-1].text is not None else '', ensure_ascii=False)
        if self._highlight_callback is not None:
            self._append_main_code('print({}, color={})'.format(msg, color), indent + 2)
        else:
            self._append_main_code('print({})'.format(msg), indent + 2)

    def _handle_tool_console_with_variable(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', block)
        color = json.dumps(fields[0].text, ensure_ascii=False)
        msg = fields[1].text
        value = self._get_node('value', block)
        expression = self._get_condition_expression(value, arg_map=arg_map)
        if msg:
            if self._highlight_callback is not None:
                self._append_main_code('print({}.format({}), color={})'.format(json.dumps(msg+'{}', ensure_ascii=False), expression, color), indent + 2)
            else:
                self._append_main_code('print({}.format({}))'.format(json.dumps(msg+'{}', ensure_ascii=False), expression), indent + 2)
        else:
            if self._highlight_callback is not None:
                self._append_main_code('print(\'{{}}\'.format({}), color={})'.format(expression, color), indent + 2)
            else:
                self._append_main_code('print(\'{{}}\'.format({}))'.format(expression), indent + 2)

    def _handle_wait(self, block, indent=0, arg_map=None):
        value = self._get_node('value', root=block)
        value = float(self._get_nodes('field', root=value, descendant=True)[0].text)
        self._append_main_code('for i in range({}):'.format(int(value / 0.1)), indent + 2)
        self._append_main_code('    time.sleep(0.1)', indent + 2)
        self._append_main_code('    if not self.is_alive:', indent=indent + 2)
        self._append_main_code('        return', indent=indent + 2)

    def _handle_gpio_get_digital(self, block, indent=0, arg_map=None):
        io = self._get_node('field', block).text
        self._append_main_code('self._arm.get_tgpio_digital({})'.format(io), indent + 2)

    def _handle_gpio_get_analog(self, block, indent=0, arg_map=None):
        io = self._get_node('field', block).text
        self._append_main_code('self._arm.get_tgpio_analog({})'.format(io), indent + 2)

    def _handle_gpio_set_digital(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        io = fields[0].text
        value = 0 if fields[1].text == 'LOW' else 1
        is_sync = fields[2].text if len(fields) > 2 else 'True'
        delay_sec = fields[3].text if len(fields) > 3 else 0
        self._append_main_code('code = self._arm.set_tgpio_digital({}, {}, delay_sec={}, sync={})'.format(io, value, delay_sec, is_sync), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_tgpio_digital\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_gpio_set_digital_with_xyz(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        x = fields[0].text
        y = fields[1].text
        z = fields[2].text
        xyz = list(map(float, [x, y, z]))
        tol_r = fields[3].text
        io = fields[4].text
        value = 0 if fields[5].text == 'LOW' else 1
        self._append_main_code('code = self._arm.set_tgpio_digital_with_xyz({}, {}, {}, {})'.format(io, value, xyz, tol_r), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_tgpio_digital_with_xyz\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_gpio_set_digital_with_xyz_var(self, block, indent=0, arg_map=None):
        value_nodes = self._get_nodes('value', root=block)
        x = self._get_block_val(value_nodes[0], arg_map)
        y = self._get_block_val(value_nodes[1], arg_map)
        z = self._get_block_val(value_nodes[2], arg_map)
        xyz_li = [x, y, z]
        xyz = []
        for i in range(3):
            if re.match('\d+(\.\d+)*$', xyz_li[i]) is not None:
                xyz.append(float(xyz_li[i]))
            else:
                xyz.append(math.log2(3) if i == 0 else math.log2(5) if i == 1 else math.log2(7))
        xyz = str(xyz).replace(str(math.log2(3)), xyz_li[0]).replace(str(math.log2(5)), xyz_li[1]).replace(
            str(math.log2(7)), xyz_li[2])
        tol_r = self._get_block_val(value_nodes[3], arg_map)
        io = self._get_block_val(value_nodes[4], arg_map)
        value = 0 if self._get_block_val(value_nodes[5], arg_map) == 'LOW' else 1
        self._append_main_code(
            'code = self._arm.set_tgpio_digital_with_xyz({}, {}, {}, {})'.format(io, value, xyz, tol_r), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_tgpio_digital_with_xyz\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_get_suction_cup(self, block, indent=0, arg_map=None):
        self._append_main_code('{}self._arm.get_vacuum_gripper(hardware_version={})'.format('{}', self._vacuum_version), indent + 2)

    def _handle_check_air_pump_state(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        state = 1 if fields[0].text == 'ON' else 0
        timeout = float(fields[1].text)
        self._append_main_code('self._arm.arm.check_air_pump_state({}, timeout={}, hardware_version={})'.format(state, timeout, self._vacuum_version), indent + 2)

    def _handle_check_bio_gripper_is_catch(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        timeout = float(fields[0].text)
        self._append_main_code('self._arm.arm.check_bio_gripper_is_catch(timeout={})'.format(timeout), indent + 2)

    def _handle_check_robotiq_is_catch(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        timeout = float(fields[0].text)
        self._append_main_code('self._arm.arm.check_robotiq_is_catch(timeout={})'.format(timeout), indent + 2)

    def _handle_set_suction_cup(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block, name='trigger')
        on = True if fields[0].text == 'ON' else False
        fields = self._get_nodes('field', root=block, name='wait')
        if fields and len(fields) > 0:
            wait = fields[0].text == 'TRUE'
        else:
            wait = False
        fields = self._get_nodes('field', root=block, name='delay')
        delay_sec = fields[0].text if len(fields) > 0 else 0
        self._append_main_code('code = self._arm.set_vacuum_gripper({}, wait={}, delay_sec={}, hardware_version={})'.format(on, wait, delay_sec, self._vacuum_version), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_vacuum_gripper\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_gpio_get_controller_digital(self, block, indent=0, arg_map=None):
        io = self._get_node('field', block).text
        self._append_main_code('self._arm.get_cgpio_digital({})'.format(io), indent + 2)

    def _handle_gpio_get_controller_digital_di(self, block, indent=0, arg_map=None):
        io = self._get_node('field', block).text
        self._append_main_code('self._arm.get_cgpio_digital({})'.format(io), indent + 2)

    def _handle_gpio_get_controller_ci_li(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        values = []
        for field in fields[:-1]:
            values.append(int(field.text))
        timeout = float(fields[-1].text)
        self._append_main_code('self._arm.arm.get_cgpio_li_state({},timeout={},is_ci=True)'.format(values, timeout),
                               indent + 2)

    def _handle_gpio_get_controller_di_li(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        values = []
        for field in fields[:-1]:
            values.append(int(field.text))
        timeout = float(fields[-1].text)
        self._append_main_code('self._arm.arm.get_cgpio_li_state({},timeout={},is_ci=False)'.format(values, timeout),
                               indent + 2)

    def _handle_gpio_get_tgpio_li(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        values = []
        for field in fields[:-1]:
            values.append(int(field.text))
        timeout = float(fields[-1].text)
        self._append_main_code('self._arm.arm.get_tgpio_li_state({},timeout={})'.format(values, timeout),
                               indent + 2)

    def _handle_gpio_get_controller_analog(self, block, indent=0, arg_map=None):
        io = self._get_node('field', block).text
        self._append_main_code('self._arm.get_cgpio_analog({})'.format(io), indent + 2)

    def _handle_gpio_set_controller_digital(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        io = fields[0].text
        value = 0 if fields[1].text == 'LOW' else 1
        is_sync = fields[2].text if len(fields) > 2 else 'True'
        delay_sec = fields[3].text if len(fields) > 3 else 0
        self._append_main_code('code = self._arm.set_cgpio_digital({}, {}, delay_sec={}, sync={})'.format(io, value, delay_sec, is_sync), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_cgpio_digital\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_gpio_set_controller_digital_with_xyz(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        x = fields[0].text
        y = fields[1].text
        z = fields[2].text
        xyz = list(map(float, [x, y, z]))
        tol_r = fields[3].text
        io = fields[4].text
        value = 0 if fields[5].text == 'LOW' else 1
        self._append_main_code(
            'code = self._arm.set_cgpio_digital_with_xyz({}, {}, {}, {})'.format(io, value, xyz, tol_r), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_cgpio_digital_with_xyz\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_gpio_set_controller_digital_with_xyz_var(self, block, indent=0, arg_map=None):
        value_nodes = self._get_nodes('value', root=block)
        x = self._get_block_val(value_nodes[0], arg_map)
        y = self._get_block_val(value_nodes[1], arg_map)
        z = self._get_block_val(value_nodes[2], arg_map)
        xyz_li = [x, y, z]
        xyz = []
        for i in range(3):
            if re.match('\d+(\.\d+)*$', xyz_li[i]) is not None:
                xyz.append(float(xyz_li[i]))
            else:
                xyz.append(math.log2(3) if i == 0 else math.log2(5) if i == 1 else math.log2(7))
        xyz = str(xyz).replace(str(math.log2(3)), xyz_li[0]).replace(str(math.log2(5)), xyz_li[1]).replace(str(math.log2(7)), xyz_li[2])
        tol_r = self._get_block_val(value_nodes[3], arg_map)
        io = self._get_block_val(value_nodes[4], arg_map)
        value = 0 if self._get_block_val(value_nodes[5], arg_map) == 'LOW' else 1
        self._append_main_code('code = self._arm.set_cgpio_digital_with_xyz({}, {}, {}, {})'.format(io, value, xyz, tol_r), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_cgpio_digital_with_xyz\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_gpio_set_controller_digital_do(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        io = fields[0].text
        value = 0 if fields[1].text == 'LOW' else 1
        is_sync = fields[2].text if len(fields) > 2 else 'True'
        delay_sec = fields[3].text if len(fields) > 3 else 0
        self._append_main_code('code = self._arm.set_cgpio_digital({}, {}, delay_sec={}, sync={})'.format(io, value, delay_sec, is_sync), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_cgpio_digital\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_gpio_set_controller_digital_with_xyz_do(self, block, indent=0, arg_map=None):
        value_nodes = self._get_nodes('value', root=block)
        x = self._get_block_val(value_nodes[0], arg_map)
        y = self._get_block_val(value_nodes[1], arg_map)
        z = self._get_block_val(value_nodes[2], arg_map)
        xyz = list(map(float, [x, y, z]))
        tol_r = self._get_block_val(value_nodes[3], arg_map)
        io = self._get_block_val(value_nodes[4], arg_map)
        value = 0 if self._get_block_val(value_nodes[5], arg_map) == 'LOW' else 1
        self._append_main_code(
            'code = self._arm.set_cgpio_digital_with_xyz({}, {}, {}, {})'.format(io, value, xyz, tol_r), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_cgpio_digital_with_xyz\'):', indent + 2)
        self._append_main_code('    return', indent + 2)
        
    def _handle_gpio_set_controller_digital_with_xyz_do_var(self, block, indent=0, arg_map=None):
        value_nodes = self._get_nodes('value', root=block)
        x = self._get_block_val(value_nodes[0], arg_map)
        y = self._get_block_val(value_nodes[1], arg_map)
        z = self._get_block_val(value_nodes[2], arg_map)
        tol_r = self._get_block_val(value_nodes[3], arg_map)
        io = self._get_block_val(value_nodes[4], arg_map)
        value = 0 if self._get_block_val(value_nodes[5], arg_map) == 'LOW' else 1
        xyz_li = [x, y, z]
        xyz = []
        for i in range(3):
            if re.match('\d+(\.\d+)*$', xyz_li[i]) is not None:
                xyz.append(float(xyz_li[i]))
            else:
                xyz.append(math.log2(3) if i == 0 else math.log2(5) if i == 1 else math.log2(7))
        xyz = str(xyz).replace(str(math.log2(3)), xyz_li[0]).replace(str(math.log2(5)), xyz_li[1]).replace(
            str(math.log2(7)), xyz_li[2])
        self._append_main_code('code = self._arm.set_cgpio_digital_with_xyz({}, {}, {}, {})'.format(io, value, xyz, tol_r), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_cgpio_digital_with_xyz\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_gpio_set_controller_analog_with_xyz(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        x = fields[0].text
        y = fields[1].text
        z = fields[2].text
        xyz = list(map(float, [x, y, z]))
        tol_r = fields[3].text
        io = fields[4].text
        value = fields[5].text
        self._append_main_code('code = self._arm.set_cgpio_analog_with_xyz({}, {}, {}, {})'.format(io, value, xyz, tol_r), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_cgpio_analog_with_xyz\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_gpio_set_controller_analog_with_xyz_var(self, block, indent=0, arg_map=None):
        value_nodes = self._get_nodes('value', root=block)
        x = self._get_block_val(value_nodes[0], arg_map)
        y = self._get_block_val(value_nodes[1], arg_map)
        z = self._get_block_val(value_nodes[2], arg_map)
        # xyz = list(map(lambda x: float(x) if re.match('\d+(\.\d+)*$', x) is not None else [x, y, z].index(x), [x, y, z]))
        xyz_li = [x, y, z]
        xyz = []
        for i in range(3):
            if re.match('\d+(\.\d+)*$', xyz_li[i]) is not None:
                xyz.append(float(xyz_li[i]))
            else:
                xyz.append(math.log2(3) if i == 0 else math.log2(5) if i == 1 else math.log2(7))
        xyz = str(xyz).replace(str(math.log2(3)), xyz_li[0]).replace(str(math.log2(5)), xyz_li[1]).replace(
            str(math.log2(7)), xyz_li[2])
        tol_r = self._get_block_val(value_nodes[3], arg_map)
        io = self._get_block_val(value_nodes[4], arg_map)
        value = self._get_block_val(value_nodes[5], arg_map)
        self._append_main_code('code = self._arm.set_cgpio_analog_with_xyz({}, {}, {}, {})'.format(io, value, xyz, tol_r), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_cgpio_analog_with_xyz\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_gpio_set_controller_analog(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        io = fields[0].text
        is_sync = fields[1].text
        value = self._get_node('value', root=block)
        value = self._get_block_val(value, arg_map=arg_map)
        self._append_main_code('code = self._arm.set_cgpio_analog({}, {}, sync={})'.format(io, value, is_sync), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_cgpio_analog\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_collision_sensitivity(self, block, indent=0, arg_map=None):
        # value = self._get_node('value', root=block)
        # value = self._get_nodes('field', root=value, descendant=True)[0].text
        value = self._get_field_value(block)[0]
        self._append_main_code('code = self._arm.set_collision_sensitivity({})'.format(value), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_collision_sensitivity\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_teach_sensitivity(self, block, indent=0, arg_map=None):
        value = self._get_node('value', root=block)
        value = self._get_nodes('field', root=value, descendant=True)[0].text
        self._append_main_code('code = self._arm.set_teach_sensitivity({})'.format(value), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_teach_sensitivity\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_tcp_load(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        weight = fields[1].text
        x = fields[2].text
        y = fields[3].text
        z = fields[4].text
        self._append_main_code('code = self._arm.set_tcp_load({}, [{}, {}, {}])'.format(weight, x, y, z), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_tcp_load\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_gravity_direction(self, block, indent=0, arg_map=None):
        values = self._get_nodes('value', root=block)
        x = self._get_nodes('field', root=values[0], descendant=True)[0].text
        y = self._get_nodes('field', root=values[1], descendant=True)[0].text
        z = self._get_nodes('field', root=values[2], descendant=True)[0].text
        self._append_main_code('code = self._arm.set_gravity_direction([{}, {}, {}])'.format(x, y, z), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_gravity_direction\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_tcp_offset(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        x = fields[1].text
        y = fields[2].text
        z = fields[3].text
        roll = fields[4].text
        pitch = fields[5].text
        yaw = fields[6].text
        self._append_main_code('code = self._arm.set_tcp_offset([{}, {}, {}, {}, {}, {}], wait=True)'.format(x, y, z, roll, pitch, yaw), indent + 2)
        self._append_main_code('self._arm.set_state(0)', indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_tcp_offset\'):', indent + 2)
        self._append_main_code('    return', indent + 2)
        self._append_main_code('time.sleep(0.5)', indent + 2)

    def _handle_set_world_offset(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        x = fields[1].text
        y = fields[2].text
        z = fields[3].text
        roll = fields[4].text
        pitch = fields[5].text
        yaw = fields[6].text
        self._append_main_code('code = self._arm.set_world_offset([{}, {}, {}, {}, {}, {}])'.format(x, y, z, roll, pitch, yaw), indent + 2)
        self._append_main_code('self._arm.set_state(0)', indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_world_offset\'):', indent + 2)
        self._append_main_code('    return', indent + 2)
        self._append_main_code('time.sleep(0.5)', indent + 2)
    
    def _handle_set_lite6_gripper(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block, name='trigger')
        op = fields[0].text.upper()
        func = ''
        if op == 'OPEN':
            func = 'open_lite6_gripper'
            self._append_main_code('code = self._arm.{}()'.format(func), indent + 2)
        elif op == 'CLOSE':
            func = 'close_lite6_gripper'
            self._append_main_code('code = self._arm.{}()'.format(func), indent + 2)
        elif op == 'STOP':
            func = 'stop_lite6_gripper'
            self._append_main_code('code = self._arm.{}()'.format(func), indent + 2)
        else:
            return
        self._append_main_code('if not self._check_code(code, \'{}\'):'.format(func), indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_gripper_set(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        if fields is not None and len(fields) >= 3:
            pos = fields[0].text
            speed = fields[1].text
            wait = fields[2].text == 'TRUE'
        else:
            values = self._get_nodes('value', root=block)
            pos = self._get_nodes('field', root=values[0], descendant=True)[0].text
            speed = self._get_nodes('field', root=values[1], descendant=True)[0].text
            wait = self._get_nodes('field', root=values[2], descendant=True)[0].text == 'TRUE'
        self._append_main_code('code = self._arm.set_gripper_position({}, wait={}, speed={}, auto_enable=True)'.format(pos, wait, speed), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_gripper_position\'):', indent + 2)
        self._append_main_code('    return', indent + 2)
        
    def _handle_gripper_set_variable(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        wait = fields[0].text == 'TRUE'
        values = self._get_nodes('value', root=block)
        pos = self._get_block_val(values[0], arg_map)
        speed = self._get_block_val(values[1], arg_map)
        self._append_main_code('code = self._arm.set_gripper_position({}, wait={}, speed={}, auto_enable=True)'.format(pos, wait, speed), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_gripper_position\'):', indent + 2)
        self._append_main_code('    return', indent + 2)
        
    def _handle_gripper_set_status(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block, name='status')
        status = True if fields[0].text == 'TRUE' else False
        fields = self._get_nodes('field', root=block, name='delay')
        delay_sec = fields[0].text if len(fields) > 0 else 0
        self._append_main_code('code = self._arm.arm.set_gripper_status({}, delay_sec={})'.format(status, delay_sec), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_gripper_status\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_bio_g2_gripper_init(self, block, indent=0, arg_map=None):
        self._append_main_code('code = self._arm.set_bio_gripper_enable(True)', indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_bio_gripper_enable\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_bio_gripper_init(self, block, indent=0, arg_map=None):
        self._append_main_code('code = self._arm.set_bio_gripper_enable(True)', indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_bio_gripper_enable\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_bio_gripper(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block, name='status')
        on = True if fields[0].text == 'TRUE' else False
        fields = self._get_nodes('field', root=block, name='speed')
        speed = int(fields[0].text) if fields and len(fields) > 0 else 0
        fields = self._get_nodes('field', root=block, name='wait')
        wait = fields[0].text == 'TRUE' if fields and len(fields) > 0 else False
        if on:
            self._append_main_code('code = self._arm.open_bio_gripper(speed={}, wait={})'.format(speed, wait), indent + 2)
            self._append_main_code('if not self._check_code(code, \'open_bio_gripper\'):', indent + 2)
            self._append_main_code('    return', indent + 2)
        else:
            self._append_main_code('code = self._arm.close_bio_gripper(speed={}, wait={})'.format(speed, wait), indent + 2)
            self._append_main_code('if not self._check_code(code, \'close_bio_gripper\'):', indent + 2)
            self._append_main_code('    return', indent + 2)

    def _handle_set_bio_gripper_pos_force(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        pos = fields[0].text
        speed = fields[1].text
        force = fields[2].text
        wait = fields[3].text == 'TRUE'
        self._append_main_code('code = self._arm.set_bio_gripper_position(pos={}, speed={}, force={}, wait={})'.format(pos, speed, force, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'open_bio_gripper\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_robotiq_init(self, block, indent=0, arg_map=None):
        self._append_main_code('code, ret = self._arm.robotiq_reset()', indent + 2)
        self._append_main_code('if not self._check_code(code, \'robotiq_reset\'):', indent + 2)
        self._append_main_code('    return', indent + 2)
        self._append_main_code('code, ret = self._arm.robotiq_set_activate(wait=True)', indent + 2)
        self._append_main_code('if not self._check_code(code, \'robotiq_set_activate\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_robotiq_gripper(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block, name='pos')
        pos = int(fields[0].text)
        fields = self._get_nodes('field', root=block, name='speed')
        speed = int(fields[0].text) if fields and len(fields) > 0 else 0xFF
        fields = self._get_nodes('field', root=block, name='force')
        force = int(fields[0].text) if fields and len(fields) > 0 else 0xFF
        fields = self._get_nodes('field', root=block, name='wait')
        wait = fields[0].text == 'TRUE' if fields and len(fields) > 0 else False
        self._append_main_code('code, ret = self._arm.robotiq_set_position({}, speed={}, force={}, wait={})'.format(pos, speed, force, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'robotiq_set_position\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def __handle_gpio_event(self, gpio_type, block, indent=0, arg_map=None):
        if gpio_type.startswith('listen'):
            if gpio_type == 'listen_tgpio_digital':
                self._listen_tgpio_digital = True
            elif gpio_type == 'listen_tgpio_analog':
                self._listen_tgpio_analog = True
            elif gpio_type == 'listen_cgpio_state':
                self._listen_cgpio_state = True
            else:
                return
        else:
            fields = self._get_nodes('field', root=block)
            io = fields[0].text
            trigger = fields[1].text

            num = 1

            self._is_main_run_code = False
            if gpio_type == 'tgpio_digital':
                num = len(self._tgpio_digital_callbacks) + 1
                name = 'tool_gpio_{}_digital_is_changed_callback_{}'.format(io, num)
                self._append_main_code('# Define Tool GPIO-{} DIGITAL is {} callback'.format(io, trigger), indent=1)
            elif gpio_type == 'tgpio_analog':
                num = len(self._tgpio_analog_callbacks) + 1
                name = 'tool_gpio_{}_analog_is_changed_callback_{}'.format(io, num)
                self._append_main_code('# Define Tool GPIO-{} ANALOG is changed callback'.format(io), indent=1)
            elif gpio_type == 'cgpio_digital':
                num = len(self._cgpio_digital_callbacks) + 1
                name = 'controller_gpio_{}_digital_is_changed_callback_{}'.format(io, num)
                self._append_main_code('# Define Contoller GPIO-{} DIGITAL is {} callback'.format(io, trigger), indent=1)
            elif gpio_type == 'cgpio_analog':
                num = len(self._cgpio_analog_callbacks) + 1
                name = 'controller_gpio_{}_analog_is_changed_callback_{}'.format(io, num)
                self._append_main_code('# Define Contoller GPIO-{} ANALOG is changed callback'.format(io), indent=1)
            else:
                self._is_main_run_code = True
                return
            self._append_main_code('def {}(self):'.format(name), indent=1)
            statement = self._get_node('statement', root=block)
            if statement:
                self._parse_block(statement, indent, arg_map=arg_map)
            else:
                self._append_main_code('pass', indent=indent+3)
            self._append_main_code('')
            
            self._is_main_run_code = True
            if gpio_type == 'tgpio_digital':
                self._tgpio_digital_callbacks.append(name)
                self._append_main_code('self._tgpio_digital_callbacks.append({{\'io\': {}, \'trigger\': {}, \'op\': \'{}\', \'callback\': self.{}}})'.format(io, 1 if trigger == 'HIGH' else 0, '==', name), indent=indent+2)
            elif gpio_type == 'tgpio_analog':
                op = OPS_MAP.get(trigger)
                trigger = fields[2].text
                self._tgpio_analog_callbacks.append(name)
                self._append_main_code('self._tgpio_analog_callbacks.append({{\'io\': {}, \'trigger\': {}, \'op\': \'{}\', \'callback\': self.{}}})'.format(io, trigger, op, name), indent=indent+2)
            elif gpio_type == 'cgpio_digital':
                self._cgpio_digital_callbacks.append(name)
                self._append_main_code('self._cgpio_digital_callbacks.append({{\'io\': {}, \'trigger\': {}, \'op\': \'{}\', \'callback\': self.{}}})'.format(io, 1 if trigger == 'HIGH' else 0, '==', name), indent=indent+2)
            elif gpio_type == 'cgpio_analog':
                op = OPS_MAP.get(trigger)
                trigger = fields[2].text
                self._cgpio_analog_callbacks.append(name)
                self._append_main_code('self._cgpio_analog_callbacks.append({{\'io\': {}, \'trigger\': {}, \'op\': \'{}\', \'callback\': self.{}}})'.format(io, trigger, op, name), indent=indent+2)
    
    def _handle_event_get_holding_condition(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        addr = int(fields[0].text.replace(' ', '').replace('0x','').replace(',','').replace('\xa0', ''), 16)
        trigger = int(fields[1].text.replace(' ', '').replace('0x','').replace(',','').replace('\xa0', ''), 16)

        num = 1

        self._is_main_run_code = False
        num = len(self._holding_callbacks) + 1
        name = 'holding_is_changed_callback_{}'.format(num)
        self._append_main_code('# Define Holding Registers {} Value is {} callback'.format(
            addr, trigger), indent=1)

        self._append_main_code('def {}(self):'.format(name), indent=1)
        statement = self._get_node('statement', root=block)
        if statement:
            self._parse_block(statement, indent, arg_map=arg_map)
        else:
            self._append_main_code('pass', indent=indent + 3)
        self._append_main_code('')

        self._is_main_run_code = True
        self._holding_callbacks.append(name)
        self._append_main_code(
            'self._holding_callbacks.append({{\'trigger\': \'{}\', \'addr\': \'{}\', \'callback\': self.{}}})'.format(
                trigger, addr, name), indent=indent + 2)
    
    def __handle_count_event(self, count_type, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        op = fields[0].text
        trigger = fields[1].text

        num = 1

        self._is_main_run_code = False
        num = len(self._count_callbacks) + 1
        name = 'count_is_changed_callback_{}'.format(num)
        self._append_main_code('# Define Counter Value is {} {} callback'.format('greater than' if op == '>' else 'less than' if op == '<' else '', trigger), indent=1)

        self._append_main_code('def {}(self):'.format(name), indent=1)
        statement = self._get_node('statement', root=block)
        if statement:
            self._parse_block(statement, indent, arg_map=arg_map)
        else:
            self._append_main_code('pass', indent=indent + 3)
        self._append_main_code('')

        self._is_main_run_code = True
        self._count_callbacks.append(name)
        self._append_main_code(
            'self._count_callbacks.append({{\'trigger\': {}, \'op\': \'{}\', \'callback\': self.{}}})'.format(
                 trigger, '==' if op == '=' else op, name), indent=indent + 2)

    def _handle_event_get_counter_condition(self, block, indent=0, arg_map=None):
        self.__handle_count_event('get_counter', block, indent, arg_map=arg_map)

    def _handle_event_gpio_digital(self, block, indent=0, arg_map=None):
        self.__handle_gpio_event('tgpio_digital', block, indent, arg_map=arg_map)

    def _handle_event_gpio_analog(self, block, indent=0, arg_map=None):
        self.__handle_gpio_event('tgpio_analog', block, indent, arg_map=arg_map)

    def _handle_event_gpio_controller_digital(self, block, indent=0, arg_map=None):
        self.__handle_gpio_event('cgpio_digital', block, indent, arg_map=arg_map)

    def _handle_event_gpio_controller_analog(self, block, indent=0, arg_map=None):
        self.__handle_gpio_event('cgpio_analog', block, indent, arg_map=arg_map)

    def _handle_gpio_controller_digitals_listen(self, block, indent=0, arg_map=None):
        self.__handle_gpio_event('listen_cgpio_state', block, indent, arg_map=arg_map)

    def _handle_event_gpio_controller_digital_di(self, block, indent=0, arg_map=None):
        self.__handle_gpio_event('cgpio_digital', block, indent, arg_map=arg_map)

    def _handle_procedures_defnoreturn(self, block, indent=0, arg_map=None):
        field = self._get_node('field', block).text
        if not field:
            field = '1'
        if field not in self._funcs:
            name = 'function_{}'.format(len(self._funcs) + 1)
        else:
            name = self._funcs[field]
        self._is_main_run_code = False
        try:
            args = self._get_nodes('arg', root=self._get_node('mutation', block))
            arg_map_ = None

            if not args:
                self._append_main_code('    def {}(self):'.format(name), indent=indent)
            else:
                arg_list = [arg.attrib['name'] for arg in args]
                arg_map_ = {arg: 'arg_{}'.format(i + 1) for i, arg in enumerate(arg_list)}
                args_str = ', '.join(map(lambda x: arg_map_[x], arg_list)).strip()
                self._append_main_code('    def {}(self, {}):'.format(name, args_str), indent=indent)
            comment_block = self._get_node('comment', block)
            comment = '' if comment_block is None else comment_block.text
            self._append_main_code('    """', indent=indent+1)
            self._append_main_code('    {}'.format(comment), indent=indent+1)
            self._append_main_code('    """', indent=indent+1)
            statement = self._get_node('statement', root=block)
            self._funcs[field] = name
            if statement:
                self._parse_block(statement, indent, arg_map=arg_map_)
            else:
                self._append_main_code('    pass', indent=indent+1)
            self._append_main_code('')
            return arg_map_
        except Exception as e:
            self._succeed = False
            print('convert procedures_defnoreturn failed, {}'.format(e))
        finally:
            self._is_main_run_code = True

    def _handle_procedures_defreturn(self, block, indent=0, arg_map=None):
        arg_map_ = self._handle_procedures_defnoreturn(block, indent=indent, arg_map=arg_map)
        value = self._get_node('value', root=block)
        expression = self._get_condition_expression(value, arg_map=arg_map_)
        self._is_main_run_code = False
        self._append_main_code('    return {}'.format(expression), indent=indent+1)
        self._append_main_code('')
        self._is_main_run_code = True

    def _handle_procedures_callnoreturn(self, block, indent=0, arg_map=None):
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
            self._append_main_code('self.{}({})'.format(name, ','.join([self._get_condition_expression(val, arg_map=arg_map) for val in values])), indent=indent+2)
        else:
            self._append_main_code('self.{}()'.format(name), indent=indent+2)
        self._append_main_code('if not self.is_alive:', indent=indent+2)
        self._append_main_code('    return', indent=indent+2)

        self._funcs[mutation] = name

    def _handle_procedures_ifreturn(self, block, indent=0, arg_map=None):
        """
        This block may be used only within a function definition
        """
        self._is_main_run_code = False
        values = self._get_nodes('value', block)
        expression = self._get_condition_expression(values[0], arg_map=arg_map)
        self._append_main_code('if {}:'.format(expression), indent=indent+2)
        expression = self._get_condition_expression(values[1], arg_map=arg_map)
        self._append_main_code('    return {}'.format(expression), indent=indent+2)
        self._append_main_code('if not self.is_alive:', indent=indent+2)
        self._append_main_code('    return', indent=indent+2)
        self._is_main_run_code = True

    def _handle_procedures_callreturn(self, block, indent=0, arg_map=None):
        self._handle_procedures_callnoreturn(block, indent, arg_map=arg_map)

    def _handle_variables_set(self, block, indent=0, arg_map=None):
        field = self._get_node('field', block).text
        value = self._get_node('value', root=block)
        expression = self._get_condition_expression(value, arg_map=arg_map)
        if arg_map and field in arg_map:
            self._append_main_code('{} = {}'.format(arg_map[field], expression), indent=indent+2)
        else:
            self._append_main_code('self._vars[\'{}\'] = {}'.format(field, expression), indent=indent+2)

    def _handle_math_change(self, block, indent=0, arg_map=None):
        field = self._get_node('field', block).text
        value = self._get_node('value', root=block)
        # shadow = self._get_node('shadow', root=value)
        # val = self._get_node('field', root=shadow).text
        val = self._get_block_val(value)
        if arg_map and field in arg_map:
            self._append_main_code('{} += {}'.format(arg_map[field], val), indent=indent+2)
        else:
            self._append_main_code('self._vars[\'{}\'] += {}'.format(field, val), indent=indent+2)

    def _handle_controls_repeat_ext(self, block, indent=0, arg_map=None):
        value = self._get_node('value', root=block)
        times = self._get_block_val(value, arg_map=arg_map)
        self._append_main_code('for i in range(int({})):'.format(times), indent=indent+2)
        self._append_main_code('    if not self.is_alive:', indent=indent+2)
        self._append_main_code('        break', indent=indent+2)
        statement = self._get_node('statement', root=block)
        if statement:
            if self._loop_interval_sec > 0:
                self._append_main_code('    t1 = time.monotonic()', indent=indent+2)
            self._parse_block(statement, indent+1, arg_map=arg_map)
            if self._loop_interval_sec > 0:
                # limit frequency
                self._append_main_code('    interval = time.monotonic() - t1', indent=indent+2)
                self._append_main_code('    if interval < {}:'.format(self._loop_interval_sec), indent=indent+2)
                self._append_main_code('        time.sleep({} - interval)'.format(self._loop_interval_sec), indent=indent+2)
        else:
            if self._loop_interval_sec > 0:
                self._append_main_code('    time.sleep({})'.format(self._loop_interval_sec), indent=indent+2)
            else:
                self._append_main_code('    pass', indent=indent+2)

    def _handle_controls_whileUntil(self, block, indent=0, arg_map=None):
        field = self._get_node('field', root=block)
        if field.text == 'WHILE':
            value = self._get_node('value', root=block)
            expression = self._get_condition_expression(value, arg_map=arg_map)
            self._append_main_code('while self.is_alive and {}:'.format(expression), indent=indent+2)
        elif field.text == 'UNTIL':
            value = self._get_node('value', root=block)
            expression = self._get_condition_expression(value, arg_map=arg_map)
            self._append_main_code('while self.is_alive and not {}:'.format(expression), indent=indent+2)
        statement = self._get_node('statement', root=block)
        if statement:
            if self._loop_interval_sec > 0:
                self._append_main_code('    t1 = time.monotonic()', indent=indent+2)
            self._parse_block(statement, indent+1, arg_map=arg_map)
            if self._loop_interval_sec > 0:
                # limit frequency
                self._append_main_code('    interval = time.monotonic() - t1', indent=indent+2)
                self._append_main_code('    if interval < {}:'.format(self._loop_interval_sec), indent=indent+2)
                self._append_main_code('        time.sleep({} - interval)'.format(self._loop_interval_sec), indent=indent+2)
        else:
            if self._loop_interval_sec > 0:
                self._append_main_code('    time.sleep({})'.format(self._loop_interval_sec), indent=indent+2)
            else:
                self._append_main_code('    pass', indent=indent+2)

    def _handle_loop_run_forever(self, block, indent=0, arg_map=None):
        self._append_main_code('while self.is_alive:', indent=indent+2)
        statement = self._get_node('statement', root=block)
        if statement:
            if self._loop_interval_sec > 0:
                self._append_main_code('    t1 = time.monotonic()', indent=indent+2)
            self._parse_block(statement, indent+1, arg_map=arg_map)
            if self._loop_interval_sec > 0:
                # limit frequency
                self._append_main_code('    interval = time.monotonic() - t1', indent=indent+2)
                self._append_main_code('    if interval < {}:'.format(self._loop_interval_sec), indent=indent+2)
                self._append_main_code('        time.sleep({} - interval)'.format(self._loop_interval_sec), indent=indent+2)
        else:
            if self._loop_interval_sec > 0:
                self._append_main_code('    time.sleep({})'.format(self._loop_interval_sec), indent=indent+2)
            else:
                self._append_main_code('    pass', indent=indent+2)

    def _handle_loop_break(self, block, indent=0, arg_map=None):
        self._append_main_code('break', indent=indent+2)

    def _handle_tool_comment(self, block, indent=0, arg_map=None):
        field = self._get_node('field', block)
        self._append_main_code('# {}'.format(field.text), indent=indent+2)
        statement = self._get_node('statement', block)
        if statement:
            self._parse_block(statement, indent, arg_map=arg_map)

    def _handle_tool_app_comment(self, block, indent=0, arg_map=None):
        field = self._get_node('field', block)
        self._append_main_code('# [APP] {}'.format(field.text), indent=indent+2)
        statement = self._get_node('statement', block)
        if statement:
            self._parse_block(statement, indent, arg_map=arg_map)

    def _handle_tool_remark(self, block, indent=0, arg_map=None):
        field = self._get_node('field', block)
        self._append_main_code('# {}'.format(field.text), indent=indent+2)

    def _handle_controls_if(self, block, indent=0, arg_map=None):
        values = self._get_nodes('value', root=block)
        statements = self._get_nodes('statement', root=block)
        old_indent = indent
        has_if = False
        for _, value in enumerate(values):
            indent = old_indent
            expression = self._get_condition_expression(value, arg_map=arg_map)
            if not has_if:
                has_if = True
                self._append_main_code('if {}:'.format(expression), indent=indent+2)
            else:
                self._append_main_code('elif {}:'.format(expression), indent=indent+2)
            old_indent = indent
            indent += 1
            statement = None
            for st in statements:
                if st.attrib['name'][2:] == value.attrib['name'][2:]:
                    statement = st
                    break
            if statement:
                self._parse_block(statement, old_indent+1, arg_map=arg_map)
            else:
                self._append_main_code('    pass', indent=old_indent+2)
        for st in statements:
            if st.attrib['name'] == 'ELSE':
                if has_if:
                    self._append_main_code('else:', indent=old_indent+2)
                self._parse_block(st, old_indent if not has_if else indent, arg_map=arg_map)
                break
    
    def _handle_set_line_track(self, block, indent=0, arg_map=None):
        self.is_run_linear_track = True
        fields = self._get_nodes('field', root=block)
        if fields is None:
            values = self._get_nodes('value', root=block)
            fields = [
                self._get_nodes('field', root=values[0], descendant=True)[0],
                self._get_nodes('field', root=values[1], descendant=True)[0]
            ]
        pos = fields[0].text
        speed = fields[1].text
        wait = True
        self._append_main_code('code = self._arm.set_linear_track_pos({}, speed={}, wait={}, auto_enable=True)'.format(pos, speed, wait), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_linear_track_pos\'):', indent + 2)
        self._append_main_code('    return', indent + 2)
        
    def _handle_set_line_track_origin(self, block, indent=0, arg_map=None):
        self._append_main_code('code = self._arm.set_linear_track_back_origin(wait=True, auto_enable=True)', indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_linear_track_back_origin\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_python_code(self, block, indent=0, arg_map=None, **kwargs):
        self._append_main_code('##### Python code #####', indent + 2)
        text = self._get_field_value(block)
        codes = text.split('\n') if isinstance(text, str) else []
        prev_is_empty = False
        length = len(codes)
        for i, code in enumerate(codes):
            if not code.strip():
                if prev_is_empty or i == length - 1:
                    continue
                prev_is_empty = True
            else:
                prev_is_empty = False
            if (self._is_exec or (not self._is_exec and not self._is_ide)) and code.strip() and code not in \
                    ['finally:', 'else:'] and all([i not in code for i in ['elif', 'except', 'def', 'class']]) \
                    and not code.startswith('@'):
                # 只有在studio执行blockly时或者SDK直接调用run_blockly_app时才会执行
                code_indent = re.match('(\s*).*', code).group(1)
                self._append_main_code(code_indent + 'if not self.is_alive:', indent + 2)
                self._append_main_code(code_indent + 'return', indent + 3)
            self._append_main_code(code, indent + 2)
        self._append_main_code('#######################', indent + 2)
        
    def _handle_set_end_level(self, block, indent=0, arg_map=None, **kwargs):
        if not self.axis_type:
            return
        self._append_main_code('self._arm.arm.wait_move()', indent + 2)
        self._append_main_code('current_angle = self._arm.angles', indent + 2)
        if self.axis_type[0] == 5:
            self._append_main_code('angle = -(current_angle[1] + current_angle[2])', indent + 2)
            self._append_main_code('code = self._arm.set_servo_angle(angle=[*current_angle[:3], angle, current_angle[4]])', indent + 2,)
        elif self.axis_type[0] == 6:
            self._append_main_code('angle_5 = {}'.format('-(current_angle[1] - current_angle[2])' if self.axis_type[1]==9 or self.axis_type[1]==12 else
                                                       '-(current_angle[1] + current_angle[2])'), indent + 2)
            self._append_main_code('angles = [*current_angle[:3],0,angle_5,current_angle[5]]', indent + 2)
            self._append_main_code('code = self._arm.set_servo_angle(angle=angles)', indent + 2)
        elif self.axis_type[0] == 7:
            self._append_main_code('code, ret = self._arm.get_inverse_kinematics(pose=[*current_angle[:3], 180, 0, current_angle[5]])', indent + 2)
            self._append_main_code('if not self._check_code(code, \'set_end_level\'):', indent + 2)
            self._append_main_code('    return', indent + 2)
            self._append_main_code('code = self._arm.set_servo_angle(angle=ret)', indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_end_level\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_modbus_rtu(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        host_id = fields[0].text
        cmd = fields[1].text
        cmd_li = re.sub(',', ' ', cmd)
        is_run_cmd = ''.join(cmd_li.split()).isalnum()
        if not is_run_cmd:
            self._append_main_code('if not self._check_code(-1, \'set_tgpio_modbus\'):', indent + 2)
            self._append_main_code('    return', indent + 2)
            return
        cmd_li = re.sub(' +', ' ', cmd_li)
        cmd_li = re.sub('\xa0', ' ', cmd_li)
        cmd_li = re.sub('\s+', ' ', cmd_li)
        cmd_li = cmd_li.strip().split(' ')
        int_li = [int(da, 16) for da in cmd_li]
        self._append_main_code('code, ret = self._arm.getset_tgpio_modbus_data({}, host_id={})'.format(int_li, host_id), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_tgpio_modbus\'):', indent + 2)
        self._append_main_code('    return', indent + 2)

    def _handle_set_ft_sensor(self, block, indent=0, arg_map=None):
        force_axis_list = ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']
        force_axis_value = [0, 0, 0, 0, 0, 0]
        force_ref_value = [0, 0, 0, 0, 0, 0]
        fields = self._get_nodes('field', root=block)
        ref_frame = fields[0].text
        force_axis = fields[1].text
        force_ref = fields[2].text
        wait_time = fields[3].text
        for index, axis in enumerate(force_axis_list):
            if axis == force_axis:
                force_axis_value[index] = 1
                force_ref_value[index] = float(force_ref)
        self._append_main_code('code = self._arm.config_force_control({}, {}, {}, [0] * 6)'.format(ref_frame, force_axis_value,
                               force_ref_value), indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_tgpio_modbus\'):', indent + 2)
        self._append_main_code('    return', indent + 2)
        self._append_main_code('code = self._arm.ft_sensor_enable(1)', indent + 2)
        self._append_main_code('if not self._check_code(code, \'set_tgpio_modbus\'):', indent + 2)
        self._append_main_code('    return', indent + 2)
        self._append_main_code('time.sleep(0.2)', indent + 2)
        self._append_main_code('self._arm.ft_sensor_app_set(2)', indent + 2)
        self._append_main_code('self._arm.set_state(0)', indent + 2)
        self._append_main_code('start_time = time.time()', indent + 2)
        self._append_main_code('while time.time() - start_time < {}:'.format(wait_time), indent + 2)
        self._append_main_code('if self._arm.state == 4:', indent + 3)
        self._append_main_code('    return', indent + 4)
        self._append_main_code('self._arm.ft_sensor_app_set(0)', indent + 2)

    def _handle_studio_run_blockly(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        projectName = fields[0].text
        fileName = fields[1].text
        times = fields[2].text
        if not self._is_exec:
            self._append_main_code('self._start_run_blockly(fileName="{}", times={})'.format(fileName.lstrip('/'), times), indent + 2)
        else:
            self._append_main_code('start_run_blockly(fileName="{}", times={})'.format(fileName, times), indent + 2)
        self._is_run_blockly = True
    
    def _handle_studio_run_gcode(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        projectName = fields[0].text
        fileName = fields[1].text
        times = fields[2].text
        if self._is_exec:
            self._append_main_code('start_run_gcode(projectName="{}", fileName="{}", times={})'.format(projectName, fileName, times), indent + 2)
        else:
            self._append_main_code("for i in range({}):".format(times), indent + 2)
            self._append_main_code('    self._arm.run_gcode_app(path="{}")'.format(projectName+fileName), indent + 2)

    def _handle_write_single_holding_register(self, block, indent=0, arg_map=None):
        fields = self._get_nodes('field', root=block)
        addr = int(fields[0].text.replace(' ', '').replace('0x','').replace(',','').replace('\xa0', ''), 16)
        value = int(fields[1].text.replace(' ', '').replace('0x','').replace(',','').replace('\xa0', ''), 16)
        self._append_main_code('self._arm.write_single_holding_register({}, {})'.format(addr, value), indent + 2)

