#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

try:
  import xml.etree.cElementTree as ET
except ImportError:
  import xml.etree.ElementTree as ET
import re
import json


class XmlTool(object):
    def __init__(self, path):
        self.tree = ET.parse(path)
        self.root = self.tree.getroot()
        self.namespace = self.get_namespace()
        self._ops = {
            'EQ': '==',
            'NEQ': '!=',
            'LT': '<',
            'LTE': '<=',
            'GT': '>',
            'GTE': '>='
        }
        self._py_list = []
        self._hasEvent = False
        self._events = {}
        self._index = -1
        self.py_code = ''

    @property
    def index(self):
        self._index += 1
        return self._index

    def _append_to_file(self, data):
        self._py_list.append(data)

    def _insert_to_file(self, i, data):
        self._py_list.insert(i, data)

    def get_namespace(self):
        try:
            r = re.compile('({.+})')
            if r.search(self.root.tag) is not None:
                ns = r.search(self.root.tag).group(1)
            else:
                ns = ''
        except Exception as e:
            # print(e)
            ns = ''
        return ns

    def get_node(self, tag, root=None):
        if root is None:
            root = self.root
        return root.find(self.namespace + tag)

    def get_nodes(self, tag, root=None, descendant=False, **kwargs):
        if root is None:
            root = self.root
        nodes = []
        if descendant:
            func = root.iter
        else:
            func = root.findall
        for node in func(self.namespace + tag):
            flag = True
            for k, v in kwargs.items():
                if node.attrib[k] != v:
                    flag = False
            if flag:
                nodes.append(node)
        return nodes

    def _init_py3(self, ip=None, clean_err_warn=True, motion_enable=True, mode=0, state=0, error_exit=True):
        self._insert_to_file(self.index, '#!/usr/bin/env python3')
        self._insert_to_file(self.index, '# Software License Agreement (BSD License)\n#')
        self._insert_to_file(self.index, '# Copyright (c) 2019, UFACTORY, Inc.')
        self._insert_to_file(self.index, '# All rights reserved.\n#')
        self._insert_to_file(self.index, '# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>\n')
        self._insert_to_file(self.index, 'import sys')
        self._insert_to_file(self.index, 'import time')
        self._insert_to_file(self.index, 'import threading')
        self._insert_to_file(self.index, 'from xarm.wrapper import XArmAPI\n')
        if ip is None:
            self._insert_to_file(self.index, 'arm = XArmAPI(sys.argv[1])')
        elif isinstance(ip, str):
            self._insert_to_file(self.index, 'arm = XArmAPI(\'{}\')'.format(ip))
        self._insert_to_file(self.index, 'time.sleep(0.5)')
        if clean_err_warn:
            self._insert_to_file(self.index, 'arm.clean_warn()')
            self._insert_to_file(self.index, 'arm.clean_error()')
        if motion_enable:
            self._insert_to_file(self.index, 'arm.motion_enable(True)')
        self._insert_to_file(self.index, 'arm.set_mode({})'.format(mode))
        self._insert_to_file(self.index, 'arm.set_state({})'.format(state))
        self._insert_to_file(self.index, 'time.sleep(1)\n')
        self._insert_to_file(self.index, 'params = {\'speed\': 100, \'acc\': 2000, \'angle_speed\': 20, \'angle_acc\': 500}')
        self._insert_to_file(self.index, 'events = {}\n')
        if error_exit:
            self._insert_to_file(self.index, '\ndef error_warn_change_callback(data):')
            self._insert_to_file(self.index, '    if data and data[\'error_code\'] != 0:')
            self._insert_to_file(self.index, '        arm.set_state(4)')
            self._insert_to_file(self.index, '        sys.exit(1)\n')
            self._insert_to_file(self.index, 'arm.register_error_warn_changed_callback(error_warn_change_callback)\n')
            # self._insert_to_file(self.index, '\ndef state_change_callback(data):')
            # self._insert_to_file(self.index, '    if data and data[\'state\'] == 4:')
            # self._insert_to_file(self.index, '        sys.exit(1)\n')
            # self._insert_to_file(self.index, 'arm.register_state_changed_callback(state_change_callback)\n')

    def _finish_py3(self):
        if self._hasEvent:
            self._append_to_file('\nwhile arm.connected and arm.error_code == 0:')
            self._append_to_file('    time.sleep(1)')

    def to_python(self, path=None, ip=None, clean_err_warn=True, motion_enable=True, mode=0, state=0, error_exit=True):
        self._init_py3(ip, clean_err_warn, motion_enable, mode, state, error_exit)
        self.parse()
        self._finish_py3()
        self.py_code = '\n'.join(self._py_list)
        if path is not None:
            with open(path, 'w', encoding='utf-8') as f:
                f.write('{}\n'.format(self.py_code))

    def parse(self, root=None, prefix=''):
        is_statement = root is None
        if root is not None:
            if root.tag == self.namespace+'statement':
                is_statement = True
        block = self.get_node('block', root=root)
        while block is not None:
            if not is_statement:
                block = self.get_node('next', root=block)
                if not block:
                    break
                block = self.get_node('block', root=block)
            else:
                is_statement = False
            if block.attrib.get('disabled', False):
                continue
            func = getattr(self, '_handle_{}'.format(block.attrib['type']), None)
            if func:
                func(block, prefix)
            else:
                print('block {} can\'t convert to python code'.format(block.attrib['type']))

    def _handle_set_speed(self, block, prefix=''):
        # print('{}{}'.format(prefix, block.attrib['type']))
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        # if len(prefix):
        #     print_to_file('{}global speed'.format(prefix))
        self._append_to_file('{}params[\'speed\'] = {}'.format(prefix, value))

    def _handle_set_acceleration(self, block, prefix=''):
        # print('{}{}'.format(prefix, block.attrib['type']))
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        # if len(prefix):
        #     print_to_file('{}global acc'.format(prefix))
        self._append_to_file('{}params[\'acc\'] = {}'.format(prefix, value))

    def _handle_set_angle_speed(self, block, prefix=''):
        # print('{}{}'.format(prefix, block.attrib['type']))
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        # if len(prefix):
        #     print_to_file('{}global angle_speed'.format(prefix))
        self._append_to_file('{}params[\'angle_speed\'] = {}'.format(prefix, value))

    def _handle_set_angle_acceleration(self, block, prefix=''):
        # print('{}{}'.format(prefix, block.attrib['type']))
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        # if len(prefix):
        #     print_to_file('{}global angle_acc'.format(prefix))
        self._append_to_file('{}params[\'angle_acc\'] = {}'.format(prefix, value))

    def _handle_reset(self, block, prefix=''):
        # print('{}{}'.format(prefix, block.attrib['type']))
        self._append_to_file('{}if arm.error_code == 0:'.format(prefix))
        self._append_to_file('{}    arm.reset()'.format(prefix))

    def _handle_sleep(self, block, prefix=''):
        # print('{}{}'.format(prefix, block.attrib['type']))
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}if arm.error_code == 0:'.format(prefix))
        self._append_to_file('{}    arm.set_sleep_time({})'.format(prefix, value))

    def _handle_move(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        orientation = fields[0].text
        wait = fields[1].text == 'TRUE'
        value = fields[2].text
        if orientation == 'forward':
            param = 'x'
        elif orientation == 'backward':
            param = 'x'
            value = '-{}'.format(value)
        elif orientation == 'left':
            param = 'y'
        elif orientation == 'right':
            param = 'y'
            value = '-{}'.format(value)
        elif orientation == 'up':
            param = 'z'
        elif orientation == 'down':
            param = 'z'
            value = '-{}'.format(value)
        else:
            return

        self._append_to_file('{}if arm.error_code == 0:'.format(prefix))
        self._append_to_file(
            '{}    arm.set_position({}={}, speed=params[\'speed\'], mvacc=params[\'acc\'], relative=True, wait={})'.format(
                prefix, param, value, wait))

    def _handle_move_arc_to(self, block, prefix=''):
        # print('{}{}'.format(prefix, block.attrib['type']))
        value = self.get_node('value', root=block)
        p_block = self.get_node('block', root=value)
        fields = self.get_nodes('field', root=p_block)
        values = []
        for field in fields[:-2]:
            values.append(float(field.text))
        radius = float(fields[-2].text)
        wait = fields[-1].text == 'TRUE'
        self._append_to_file('{}if arm.error_code == 0:'.format(prefix))
        self._append_to_file('{}    arm.set_position(*{}, speed=params[\'speed\'], mvacc=params[\'acc\'], radius={}, wait={})'.format(prefix, values, radius, wait))

    def _handle_move_7(self, block, prefix=''):
        # print('{}{}'.format(prefix, block.attrib['type']))
        value = self.get_node('value', root=block)
        p_block = self.get_node('block', root=value)
        fields = self.get_nodes('field', root=p_block)
        values = []
        for field in fields[:-1]:
            values.append(float(field.text))
        wait = fields[-1].text == 'TRUE'
        self._append_to_file('{}if arm.error_code == 0:'.format(prefix))
        self._append_to_file(
            '{}    arm.set_servo_angle(angle={}, speed=params[\'angle_speed\'], mvacc=params[\'angle_acc\'], wait={})'.format(prefix, values, wait))

    def _handle_motion_stop(self, block, prefix=''):
        # print('{}{}'.format(prefix, block.attrib['type']))
        self._append_to_file('{}arm.emergency_stop()'.format(prefix))

    def _handle_tool_message(self, block, prefix=''):
        fields = self.get_nodes('field', block)
        message = json.loads(json.dumps(fields[-1].text))
        self._append_to_file('{}print(\'{}\')'.format(prefix, message))

    def _handle_wait(self, block, prefix=''):
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}time.sleep({})'.format(prefix, value))

    def _handle_gpio_set_digital(self, block, prefix=''):
        io = self.get_node('field', block).text
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}arm.set_gpio_digital({}, {})'.format(prefix, io, value))

    def _handle_set_collision_sensitivity(self, block, prefix=''):
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}arm.set_collision_sensitivity({})'.format(prefix, value))

    def _handle_set_teach_sensitivity(self, block, prefix=''):
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}arm.set_teach_sensitivity({})'.format(prefix, value))

    def _handle_set_tcp_load(self, block, prefix=''):
        values = self.get_nodes('value', root=block)
        weight = self.get_nodes('field', root=values[0], descendant=True)[0].text
        x = self.get_nodes('field', root=values[1], descendant=True)[0].text
        y = self.get_nodes('field', root=values[2], descendant=True)[0].text
        z = self.get_nodes('field', root=values[3], descendant=True)[0].text
        self._append_to_file('{}arm.set_tcp_load({}, [{}, {}, {}])'.format(prefix, weight, x, y, z))

    def _handle_set_tcp_offset(self, block, prefix=''):
        values = self.get_nodes('value', root=block)
        x = self.get_nodes('field', root=values[0], descendant=True)[0].text
        y = self.get_nodes('field', root=values[1], descendant=True)[0].text
        z = self.get_nodes('field', root=values[2], descendant=True)[0].text
        roll = self.get_nodes('field', root=values[3], descendant=True)[0].text
        pitch = self.get_nodes('field', root=values[4], descendant=True)[0].text
        yaw = self.get_nodes('field', root=values[5], descendant=True)[0].text
        self._append_to_file('{}arm.set_tcp_offset([{}, {}, {}, {}, {}, {}])'.format(prefix, x, y, z, roll, pitch, yaw))

    def _handle_gripper_set(self, block, prefix=''):
        values = self.get_nodes('value', root=block)
        pos = self.get_nodes('field', root=values[0], descendant=True)[0].text
        speed = self.get_nodes('field', root=values[1], descendant=True)[0].text
        wait = self.get_nodes('field', root=values[2], descendant=True)[0].text == 'TRUE'
        self._append_to_file('{}arm.set_gripper_position({}, wait={}, speed={})'.format(prefix, pos, wait, speed))

    def _handle_event_gpio_digital(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        io = fields[0].text
        trigger = fields[1].text
        # print('{}[{}] [当IO{}为{}时，执行]'.format(prefix, block.attrib['type'], io, '高电平' if trigger == 'HIGH' else '低电平'))

        if 'gpio' not in self._events:
            num = 1
        else:
            num = self._events['gpio'] + 1
        name = '{}_io{}_is_{}_{}'.format(block.attrib['type'], io, trigger.lower(), num)
        self._append_to_file('\n{}def {}():'.format(prefix, name))
        old_prefix = prefix
        prefix = '    ' + prefix
        statement = self.get_node('statement', root=block)
        if statement:
            self.parse(statement, prefix)
        else:
            # print('{}没操作'.format(prefix))
            self._append_to_file('{}pass'.format(prefix))
        self._append_to_file('\n{}events[\'gpio\'].callbacks[\'IO{}\'][{}].append({})'.format(old_prefix, io,
                                                                                       1 if trigger == 'HIGH' else 0,
                                                                                       name))
        self._append_to_file('{}if not events[\'gpio\'].isAlive():'.format(old_prefix))
        self._append_to_file('{}    events[\'gpio\'].start()'.format(old_prefix))

        if 'gpio' not in self._events:
            name2 = 'EventGPIOThread'.format(io, trigger.capitalize())
            self._insert_to_file(self.index, '\nclass {}(threading.Thread):'.format(name2))
            self._insert_to_file(self.index, '    def __init__(self, *args, **kwargs):\n        threading.Thread.__init__(self, *args, **kwargs)')
            self._insert_to_file(self.index, '        self.daemon = True')
            self._insert_to_file(self.index, '        self.digital = [-1, -1]')
            self._insert_to_file(self.index, '        self.callbacks = {\'IO1\': {0: [], 1: []}, \'IO2\': {0: [], 1: []}}')
            self._insert_to_file(self.index, '\n    def run(self):')
            self._insert_to_file(self.index, '        while arm.connected and arm.error_code == 0:')
            self._insert_to_file(self.index, '            _, digital = arm.get_gpio_digital()')
            self._insert_to_file(self.index, '            if _ == 0:')
            self._insert_to_file(self.index, '                if digital[0] != self.digital[0]:')
            self._insert_to_file(self.index, '                    for callback in self.callbacks[\'IO1\'][digital[0]]:')
            self._insert_to_file(self.index, '                        callback()')
            self._insert_to_file(self.index, '                if digital[1] != self.digital[1]:')
            self._insert_to_file(self.index, '                    for callback in self.callbacks[\'IO2\'][digital[1]]:')
            self._insert_to_file(self.index, '                        callback()')
            self._insert_to_file(self.index, '            if _ == 0:')
            self._insert_to_file(self.index, '                self.digital = digital')
            self._insert_to_file(self.index, '            time.sleep(0.1)')
            self._insert_to_file(self.index, '\nevents[\'gpio\'] = {}()\n'.format(name2))

        if 'gpio' not in self._events:
            self._events['gpio'] = 2
        else:
            self._events['gpio'] += 1

        self._hasEvent = True

    def _handle_controls_repeat_ext(self, block, prefix=''):
        value = self.get_node('value', root=block)
        times = self.get_nodes('field', root=value, descendant=True)[0].text
        # print('{}[{}] [循环执行{}次]'.format(prefix, block.attrib['type'], times))
        self._append_to_file('{}for i in range({}):'.format(prefix, times))
        prefix = '    ' + prefix
        statement = self.get_node('statement', root=block)
        if statement:
            self.parse(statement, prefix)
        else:
            # print('{}没操作'.format(prefix))
            self._append_to_file('{}pass'.format(prefix))

    # def handle_controls_for(self, block, prefix=''):
    #     print(block.attrib.get('disabled', False))

    def _handle_controls_whileUntil(self, block, prefix=''):
        field = self.get_node('field', root=block)
        if field.text == 'WHILE':
            # print('{}[{}] [当条件为真时，循环]'.format(prefix, block.attrib['type']))
            value = self.get_node('value', root=block)
            expression = self._get_condition_expression(value)
            self._append_to_file('{}while {}:'.format(prefix, expression))
        elif field.text == 'UNTIL':
            # print('{}[{}] [循环直到条件为真]'.format(prefix, block.attrib['type']))
            value = self.get_node('value', root=block)
            expression = self._get_condition_expression(value)
            self._append_to_file('{}while not {}:'.format(prefix, expression))
        prefix = '    ' + prefix
        statement = self.get_node('statement', root=block)
        if statement:
            self.parse(statement, prefix)
        else:
            # print('{}没操作'.format(prefix))
            self._append_to_file('{}pass'.format(prefix))

    def _handle_loop_run_forever(self, block, prefix=''):
        self._append_to_file('{}while True:'.format(prefix))
        prefix = '    ' + prefix
        statement = self.get_node('statement', root=block)
        if statement:
            self.parse(statement, prefix)
        else:
            # print('{}没操作'.format(prefix))
            self._append_to_file('{}pass'.format(prefix))

    def _handle_loop_break(self, block, prefix=''):
        self._append_to_file('{}break'.format(prefix))

    def _handle_tool_comment(self, block, prefix=''):
        field = self.get_node('field', block)
        self._append_to_file('{}# {}'.format(prefix, field.text))
        statement = self.get_node('statement', block)
        if statement:
            self.parse(statement, prefix)

    def _handle_tool_remark(self, block, prefix=''):
        field = self.get_node('field', block)
        self._append_to_file('{}# {}'.format(prefix, field.text))

    def _handle_controls_if(self, block, prefix=''):
        value = self.get_node('value', root=block)
        expression = self._get_condition_expression(value)
        self._append_to_file('{}if {}:'.format(prefix, expression))
        prefix = '    ' + prefix
        # print_to_file('{}pass'.format(prefix))
        statement = self.get_node('statement', root=block)
        if statement:
            self.parse(statement, prefix)
        else:
            # print('{}没操作'.format(prefix))
            self._append_to_file('{}pass'.format(prefix))

    def _get_condition_expression(self, value_block):
        block = self.get_node('block', value_block)
        if block.attrib['type'] == 'logic_boolean':
            return str(self.get_node('field', block).text == 'TRUE')
        elif block.attrib['type'] == 'logic_compare':
            op = self._ops.get(self.get_node('field', block).text)
            A = 0
            B = 0
            values = self.get_nodes('value', block)
            if len(values) > 0:
                A = self._get_condition_expression(values[0])
                if len(values) > 1:
                    B = self._get_condition_expression(values[1])
            return '{} {} {}'.format(A, op, B)
        elif block.attrib['type'] == 'logic_operation':
            op = self.get_node('field', block).text.lower()
            A = False
            B = False
            values = self.get_nodes('value', block)
            if len(values) > 0:
                A = self._get_condition_expression(values[0])
                if len(values) > 1:
                    B = self._get_condition_expression(values[1])
            return '{} {} {}'.format(A, op, B)
        elif block.attrib['type'] == 'logic_negate':
            value = self.get_node('value', root=block)
            return 'not ({})'.format(self._get_condition_expression(value))
        elif block.attrib['type'] == 'gpio_get_digital':
            io = self.get_node('field', block).text
            return 'arm.gpio_get_digital({})[{}]'.format(io, 1)
        elif block.attrib['type'] == 'gpio_get_analog':
            io = self.get_node('field', block).text
            return 'arm.get_gpio_analog({})[{}]'.format(io, 1)
        elif block.attrib['type'] == 'math_number':
            val = self.get_node('field', block).text
            return val


if __name__ == '__main__':
    # xmlTool = XmlTool('C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm6\\app\\myapp\local_test_1\\app.xml')
    # xmlTool = XmlTool('C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm6\\app\\myapp\\app_template\\app.xml')
    # xmlTool = XmlTool('C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm6\\app\\myapp\\test_gpio\\app.xml')
    # xmlTool = XmlTool('C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm7\\app\\myapp\\pour_water\\app.xml')
    xmlTool = XmlTool('C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm7\\app\\myapp\\233\\app.xml')
    import os
    target_path = os.path.join(os.path.expanduser('~'), '.UFACTORY', 'app', 'tmp')
    if not os.path.exists(target_path):
        os.makedirs(target_path)
    target_file = os.path.join(target_path, 'xml_to_py.py')
    xmlTool.to_python(target_file, ip='192.168.1.145')
