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
import sys
import json
import time


def line_filter(line):
    ret = re.search('\S', line)
    return ret.span()[0] if ret is not None else ret


def get_code_list(codes):
    code_list = list(filter(lambda x: x, codes.splitlines()))
    inx = min(list(filter(lambda x: x is not None, map(line_filter, code_list))))
    code_list = list(map(lambda x: x[inx:], code_list))
    if not code_list[0]:
        code_list = code_list[1:]
    if len(code_list) > 2 and not code_list[-1] and not code_list[-2]:
        code_list = code_list[:-1]
    return code_list


class BlocklyTool(object):
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
        self._ops2 = {
            '===': '==',
            '!==': '!=',
            '>=': '>=',
            '>': '>',
            '<=': '<=',
            '<': '<',
        }
        self._code_list = []
        self._hasEvent = False
        self._events = {}
        self._funcs = {}
        self._func_index = 0
        self._index = -1
        self._first_index = 0
        self._is_insert = False
        self.codes = ''
        self._succeed = True
        self._show_comment = False

    @property
    def index(self):
        self._index += 1
        return self._index

    @property
    def func_index(self):
        self._func_index += 1
        return self._func_index

    @property
    def first_index(self):
        self._first_index += 1
        self._index += 1
        return self._first_index

    def _append_to_file(self, data):
        if not self._is_insert:
            self._code_list.append(data)
        else:
            self._code_list.insert(self.first_index, data)

    def _insert_to_file(self, i, data):
        self._code_list.insert(i, data)

    def _insert_codes_to_file(self, codes, prefix=''):
        for code in get_code_list(codes):
            self._insert_to_file(self.index, '{}{}'.format(prefix, code))

    def _append_codes_to_file(self, codes, prefix=''):
        for code in get_code_list(codes):
            self._append_to_file('{}{}'.format(prefix, code))

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

    def _init_py3(self, arm=None, init=True, wait_seconds=1, mode=0, state=0, error_exit=True, stop_exit=True):
        codes = """
            #!/usr/bin/env python3
            # Software License Agreement (BSD License)
            #
            # Copyright (c) {}, UFACTORY, Inc.'
            # All rights reserved.
            # 
            # Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
            \"\"\"
            # Notice
            #   1. Changes to this file on Studio will not be preserved
            #   2. The next conversion will overwrite the file with the same name
            \"\"\"
            import sys
            import time
            import datetime
            import threading
            \"\"\"
            # xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
            # git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
            # cd xArm-Python-SDK
            # python setup.py install
            \"\"\"
            from xarm import version
            from xarm.wrapper import XArmAPI
            print('xArm-Python-SDK Version: {{}}'.format(version.__version__))
            """.format(
            time.localtime(time.time()).tm_year
        )
        self._insert_codes_to_file(codes)

        if arm is None:
            self._insert_to_file(self.index, 'arm = XArmAPI(sys.argv[1])')
        elif isinstance(arm, str):
            self._insert_to_file(self.index, 'arm = XArmAPI(\'{}\')'.format(arm))
        if init:
            codes = """
            arm.clean_warn()
            arm.clean_error()
            arm.motion_enable(True)
            arm.set_mode({})
            arm.set_state({})
            """.format(mode, state)
            self._insert_codes_to_file(codes)
        if wait_seconds > 0:
            self._insert_to_file(self.index, 'time.sleep({})\n'.format(wait_seconds))
        codes = """
        params = {
            'speed': 100,
            'acc': 2000,
            'angle_speed': 20,
            'angle_acc': 500,
            'quit': False,
            'variables': {},
            'events': {},
        }
        """
        self._insert_codes_to_file(codes)

        if error_exit:
            codes = """
            
            
            # Register error/warn changed callback
            def error_warn_change_callback(data):
                if data and data[\'error_code\'] != 0:
                    arm.set_state(4)
                    params['quit'] = True
                    print('err={}, quit'.format(data['error_code']))
            arm.register_error_warn_changed_callback(error_warn_change_callback)
            """
            self._insert_codes_to_file(codes)
        if stop_exit:
            codes = """
            
            
            # Register state changed callback
            def state_changed_callback(data):
                if arm.version_number[0] >= 1 and arm.version_number[1] >= 1 and arm.version_number[2] > 0:
                    params['quit'] = True')
                    print('state=4, quit')
            arm.register_state_changed_callback(state_changed_callback)
            """
            self._insert_codes_to_file(codes)

        codes = """
        
        
        # Register counter value changed callback
        if hasattr(arm, 'register_count_changed_callback'):
            def count_changed_callback(data):
                print('counter val: {}'.format(data['count']))
            arm.register_count_changed_callback(count_changed_callback)
        """
        self._insert_codes_to_file(codes)

        self._first_index = self._index

    def _finish_py3(self, error_exit=True, stop_exit=True):
        if self._hasEvent:
            codes = """
            
            # Main loop
            while arm.connected and arm.error_code == 0 and not params['quit']:
                time.sleep(1)
            """
            self._append_codes_to_file(codes)
        codes = """
        
        if hasattr(arm, 'release_count_changed_callback'):
            arm.release_count_changed_callback(count_changed_callback)
        """
        self._append_codes_to_file(codes)
        if error_exit:
            self._append_to_file('arm.release_error_warn_changed_callback(state_changed_callback)')
        if stop_exit:
            self._append_to_file('arm.release_state_changed_callback(state_changed_callback)')
        self._append_to_file('')

    def to_python(self, path=None, arm=None, init=True, wait_seconds=1, mode=0, state=0,
                  error_exit=True, stop_exit=True, show_comment=False, **kwargs):
        self._show_comment = show_comment
        self._succeed = True
        self._init_py3(arm=arm, init=init, wait_seconds=wait_seconds, mode=mode, state=state, error_exit=error_exit, stop_exit=stop_exit)
        self.parse()
        self._finish_py3(error_exit=error_exit, stop_exit=stop_exit)
        self.codes = '\n'.join(self._code_list)
        if path is not None:
            with open(path, 'w', encoding='utf-8') as f:
                f.write('{}\n'.format(self.codes))
        return self._succeed

    def parse(self, root=None, prefix=''):
        blocks = self.get_nodes('block', root=root)
        if blocks:
            for block in blocks:
                is_statement = root is None
                if root is not None:
                    if root.tag == self.namespace + 'statement':
                        is_statement = True
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
                        self._succeed = False
                        print('block {} can\'t convert to python code'.format(block.attrib['type']))

    def _handle_set_speed(self, block, prefix=''):
        field = self.get_node('field', root=block)
        if field is not None:
            value = field.text
        else:
            value = self.get_node('value', root=block)
            value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}params[\'speed\'] = {}'.format(prefix, value))

    def _handle_set_acceleration(self, block, prefix=''):
        field = self.get_node('field', root=block)
        if field is not None:
            value = field.text
        else:
            value = self.get_node('value', root=block)
            value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}params[\'acc\'] = {}'.format(prefix, value))

    def _handle_set_angle_speed(self, block, prefix=''):
        field = self.get_node('field', root=block)
        if field is not None:
            value = field.text
        else:
            value = self.get_node('value', root=block)
            value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}params[\'angle_speed\'] = {}'.format(prefix, value))

    def _handle_set_angle_acceleration(self, block, prefix=''):
        field = self.get_node('field', root=block)
        if field is not None:
            value = field.text
        else:
            value = self.get_node('value', root=block)
            value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}params[\'angle_acc\'] = {}'.format(prefix, value))

    def _handle_set_counter_increase(self, block, prefix=''):
        # field = self.get_node('field', root=block)
        # if field is not None:
        #     value = field.text
        # else:
        #     value = self.get_node('value', root=block)
        #     value = self.get_nodes('field', root=value, descendant=True)[0].text
        if self._show_comment:
            self._append_to_file('{}# set counter increase'.format(prefix))
        self._append_to_file('{}arm.set_counter_increase()'.format(prefix))

    def _handle_set_counter_reset(self, block, prefix=''):
        if self._show_comment:
            self._append_to_file('{}# set counter reset'.format(prefix))
        self._append_to_file('{}arm.set_counter_reset()'.format(prefix))

    def _handle_reset(self, block, prefix=''):
        self._append_to_file('{}if arm.error_code == 0 and not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.reset()'.format(prefix))

    def _handle_sleep(self, block, prefix=''):
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        if self._show_comment:
            self._append_to_file('{}# set pause time'.format(prefix))
        self._append_to_file('{}if arm.error_code == 0 and not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.set_pause_time({})'.format(prefix, value))

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

        if self._show_comment:
            self._append_to_file('{}# relative move'.format(prefix))
        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_position(
                {}={}, speed=params['speed'], mvacc=params['acc'], relative=True, wait={})
            if code != 0:
                print('set_position, code={{}}, {}={}'.format(code))
                params['quit'] = True
        """.format(param, value, wait, param, value)
        self._append_codes_to_file(codes, prefix)

    def _handle_move_arc_to(self, block, prefix=''):
        value = self.get_node('value', root=block)
        p_block = self.get_node('block', root=value)
        fields = self.get_nodes('field', root=p_block)
        values = []
        for field in fields[:-2]:
            values.append(float(field.text))
        radius = float(fields[-2].text)
        wait = fields[-1].text == 'TRUE'
        if self._show_comment:
            self._append_to_file('{}# move{}line and {}'.format(
                prefix, ' arc ' if float(radius) >= 0 else ' ', 'wait' if wait else 'no wait'))

        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_position(
                *{}, speed=params['speed'], mvacc=params['acc'], radius={}, wait={})
            if code != 0:
                print('set_position, code={{}}, pose={}, radius={}'.format(code))
                params['quit'] = True
        """.format(values, radius, wait, values, radius)
        self._append_codes_to_file(codes, prefix)

    def _handle_move_circle(self, block, prefix=''):
        values = self.get_nodes('value', root=block)
        percent = self.get_nodes('field', root=values[2], descendant=True)[0].text
        percent = round(float(percent) / 360 * 100, 2)
        wait = self.get_nodes('field', root=values[3], descendant=True)[0].text == 'TRUE'

        p1_block = self.get_node('block', root=values[0])
        fields = self.get_nodes('field', root=p1_block)
        pose1 = []
        for field in fields:
            pose1.append(float(field.text))

        p2_block = self.get_node('block', root=values[1])
        fields = self.get_nodes('field', root=p2_block)
        pose2 = []
        for field in fields:
            pose2.append(float(field.text))
        if self._show_comment:
            self._append_to_file('{}# move circle and {}'.format(
                prefix, 'wait' if wait else 'no wait'))
        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.move_circle(
                {}, {}, {}, speed=params['speed'], mvacc=params['acc'], wait={})
            if code != 0:
                print('move_circle, code={{}}, pose1={}, pose2={}, percent={}'.format(code))
                params['quit'] = True
        """.format(pose1, pose2, percent, wait, pose1, pose2, percent)
        self._append_codes_to_file(codes, prefix)

    def _handle_move_7(self, block, prefix=''):
        value = self.get_node('value', root=block)
        p_block = self.get_node('block', root=value)
        fields = self.get_nodes('field', root=p_block)
        values = []
        for field in fields[:-1]:
            values.append(float(field.text))
        wait = fields[-1].text == 'TRUE'
        if self._show_comment:
            self._append_to_file('{}# move joint and {}'.format(prefix, 'wait' if wait else 'no wait'))

        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_servo_angle(
                angle={}, speed=params['angle_speed'], mvacc=params['angle_acc'], wait={})
            if code != 0:
                print('set_servo_angle, code={{}}, angle={}'.format(code))
                params['quit'] = True
        """.format(values, wait, values)
        self._append_codes_to_file(codes, prefix)

    def _handle_move_joints(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        values = []
        for field in fields[:-1]:
            values.append(float(field.text))
        wait = fields[-1].text == 'TRUE'
        if self._show_comment:
            self._append_to_file('{}# move joint and {}'.format(prefix, 'wait' if wait else 'no wait'))

        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_servo_angle(
                angle={}, speed=params['angle_speed'], mvacc=params['angle_acc'], wait={})
            if code != 0:
                print('set_servo_angle, code={{}}, angle={}'.format(code))
                params['quit'] = True
        """.format(values, wait, values)
        self._append_codes_to_file(codes, prefix)

    def _handle_move_cartesian(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        values = []
        for field in fields[:-2]:
            values.append(float(field.text))
        radius = float(fields[-2].text)
        wait = fields[-1].text == 'TRUE'
        if self._show_comment:
            self._append_to_file('{}# move{}line and {}'.format(
                prefix, ' arc ' if float(radius) >= 0 else ' ', 'wait' if wait else 'no wait'))
        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_position(
                *{}, speed=params['speed'], mvacc=params['acc'], radius={}, wait={})
            if code != 0:
                print('set_position, code={{}}, pose={}, radius={}'.format(code))
                params['quit'] = True
        """.format(values, radius, wait, values, radius)
        self._append_codes_to_file(codes, prefix)

    def _handle_move_tool_line(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        values = []
        for field in fields[:-1]:
            values.append(float(field.text))
        wait = fields[-1].text == 'TRUE'
        if self._show_comment:
            self._append_to_file('{}# move tool line and {}'.format(prefix, 'wait' if wait else 'no wait'))
        codes = """
                if arm.error_code == 0 and not params['quit']:
                    code = arm.set_tool_position(
                        *{}, speed=params['speed'], mvacc=params['acc'], wait={})
                    if code != 0:
                        print('set_tool_position, code={{}}, pose={}'.format(code))
                        params['quit'] = True
                """.format(values, wait, values)
        self._append_codes_to_file(codes, prefix)

    def _handle_move_joints_var(self, block, prefix=''):
        field = self.get_node('field', root=block)
        wait = field.text == 'TRUE'
        value_nodes = self.get_nodes('value', root=block)
        values = []
        for val_node in value_nodes:
            val = self.__get_condition_expression(val_node)
            values.append(val)
        values = '[{}]'.format(','.join(values))
        if self._show_comment:
            self._append_to_file('{}# move joint and {}'.format(prefix, 'wait' if wait else 'no wait'))
        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_servo_angle(
                angle={}, speed=params['angle_speed'], mvacc=params['angle_acc'], wait={})
            if code != 0:
                print('set_servo_angle, code={{}}, angle={}'.format(code))
                params['quit'] = True
        """.format(values, wait, values)
        self._append_codes_to_file(codes, prefix)

    def _handle_move_cartesian_var(self, block, prefix=''):
        field = self.get_node('field', root=block)
        wait = field.text == 'TRUE'
        value_nodes = self.get_nodes('value', root=block)
        values = []
        for val_node in value_nodes:
            val = self.__get_condition_expression(val_node)
            values.append(val)
        radius = values.pop()
        values = '[{}]'.format(','.join(values))
        if self._show_comment:
            try:
                self._append_to_file('{}# move{}line and {}'.format(
                    prefix, ' arc ' if float(radius) >= 0 else ' ', 'wait' if wait else 'no wait'))
            except:
                pass
        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_position(
                *{}, speed=params['speed'], mvacc=params['acc'], radius={}, wait={})
            if code != 0:
                print('set_position, code={{}}, pose={}, radius={}'.format(code))
                params['quit'] = True
        """.format(values, radius, wait, values, radius)
        self._append_codes_to_file(codes, prefix)

    def _handle_motion_set_state(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        state = fields[0].text
        if self._show_comment:
            self._append_to_file('{}# set state'.format(prefix))
        self._append_to_file('{}arm.set_state({})'.format(prefix, state))

    def _handle_motion_stop(self, block, prefix=''):
        if self._show_comment:
            self._append_to_file('{}# emergency stop'.format(prefix))
        self._append_to_file('{}arm.emergency_stop()'.format(prefix))

    def _handle_studio_run_traj(self, block, prefix=''):
        filename = self.get_node('field', root=block).text
        value = self.get_node('value', root=block)
        times = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    if arm.playback_trajectory(times={}, filename=\'{}\', wait=True) != 0:'.format(prefix, times, filename))
        self._append_to_file('{}        params[\'quit\'] = True'.format(prefix))

    def _handle_app_studio_traj(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        filename = fields[0].text
        speed = fields[1].text
        value = self.get_node('value', root=block)
        times = self.get_nodes('field', root=value, descendant=True)[0].text
        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.playback_trajectory(
                times={}, filename=\'{}\', wait=True, double_speed={})
            if code != 0:
                print('playback_trajectory, code={{}}, filename={}, speed={}'.format(code))
                params['quit'] = True
        """.format(times, filename, speed, filename, speed)
        self._append_codes_to_file(codes, prefix)

    def _handle_tool_message(self, block, prefix=''):
        fields = self.get_nodes('field', block)
        msg = json.dumps(fields[-1].text, ensure_ascii=False)
        self._append_to_file('{}print({})'.format(prefix, msg))
        # msg = fields[-1].text
        # self._append_to_file('{}print(\'{}\')'.format(prefix, message))
        # self._append_to_file('{}print(\'{{}}\'.format(\'{}\'))'.format(prefix, message))

    def _handle_tool_console(self, block, prefix=''):
        fields = self.get_nodes('field', block)
        msg = json.dumps(fields[1].text, ensure_ascii=False)
        self._append_to_file('{}print({})'.format(prefix, msg))
        # msg = fields[1].text
        # self._append_to_file('{}print(\'{}\')'.format(prefix, msg))

    def _handle_tool_console_with_variable(self, block, prefix=''):
        fields = self.get_nodes('field', block)
        msg = fields[1].text
        # msg = json.dumps(fields[1].text, ensure_ascii=False)
        value = self.get_node('value', block)
        expression = self.__get_condition_expression(value)
        # self._append_to_file('{}value = {}'.format(prefix, expression))
        if msg:
            self._append_to_file('{}print({}.format({}))'.format(prefix, json.dumps(msg+'{}', ensure_ascii=False), expression))
            # self._append_to_file('{}print(\'{}{{}}\'.format({}))'.format(prefix, msg, expression))
        else:
            self._append_to_file('{}print(\'{{}}\'.format({}))'.format(prefix, expression))

    def _handle_wait(self, block, prefix=''):
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    time.sleep({})'.format(prefix, value))

    def _handle_gpio_get_digital(self, block, prefix=''):
        io = self.get_node('field', block).text
        if self._show_comment:
            self._append_to_file('{}# get tgpio-{} digital'.format(prefix, io))
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.get_tgpio_digital({})'.format(prefix, io))

    def _handle_gpio_get_analog(self, block, prefix=''):
        io = self.get_node('field', block).text
        if self._show_comment:
            self._append_to_file('{}# get tgpio-{} analog'.format(prefix, io))
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.get_tgpio_analog({})'.format(prefix, io))

    def _handle_gpio_set_digital(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        io = fields[0].text
        value = 0 if fields[1].text == 'LOW' else 1
        delay_sec = fields[2].text if len(fields) > 2 else 0
        # io = self.get_node('field', block).text
        # value = self.get_node('value', root=block)
        # value = self.get_nodes('field', root=value, descendant=True)[0].text
        if self._show_comment:
            self._append_to_file('{}# set tgpio-{} digital'.format(prefix, io))

        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_tgpio_digital(
                {}, {}, delay_sec={})
            if code != 0:
                print('set_tgpio_digital, code={{}}, io={}, value={}, delay_sec={}'.format(code))
                params['quit'] = True
        """.format(io, value, delay_sec, io, value, delay_sec)
        self._append_codes_to_file(codes, prefix)

    def _handle_gpio_set_digital_with_xyz(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        x = fields[0].text
        y = fields[1].text
        z = fields[2].text
        xyz = list(map(float, [x, y, z]))
        tol_r = fields[3].text
        io = fields[4].text
        value = 0 if fields[5].text == 'LOW' else 1
        # io = self.get_node('field', block).text
        # value = self.get_node('value', root=block)
        # value = self.get_nodes('field', root=value, descendant=True)[0].text
        if self._show_comment:
            self._append_to_file('{}# set tgpio-{} digital with pos {}'.format(prefix, io, xyz))
        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_tgpio_digital_with_xyz(
                {}, {}, {}, {})
            if code != 0:
                print('set_tgpio_digital_with_xyz, code={{}}, io={}, value={}, xyz={}, tol_r={}'.format(code))
                params['quit'] = True
        """.format(io, value, xyz, tol_r, io, value, xyz, tol_r)
        self._append_codes_to_file(codes, prefix)

    def _handle_get_suction_cup(self, block, prefix=''):
        if self._show_comment:
            self._append_to_file('{}# get suction cup status'.format(prefix))
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.get_suction_cup()'.format(prefix))

    def _handle_check_air_pump_state(self, block, prefix=''):
        if self._show_comment:
            self._append_to_file('{}# check air pump state'.format(prefix))
        fields = self.get_nodes('field', root=block)
        state = 1 if fields[0].text == 'ON' else 0
        timeout = float(fields[1].text)
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.arm.check_air_pump_state({}, timeout={})'.format(prefix, state, timeout))

    def _handle_set_suction_cup(self, block, prefix=''):
        fields = self.get_nodes('field', root=block, name='trigger')
        on = True if fields[0].text == 'ON' else False
        fields = self.get_nodes('field', root=block, name='wait')
        if fields and len(fields) > 0:
            wait = fields[0].text == 'TRUE'
        else:
            wait = False
        fields = self.get_nodes('field', root=block, name='delay')
        delay_sec = fields[0].text if len(fields) > 0 else 0

        # io = self.get_node('field', block).text
        # value = self.get_node('value', root=block)
        # value = self.get_nodes('field', root=value, descendant=True)[0].text
        if self._show_comment:
            self._append_to_file('{}# set_suction_cup({}, wait={}, delay_sec={})'.format(prefix, on, wait, delay_sec))
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.set_suction_cup({}, wait={}, delay_sec={})'.format(prefix, on, wait, delay_sec))
        # self._append_to_file('{}    if arm.set_suction_cup({}, {}) != 0:'.format(prefix, on, wait))
        # self._append_to_file('{}        params[\'quit\'] = True'.format(prefix))

    def _handle_gpio_get_controller_digital(self, block, prefix=''):
        io = self.get_node('field', block).text
        if self._show_comment:
            self._append_to_file('{}# get cgpio-{} digital'.format(prefix, io))
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.get_cgpio_digital({})'.format(prefix, io))

    def _handle_gpio_get_controller_analog(self, block, prefix=''):
        io = self.get_node('field', block).text
        if self._show_comment:
            self._append_to_file('{}# get cgpio-{} analog'.format(prefix, io))
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.get_cgpio_analog({})'.format(prefix, io))

    def _handle_gpio_set_controller_digital(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        io = fields[0].text
        value = 0 if fields[1].text == 'LOW' else 1
        delay_sec = fields[2].text if len(fields) > 2 else 0
        # io = self.get_node('field', block).text
        # value = self.get_node('value', root=block)
        # value = self.get_nodes('field', root=value, descendant=True)[0].text
        if self._show_comment:
            self._append_to_file('{}# set cgpio-{} digital'.format(prefix, io))
        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_cgpio_digital(
                {}, {}, delay_sec={})
            if code != 0:
                print('set_cgpio_digital, code={{}}, io={}, value={}, delay_sec={}'.format(code))
                params['quit'] = True
        """.format(io, value, delay_sec, io, value, delay_sec)
        self._append_codes_to_file(codes, prefix)

    def _handle_gpio_set_controller_digital_with_xyz(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        x = fields[0].text
        y = fields[1].text
        z = fields[2].text
        xyz = list(map(float, [x, y, z]))
        tol_r = fields[3].text
        io = fields[4].text
        value = 0 if fields[5].text == 'LOW' else 1
        # io = self.get_node('field', block).text
        # value = self.get_node('value', root=block)
        # value = self.get_nodes('field', root=value, descendant=True)[0].text
        if self._show_comment:
            self._append_to_file('{}# set cgpio-{} digital with pos {}'.format(prefix, io, xyz))
        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_cgpio_digital_with_xyz(
                {}, {}, {}, {})
            if code != 0:
                print('set_cgpio_digital_with_xyz, code={{}}, io={}, value={}, xyz={}, tol_r={}'.format(code))
                params['quit'] = True
        """.format(io, value, xyz, tol_r, io, value, xyz, tol_r)
        self._append_codes_to_file(codes, prefix)

    def _handle_gpio_set_controller_analog(self, block, prefix=''):
        io = self.get_node('field', block).text
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        if self._show_comment:
            self._append_to_file('{}# set cgpio-{} digital'.format(prefix, io))
        codes = """
        if arm.error_code == 0 and not params['quit']:
            code = arm.set_cgpio_analog(
                {}, {})
            if code != 0:
                print('set_cgpio_analog, code={{}}, io={}, value={}'.format(code))
                params['quit'] = True
        """.format(io, value, io, value)
        self._append_codes_to_file(codes, prefix)

    def _handle_set_collision_sensitivity(self, block, prefix=''):
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.set_collision_sensitivity({})'.format(prefix, value))

    def _handle_set_teach_sensitivity(self, block, prefix=''):
        value = self.get_node('value', root=block)
        value = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.set_teach_sensitivity({})'.format(prefix, value))

    def _handle_set_tcp_load(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        weight = fields[1].text
        x = fields[2].text
        y = fields[3].text
        z = fields[4].text
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.set_tcp_load({}, [{}, {}, {}])'.format(prefix, weight, x, y, z))
        self._append_to_file('{}    arm.set_state(0)'.format(prefix))

        # values = self.get_nodes('value', root=block)
        # weight = self.get_nodes('field', root=values[0], descendant=True)[0].text
        # x = self.get_nodes('field', root=values[1], descendant=True)[0].text
        # y = self.get_nodes('field', root=values[2], descendant=True)[0].text
        # z = self.get_nodes('field', root=values[3], descendant=True)[0].text
        # self._append_to_file('{}arm.set_tcp_load({}, [{}, {}, {}])'.format(prefix, weight, x, y, z))
        # self._append_to_file('{}arm.set_state(0)'.format(prefix))

    def _handle_set_gravity_direction(self, block, prefix=''):
        values = self.get_nodes('value', root=block)
        x = self.get_nodes('field', root=values[0], descendant=True)[0].text
        y = self.get_nodes('field', root=values[1], descendant=True)[0].text
        z = self.get_nodes('field', root=values[2], descendant=True)[0].text
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.set_gravity_direction([{}, {}, {}])'.format(prefix, x, y, z))

    def _handle_set_tcp_offset(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        x = fields[1].text
        y = fields[2].text
        z = fields[3].text
        roll = fields[4].text
        pitch = fields[5].text
        yaw = fields[6].text
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.set_tcp_offset([{}, {}, {}, {}, {}, {}])'.format(prefix, x, y, z, roll, pitch, yaw))
        self._append_to_file('{}    arm.set_state(0)'.format(prefix))

        # values = self.get_nodes('value', root=block)
        # x = self.get_nodes('field', root=values[0], descendant=True)[0].text
        # y = self.get_nodes('field', root=values[1], descendant=True)[0].text
        # z = self.get_nodes('field', root=values[2], descendant=True)[0].text
        # roll = self.get_nodes('field', root=values[3], descendant=True)[0].text
        # pitch = self.get_nodes('field', root=values[4], descendant=True)[0].text
        # yaw = self.get_nodes('field', root=values[5], descendant=True)[0].text
        # self._append_to_file('{}arm.set_tcp_offset([{}, {}, {}, {}, {}, {}])'.format(prefix, x, y, z, roll, pitch, yaw))
        # self._append_to_file('{}arm.set_state(0)'.format(prefix))

    def _handle_set_world_offset(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        x = fields[1].text
        y = fields[2].text
        z = fields[3].text
        roll = fields[4].text
        pitch = fields[5].text
        yaw = fields[6].text
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    arm.set_world_offset([{}, {}, {}, {}, {}, {}])'.format(prefix, x, y, z, roll, pitch, yaw))
        self._append_to_file('{}    arm.set_state(0)'.format(prefix))

    def _handle_gripper_set(self, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        if fields is not None and len(fields) >= 3:
            pos = fields[0].text
            speed = fields[1].text
            wait = fields[2].text == 'TRUE'
        else:
            values = self.get_nodes('value', root=block)
            pos = self.get_nodes('field', root=values[0], descendant=True)[0].text
            speed = self.get_nodes('field', root=values[1], descendant=True)[0].text
            wait = self.get_nodes('field', root=values[2], descendant=True)[0].text == 'TRUE'
        if self._show_comment:
            self._append_to_file('{}# set gripper position and '.format(prefix, 'wait' if wait else 'no wait'))
        self._append_to_file('{}if not params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    if arm.set_gripper_position({}, wait={}, speed={}, auto_enable=True) != 0:'.format(prefix, pos, wait, speed))
        self._append_to_file('{}        params[\'quit\'] = True'.format(prefix))

    def _handle_gripper_set_status(self, block, prefix=''):
        fields = self.get_nodes('field', root=block, name='status')
        status = True if fields[0].text == 'TRUE' else False
        fields = self.get_nodes('field', root=block, name='delay')
        delay_sec = fields[0].text if len(fields) > 0 else 0
        if self._show_comment:
            self._append_to_file('{}# set_gripper_status({}, delay_sec={})'.format(prefix, status, delay_sec))
        self._append_to_file('{}if not params[\'quit\'] and arm._arm.set_gripper_status({}, delay_sec={}) != 0:'.format(prefix, status, delay_sec))
        self._append_to_file('{}    params[\'quit\'] = True'.format(prefix))

    def _handle_set_bio_gripper_init(self, block, prefix=''):
        if self._show_comment:
            self._append_to_file('{}# set_bio_gripper_enable(True)'.format(prefix))
        self._append_to_file('{}if not params[\'quit\'] and arm.set_bio_gripper_enable(True) != 0:'.format(prefix))
        self._append_to_file('{}    params[\'quit\'] = True'.format(prefix))
        self._append_to_file('{}expired = time.time() + 2'.format(prefix))
        self._append_to_file('{}while not params[\'quit\'] and time.time() < expired:'.format(prefix))
        self._append_to_file('{}    time.sleep(0.1)'.format(prefix))

    def _handle_set_bio_gripper(self, block, prefix=''):
        fields = self.get_nodes('field', root=block, name='status')
        on = True if fields[0].text == 'TRUE' else False
        fields = self.get_nodes('field', root=block, name='speed')
        speed = int(fields[0].text) if fields and len(fields) > 0 else 0
        fields = self.get_nodes('field', root=block, name='wait')
        wait = fields[0].text == 'TRUE' if fields and len(fields) > 0 else False
        if on:
            if self._show_comment:
                self._append_to_file('{}# open_bio_gripper(speed={}, wait={})'.format(prefix, speed, wait))
            self._append_to_file('{}if not params[\'quit\'] and arm.open_bio_gripper(speed={}, wait={}) != 0:'.format(prefix, speed, wait))
            self._append_to_file('{}    params[\'quit\'] = True'.format(prefix))
        else:
            if self._show_comment:
                self._append_to_file('{}# close_bio_gripper(speed={}, wait={})'.format(prefix, speed, wait))
            self._append_to_file('{}if not params[\'quit\'] and arm.close_bio_gripper(speed={}, wait={}) != 0:'.format(prefix, speed, wait))
            self._append_to_file('{}    params[\'quit\'] = True'.format(prefix))

    def _handle_set_robotiq_init(self, block, prefix=''):
        if self._show_comment:
            self._append_to_file('{}# set_robotiq_init()'.format(prefix))
        self._append_to_file('{}if not params[\'quit\'] and arm.robotiq_reset() != 0:'.format(prefix))
        self._append_to_file('{}    params[\'quit\'] = True'.format(prefix))
        self._append_to_file('{}if not params[\'quit\'] and arm.robotiq_set_activate(wait=True) != 0:'.format(prefix))
        self._append_to_file('{}    params[\'quit\'] = True'.format(prefix))

    def _handle_set_robotiq_gripper(self, block, prefix=''):
        fields = self.get_nodes('field', root=block, name='pos')
        pos = int(fields[0].text)
        fields = self.get_nodes('field', root=block, name='speed')
        speed = int(fields[0].text) if fields and len(fields) > 0 else 0xFF
        fields = self.get_nodes('field', root=block, name='force')
        force = int(fields[0].text) if fields and len(fields) > 0 else 0xFF
        fields = self.get_nodes('field', root=block, name='wait')
        wait = fields[0].text == 'TRUE' if fields and len(fields) > 0 else False
        if self._show_comment:
            self._append_to_file('{}# robotiq_set_position({}, speed={}, force={}, wait={})'.format(prefix, pos, speed, force, wait))
        self._append_to_file('{}if not params[\'quit\'] and arm.robotiq_set_position({}, speed={}, force={}, wait={}) != 0:'.format(prefix, pos, speed, force, wait))
        self._append_to_file('{}    params[\'quit\'] = True'.format(prefix))

    def __handle_gpio_event(self, gpio_type, block, prefix=''):
        fields = self.get_nodes('field', root=block)
        io = fields[0].text
        trigger = fields[1].text

        if 'gpio' not in self._events:
            num = 1
        else:
            if gpio_type not in self._events['gpio']:
                num = 1
            else:
                num = self._events['gpio'][gpio_type] + 1

        if gpio_type == 'tgpio_digital':
            name = 'tool_gpio_{}_digital_is_changed_callback_{}'.format(io, num)
            self._append_to_file('\n\n{}# Define Tool GPIO-{} DIGITAL is changed callback'.format(prefix, io))
        elif gpio_type == 'tgpio_analog':
            name = 'tool_gpio_{}_analog_is_changed_callback_{}'.format(io, num)
            self._append_to_file('\n\n{}# Define Tool GPIO-{} ANALOG is changed callback'.format(prefix, io))
        elif gpio_type == 'cgpio_digital':
            name = 'controller_gpio_{}_digital_is_changed_callback_{}'.format(io, num)
            self._append_to_file('\n\n{}# Define Contoller GPIO-{} DIGITAL is {} callback'.format(prefix, io, trigger))
        elif gpio_type == 'cgpio_analog':
            name = 'controller_gpio_{}_digital_is_changed_callback_{}'.format(io, num)
            self._append_to_file('\n\n{}# Define Contoller GPIO-{} ANALOG is changed callback'.format(prefix, io))
        else:
            return
        self._append_to_file('{}def {}():'.format(prefix, name))
        old_prefix = prefix
        prefix = '    ' + prefix
        statement = self.get_node('statement', root=block)
        if statement:
            self.parse(statement, prefix)
        else:
            self._append_to_file('{}pass'.format(prefix))

        if gpio_type == 'tgpio_digital':
            self._append_to_file(
                '\n{}params[\'events\'][\'gpio\'].tgpio_digital_callbacks.append({{'
                '\'io\': {}, \'trigger\': {}, \'op\': \'==\', \'callback\': {}}})'.format(
                    old_prefix, io, 1 if trigger == 'HIGH' else 0, name))
        elif gpio_type == 'tgpio_analog':
            op = self._ops2.get(trigger)
            trigger = fields[2].text
            self._append_to_file(
                '\n{}params[\'events\'][\'gpio\'].tgpio_analog_callbacks.append({{'
                '\'io\': {}, \'trigger\': {}, \'op\': \'{}\', \'callback\': {}}})'.format(
                    old_prefix, io, trigger, op, name))
        elif gpio_type == 'cgpio_digital':
            self._append_to_file(
                '\n{}params[\'events\'][\'gpio\'].cgpio_callbacks.append({{'
                '\'type\': \'digital\', \'io\': {}, \'trigger\': {}, \'op\': \'{}\', \'callback\': {}}})'.format(
                    old_prefix, io, 1 if trigger == 'HIGH' else 0, '==', name))
        elif gpio_type == 'cgpio_analog':
            op = self._ops2.get(trigger)
            trigger = fields[2].text
            self._append_to_file(
                '\n{}params[\'events\'][\'gpio\'].cgpio_callbacks.append({{'
                '\'type\': \'analog\', \'io\': {}, \'trigger\': {}, \'op\': \'{}\', \'callback\': {}}})'.format(
                    old_prefix, io, trigger, op, name))
        else:
            return
        self._append_to_file('{}if not params[\'events\'][\'gpio\'].alive:'.format(old_prefix))
        self._append_to_file('{}    params[\'events\'][\'gpio\'].start()'.format(old_prefix))

        if 'gpio' not in self._events:
            name2 = 'EventGPIOThread'
            self._insert_to_file(self.index, '\n\n# Define GPIO callback handle thread')
            self._insert_to_file(self.index, 'class {}(threading.Thread):'.format(name2))
            self._insert_to_file(self.index, '    def __init__(self, *args, **kwargs):'
                                             '\n        threading.Thread.__init__(self, *args, **kwargs)')
            self._insert_to_file(self.index, '        self.daemon = True')
            self._insert_to_file(self.index, '        self.alive = False')

            self._insert_to_file(self.index, '        self.values = {'
                                             '\'tgpio\': {\'digital\': [-1] * 2, \'analog\': [-1] * 2},'
                                             '\'cgpio\': {\'digital\': [-1] * 8, \'analog\': [-1] * 2}}')

            self._insert_to_file(self.index, '        self.tgpio_digital_callbacks = []')
            self._insert_to_file(self.index, '        self.tgpio_analog_callbacks = []')
            self._insert_to_file(self.index, '        self.cgpio_callbacks = []')

            self._insert_to_file(self.index, '\n    def run(self):')
            self._insert_to_file(self.index, '        self.alive = True')
            self._insert_to_file(self.index, '        while arm.connected and arm.error_code == 0 and not params[\'quit\']:')

            self._insert_to_file(self.index, '            if len(self.tgpio_digital_callbacks) > 0:')
            self._insert_to_file(self.index, '                _, values = arm.get_tgpio_digital()')
            self._insert_to_file(self.index, '                if _ == 0:')
            self._insert_to_file(self.index, '                    for item in self.tgpio_digital_callbacks:')
            self._insert_to_file(self.index, '                        for io in range(2):')
            self._insert_to_file(self.index, '                            if item[\'io\'] == io and eval(\'{} {} {}\'.format(values[io], item[\'op\'], item[\'trigger\'])) and not eval(\'{} {} {}\'.format(self.values[\'tgpio\'][\'digital\'][io], item[\'op\'], item[\'trigger\'])):')
            # self._insert_to_file(self.index, '                            if item[\'io\'] == io and values[io] {op} item[\'trigger\'] and not (values[io] {op} self.values[\'tgpio\'][\'digital\'][io]):'.format(op='item[\'op\']'))
            self._insert_to_file(self.index, '                                item[\'callback\']()')
            self._insert_to_file(self.index, '                    self.values[\'tgpio\'][\'digital\'] = values')

            self._insert_to_file(self.index, '            if len(self.tgpio_analog_callbacks) > 0:')
            self._insert_to_file(self.index, '                _, values = arm.get_tgpio_analog()')
            self._insert_to_file(self.index, '                if _ == 0:')
            self._insert_to_file(self.index, '                    for item in self.tgpio_analog_callbacks:')
            self._insert_to_file(self.index, '                        for io in range(2):')
            self._insert_to_file(self.index, '                            if item[\'io\'] == io and eval(\'{} {} {}\'.format(values[io], item[\'op\'], item[\'trigger\'])) and not eval(\'{} {} {}\'.format(self.values[\'tgpio\'][\'analog\'][io], item[\'op\'], item[\'trigger\'])):')
            # self._insert_to_file(self.index, '                            if item[\'io\'] == io and values[io] {op} item[\'trigger\'] and not (values[io] {op} self.values[\'tgpio\'][\'analog\'][io]):'.format(op='item[\'op\']'))
            self._insert_to_file(self.index, '                                item[\'callback\']()')
            self._insert_to_file(self.index, '                    self.values[\'tgpio\'][\'analog\'] = values')

            self._insert_to_file(self.index, '            if len(self.cgpio_callbacks) > 0:')
            self._insert_to_file(self.index, '                _, values = arm.get_cgpio_state()')
            self._insert_to_file(self.index, '                if _ == 0:')
            self._insert_to_file(self.index, '                    digitals = [values[3] >> i & 0x01 if values[10][i] in [0, 255] else 1 for i in range(8)]')
            self._insert_to_file(self.index, '                    analogs = [values[6], values[7]]')
            self._insert_to_file(self.index, '                    for item in self.cgpio_callbacks:')
            self._insert_to_file(self.index, '                        if item[\'type\'] == \'digital\':')
            self._insert_to_file(self.index, '                            for io in range(8):')
            self._insert_to_file(self.index, '                                if item[\'io\'] == io and eval(\'{} {} {}\'.format(digitals[io], item[\'op\'], item[\'trigger\'])) and not eval(\'{} {} {}\'.format(self.values[\'cgpio\'][\'digital\'][io], item[\'op\'], item[\'trigger\'])):')
            # self._insert_to_file(self.index, '                                if item[\'io\'] == io and values[io] {op} item[\'trigger\'] and not (values[io] {op} self.values[\'cgpio\'][\'digital\'][io]):'.format(op='item[\'op\']'))
            self._insert_to_file(self.index, '                                    item[\'callback\']()')
            self._insert_to_file(self.index, '                        elif item[\'type\'] == \'analog\':')
            self._insert_to_file(self.index, '                            for io in range(2):')
            self._insert_to_file(self.index, '                                if item[\'io\'] == io and eval(\'{} {} {}\'.format(analogs[io], item[\'op\'], item[\'trigger\'])) and not eval(\'{} {} {}\'.format(self.values[\'cgpio\'][\'analog\'][io], item[\'op\'], item[\'trigger\'])):')
            # self._insert_to_file(self.index, '                                if item[\'io\'] == io and values[io] {op} item[\'trigger\'] and not (values[io] {op} self.values[\'cgpio\'][\'analog\'][io]):'.format(op='item[\'op\']'))
            self._insert_to_file(self.index, '                                    item[\'callback\']()')
            self._insert_to_file(self.index, '                    self.values[\'cgpio\'][\'digital\'] = digitals')
            self._insert_to_file(self.index, '                    self.values[\'cgpio\'][\'analog\'] = analogs')

            self._insert_to_file(self.index, '            time.sleep(0.1)')
            self._insert_to_file(self.index, '\nparams[\'events\'][\'gpio\'] = {}()'.format(name2))
            self._events['gpio'] = {}

        if gpio_type not in self._events['gpio']:
            self._events['gpio'][gpio_type] = 2
        else:
            self._events['gpio'][gpio_type] += 1

        self._hasEvent = True

    def _handle_event_gpio_digital(self, block, prefix=''):
        self.__handle_gpio_event('tgpio_digital', block, prefix)

    def _handle_event_gpio_analog(self, block, prefix=''):
        self.__handle_gpio_event('tgpio_analog', block, prefix)

    def _handle_event_gpio_controller_digital(self, block, prefix):
        self.__handle_gpio_event('cgpio_digital', block, prefix)

    def _handle_event_gpio_controller_analog(self, block, prefix):
        self.__handle_gpio_event('cgpio_analog', block, prefix)

    # def _handle_event_gpio_digital(self, block, prefix=''):
    #     fields = self.get_nodes('field', root=block)
    #     io = fields[0].text
    #     trigger = fields[1].text
    #
    #     if 'gpio' not in self._events:
    #         num = 1
    #     else:
    #         num = self._events['gpio'] + 1
    #     name = '{}_io{}_is_{}_{}'.format(block.attrib['type'], io, trigger.lower(), num)
    #     self._append_to_file('\n\n{}# Define TGPIO-{} is {} callback'.format(prefix, io, trigger))
    #     self._append_to_file('{}def {}():'.format(prefix, name))
    #     old_prefix = prefix
    #     prefix = '    ' + prefix
    #     statement = self.get_node('statement', root=block)
    #     if statement:
    #         self.parse(statement, prefix)
    #     else:
    #         self._append_to_file('{}pass'.format(prefix))
    #     self._append_to_file('\n{}params[\'events\'][\'gpio\'].callbacks[\'IO{}\'][{}].append({})'.format(
    #         old_prefix, io, 1 if trigger == 'HIGH' else 0, name))
    #     self._append_to_file('{}if not params[\'events\'][\'gpio\'].alive:'.format(old_prefix))
    #     self._append_to_file('{}    params[\'events\'][\'gpio\'].start()'.format(old_prefix))
    #
    #     if 'gpio' not in self._events:
    #         name2 = 'EventGPIOThread'.format(io, trigger.capitalize())
    #         self._insert_to_file(self.index, '\n\n# Define GPIO callback handle thread')
    #         self._insert_to_file(self.index, 'class {}(threading.Thread):'.format(name2))
    #         self._insert_to_file(self.index, '    def __init__(self, *args, **kwargs):'
    #                                          '\n        threading.Thread.__init__(self, *args, **kwargs)')
    #         self._insert_to_file(self.index, '        self.daemon = True')
    #         self._insert_to_file(self.index, '        self.alive = False')
    #         self._insert_to_file(self.index, '        self.digital = [-1, -1]')
    #         self._insert_to_file(self.index, '        self.callbacks = {\'IO0\': {0: [], 1: []}, '
    #                                          '\'IO1\': {0: [], 1: []}}')
    #         self._insert_to_file(self.index, '\n    def run(self):')
    #         self._insert_to_file(self.index, '        self.alive = True')
    #         self._insert_to_file(self.index, '        while arm.connected and arm.error_code == 0:')
    #         self._insert_to_file(self.index, '            _, digital = arm.get_tgpio_digital()')
    #         self._insert_to_file(self.index, '            if _ == 0:')
    #         self._insert_to_file(self.index, '                if digital[0] != self.digital[0]:')
    #         self._insert_to_file(self.index, '                    for callback in self.callbacks[\'IO0\'][digital[0]]:')
    #         self._insert_to_file(self.index, '                        callback()')
    #         self._insert_to_file(self.index, '                if digital[1] != self.digital[1]:')
    #         self._insert_to_file(self.index, '                    for callback in self.callbacks[\'IO1\'][digital[1]]:')
    #         self._insert_to_file(self.index, '                        callback()')
    #         self._insert_to_file(self.index, '            if _ == 0:')
    #         self._insert_to_file(self.index, '                self.digital = digital')
    #         self._insert_to_file(self.index, '            time.sleep(0.1)')
    #         self._insert_to_file(self.index, '\nparams[\'events\'][\'gpio\'] = {}()'.format(name2))
    #
    #     if 'gpio' not in self._events:
    #         self._events['gpio'] = 2
    #     else:
    #         self._events['gpio'] += 1
    #
    #     self._hasEvent = True

    def _handle_procedures_defnoreturn(self, block, prefix=''):
        if not self._funcs:
            name = 'MyDef'
            self._insert_to_file(self.first_index, '\n\n# Define Mydef class')
            self._insert_to_file(self.first_index, 'class {}(object):'.format(name))
            self._insert_to_file(self.first_index,
                                 '    def __init__(self, *args, **kwargs):\n        pass')
        field = self.get_node('field', block).text
        if not field:
            field = '1'
        if field not in self._funcs:
            name = 'function_{}'.format(self.func_index)
        else:
            name = self._funcs[field]
        self._is_insert = True
        try:
            self._append_to_file('\n    @classmethod')
            args = self.get_nodes('arg', root=self.get_node('mutation', block))
            if not args:
                self._append_to_file('    def {}(cls):'.format(name))
            else:
                self._append_to_file('    def {}(cls, {}):'.format(name, ','.join([arg.attrib['name'] for arg in args])))
            # self._append_to_file('    def {}(cls):'.format(name))
            prefix = '        '
            comment = self.get_node('comment', block).text
            self._append_to_file('{}"""'.format(prefix))
            self._append_to_file('{}{}'.format(prefix, comment))
            self._append_to_file('{}"""'.format(prefix))
            statement = self.get_node('statement', root=block)
            if statement:
                self.parse(statement, prefix)
            else:
                self._append_to_file('{}pass'.format(prefix))
            self._funcs[field] = name
        except:
            self._succeed = False
        self._is_insert = False

    def _handle_procedures_defreturn(self, block, prefix=''):
        self._handle_procedures_defnoreturn(block, prefix)
        value = self.get_node('value', root=block)
        expression = self.__get_condition_expression(value)
        self._is_insert = True
        prefix = '        '
        self._append_to_file('{}return {}'.format(prefix, expression))
        self._is_insert = False

    def _handle_procedures_callnoreturn(self, block, prefix=''):
        mutation = self.get_node('mutation', block).attrib['name']
        if not mutation:
            mutation = '1'
        if mutation in self._funcs:
            name = self._funcs[mutation]
        else:
            name = 'function_{}'.format(self.func_index)

        args = self.get_nodes('arg', root=self.get_node('mutation', block))
        values = self.get_nodes('value', root=block)
        if args and values and len(args) == len(values):
            self._append_to_file('{}MyDef.{}({})'.format(prefix, name, ','.join([self.__get_condition_expression(val) for val in values])))
        else:
            self._append_to_file('{}MyDef.{}()'.format(prefix, name))

        # self._append_to_file('{}MyDef.{}()'.format(prefix, name))
        self._funcs[mutation] = name

    def _handle_procedures_ifreturn(self, block, prefix=''):
        self._is_insert = True
        values = self.get_nodes('value', block)
        expression = self.__get_condition_expression(values[0])
        self._append_to_file('{}if {}:'.format(prefix, expression))
        expression = self.__get_condition_expression(values[1])
        self._append_to_file('{}    return {}'.format(prefix, expression))
        self._is_insert = False

    def _handle_procedures_callreturn(self, block, prefix=''):
        self._handle_procedures_callnoreturn(block, prefix)

    def _handle_variables_set(self, block, prefix=''):
        field = self.get_node('field', block).text
        value = self.get_node('value', root=block)
        expression = self.__get_condition_expression(value)
        # self._append_to_file('{}params[\'variables\'][\'{}\'] = {}'.format(prefix, field, expression))

        self._append_to_file('{}if \'{}\' in locals():'.format(prefix, field))
        self._append_to_file('{}    {} = {}'.format(prefix, field, expression))
        self._append_to_file('{}else:'.format(prefix))
        self._append_to_file('{}    params[\'variables\'][\'{}\'] = {}'.format(prefix, field, expression))

    def _handle_math_change(self, block, prefix=''):
        field = self.get_node('field', block).text
        value = self.get_node('value', root=block)
        shadow = self.get_node('shadow', root=value)
        val = self.get_node('field', root=shadow).text
        # self._append_to_file('{}params[\'variables\'][\'{}\'] += {}'.format(prefix, field, val))

        self._append_to_file('{}if \'{}\' in locals():'.format(prefix, field))
        self._append_to_file('{}    {} += {}'.format(prefix, field, val))
        self._append_to_file('{}else:'.format(prefix))
        self._append_to_file('{}    params[\'variables\'][\'{}\'] += {}'.format(prefix, field, val))

    def _handle_controls_repeat_ext(self, block, prefix=''):
        value = self.get_node('value', root=block)
        times = self.get_nodes('field', root=value, descendant=True)[0].text
        self._append_to_file('{}for i in range(int({})):'.format(prefix, times))
        prefix = '    ' + prefix
        self._append_to_file('{}if params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    break'.format(prefix))
        statement = self.get_node('statement', root=block)
        if statement:
            self.parse(statement, prefix)
        else:
            self._append_to_file('{}pass'.format(prefix))

    # def handle_controls_for(self, block, prefix=''):
    #     print(block.attrib.get('disabled', False))

    def _handle_controls_whileUntil(self, block, prefix=''):
        field = self.get_node('field', root=block)
        if field.text == 'WHILE':
            value = self.get_node('value', root=block)
            expression = self.__get_condition_expression(value)
            self._append_to_file('{}while {} and not params[\'quit\']:'.format(prefix, expression))
        elif field.text == 'UNTIL':
            value = self.get_node('value', root=block)
            expression = self.__get_condition_expression(value)
            self._append_to_file('{}while not {} and not params[\'quit\']:'.format(prefix, expression))
        prefix = '    ' + prefix
        statement = self.get_node('statement', root=block)
        if statement:
            self.parse(statement, prefix)
        else:
            self._append_to_file('{}pass'.format(prefix))

    def _handle_loop_run_forever(self, block, prefix=''):
        self._append_to_file('{}while True:'.format(prefix))
        prefix = '    ' + prefix
        self._append_to_file('{}if params[\'quit\']:'.format(prefix))
        self._append_to_file('{}    break'.format(prefix))
        statement = self.get_node('statement', root=block)
        if statement:
            self.parse(statement, prefix)
        else:
            self._append_to_file('{}pass'.format(prefix))

    def _handle_loop_break(self, block, prefix=''):
        self._append_to_file('{}break'.format(prefix))

    def _handle_tool_comment(self, block, prefix=''):
        field = self.get_node('field', block)
        self._append_to_file('{}# {}'.format(prefix, field.text))
        statement = self.get_node('statement', block)
        if statement:
            self.parse(statement, prefix)

    def _handle_tool_app_comment(self, block, prefix=''):
        field = self.get_node('field', block)
        self._append_to_file('{}# [APP] {}'.format(prefix, field.text))
        statement = self.get_node('statement', block)
        if statement:
            self.parse(statement, prefix)

    def _handle_tool_remark(self, block, prefix=''):
        field = self.get_node('field', block)
        self._append_to_file('{}# {}'.format(prefix, field.text))

    def _handle_controls_if(self, block, prefix=''):
        value = self.get_node('value', root=block)
        expression = self.__get_condition_expression(value)
        self._append_to_file('{}if {}:'.format(prefix, expression))
        old_prefix = prefix
        prefix = '    ' + prefix
        statement_if = self.get_nodes('statement', root=block, name='DO0')
        statement_else = self.get_nodes('statement', root=block, name='ELSE')
        if statement_if:
            self.parse(statement_if[0], prefix)
            if statement_else:
                self._append_to_file('{}else:'.format(old_prefix))
                self.parse(statement_else[0], prefix)
        else:
            self._append_to_file('{}pass'.format(prefix))
        # statement = self.get_node('statement', root=block)
        # if statement:
        #     self.parse(statement, prefix)
        # else:
        #     self._append_to_file('{}pass'.format(prefix))

    def __get_condition_expression(self, value_block):
        block = self.get_node('block', value_block)
        if block is None:
            shadow = self.get_node('shadow', root=value_block)
            return self.get_node('field', root=shadow).text
        if block.attrib['type'] == 'logic_boolean':
            return str(self.get_node('field', block).text == 'TRUE')
        elif block.attrib['type'] == 'logic_compare':
            op = self._ops.get(self.get_node('field', block).text)
            cond_a = 0
            cond_b = 0
            values = self.get_nodes('value', block)
            if len(values) > 0:
                cond_a = self.__get_condition_expression(values[0])
                if len(values) > 1:
                    cond_b = self.__get_condition_expression(values[1])
            return '{} {} {}'.format(cond_a, op, cond_b)
        elif block.attrib['type'] == 'logic_operation':
            op = self.get_node('field', block).text.lower()
            cond_a = False
            cond_b = False
            values = self.get_nodes('value', block)
            if len(values) > 0:
                cond_a = self.__get_condition_expression(values[0])
                if len(values) > 1:
                    cond_b = self.__get_condition_expression(values[1])
            return '{} {} {}'.format(cond_a, op, cond_b)
        elif block.attrib['type'] == 'logic_negate':
            value = self.get_node('value', root=block)
            return 'not ({})'.format(self.__get_condition_expression(value))
        elif block.attrib['type'] == 'gpio_get_digital':
            io = self.get_node('field', block).text
            return 'arm.get_tgpio_digital({})[{}]'.format(io, 1)
        elif block.attrib['type'] == 'gpio_get_analog':
            io = self.get_node('field', block).text
            return 'arm.get_tgpio_analog({})[{}]'.format(io, 1)
        elif block.attrib['type'] == 'gpio_get_controller_digital':
            io = self.get_node('field', block).text
            return 'arm.get_cgpio_digital({})[{}]'.format(io, 1)
        elif block.attrib['type'] == 'gpio_get_controller_analog':
            io = self.get_node('field', block).text
            return 'arm.get_cgpio_analog({})[{}]'.format(io, 1)
        elif block.attrib['type'] == 'get_suction_cup':
            return 'arm.get_suction_cup()[{}]'.format(1)
        elif block.attrib['type'] == 'check_air_pump_state':
            fields = self.get_nodes('field', root=block)
            state = 1 if fields[0].text == 'ON' else 0
            timeout = float(fields[1].text)
            return 'arm.arm.check_air_pump_state({}, timeout={})'.format(state, timeout)
        elif block.attrib['type'] == 'math_number':
            val = self.get_node('field', block).text
            return val
        elif block.attrib['type'] == 'math_arithmetic':
            field = self.get_node('field', block).text
            values = self.get_nodes('value', block)
            if len(values) > 1:
                val_a = self.__get_block_val(values[0])
                val_b = self.__get_block_val(values[1])
                if field == 'ADD':
                    return '{} + {}'.format(val_a, val_b)
                elif field == 'MINUS':
                    return '{} - {}'.format(val_a, val_b)
                elif field == 'MULTIPLY':
                    return '{} * {}'.format(val_a, val_b)
                elif field == 'DIVIDE':
                    return '{} / {}'.format(val_a, val_b)
                elif field == 'POWER':
                    return 'pow({}, {})'.format(val_a, val_b)
        elif block.attrib['type'] == 'math_number_property':
            field = self.get_node('field', block).text
            values = self.get_nodes('value', block)
            if len(values) >= 1:
                val_a = self.__get_block_val(values[0])

                if field == 'EVEN':
                    # 偶数
                    return '{} % 2 == 0'.format(val_a)
                elif field == 'ODD':
                    # 奇数
                    return '{} % 2 == 1'.format(val_a)
                elif field == 'PRIME':
                    # 质数
                    return 'utils.is_prime({})'.format(val_a)
                elif field == 'WHOLE':
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
                        val_b = self.__get_block_val(values[1])
                    else:
                        val_b = 0
                    return '{} % {} == 0'.format(val_a, val_b)
        elif block.attrib['type'] == 'math_random_int':
            values = self.get_nodes('value', block)
            if len(values) > 1:
                val_a = self.__get_block_val(values[0])
                val_b = self.__get_block_val(values[1])
                return 'random.randint({}, {})'.format(val_a, val_b)
        elif block.attrib['type'] == 'math_round':
            field = self.get_node('field', block).text
            values = self.get_nodes('value', block)
            if len(values) >= 1:
                val_a = self.__get_block_val(values[0])
                if field == 'ROUND':
                    # 四舍五入
                    return 'round({})'.format(val_a)
                elif field == 'ROUNDUP':
                    # 上舍入
                    return 'math.ceil({})'.format(val_a)
                elif field == 'ROUNDDOWN':
                    # 下舍入
                    return 'math.floor({})'.format(val_a)
        elif block.attrib['type'] == 'math_single':
            # 算术函数
            field = self.get_node('field', block).text
            values = self.get_nodes('value', block)
            if len(values) >= 1:
                val_a = self.__get_block_val(values[0])
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
                    return 'math.log({}) / math.log(10)'.format(val_a)
                elif field == 'EXP':
                    # exp
                    return 'math.exp({})'.format(val_a)
                elif field == 'POW10':
                    # 10的多少次方
                    return 'math.pow(10, {})'.format(val_a)
        elif block.attrib['type'] == 'math_trig':
            # 三角函数
            field = self.get_node('field', block).text
            values = self.get_nodes('value', block)
            if len(values) >= 1:
                val_a = self.__get_block_val(values[0])
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
        elif block.attrib['type'] == 'math_constant':
            # 常量
            field = self.get_node('field', block).text
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
        elif block.attrib['type'] == 'math_modulo':
            values = self.get_nodes('value', block)
            if len(values) > 1:
                val_a = self.__get_block_val(values[0])
                val_b = self.__get_block_val(values[1])
                return '{} % {}'.format(val_a, val_b)
        elif block.attrib['type'] == 'math_constrain':
            values = self.get_nodes('value', block)
            if len(values) > 2:
                val_a = self.__get_block_val(values[0])
                val_b = self.__get_block_val(values[1])
                val_c = self.__get_block_val(values[2])
                return 'min(max({}, {}), {})'.format(val_a, val_b, val_c)
        # elif block.attrib['type'] == 'math_round':
        #     pass
        elif block.attrib['type'] == 'variables_get':
            field = self.get_node('field', block).text
            return '(params[\'variables\'].get(\'{}\', 0) if \'{}\' not in locals() else {})'.format(field, field, field)
            # return 'params[\'variables\'].get(\'{}\', 0)'.format(field)
        elif block.attrib['type'] == 'move_var':
            val = self.get_node('field', block).text
            return val
        elif block.attrib['type'] == 'tool_get_date':
            return 'datetime.datetime.now()'
        elif block.attrib['type'] == 'tool_combination':
            field = self.get_node('field', block).text
            values = self.get_nodes('value', block)
            var1 = self.__get_condition_expression(values[0])
            var2 = self.__get_condition_expression(values[1])
            return '\'{{}}{{}}{{}}\'.format({}, \'{}\', {})'.format(var1, field, var2)
        elif block.attrib['type'] == 'procedures_callreturn':
            mutation = self.get_node('mutation', block).attrib['name']
            if not mutation:
                mutation = '1'
            if mutation in self._funcs:
                name = self._funcs[mutation]
            else:
                name = 'function_{}'.format(self.func_index)

            args = self.get_nodes('arg', root=self.get_node('mutation', block))
            values = self.get_nodes('value', root=block)
            if args and values and len(args) == len(values):
                return 'MyDef.{}({})'.format(name, ','.join(
                    [self.__get_condition_expression(val) for val in values]))
            else:
                return 'MyDef.{}()'.format(name)
            # return 'MyDef.{}()'.format(name)

    def __get_block_val(self, block):
        block_v = self.get_node('block', root=block)
        if block_v is not None:
            val = self.__get_condition_expression(block)
        else:
            shadow = self.get_node('shadow', root=block)
            val = self.get_node('field', root=shadow).text
        return val


if __name__ == '__main__':
    blockly = BlocklyTool('C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm6\\app\\myapp\local_test_1\\app.xml')
    # blockly = BlocklyTool('C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm6\\app\\myapp\\app_template\\app.xml')
    # blockly = BlocklyTool('C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm6\\app\\myapp\\test_gpio\\app.xml')
    # blockly = BlocklyTool('C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm7\\app\\myapp\\pour_water\\app.xml')
    # blockly = BlocklyTool('C:\\Users\\ufactory\\.UFACTORY\projects\\test\\xarm7\\app\\myapp\\233\\app.xml')
    import os
    target_path = os.path.join(os.path.expanduser('~'), '.UFACTORY', 'app', 'tmp')
    if not os.path.exists(target_path):
        os.makedirs(target_path)
    target_file = os.path.join(target_path, 'blockly_app.py')
    blockly.to_python(target_file, arm='192.168.1.145')
