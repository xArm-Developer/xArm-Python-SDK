#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import json
from ._blockly_handler import _BlocklyHandler


class BlocklyTool(_BlocklyHandler):
    def __init__(self, xml_path):
        super(BlocklyTool, self).__init__(xml_path)
        self._is_converted = False
        self._codes = []
    
    @property
    def codes(self):
        return '\n'.join(self._codes)

    def to_python(self, path=None, arm=None, init=True, wait_seconds=1, mode=0, state=0, error_exit=True, stop_exit=True, **kwargs):
        if not self._is_converted:
            self._is_exec = kwargs.get('is_exec', False)
            self._is_ide = kwargs.get('is_ide', False)
            self._vacuum_version = kwargs.get('vacuum_version', '1')
            # highlight_callback: only use pack to run blockly in studio
            self._highlight_callback = kwargs.get('highlight_callback', None)
            # axis_type: Obtain the type of mechanical arm axis for end leveling use
            if 'axis_type' in kwargs:
                self.axis_type = kwargs.get('axis_type', [])
            else:
                self.axis_type = []
            # loop_max_frequency: limit frequency in loop, only use pack to run blockly in studio
            if 'loop_max_frequency' in kwargs:
                loop_max_freq = kwargs.get('loop_max_frequency', -1)
                if loop_max_freq < 0 or loop_max_freq > 10000:
                    self._loop_interval_sec = 0
                else:
                    self._loop_interval_sec = 1 / loop_max_freq
            elif self._highlight_callback:
                self._loop_interval_sec = 0.001
            self._init_header_codes()
            self._init_robot_main_run_codes()
            self._parse_block()
            self._finish_robot_main_run_codes(error_exit, stop_exit)
            self._init_robot_main_class_codes(init=init, wait_seconds=wait_seconds, mode=mode, state=state, error_exit=error_exit, stop_exit=stop_exit)
            self._init_main_codes(arm=arm)
            self._codes.extend(self._init_code_list)
            self._codes.extend(self._main_init_code_list)
            self._codes.extend(self._main_func_code_list)
            self._codes.extend(self._main_run_code_list)
            self._is_converted = True
        if path is not None:
            with open(path, 'w', encoding='utf-8') as f:
                f.write('{}\n'.format(self.codes))
        return self._succeed

    def _init_header_codes(self):
        self._append_init_code('#!/usr/bin/env python3')
        self._append_init_code('# Software License Agreement (BSD License)')
        self._append_init_code('#')
        self._append_init_code('# Copyright (c) 2022, UFACTORY, Inc.')
        self._append_init_code('# All rights reserved.')
        self._append_init_code('#')
        self._append_init_code('# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>\n')
        self._append_init_code('"""')
        self._append_init_code('# Notice')
        self._append_init_code('#   1. Changes to this file on Studio will not be preserved')
        self._append_init_code('#   2. The next conversion will overwrite the file with the same name')
        self._append_init_code('# ')
        self._append_init_code('# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK')
        self._append_init_code('#   1. git clone git@github.com:xArm-Developer/xArm-Python-SDK.git')
        self._append_init_code('#   2. cd xArm-Python-SDK')
        self._append_init_code('#   3. python setup.py install')
        self._append_init_code('"""')
        self._append_init_code('import sys')
        self._append_init_code('import math')
        self._append_init_code('import time')
        self._append_init_code('import queue')
        self._append_init_code('import datetime')
        self._append_init_code('import random')
        self._append_init_code('import traceback')
        self._append_init_code('import threading')
        self._append_init_code('from xarm import version')
        self._append_init_code('from xarm.wrapper import XArmAPI\n\n')

    def _init_robot_main_run_codes(self):
        self._append_main_code('    # Robot Main Run', indent=-1)
        self._append_main_code('    def run(self):', indent=-1)
        self._append_main_code('        try:', indent=-1)
    
    def _finish_robot_main_run_codes(self, error_exit=True, stop_exit=True):
        if self._listen_tgpio_digital or self._listen_tgpio_analog or self._listen_cgpio_state \
            or len(self._tgpio_digital_callbacks) or len(self._tgpio_analog_callbacks) or len(self._cgpio_digital_callbacks) or len(self._cgpio_analog_callbacks) or len(self._count_callbacks) or len(self._holding_callbacks):
            self._append_main_code('            # Event Loop', indent=-1)
            self._append_main_code('            while self.is_alive:', indent=-1)
            self._append_main_code('                time.sleep(0.5)', indent=-1)
        # catch exception and release callback
        self._append_main_code('        except Exception as e:', indent=-1)
        self._append_main_code('            self.pprint(\'MainException: {}\'.format(e))', indent=-1)
        self._append_main_code('        finally:', indent=-1)
        self._append_main_code('            self.alive = False', indent=-1)
        if stop_exit or error_exit:
            if error_exit:
                self._append_main_code('            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)', indent=-1)
            if stop_exit:
                self._append_main_code('            self._arm.release_state_changed_callback(self._state_changed_callback)', indent=-1)
        if self._listen_counter:
            self._append_main_code('            if hasattr(self._arm, \'release_count_changed_callback\'):', indent=-1)
            self._append_main_code('                self._arm.release_count_changed_callback(self._count_changed_callback)', indent=-1)

        # if self._listen_tgpio_digital or self._listen_tgpio_analog or self._listen_cgpio_state \
        #     or len(self._tgpio_digital_callbacks) or len(self._tgpio_analog_callbacks) or len(self._cgpio_digital_callbacks) or len(self._cgpio_analog_callbacks):
        #     self._append_main_code('        # Event Loop', indent=-1)
        #     self._append_main_code('        while self.is_alive:', indent=-1)
        #     self._append_main_code('            time.sleep(0.5)', indent=-1)
        # self._append_main_code('        self.alive = False', indent=-1)
        # if stop_exit or error_exit:
        #     if error_exit:
        #         self._append_main_code('        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)', indent=-1)
        #     if stop_exit:
        #         self._append_main_code('        self._arm.release_state_changed_callback(self._state_changed_callback)', indent=-1)
        # self._append_main_code('        if hasattr(self._arm, \'release_count_changed_callback\'):', indent=-1)
        # self._append_main_code('            self._arm.release_count_changed_callback(self._count_changed_callback)', indent=-1)

    def _init_robot_main_class_codes(self, init=True, wait_seconds=1, mode=0, state=0, error_exit=True, stop_exit=True):
        self._append_main_init_code('class RobotMain(object):')
        self._append_main_init_code('    """Robot Main Class"""')
        self._append_main_init_code('    def __init__(self, robot, **kwargs):')
        self._append_main_init_code('        self.alive = True')
        self._append_main_init_code('        self._arm = robot')
        self._append_main_init_code('        self._ignore_exit_state = False')
        self._append_main_init_code('        self._tcp_speed = 100')
        self._append_main_init_code('        self._tcp_acc = 2000')
        self._append_main_init_code('        self._angle_speed = 20')
        self._append_main_init_code('        self._angle_acc = 500')
        self._append_main_init_code('        self._vars = {}'.format({var: 0 for var in self._parse_variables()}))
        if len(self._funcs):
            self._append_main_init_code('        self._funcs = {')
            for key, val in self._funcs.items():
                self._append_main_init_code('            {}: self.{},'.format(json.dumps(key, ensure_ascii=False), val))
            self._append_main_init_code('        }')
        else:
            self._append_main_init_code('        self._funcs = {}')
        self._append_main_init_code('        self._robot_init()')
        if len(self._tgpio_digital_callbacks):
            self._append_main_init_code('        self._tgpio_digital_callbacks = []')
        if len(self._tgpio_analog_callbacks):
            self._append_main_init_code('        self._tgpio_analog_callbacks = []')
        if len(self._cgpio_digital_callbacks):
            self._append_main_init_code('        self._cgpio_digital_callbacks = []')
        if len(self._cgpio_analog_callbacks):
            self._append_main_init_code('        self._cgpio_analog_callbacks = []')
        if len(self._count_callbacks):
            self._append_main_init_code('        self._count_callbacks = []')
        if len(self._holding_callbacks):
            self._append_main_init_code('        self._holding_callbacks = []')
        if self._listen_cgpio_state or len(self._cgpio_digital_callbacks) or len(self._cgpio_analog_callbacks):
            self._append_main_init_code('        self._cgpio_state = None')

        if self._listen_count or len(self._count_callbacks):
            self._append_main_init_code('        self._counter_val = None')
        if self._listen_holding or len(self._holding_callbacks):
            self._append_main_init_code('        self._holding_dict = {}')
        
        if len(self._tgpio_digital_callbacks) or len(self._tgpio_analog_callbacks) or len(self._cgpio_digital_callbacks) or len(self._cgpio_analog_callbacks)\
                or len(self._count_callbacks) or len(self._holding_callbacks):
            self._append_main_init_code('        self._callback_in_thread = kwargs.get(\'callback_in_thread\', True)')
            self._append_main_init_code('        self._callback_que = queue.Queue()')

        if self._listen_tgpio_digital or self._listen_tgpio_analog or self._listen_cgpio_state \
            or len(self._tgpio_digital_callbacks) or len(self._tgpio_analog_callbacks) or len(self._cgpio_digital_callbacks) or len(self._cgpio_analog_callbacks):
            self._append_main_init_code('        gpio_t = threading.Thread(target=self._listen_gpio_thread, daemon=True)')
            self._append_main_init_code('        gpio_t.start()')

        if len(self._tgpio_digital_callbacks) or len(self._tgpio_analog_callbacks) or len(self._cgpio_digital_callbacks) or len(self._cgpio_analog_callbacks)\
                or len(self._count_callbacks) or len(self._holding_callbacks):
            self._append_main_init_code('        callback_t = threading.Thread(target=self._event_callback_handle_thread, daemon=True)')
            self._append_main_init_code('        callback_t.start()')
        
        if self._listen_count or len(self._count_callbacks):
            self._append_main_init_code(
                '        count_t = threading.Thread(target=self._listen_count_thread, daemon=True)')
            self._append_main_init_code('        count_t.start()')
        
        if self._holding_callbacks or len(self._holding_callbacks):
            self._append_main_init_code(
                '        holding_t = threading.Thread(target=self._listen_holding_thread, daemon=True)')
            self._append_main_init_code('        holding_t.start()')
        
        self._append_main_init_code('')

        self.__define_callback_thread_func()
        self.__define_listen_gpio_thread_func()
        self.__define_listen_count_thread_func()
        self.__define_listen_holding_registers_thread_func()
        self.__define_run_blockly_func()
        self.__define_robot_init_func(init=init, wait_seconds=wait_seconds, mode=mode, state=state, error_exit=error_exit, stop_exit=stop_exit)
        self.__define_error_warn_changed_callback_func(error_exit=error_exit)
        self.__define_state_changed_callback_func(stop_exit=stop_exit)
        self.__define_count_changed_callback_func()
        self.__define_cgpio_digitals_is_matchs_bin_func()
        self.__define_check_code_func()
        self.__define_is_prime_func()
        self.__define_pprint_func()
        self.__define_property()
        self.__define_is_alive_property()

    def __define_is_prime_func(self):
        # Define Function: is_prime
        if self._define_is_prime_func:
            self._append_main_init_code('    @staticmethod')
            self._append_main_init_code('    def is_prime(n):')
            self._append_main_init_code('        def _is_prime()')
            self._append_main_init_code('            for i in range(6, int(math.sqrt(n) + 1), 6):')
            self._append_main_init_code('                if n % (i - 1) == 0 or n % (i + 1) == 0:')
            self._append_main_init_code('                    return False')
            self._append_main_init_code('            return True')
            self._append_main_init_code('        return n == 2 or n == 3 or (n > 1 and n % 2 != 0 and n % 3 != 0 and _is_prime())\n')
    
    def __define_cgpio_digitals_is_matchs_bin_func(self):
        # Define Funciton: cgpio_digitals_is_matchs_bin
        if self._define_bin_matchs_func:
            self._append_main_init_code('    def _cgpio_digitals_is_matchs_bin(self, bin_val):')
            self._append_main_init_code('        if self._cgpio_state is None:')
            self._append_main_init_code('            return True')
            self._append_main_init_code('        digitals_bin = \'\'.join(map(str, [self._cgpio_state[3] >> i & 0x0001 if self._cgpio_state[10][i] in [0, 255] else 1 for i in range(len(self._cgpio_state[10]))]))')
            self._append_main_init_code('        length = min(len(digitals_bin), len(bin_val))')
            self._append_main_init_code('        bin_val_ = bin_val[::-1]')
            self._append_main_init_code('        for i in range(length):')
            self._append_main_init_code('            if bin_val_[i] != digitals_bin[i]:')
            self._append_main_init_code('                return False')
            self._append_main_init_code('        return True\n')
    
    def __define_property(self):
        # define property: self.arm -> self._arm
        self._append_main_init_code('    @property')
        self._append_main_init_code('    def arm(self):')
        self._append_main_init_code('        return self._arm\n')
        # define property: self.VARS -> self._vars
        self._append_main_init_code('    @property')
        self._append_main_init_code('    def VARS(self):')
        self._append_main_init_code('        return self._vars\n')
        # define property: self.FUNCS -> self._funcs
        self._append_main_init_code('    @property')
        self._append_main_init_code('    def FUNCS(self):')
        self._append_main_init_code('        return self._funcs\n')

    def __define_callback_thread_func(self):
        # Define callback thread function
        if  len(self._tgpio_digital_callbacks) or len(self._tgpio_analog_callbacks) or len(self._cgpio_digital_callbacks) or len(self._cgpio_analog_callbacks) or len(self._count_callbacks) or len(self._holding_callbacks):
            self._append_main_init_code('    def _event_callback_handle_thread(self):')
            self._append_main_init_code('        while self.alive:')
            self._append_main_init_code('            try:')
            self._append_main_init_code('                callback = self._callback_que.get(timeout=1)')
            self._append_main_init_code('                callback() if not self._callback_in_thread else threading.Thread(target=callback, daemon=True).start()')
            self._append_main_init_code('            except queue.Empty:')
            self._append_main_init_code('                pass')
            self._append_main_init_code('            except Exception as e:')
            self._append_main_init_code('                self.pprint(e)\n')

    def __define_listen_gpio_thread_func(self):
        # Define listen gpio thread function
        if self._listen_tgpio_digital or self._listen_tgpio_analog or self._listen_cgpio_state \
            or len(self._tgpio_digital_callbacks) or len(self._tgpio_analog_callbacks) or len(self._cgpio_digital_callbacks) or len(self._cgpio_analog_callbacks):
            
            self._append_main_init_code('    def _listen_gpio_thread(self):')
            if self._listen_tgpio_digital or len(self._tgpio_digital_callbacks):
                self._append_main_init_code('        _, values2 = self._arm.get_tgpio_digital(2)')
                self._append_main_init_code('        _, values = self._arm.get_tgpio_digital()')
                self._append_main_init_code('        values.insert(2, values2)')
                self._append_main_init_code('        tgpio_digitals = values if _ == 0 else [0] * 5')
            if self._listen_tgpio_analog or len(self._tgpio_analog_callbacks):
                self._append_main_init_code('        _, values = self._arm.get_tgpio_analog()')
                self._append_main_init_code('        tgpio_analogs = values if _ == 0 else [0] * 2')
            if self._listen_cgpio_state or len(self._cgpio_digital_callbacks) or len(self._cgpio_analog_callbacks):
                self._append_main_init_code('        _, values = self._arm.get_cgpio_state()')
            if self._listen_cgpio_state or len(self._cgpio_digital_callbacks):
                self._append_main_init_code('        cgpio_digitals = [values[3] >> i & 0x0001 if values[10][i] in [0, 255] else 1 for i in range(len(values[10]))] if _ == 0 else [0] * 16')
            if self._listen_cgpio_state or len(self._cgpio_analog_callbacks):
                self._append_main_init_code('        cgpio_analogs = [values[6], values[7]] if _ == 0 else [0] * 2')

            self._append_main_init_code('        while self.alive:')
            if self._listen_tgpio_digital or len(self._tgpio_digital_callbacks):
                self._append_main_init_code('            _, values2 = self._arm.get_tgpio_digital(2)')
                self._append_main_init_code('            _, values = self._arm.get_tgpio_digital()')
                self._append_main_init_code('            values.insert(2, values2)')
                self._append_main_init_code('            if _ == 0 and tgpio_digitals is not None:')
                self._append_main_init_code('                for item in self._tgpio_digital_callbacks:')
                self._append_main_init_code('                    for io in range(5):')
                self._append_main_init_code('                        if item[\'io\'] == io and eval(\'{} {} {}\'.format(values[io], item[\'op\'], item[\'trigger\'])) and not eval(\'{} {} {}\'.format(tgpio_digitals[io], item[\'op\'], item[\'trigger\'])):')
                self._append_main_init_code('                            self._callback_que.put(item[\'callback\'])')
                self._append_main_init_code('            tgpio_digitals = values if _ == 0 else tgpio_digitals')
            if self._listen_tgpio_digital or len(self._tgpio_analog_callbacks):
                self._append_main_init_code('            _, values = self._arm.get_tgpio_analog()')
                self._append_main_init_code('            if _ == 0 and tgpio_analogs is not None:')
                self._append_main_init_code('                for item in self._tgpio_analog_callbacks:')
                self._append_main_init_code('                    for io in range(2):')
                self._append_main_init_code('                        if item[\'io\'] == io and eval(\'{} {} {}\'.format(values[io], item[\'op\'], item[\'trigger\'])) and not eval(\'{} {} {}\'.format(tgpio_analogs[io], item[\'op\'], item[\'trigger\'])):')
                self._append_main_init_code('                            self._callback_que.put(item[\'callback\'])')
                self._append_main_init_code('            tgpio_analogs = values if _ == 0 else tgpio_analogs')

            if not self._listen_cgpio_state and len(self._cgpio_digital_callbacks) == 0 and len(self._cgpio_analog_callbacks) == 0:
                self._append_main_init_code('            time.sleep(0.01)\n')
                return

            self._append_main_init_code('            _, values = self._arm.get_cgpio_state()')
            self._append_main_init_code('            if _ == 0 and self._cgpio_state is not None and self._cgpio_state != values:')
            if self._listen_cgpio_state or len(self._cgpio_digital_callbacks):
                self._append_main_init_code('                digitals = [values[3] >> i & 0x0001 if values[10][i] in [0, 255] else 1 for i in range(len(values[10]))]')
                self._append_main_init_code('                for item in self._cgpio_digital_callbacks:')
                self._append_main_init_code('                    for io in range(len(digitals)):')
                self._append_main_init_code('                        if item[\'io\'] == io and eval(\'{} {} {}\'.format(digitals[io], item[\'op\'], item[\'trigger\'])) and not eval(\'{} {} {}\'.format(cgpio_digitals[io], item[\'op\'], item[\'trigger\'])):')
                self._append_main_init_code('                            self._callback_que.put(item[\'callback\'])')
                self._append_main_init_code('                cgpio_digitals = digitals')
            if self._listen_cgpio_state or len(self._cgpio_analog_callbacks):
                self._append_main_init_code('                analogs = [values[6], values[7]]')
                self._append_main_init_code('                for item in self._cgpio_analog_callbacks:')
                self._append_main_init_code('                    for io in range(len(analogs)):')
                self._append_main_init_code('                        if item[\'io\'] == io and eval(\'{} {} {}\'.format(analogs[io], item[\'op\'], item[\'trigger\'])) and not eval(\'{} {} {}\'.format(cgpio_analogs[io], item[\'op\'], item[\'trigger\'])):')
                self._append_main_init_code('                            self._callback_que.put(item[\'callback\'])')
                self._append_main_init_code('                cgpio_analogs = analogs')
            self._append_main_init_code('            self._cgpio_state = values if _ == 0 else self._cgpio_state')
            self._append_main_init_code('            time.sleep(0.01)\n')

    def __define_listen_count_thread_func(self):
        # Define listen counter value thread function
        if self._listen_count or len(self._count_callbacks):
            self._append_main_init_code('    def _listen_count_thread(self):')
            if self._listen_count or len(self._count_callbacks):
                self._append_main_init_code('        values = self._arm.count')

            self._append_main_init_code('        while self.alive:')

            self._append_main_init_code('            values = self._arm.count')
            self._append_main_init_code(
                '            if self._counter_val is not None and self._counter_val != values:')
            if self._listen_count or len(self._count_callbacks):
                self._append_main_init_code('                for item in self._count_callbacks:')
                self._append_main_init_code(
                    '                    if eval(\'{} {} {}\'.format(values, item[\'op\'], item[\'trigger\'])) and not eval(\'{} {} {}\'.format(self._counter_val, item[\'op\'], item[\'trigger\'])):')
                self._append_main_init_code('                            self._callback_que.put(item[\'callback\'])')
            self._append_main_init_code('            self._counter_val = values')
            self._append_main_init_code('            time.sleep(0.01)\n')
    
    def __define_listen_holding_registers_thread_func(self):
        # Define listen holding registers value thread function
        if self._listen_holding or len(self._holding_callbacks):
            self._append_main_init_code('    def _listen_holding_thread(self):')

            self._append_main_init_code('        while self.alive:')
            self._append_main_init_code('            for index, item in enumerate(self._holding_callbacks):')
            self._append_main_init_code('                for i in range(2):')
            self._append_main_init_code('                    _, values = self._arm.read_holding_registers(int(item[\'addr\']), 1)')
            self._append_main_init_code(
                '                    if _ == 0 and self._holding_dict.get(index, None) is not None:')
            self._append_main_init_code(
                '                        if eval(\'{} == {}\'.format(values[0], item[\'trigger\'])) and not eval(\'{} == {}\'.format(self._holding_dict[index], item[\'trigger\'])):')
            self._append_main_init_code('                            self._callback_que.put(item[\'callback\'])')
            self._append_main_init_code('                    self._holding_dict[index] = values[0] if _ == 0 else self._holding_dict.get(index, None)')
            self._append_main_init_code('                    time.sleep(0.01)\n')

    def __define_run_blockly_func(self):
        if self._is_run_blockly and not self._is_exec:
            self._append_main_init_code('    def _start_run_blockly(self, fileName, times):')
            self._append_main_init_code('       for i in range(times):')
            self._append_main_init_code('           code = self._arm.run_blockly_app(fileName, init=False, is_exec=True, axis_type=[self._arm.axis, self._arm.device_type])\n')

    def __define_robot_init_func(self, init=True, wait_seconds=1, mode=0, state=0, error_exit=True, stop_exit=True):
        # Define XArm Init Function:
        self._append_main_init_code('    # Robot init')
        self._append_main_init_code('    def _robot_init(self):')
        if init:
            self._append_main_init_code('        self._arm.clean_warn()')
            self._append_main_init_code('        self._arm.clean_error()')
            self._append_main_init_code('        self._arm.motion_enable(True)')
            self._append_main_init_code('        self._arm.set_mode({})'.format(mode))
            self._append_main_init_code('        self._arm.set_state({})'.format(state))
        if wait_seconds > 0:
            self._append_main_init_code('        time.sleep({})'.format(wait_seconds))
        if error_exit:
            self._append_main_init_code('        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)')
        if stop_exit:
            self._append_main_init_code('        self._arm.register_state_changed_callback(self._state_changed_callback)')
        if self._listen_counter:
            self._append_main_init_code('        if hasattr(self._arm, \'register_count_changed_callback\'):')
            self._append_main_init_code('            self._arm.register_count_changed_callback(self._count_changed_callback)')
        self._append_main_init_code('')

    def __define_error_warn_changed_callback_func(self, error_exit=True):
        # Define error/warn changed callback
        if error_exit:
            self._append_main_init_code('    # Register error/warn changed callback')
            self._append_main_init_code('    def _error_warn_changed_callback(self, data):')
            self._append_main_init_code('        if data and data[\'error_code\'] != 0:')
            self._append_main_init_code('            self.alive = False')
            self._append_main_init_code('            self.pprint(\'err={}, quit\'.format(data[\'error_code\']))')
            if self.is_run_linear_track:
                self._append_main_init_code('            self._arm.set_linear_track_stop()')
            self._append_main_init_code('            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)\n')
    
    def __define_state_changed_callback_func(self, stop_exit=True):
        # Define state changed callback
        if stop_exit:
            self._append_main_init_code('    # Register state changed callback')
            self._append_main_init_code('    def _state_changed_callback(self, data):')
            self._append_main_init_code('        if not self._ignore_exit_state and data and data[\'state\'] == 4:')
            self._append_main_init_code('            self.alive = False')
            self._append_main_init_code('            self.pprint(\'state=4, quit\')')
            self._append_main_init_code('            self._arm.release_state_changed_callback(self._state_changed_callback)\n')

    def __define_count_changed_callback_func(self):
        # Define count changed callback
        if self._listen_counter:
            self._append_main_init_code('    # Register count changed callback')
            self._append_main_init_code('    def _count_changed_callback(self, data):')
            self._append_main_init_code('        if self.is_alive:')
            self._append_main_init_code('            self.pprint(\'counter val: {}\'.format(data[\'count\']))\n')

    def __define_pprint_func(self):
        self._append_main_init_code('    @staticmethod')
        self._append_main_init_code('    def pprint(*args, **kwargs):')
        self._append_main_init_code('        try:')
        self._append_main_init_code('            stack_tuple = traceback.extract_stack(limit=2)[0]')
        self._append_main_init_code('            print(\'[{}][{}] {}\'.format('
                                                'time.strftime(\'%Y-%m-%d %H:%M:%S\', time.localtime(time.time())), '
                                                'stack_tuple[1], \' \'.join(map(str, args))))')
        self._append_main_init_code('        except:')
        self._append_main_init_code('            print(*args, **kwargs)\n')

    def __define_is_alive_property(self):
        self._append_main_init_code('    @property')
        self._append_main_init_code('    def is_alive(self):')
        self._append_main_init_code('        if self.alive and self._arm.connected and self._arm.error_code == 0:')
        self._append_main_init_code('            if self._ignore_exit_state:')
        self._append_main_init_code('                return True')
        self._append_main_init_code('            if self._arm.state == 5:')
        self._append_main_init_code('                cnt = 0')
        self._append_main_init_code('                while self._arm.state == 5 and cnt < 5:')
        self._append_main_init_code('                    cnt += 1')
        self._append_main_init_code('                    time.sleep(0.1)')
        self._append_main_init_code('            return self._arm.state < 4')
        self._append_main_init_code('        else:')
        self._append_main_init_code('            return False\n')
        # self._append_main_init_code('        return self.alive and self._arm.connected and self._arm.error_code == 0 and self._arm.state < 4\n')

    def __define_check_code_func(self):
        self._append_main_init_code('    def _check_code(self, code, label):')
        self._append_main_init_code('        if not self.is_alive or code != 0:')
        self._append_main_init_code('            self.alive = False')
        # self._append_main_init_code('            self.pprint(\'{{}}, code={{}}, connected={{}}, state={{}}, error={{}}\'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code))')
        self._append_main_init_code('            ret1 = self._arm.get_state()')
        self._append_main_init_code('            ret2 = self._arm.get_err_warn_code()')
        self._append_main_init_code('            self.pprint(\'{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}\'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))')
        self._append_main_init_code('        return self.is_alive\n')

    def _init_main_codes(self, arm=None):
        # exec can not run main function, if run in exec(), the parameter is_ide must set False
        self._append_main_code('\n', indent=-1)
        if self._is_ide:
            self._append_main_code('if __name__ == \'__main__\':', indent=-1)
            indent = 0
        else:
            indent = -1
        self._append_main_code('RobotMain.pprint(\'xArm-Python-SDK Version:{}\'.format(version.__version__))', indent=indent)
        if arm is None:
            self._append_main_code('arm = XArmAPI(sys.argv[1], baud_checkset=False)', indent=indent)
            self._append_main_code('time.sleep(0.5)', indent=indent)
        elif isinstance(arm, str):
            self._append_main_code('arm = XArmAPI(\'{}\', baud_checkset=False)'.format(arm), indent=indent)
            self._append_main_code('time.sleep(0.5)', indent=indent)
            
        self._append_main_code('robot_main = RobotMain(arm)', indent=indent)
        self._append_main_code('robot_main.run()', indent=indent)
        self._append_main_code('', indent=-1)
