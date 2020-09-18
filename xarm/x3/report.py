from xarm.core.utils import convert


class ReportHandler(object):
    def __init__(self, report_type):
        self.buffer = b''
        self.report_size = 0
        self.report_type = report_type
        if self.report_type == 'devlop':
            self.parse_handler = self._parse_report_tcp_develop_data
        elif self.report_type == 'normal':
            self.parse_handler = self._parse_report_tcp_normal_data
        elif self.report_type == 'rich':
            self.parse_handler = self._parse_report_tcp_rich_data
        else:
            self.parse_handler = None
        self.source_data = b''
        self.parse_dict = {}

    def reset(self):
        self.buffer = b''
        self.report_size = 0

    def process_report_data(self, recv_data):
        if recv_data == -1:
            return
        self.buffer += recv_data
        if len(self.buffer) < 4:
            return
        if self.report_size == 0:
            self.report_size = convert.bytes_to_u32(self.buffer[:4])
        if len(self.buffer) < self.report_size:
            return
        if self.report_type == 'rich' and self.report_size == 233 and len(self.buffer) >= 245:
            # 兼容某几版固件上报的数据的数据长度和实际数据的长度不一致
            try:
                if len(self.buffer) >= 249:
                    if convert.bytes_to_u32(self.buffer[245:249]) != self.report_size:
                        if convert.bytes_to_u32(self.buffer[233:237]) == self.report_size:
                            data = self.buffer[:self.report_size]
                            self.buffer = self.buffer[self.report_size:]
                        else:
                            self.reset()
                            # TODO reconnect
                            return -1
                    else:
                        data = self.buffer[:245]
                        self.buffer = self.buffer[245:]
                else:
                    if convert.bytes_to_u32(self.buffer[233:237]) != self.report_size:
                        data = self.buffer[:245]
                        self.buffer = self.buffer[245:]
                    else:
                        data = self.buffer[:self.report_size]
                        self.buffer = self.buffer[self.report_size:]
            except:
                self.reset()
                # TODO reconnect
                return -1
        else:
            data = self.buffer[:self.report_size]
            self.buffer = self.buffer[self.report_size:]
        self.source_data = data
        if self.parse_handler:
            return self.parse_handler(data)

    def __parse_report_common_data(self, rx_data):
        # length = convert.bytes_to_u32(rx_data[0:4])
        length = len(rx_data)
        state, mode = rx_data[4] & 0x0F, rx_data[4] >> 4
        cmd_num = convert.bytes_to_u16(rx_data[5:7])
        angles = convert.bytes_to_fp32s(rx_data[7:7 * 4 + 7], 7)
        pose = convert.bytes_to_fp32s(rx_data[35:6 * 4 + 35], 6)
        torque = convert.bytes_to_fp32s(rx_data[59:7 * 4 + 59], 7)
        self.parse_dict['length'] = length
        self.parse_dict['state'] = state
        self.parse_dict['mode'] = mode
        self.parse_dict['cmd_num'] = cmd_num
        self.parse_dict['angles'] = angles
        self.parse_dict['pose'] = pose
        self.parse_dict['torque'] = torque
        return [length, state, mode, cmd_num, angles, pose, torque]

    def _parse_report_tcp_develop_data(self, rx_data):
        ret = self.__parse_report_common_data(rx_data)
        return ret

    def _parse_report_tcp_normal_data(self, rx_data):
        ret = self.__parse_report_common_data(rx_data)
        mtbrake, mtable, error_code, warn_code = rx_data[87:91]
        tcp_offset = convert.bytes_to_fp32s(rx_data[91:6 * 4 + 91], 6)
        tcp_load = convert.bytes_to_fp32s(rx_data[115:4 * 4 + 115], 4)
        collis_sens, teach_sens = rx_data[131:133]
        gravity_direction = convert.bytes_to_fp32s(rx_data[133:3 * 4 + 133], 3)
        mtbrake = [mtbrake & 0x01, mtbrake >> 1 & 0x01, mtbrake >> 2 & 0x01, mtbrake >> 3 & 0x01,
                   mtbrake >> 4 & 0x01, mtbrake >> 5 & 0x01, mtbrake >> 6 & 0x01, mtbrake >> 7 & 0x01]
        mtable = [mtable & 0x01, mtable >> 1 & 0x01, mtable >> 2 & 0x01, mtable >> 3 & 0x01,
                  mtable >> 4 & 0x01, mtable >> 5 & 0x01, mtable >> 6 & 0x01, mtable >> 7 & 0x01]
        self.parse_dict['mtbrake'] = mtbrake
        self.parse_dict['mtable'] = mtable
        self.parse_dict['tcp_offset'] = tcp_offset
        self.parse_dict['tcp_load'] = tcp_load
        self.parse_dict['error_code'] = error_code
        self.parse_dict['warn_code'] = warn_code
        self.parse_dict['collis_sens'] = collis_sens
        self.parse_dict['teach_sens'] = teach_sens
        self.parse_dict['gravity_direction'] = gravity_direction
        ret.extend([mtbrake, mtable, error_code, warn_code, tcp_offset, tcp_load, collis_sens, teach_sens, gravity_direction])
        return ret

    def _parse_report_tcp_rich_data(self, rx_data):
        ret = self._parse_report_tcp_normal_data(rx_data)
        length = ret[0]
        if length >= 151:
            arm_type, arm_axis, arm_master_id, arm_slave_id, arm_motor_tid, arm_motor_fid = rx_data[145:151]
            self.parse_dict['arm_type'] = arm_type
            self.parse_dict['arm_axis'] = arm_axis
            self.parse_dict['arm_master_id'] = arm_master_id
            self.parse_dict['arm_slave_id'] = arm_slave_id
            self.parse_dict['arm_motor_tid'] = arm_motor_tid
            self.parse_dict['arm_motor_fid'] = arm_motor_fid
            ret.extend([arm_type, arm_axis, arm_master_id, arm_slave_id, arm_motor_tid, arm_motor_fid])
        if length >= 181:
            version = str(rx_data[151:180], 'utf-8')
            self.parse_dict['version'] = version
            ret.append(version)
        if length >= 201:
            trs_msg = convert.bytes_to_fp32s(rx_data[181:201], 5)
            tcp_jerk, min_tcp_acc, max_tcp_acc, min_tcp_speed, max_tcp_speed = trs_msg
            self.parse_dict['tcp_jerk'] = tcp_jerk
            self.parse_dict['min_tcp_acc'] = min_tcp_acc
            self.parse_dict['max_tcp_acc'] = max_tcp_acc
            self.parse_dict['min_tcp_speed'] = min_tcp_speed
            self.parse_dict['max_tcp_speed'] = max_tcp_speed
            ret.extend([tcp_jerk, min_tcp_acc, max_tcp_acc, min_tcp_speed, max_tcp_speed])
        if length >= 221:
            p2p_msg = convert.bytes_to_fp32s(rx_data[201:221], 5)
            joint_jerk, min_joint_acc, max_joint_acc, min_joint_speed, max_joint_speed = p2p_msg
            self.parse_dict['joint_jerk'] = joint_jerk
            self.parse_dict['min_joint_acc'] = min_joint_acc
            self.parse_dict['max_joint_acc'] = max_joint_acc
            self.parse_dict['min_joint_speed'] = min_joint_speed
            self.parse_dict['max_joint_speed'] = max_joint_speed
            ret.extend([joint_jerk, min_joint_acc, max_joint_acc, min_joint_speed, max_joint_speed])
        if length >= 229:
            rot_msg = convert.bytes_to_fp32s(rx_data[221:229], 2)
            rot_jerk, max_rot_acc = rot_msg
            self.parse_dict['rot_jerk'] = rot_jerk
            self.parse_dict['max_rot_acc'] = max_rot_acc
            ret.extend([rot_jerk, max_rot_acc])
        if length >= 245:
            servo_code = [val for val in rx_data[229:245]]
            self.parse_dict['servo_code'] = servo_code
            ret.extend([servo_code[:-2], servo_code[-2:]])
        if length >= 252:
            temperatures = list(map(int, rx_data[245:252]))
            self.parse_dict['temperatures'] = temperatures
            ret.append(temperatures)
        if length >= 284:
            speeds = convert.bytes_to_fp32s(rx_data[252:8 * 4 + 252], 8)
            self.parse_dict['speeds'] = speeds
            ret.append(speeds)
        if length >= 288:
            count = convert.bytes_to_u32(rx_data[284:288])
            self.parse_dict['count'] = count
            ret.append(count)
        if length >= 312:
            world_offset = convert.bytes_to_fp32s(rx_data[288:6 * 4 + 288], 6)
            self.parse_dict['world_offset'] = world_offset
            ret.append(world_offset)
        if length >= 314:
            cgpio_reset_enable, tgpio_reset_enable = rx_data[312:314]
            self.parse_dict['cgpio_reset_enable'] = cgpio_reset_enable
            self.parse_dict['tgpio_reset_enable'] = tgpio_reset_enable
            ret.extend([cgpio_reset_enable, tgpio_reset_enable])
        if length >= 416:
        # if length >= 340:
            is_collision_check, collision_tool_type = rx_data[314:316]
            collision_tool_params = convert.bytes_to_fp32s(rx_data[316:340], 6)
            self.parse_dict['is_collision_check'] = is_collision_check
            self.parse_dict['collision_tool_type'] = collision_tool_type
            self.parse_dict['collision_tool_params'] = collision_tool_params
            ret.extend([is_collision_check, collision_tool_type, collision_tool_params])
        # if length >= 354:
            voltages = convert.bytes_to_u16s(rx_data[340:354], 7)
            voltages = list(map(lambda x: x / 100, voltages))
            self.parse_dict['voltages'] = voltages
            ret.append(voltages)
        # if length >= 382:
            currents = convert.bytes_to_fp32s(rx_data[354:382], 7)
            self._currents = currents
            self.parse_dict['currents'] = currents
            ret.append(currents)
        # if length >= 416:
            cgpio_states = []
            cgpio_states.extend(rx_data[382:384])
            cgpio_states.extend(convert.bytes_to_u16s(rx_data[384:400], 8))
            cgpio_states[6:10] = list(map(lambda x: x / 4095.0 * 10.0, cgpio_states[6:10]))
            cgpio_states.append(list(map(int, rx_data[400:408])))
            cgpio_states.append(list(map(int, rx_data[408:416])))
            self.parse_dict['cgpio_states'] = cgpio_states
            ret.append(cgpio_states)
        return ret

