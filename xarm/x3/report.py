from xarm.core.config.x_config import XCONF
from xarm.core.utils import convert


class ReportHandler(object):
    def __init__(self, arm):
        self.arm = arm
        self.buffer = b''
        self.report_size = 0

    def reset(self):
        self.buffer = b''
        self.report_size = 0

    def process(self, recv_data):
        if recv_data == -1:
            return
        self.buffer += recv_data
        if len(self.buffer) < 4:
            return
        if self.report_size == 0:
            self.report_size = convert.bytes_to_u32(self.buffer[:4])
            if self.report_size < 87:
                self.reset()
                # TODO reconnect
                return
        if len(self.buffer) < self.report_size:
            return
        report_type = self.arm._report_type
        if report_type == 'rich' and self.report_size == 233 and len(self.buffer) >= 245:
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
                            return
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
                return
        else:
            data = self.buffer[:self.report_size]
            self.buffer = self.buffer[self.report_size:]
        self.parse_data(data, report_type=report_type)

    def parse_data(self, rx_data, report_type='real'):
        # length = convert.bytes_to_u32(rx_data[0:4])
        length = len(rx_data)
        state, mode = rx_data[4] & 0x0F, rx_data[4] >> 4
        cmd_num = convert.bytes_to_u16(rx_data[5:7])
        angles = convert.bytes_to_fp32s(rx_data[7:7 * 4 + 7], 7)
        pose = convert.bytes_to_fp32s(rx_data[35:6 * 4 + 35], 6)
        torque = convert.bytes_to_fp32s(rx_data[59:7 * 4 + 59], 7)
        if report_type == 'real':
            return
        mtbrake, mtable, error_code, warn_code = rx_data[87:91]
        pose_offset = convert.bytes_to_fp32s(rx_data[91:6 * 4 + 91], 6)
        tcp_load = convert.bytes_to_fp32s(rx_data[115:4 * 4 + 115], 4)
        collis_sens, teach_sens = rx_data[131:133]

        gravity_direction = convert.bytes_to_fp32s(rx_data[133:3 * 4 + 133], 3)

        mtbrake = [mtbrake & 0x01, mtbrake >> 1 & 0x01, mtbrake >> 2 & 0x01, mtbrake >> 3 & 0x01,
                   mtbrake >> 4 & 0x01, mtbrake >> 5 & 0x01, mtbrake >> 6 & 0x01, mtbrake >> 7 & 0x01]
        mtable = [mtable & 0x01, mtable >> 1 & 0x01, mtable >> 2 & 0x01, mtable >> 3 & 0x01,
                  mtable >> 4 & 0x01, mtable >> 5 & 0x01, mtable >> 6 & 0x01, mtable >> 7 & 0x01]

        if length >= 151:
            arm_type, arm_axis, arm_master_id, arm_slave_id, arm_motor_tid, arm_motor_fid = rx_data[145:151]
        if length >= 201:
            trs_msg = convert.bytes_to_fp32s(rx_data[181:201], 5)
            tcp_jerk, min_tcp_acc, max_tcp_acc, min_tcp_speed, max_tcp_speed = trs_msg
        if length >= 221:
            p2p_msg = convert.bytes_to_fp32s(rx_data[201:221], 5)
            joint_jerk, min_joint_acc, max_joint_acc, min_joint_speed, max_joint_speed = p2p_msg
        if length >= 229:
            rot_msg = convert.bytes_to_fp32s(rx_data[221:229], 2)
            rot_jerk, max_rot_acc = rot_msg
        if length >= 245:
            sv3_msg = rx_data[229:245]
        if length >= 252:
            temperatures = list(map(int, rx_data[245:252]))
        if length >= 284:
            speeds = convert.bytes_to_fp32s(rx_data[252:8 * 4 + 252], 8)
        if length >= 288:
            count = convert.bytes_to_u32(rx_data[284:288])
        if length >= 312:
            world_offset = convert.bytes_to_fp32s(rx_data[288:6 * 4 + 288], 6)
        if length >= 314:
            cgpio_reset_enable, tgpio_reset_enable = rx_data[312:314]

