xArm-Python-SDK API Documentation (V1.17.0): class XArmAPI in module xarm.wrapper.xarm_api

## class __XArmAPI__
****************************************

>   

****************************************
### __Methods__
****************************************

#### def __\__init__\__(self, port=None, is_radian=False, do_not_open=False, **kwargs):

> The API wrapper of xArm  
> Note: Orientation of attitude angle  
> &ensp;&ensp;&ensp;&ensp;roll: rotate around the X axis  
> &ensp;&ensp;&ensp;&ensp;pitch: rotate around the Y axis  
> &ensp;&ensp;&ensp;&ensp;yaw: rotate around the Z axis  
>   
> :param port: ip-address(such as '192.168.1.185')  
> &ensp;&ensp;&ensp;&ensp;Note: this parameter is required if parameter do_not_open is False  
> :param is_radian: set the default unit is radians or not, default is False  
> &ensp;&ensp;&ensp;&ensp;Note: (aim of design)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. Default value for unified interface parameters  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2: Unification of the external unit system  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. For compatibility with previous interfaces  
> &ensp;&ensp;&ensp;&ensp;Note: the conversion of degree (°) and radians (rad)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;* 1 rad == 57.29577951308232 °  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;* 1 ° == 0.017453292519943295 rad  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;* 1 rad/s == 57.29577951308232 °/s  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;* 1 °/s == 0.017453292519943295 rad/s  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;* 1 rad/s^2 == 57.29577951308232 °/s^2  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;* 1 °/s^2 == 0.017453292519943295 rad/s^2  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;* 1 rad/s^3 == 57.29577951308232 °/s^3  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;* 1 °/s^3 == 0.017453292519943295 rad/s^3  
> &ensp;&ensp;&ensp;&ensp;Note: This parameter determines the value of the property self.default_is_radian  
> &ensp;&ensp;&ensp;&ensp;Note: This parameter determines the default value of the interface with the is_radian/input_is_radian/return_is_radian parameter  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;The list of affected interfaces is as follows:  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. method: get_position  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. method: set_position  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. method: get_servo_angle  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4. method: set_servo_angle  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;5. method: set_servo_angle_j  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;6. method: move_gohome  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;7. method: reset  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;8. method: set_tcp_offset  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;9. method: set_joint_jerk  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;10. method: set_joint_maxacc  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;11. method: get_inverse_kinematics  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;12. method: get_forward_kinematics  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;13. method: is_tcp_limit  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;14. method: is_joint_limit  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;15. method: get_params  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;16: method: move_arc_lines  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;17: method: move_circle  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;18: method: set_servo_cartesian  
> &ensp;&ensp;&ensp;&ensp;Note: This parameter determines the default return type for some interfaces (such as the position, velocity, and acceleration associated with the return angle arc).  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;The affected attributes are as follows:  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. property: position  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. property: last_used_position  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. property: angles  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4. property: last_used_angles  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;5. property: last_used_joint_speed  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;6. property: last_used_joint_acc  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;7. property: tcp_offset  
> :param do_not_open: do not open, default is False, if true, you need to manually call the connect interface.  
> :param kwargs: keyword parameters, generally do not need to set  
> &ensp;&ensp;&ensp;&ensp;axis: number of axes, required only when using a serial port connection, default is 7  
> &ensp;&ensp;&ensp;&ensp;baudrate: serial baudrate, invalid, reserved.  
> &ensp;&ensp;&ensp;&ensp;timeout: serial timeout, invalid, reserved.  
> &ensp;&ensp;&ensp;&ensp;filters: serial port filters, invalid, reserved.  
> &ensp;&ensp;&ensp;&ensp;check_tcp_limit: check the tcp param value out of limit or not, default is False  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: only check the param roll/pitch/yaw of the interface `set_position`/`move_arc_lines`  
> &ensp;&ensp;&ensp;&ensp;check_joint_limit: check the joint param value out of limit or not, default is True  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: only check the param angle of the interface `set_servo_angle` and the param angles of the interface `set_servo_angle_j`  
> &ensp;&ensp;&ensp;&ensp;check_cmdnum_limit: check the cmdnum out of limit or not, default is True  
> &ensp;&ensp;&ensp;&ensp;max_cmdnum: max cmdnum, default is 512  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: only available in the param `check_cmdnum_limit` is True  
> &ensp;&ensp;&ensp;&ensp;check_is_ready: check if the arm is ready to move or not, default is True  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version < 1.5.20


#### def __calibrate_tcp_coordinate_offset__(self, four_points, is_radian=None):

> Four-point method to calibrate tool coordinate system position offset  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.6.9  
>   
> :param four_points: a list of four teaching coordinate positions [x, y, z, roll, pitch, yaw]  
> :param is_radian: the roll/pitch/yaw value of the each point in radians or not, default is self.default_is_radian  
> :return: tuple((code, xyz_offset)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;xyz_offset: calculated xyz(mm) TCP offset, [x, y, z]


#### def __calibrate_tcp_orientation_offset__(self, rpy_be, rpy_bt, input_is_radian=None, return_is_radian=None):

> An additional teaching point to calibrate the tool coordinate system attitude offset  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.6.9  
>   
> :param rpy_be: the rpy value of the teaching point without TCP offset [roll, pitch, yaw]  
> :param rpy_bt: the rpy value of the teaching point with TCP offset [roll, pitch, yaw]  
> :param input_is_radian: the roll/pitch/yaw value of rpy_be and rpy_bt in radians or not, default is self.default_is_radian  
> :param return_is_radian: the roll/pitch/yaw value of result in radians or not, default is self.default_is_radian  
> :return: tuple((code, rpy_offset)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;rpy_offset: calculated rpy TCP offset, [roll, pitch, yaw]


#### def __calibrate_user_coordinate_offset__(self, rpy_ub, pos_b_uorg, is_radian=None):

> An additional teaching point determines the position offset of the user coordinate system.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.6.9  
>   
> :param rpy_ub: the confirmed offset of the base coordinate system in the user coordinate system [roll, pitch, yaw], which is the result of calibrate_user_orientation_offset()  
> :param pos_b_uorg: the position of the teaching point in the base coordinate system [x, y, z], if the arm cannot reach the target position, the user can manually input the position of the target in the base coordinate.  
> :param is_radian: the roll/pitch/yaw value of rpy_ub in radians or not, default is self.default_is_radian  
> :return: tuple((code, xyz_offset)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;xyz_offset: calculated xyz(mm) user offset, [x, y, z]


#### def __calibrate_user_orientation_offset__(self, three_points, mode=0, trust_ind=0, input_is_radian=None, return_is_radian=None):

> Three-point method teaches user coordinate system posture offset  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.6.9  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;First determine a point in the working space, move along the desired coordinate system x+ to determine the second point,  
> &ensp;&ensp;&ensp;&ensp;and then move along the desired coordinate system y+ to determine the third point.   
> &ensp;&ensp;&ensp;&ensp;Note that the x+ direction is as accurate as possible.   
> &ensp;&ensp;&ensp;&ensp;If the y+ direction is not completely perpendicular to x+, it will be corrected in the calculation process.  
>   
> :param three_points: a list of teaching TCP coordinate positions [x, y, z, roll, pitch, yaw]  
> :param input_is_radian: the roll/pitch/yaw value of the each point in radians or not, default is self.default_is_radian  
> :param return_is_radian: the roll/pitch/yaw value of result in radians or not, default is self.default_is_radian  
> :return: tuple((code, rpy_offset)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;rpy_offset: calculated rpy user offset, [roll, pitch, yaw]


#### def __check_verification__(self):

> check verification  
>   
> :return: tuple((code, status)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status:  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: verified  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;other: not verified


#### def __clean_bio_gripper_error__(self):

> Clean the error code of the bio gripper  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __clean_conf__(self):

> Clean current config and restore system default settings  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface will clear the current settings and restore to the original settings (system default settings)  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __clean_error__(self):

> Clean the error, need to be manually enabled motion(arm.motion_enable(True)) and set state(arm.set_state(state=0))after clean error  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __clean_gripper_error__(self, **kwargs):

> Clean the gripper error  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __clean_linear_motor_error__(self):

> Clean the linear motor error  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __clean_warn__(self):

> Clean the warn  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __close_bio_gripper__(self, speed=0, wait=True, timeout=5, **kwargs):

> Close the bio gripper  
>   
> :param speed: speed value, default is 0 (not set the speed)  
> :param wait: whether to wait for the bio gripper motion complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 5, only available if wait=True  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __close_lite6_gripper__(self, sync=True):

> Close the gripper of Lite6 series robotic arms  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.10.0  
> &ensp;&ensp;&ensp;&ensp;2. this API can only be used on Lite6 series robotic arms  
>   
> :param sync: whether to execute in the motion queue, set to False to execute immediately(default is True)  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __config_cgpio_reset_when_stop__(self, on_off):

> Config the Controller GPIO reset the digital output when the robot is in stop state  
>   
> :param on_off: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __config_tgpio_reset_when_stop__(self, on_off):

> Config the Tool GPIO reset the digital output when the robot is in stop state  
>   
> :param on_off: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __connect__(self, port=None, baudrate=None, timeout=None, axis=None, **kwargs):

> Connect to xArm  
>   
> :param port: port name or the ip address, default is the value when initializing an instance  
> :param baudrate: baudrate, only available in serial way, default is the value when initializing an instance  
> :param timeout: timeout, only available in serial way, default is the value when initializing an instance  
> :param axis: number of axes, required only when using a serial port connection, default is 7


#### def __delete_blockly_app__(self, name):

> Delete blockly app  
>   
> :param name: blockly app name  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __delete_trajectory__(self, name):

> Delete trajectory  
>   
> :param name: trajectory name  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __disconnect__(self):

> Disconnect


#### def __emergency_stop__(self):

> Emergency stop (set_state(4) -> motion_enable(True) -> set_state(0))  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface does not automatically clear the error. If there is an error, you need to handle it according to the error code.


#### def __get_allow_approx_motion__(self):

> Obtain whether to enable approximate solutions to avoid certain singularities  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.9.0  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_base_board_version__(self, board_id=10):

> &ensp;Get base board version  
>   
> :param board_id: int  
> :return: : (code, version)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_bio_gripper_error__(self):

> Get the error code of the bio gripper  
>   
> :return: tuple((code, error_code))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;error_code: See the [Bio Gripper Error Code Documentation](./xarm_api_code.md#bio-gripper-error-code) for details.


#### def __get_bio_gripper_g2_position__(self, **kwargs):

> Get the position (mm) of the BIO Gripper G2  
>   
> :return: tuple((code, pos)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_bio_gripper_status__(self):

> Get the status of the bio gripper  
>   
> :return: tuple((code, status))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status: status  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x03 == 0: stop  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x03 == 1: motion  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x03 == 2: catch  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x03 == 3: error  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;(status >> 2) & 0x03 == 0: not enabled  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;(status >> 2) & 0x03 == 1: enabling  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;(status >> 2) & 0x03 == 2: enabled


#### def __get_c23_error_info__(self, is_radian=None):

> Get joint angle limit error (C23) info  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :return: tuple((code, err_info)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;err_info: [(servo_id, angle), ...]


#### def __get_c24_error_info__(self, is_radian=None):

> Get joint speed limit error (C24) info  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :return: tuple((code, err_info)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;err_info: [servo_id, speed]


#### def __get_c31_error_info__(self):

> Get collision error (C31) info  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :return: tuple((code, err_info)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;err_info: [servo_id, theoratival tau, actual tau]


#### def __get_c37_error_info__(self, is_radian=None):

> Get payload error (C37) info  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :return: tuple((code, err_info)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;err_info: [servo_id, angle]


#### def __get_c38_error_info__(self, is_radian=None):

> Get joint hard angle limit error (C38) info  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.4.0  
>   
> :return: tuple((code, err_info)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;err_info: [(servo_id, angle), ...]


#### def __get_c54_error_info__(self):

> Get (Six-axis Force Torque Sensor) collision error (C54) info  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.6.103  
>   
> :return: tuple((code, err_info)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;err_info: [dir, tau threshold, actual tau]


#### def __get_c60_error_info__(self):

> Get linear speed limit error (C60) info  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Only available if firmware_version >= 2.3.0  
> &ensp;&ensp;&ensp;&ensp;2. Only available in mode 1  
>   
> :return: tuple((code, err_info)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;err_info: [max_linear_speed, curr_linear_speed]


#### def __get_cgpio_analog__(self, ionum=None):

> Get the analog value of the specified Controller GPIO  
> :param ionum: 0 or 1 or None(both 0 and 1), default is None  
> :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_cgpio_digital__(self, ionum=None):

> Get the digital value of the specified Controller GPIO  
>   
> :param ionum: 0~7(CI0~CI7), 8~15(DI0~DI7) or None(both 0~15), default is None  
> :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_cgpio_state__(self):

> Get the state of the Controller GPIO  
> :return: code, states  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;states: [...]  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[0]: contorller gpio module state  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[0] == 0: normal  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[0] == 1: wrong  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[0] == 6: communication failure  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[1]: controller gpio module error code  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[1] == 0: normal  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[1] != 0: error code  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[2]: digital input functional gpio state  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: digital-i-input functional gpio state = states[2] >> i & 0x0001  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[3]: digital input configuring gpio state  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: digital-i-input configuring gpio state = states[3] >> i & 0x0001  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[4]: digital output functional gpio state  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: digital-i-output functional gpio state = states[4] >> i & 0x0001  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[5]: digital output configuring gpio state  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: digital-i-output configuring gpio state = states[5] >> i & 0x0001  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[6]: analog-0 input value  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[7]: analog-1 input value  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[8]: analog-0 output value  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[9]: analog-1 output value  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[10]: digital input functional info, [digital-0-input-functional-mode, ... digital-15-input-functional-mode]  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[11]: digital output functional info, [digital-0-output-functional-mode, ... digital-15-output-functional-mode]


#### def __get_checkset_default_baud__(self, type_):

> Get the checkset baud value  
>   
> :param type_: checkset type  
> &ensp;&ensp;&ensp;&ensp;1: xarm gripper  
> &ensp;&ensp;&ensp;&ensp;2: bio gripper  
> &ensp;&ensp;&ensp;&ensp;3: robotiq gripper  
> &ensp;&ensp;&ensp;&ensp;4: linear motor  
> :return: tuple((code, baud))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;baud: the checkset baud value


#### def __get_cmd_mat_history_num__(self):

> Get cmd mat history num  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :return: tuple((code, num)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;num: cmd mat history num


#### def __get_cmdnum__(self):

> Get the cmd count in cache  
> :return: tuple((code, cmd_num)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_dh_params__(self):

> Get the DH parameters  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.0.0  
>   
> :return: tuple((code, dh_params)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;dh_params: DH parameters  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;dh_params[0:4]: DH parameters of Joint-1  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;dh_params[4:8]: DH parameters of Joint-2  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;...  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;dh_params[24:28]: DH parameters of Joint-7


#### def __get_err_warn_code__(self, show=False, lang='en'):

> Get the controller error and warn code  
>   
> :param show: show the detail info if True  
> :param lang: show language, en/cn, degault is en, only available if show is True  
> :return: tuple((code, [error_code, warn_code])), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;error_code: See the [Controller Error Code Documentation](./xarm_api_code.md#controller-error-code) for details.  
> &ensp;&ensp;&ensp;&ensp;warn_code: See the [Controller Error Code Documentation](./xarm_api_code.md#controller-warn-code) for details.


#### def __get_fdb_mat_history_num__(self):

> Get fdb mat history num  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :return: tuple((code, num)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;num: fdb mat history num


#### def __get_forward_kinematics__(self, angles, input_is_radian=None, return_is_radian=None):

> Get forward kinematics  
>   
> :param angles: [angle-1, angle-2, ..., angle-n], n is the number of axes of the arm  
> :param input_is_radian: the param angles value is in radians or not, default is self.default_is_radian  
> :param return_is_radian: the returned value is in radians or not, default is self.default_is_radian  
> :return: tuple((code, pose)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;pose: [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)] or []  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: the roll/pitch/yaw value is radians if return_is_radian is True, else °


#### def __get_ft_admittance_ctrl_threshold__(self):

> Get the reaction thresholds in each direction under the admittance control mode of the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.110  
>   
> :return: tuple((code, thresholds)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;threshold: [x(N), y(N), z(N), Rx(Nm), Ry(Nm), Rz(Nm)]


#### def __get_ft_collision_detection__(self):

> Get the collision detection with the Six-axis Force Torque Sensor is enable or not  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103  
>   
> :return: tuple((code, on_off)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;on_off: enable or not


#### def __get_ft_collision_reb_distance__(self, is_radian=None):

> Get the collision rebound distance with the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103  
>   
> :param is_radian: the returned value (only Rx/Ry/Rz) is in radians or not, default is self.default_is_radian  
>   
> :return: tuple((code, distance)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;distance: [x(mm), y(mm), z(mm), Rx(° or rad), Ry(° or rad), Rz(° or rad)]


#### def __get_ft_collision_rebound__(self):

> Get the collision rebound with the Six-axis Force Torque Sensor is enable or not  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103  
>   
> :return: tuple((code, on_off)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;on_off: enable or not


#### def __get_ft_collision_threshold__(self):

> Get the collision thresholds with the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103  
>   
> :return: tuple((code, thresholds)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;threshold: [x(N), y(N), z(N), Rx(Nm), Ry(Nm), Rz(Nm)]


#### def __get_ft_sensor_config__(self):

> Get the config of the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
> &ensp;&ensp;&ensp;&ensp;  
> :return: tuple((code, config))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;config: [...], the config of the Six-axis Force Torque Sensor, only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[0] ft_mode: force mode  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: non-force mode  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: admittance control  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2: force control  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[1] ft_is_started: ft sensor is enable or not  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[2] ft_type: ft sensor type  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[3] ft_id: ft sensor id  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[4] ft_freq: ft sensor frequency  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[5] ft_mass: load mass  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[6] ft_dir_bias: reversed  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[7] ft_centroid: [x_centroid, y_centroid, z_centroid]  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[8] ft_zero: [Fx_offset, Fy_offset, Fz_offset, Tx_offset, Ty_offset, Tz_ffset]  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[9] imp_coord: task frame of admittance control mode.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: base frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: tool frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[10] imp_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be admittance in the corresponding axis of the task frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[11] M: mass. (kg)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[12] K: stiffness coefficient.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[13] B: damping coefficient. invalid.   Note: the value is set to 2*sqrt(M*K) in controller.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[14] f_coord: task frame of force control mode.   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: base frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: tool frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[15] f_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be compliant in the corresponding axis of the task frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[16] f_ref:  the forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[17] f_limits: reversed.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[18] kp: proportional gain  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[19] ki: integral gain.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[20] kd: differential gain.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[21] xe_limit: 6d vector. for compliant axes, these values are the maximum allowed tcp speed along/about the axis. mm/s


#### def __get_ft_sensor_data__(self, is_raw=False):

> Get the data of the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :param is_raw: get the raw data or not.  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.109  
> :return: tuple((code, exe_ft))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;ft_data: only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: The external force detection value of the Six-axis Force Torque Sensor after filtering, load and offset compensation


#### def __get_ft_sensor_error__(self):

> Get the error code of the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :return: tuple((code, error))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;error: See the [Six-axis Force Torque Sensor Error Code Documentation](./xarm_api_code.md#six-axis-force-torque-sensor-error-code) for details.


#### def __get_ft_sensor_mode__(self):

> Get force mode  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :return: tuple((code, app_code))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;app_code:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: non-force mode  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: admittance control mode  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2: force control mode


#### def __get_gripper_err_code__(self, **kwargs):

> Get the gripper error code  
>   
> :return: tuple((code, err_code)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;err_code: See the [Gripper Error Code Documentation](./xarm_api_code.md#gripper-error-code) for details.


#### def __get_gripper_g2_position__(self, **kwargs):

> Get the position (mm) of the xArm Gripper G2  
>   
> :return: tuple((code, pos)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_gripper_position__(self, **kwargs):

> Get the gripper position (pulse)  
>   
> :return: tuple((code, pos)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_gripper_status__(self):

> Get the status of the xArm Gripper  
> Note:  
> &ensp;&ensp;&ensp;&ensp;only available if gripper_version >= 3.4.3  
>   
> :return: tuple((code, status)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status:  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x03 == 0: stop state  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x03 == 1: move state   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x03 == 2: grasp state


#### def __get_gripper_version__(self):

> Get gripper version, only for debug  
>   
> :return: (code, version)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_harmonic_type__(self, servo_id=1):

> Get harmonic type, only for debug  
>   
> :return: (code, type)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_hd_types__(self):

> Get harmonic types, only for debug  
>   
> :return: (code, types)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_iden_status__(self):

> Get iden status  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :return: tuple((code, status)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status: 1 means in identifying, 0 means not in identifying


#### def __get_initial_point__(self):

> Get the initial point from studio  
>   
> :return: tuple((code, point)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;point: initial point, [J1, J2, ..., J7]


#### def __get_inverse_kinematics__(self, pose, input_is_radian=None, return_is_radian=None):

> Get inverse kinematics  
>   
> :param pose: [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
> &ensp;&ensp;&ensp;&ensp;Note: the roll/pitch/yaw unit is radian if input_is_radian is True, else °  
> :param input_is_radian: the param pose value(only roll/pitch/yaw) is in radians or not, default is self.default_is_radian  
> :param return_is_radian: the returned value is in radians or not, default is self.default_is_radian  
> :return: tuple((code, angles)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;angles: [angle-1(rad or °), angle-2, ..., angle-(Number of axes)] or []  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: the returned angle value is radians if return_is_radian is True, else °


#### def __get_is_moving__(self):

> Check xArm is moving or not  
> :return: True/False


#### def __get_joint_states__(self, is_radian=None, num=3):

> Get the joint states  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.9.0  
>   
> :param is_radian: the returned value(position and velocity) is in radians or not, default is self.default_is_radian  
> :return: tuple((code, [position, velocity, effort])), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;position: the angles of joints, like [angle-1, ..., angle-7]  
> &ensp;&ensp;&ensp;&ensp;velocity: the velocities of joints, like [velo-1, ..., velo-7]  
> &ensp;&ensp;&ensp;&ensp;effort: the efforts of joints, like [effort-1, ..., effort-7]


#### def __get_joints_torque__(self):

> Get joints torque  
>   
> :return: tuple((code, joints_torque))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;joints_torque: joints torque


#### def __get_linear_motor_error__(self):

> Get the error code of the linear motor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, error)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;error: See the [Linear Motor Error Code Documentation](./xarm_api_code.md#linear-motor-error-code) for details.


#### def __get_linear_motor_is_enabled__(self):

> Get the linear motor is enabled or not  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, status)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: linear motor is not enabled  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: linear motor is enabled


#### def __get_linear_motor_on_zero__(self):

> Get the linear motor is on zero positon or not  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, status)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: linear motor is not on zero  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: linear motor is on zero


#### def __get_linear_motor_pos__(self):

> Get the pos of the linear motor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, position)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;position: position


#### def __get_linear_motor_registers__(self, **kwargs):

> Get the status of the linear motor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, status)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status: status, like  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'pos': 0,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'status': 0,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'error': 0,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'is_enabled': 0,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'on_zero': 0,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'sci': 1,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'sco': [0, 0],  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;}


#### def __get_linear_motor_sci__(self):

> Get the sci1 value of the linear motor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, sci1)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_linear_motor_sco__(self):

> Get the sco value of the linear motor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, sco)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;sco: [sco0, sco1]


#### def __get_linear_motor_status__(self):

> Get the status of the linear motor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, status)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status: status  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x00: motion finish  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x01: in motion  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x02: has stop


#### def __get_linear_spd_limit_factor__(self):

> Get linear speed limit factor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :return: tuple((code, factor)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;factor: linear speed limit factor


#### def __get_mount_direction__(self):

> Get the mount degrees from studio  
>   
> :return: tuple((code, degrees)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;degrees: mount degrees, [tilt angle, rotate angle]


#### def __get_poe_status__(self):

> Get poe status  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :return: tuple((code, status)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status: 1 means poe is valid, 0 means poe is invalid


#### def __get_pose_offset__(self, pose1, pose2, orient_type_in=0, orient_type_out=0, is_radian=None):

> Calculate the pose offset of two given points  
>   
> :param pose1: [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]  
> :param pose2: [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]  
> :param orient_type_in: input attitude notation, 0 is RPY(roll/pitch/yaw) (default), 1 is axis angle(rx/ry/rz)  
> :param orient_type_out: notation of output attitude, 0 is RPY (default), 1 is axis angle  
> :param is_radian: the roll/rx/pitch/ry/yaw/rz of pose1/pose2/return_pose is radian or not  
> :return: tuple((code, pose)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;pose: [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]


#### def __get_position__(self, is_radian=None):

> Get the cartesian position  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. If the value(roll/pitch/yaw) you want to return is an radian unit, please set the parameter is_radian to True  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code, pos = arm.get_position(is_radian=True)  
>   
> :param is_radian: the returned value (only roll/pitch/yaw) is in radians or not, default is self.default_is_radian  
> :return: tuple((code, [x, y, z, roll, pitch, yaw])), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_position_aa__(self, is_radian=None):

> Get the pose represented by the axis angle pose  
>   
> :param is_radian: the returned value (only rx/ry/rz) is in radians or not, default is self.default_is_radian  
> :return: tuple((code, [x, y, z, rx, ry, rz])), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_record_seconds__(self):

> Get record seconds  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Only available if firmware_version >= 2.4.0  
> &ensp;&ensp;&ensp;&ensp;2. Only valid during recording or after recording but before saving  
>   
> :return: tuple((code, seconds)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;seconds: The actual duration of the recorded trajectory


#### def __get_reduced_mode__(self):

> Get reduced mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
>   
> :return: tuple((code, mode))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;mode: 0 or 1, 1 means that the reduced mode is turned on. 0 means that the reduced mode is not turned on


#### def __get_reduced_states__(self, is_radian=None):

> Get states of the reduced mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
>   
> :param is_radian: the max_joint_speed of the states is in radians or not, default is self.default_is_radian  
> :return: tuple((code, states))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;states: [....]  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;if version > 1.2.11:  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states: [  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;reduced_mode_is_on,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[reduced_x_max, reduced_x_min, reduced_y_max, reduced_y_min, reduced_z_max, reduced_z_min],  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;reduced_max_tcp_speed,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;reduced_max_joint_speed,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;joint_ranges([joint-1-min, joint-1-max, ..., joint-7-min, joint-7-max]),  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;safety_boundary_is_on,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;collision_rebound_is_on,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;]  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;if version <= 1.2.11:  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states: [  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;reduced_mode_is_on,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[reduced_x_max, reduced_x_min, reduced_y_max, reduced_y_min, reduced_z_max, reduced_z_min],  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;reduced_max_tcp_speed,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;reduced_max_joint_speed,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;]


#### def __get_report_tau_or_i__(self):

> Get the reported torque or electric current  
>   
> :return: tuple((code, tau_or_i))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;tau_or_i:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: torque  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: electric current


#### def __get_robot_sn__(self):

> Get the xArm sn  
>   
> :return: tuple((code, sn)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_servo_angle__(self, servo_id=None, is_radian=None, is_real=False):

> Get the servo angle  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. If the value you want to return is an radian unit, please set the parameter is_radian to True  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code, angles = arm.get_servo_angle(is_radian=True)  
> &ensp;&ensp;&ensp;&ensp;2. If you want to return only the angle of a single joint, please set the parameter servo_id  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code, angle = arm.get_servo_angle(servo_id=2)  
> &ensp;&ensp;&ensp;&ensp;3. This interface is only used in the base coordinate system.  
>   
> :param servo_id: 1-(Number of axes), None(8), default is None  
> :param is_radian: the returned value is in radians or not, default is self.default_is_radian  
> :return: tuple((code, angle list if servo_id is None or 8 else angle)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_servo_debug_msg__(self, show=False, lang='en'):

> Get the servo debug msg, used only for debugging  
>   
> :param show: show the detail info if True  
> :param lang: language, en/cn, default is en  
> :return: tuple((code, servo_info_list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_servo_version__(self, servo_id=1):

> Get servo version, only for debug  
>   
> :param servo_id: servo id(1~7)  
> :return: (code, version)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_state__(self):

> Get state  
>   
> :return: tuple((code, state)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;state:  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: in motion  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2: sleeping  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3: suspended  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4: stopping


#### def __get_tgpio_analog__(self, ionum=None):

> Get the analog value of the specified Tool GPIO  
> :param ionum: 0 or 1 or None(both 0 and 1), default is None  
> :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_tgpio_digital__(self, ionum=None):

> Get the digital value of the specified Tool GPIO  
>   
> :param ionum: 0 or 1 or None(both 0 and 1), default is None  
> :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_tgpio_modbus_baudrate__(self):

> Get the modbus baudrate of the tool gpio  
>   
> :return: tuple((code, baudrate)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;baudrate: the modbus baudrate of the tool gpio


#### def __get_tgpio_modbus_timeout__(self, is_transparent_transmission=False):

> Get tgpio modbus timeout  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :param is_transparent_transmission: is transparent transmission or not  
> :return: tuple((code, timeout)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;timeout: timeout of the tgpio modbus, milliseconds


#### def __get_tgpio_output_digital__(self, ionum=None):

> Get the digital value of the specified Tool GPIO output  
>   
> :param ionum: 0 or 1 or 2 or 3 or 4 or None(both 0 and 1 and 2 and 3 and 4), default is None  
> :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_tgpio_version__(self):

> Get tool gpio version, only for debug  
>   
> :return: (code, version)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_tool_digital_input__(self, ionum=None):

> Get the digital value of the specified Tool GPIO,Compared with the "get_tgpio_digital" interface,  
> &ensp;&ensp;&ensp;&ensp;the value of TI2 is obtained when the ionum is not transmitted.  
>   
> :param ionum: 0 or 1 or or 2 or 3 or 4 (both 0 and 4), default is None  
> :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __get_traj_speeding__(self, rate):

> Obtain the joint and velocity values of joint overspeed during trajectory recording  
> :param rate: speed rate, It can only be 1/2/4  
>   
> :return: tuple((code, speed_info)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;speed_info: [result_code, servo_id, servo_speed]  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;result_code: 0: Pass, -1: Fail, >0: abnormal(1:Trajectory not loaded or incorrect status;2:The input magnification is incorrect)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;servo_id: Effective only when result_code is -1  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;servo_speed: Effective only when result_code is -1


#### def __get_trajectories__(self):

> get the trajectories  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on xArmStudio 1.2.0 or above  
> &ensp;&ensp;&ensp;&ensp;2. This interface relies on Firmware 1.2.0 or above  
>   
> :return: tuple((code, trajectories))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;trajectories: [{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'name': name, # The name of the trajectory  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'duration': duration, # The duration of the trajectory (seconds)  
> &ensp;&ensp;&ensp;&ensp;}]


#### def __get_trajectory_rw_status__(self):

> Get trajectory read/write status  
>   
> :return: (code, status)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status:  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: no read/write  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: loading  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2: load success  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3: load failed  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4: saving  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;5: save success  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;6: save failed


#### def __get_vacuum_gripper__(self, hardware_version=1):

> Get the state of the Vacuum Gripper  
>   
> :param hardware_version: hardware version  
> &ensp;&ensp;&ensp;&ensp;1: Plug-in Connection, default  
> &ensp;&ensp;&ensp;&ensp;2: Contact Connection  
> :return: tuple((code, state)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;state: state of the Vacuum Gripper  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;-1: Vacuum Gripper is off    
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: Object not picked by vacuum gripper   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: Object picked by vacuum gripper


#### def __get_version__(self):

> Get the xArm firmware version  
>   
> :return: tuple((code, version)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __getset_tgpio_modbus_data__(self, datas, min_res_len=0, host_id=9, is_transparent_transmission=False, use_503_port=False, **kwargs):

> Send the modbus data to the tool gpio  
>   
> :param datas: data_list  
> :param min_res_len: the minimum length of modbus response data. Used to check the data length, if not specified, no check  
> :param host_id: host_id, default is 9 (TGPIO_HOST_ID)  
> &ensp;&ensp;&ensp;&ensp;9: END RS485  
> &ensp;&ensp;&ensp;&ensp;11: CONTROLLER RS485  
> :param is_transparent_transmission: whether to choose transparent transmission, default is False  
> &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.11.0  
> :param use_503_port: whether to use port 503 for communication, default is False  
> &ensp;&ensp;&ensp;&ensp;Note: if it is True, it will connect to 503 port for communication when it is used for the first time, which is generally only useful for transparent transmission.  
> &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.11.0  
>   
> :return: tuple((code, modbus_response))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;modbus_response: modbus response data


#### def __iden_ft_sensor_load_offset__(self):

> Identification the tcp load and offset with the the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
> &ensp;&ensp;&ensp;&ensp;3. starting from SDK v1.11.0, the centroid unit is millimeters (originally meters)  
>   
> :return: tuple((code, load_offset)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;load:  [mass(kg), x_centroid(mm), y_centroid(mm), z_centroid(mm), Fx_offset, Fy_offset, Fz_offset, Tx_offset, Ty_offset, Tz_ffset]


#### def __iden_joint_friction__(self, sn=None):

> Identification the friction  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.9.0  
>   
> :param sn: sn value  
> :return: tuple((code, result)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;result:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: success  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;-1: failure


#### def __iden_tcp_load__(self, estimated_mass=0):

> Identification the tcp load with current  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :param estimated_mass: estimated mass  
> &ensp;&ensp;&ensp;&ensp;Note: this parameter is only available on the lite6 model manipulator, and this parameter must be specified for the lite6 model manipulator  
> :return: tuple((code, load)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;load:  [mass, x_centroid, y_centroid, z_centroid]


#### def __is_joint_limit__(self, joint, is_radian=None):

> Check the joint angle is in limit  
>   
> :param joint: [angle-1, angle-2, ..., angle-n], n is the number of axes of the arm  
> :param is_radian: angle value is radians or not, default is self.default_is_radian  
> :return: tuple((code, limit)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;limit: True/False/None, limit or not, or failed


#### def __is_tcp_limit__(self, pose, is_radian=None):

> Check the tcp pose is in limit  
>   
> :param pose: [x, y, z, roll, pitch, yaw]  
> :param is_radian: roll/pitch/yaw value is radians or not, default is self.default_is_radian  
> :return: tuple((code, limit)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;limit: True/False/None, limit or not, or failed


#### def __load_trajectory__(self, filename, wait=True, timeout=None, **kwargs):

> Load the trajectory  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
>   
> :param filename: The name of the trajectory to load  
> :param wait: Whether to wait for loading, default is True  
> :param timeout: Timeout waiting for loading to complete  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __mask_write_holding_register__(self, addr, and_mask, or_mask):

> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Mask Write Holding Register (0x16)  
>   
> :param addr: register address  
> :param and_mask: mask to be AND with  
> :param or_mask: mask to be OR with  
> :return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)


#### def __motion_enable__(self, enable=True, servo_id=None):

> Motion enable  
>   
> :param enable:True/False  
> :param servo_id: 1-(Number of axes), None(8)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __move_arc_lines__(self, paths, is_radian=None, times=1, first_pause_time=0.1, repeat_pause_time=0, automatic_calibration=True, speed=None, mvacc=None, mvtime=None, wait=False):

> Continuous linear motion with interpolation.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. If an error occurs, it will return early.  
> &ensp;&ensp;&ensp;&ensp;2. If the emergency_stop interface is called actively, it will return early.  
> &ensp;&ensp;&ensp;&ensp;3. The last_used_position/last_used_tcp_speed/last_used_tcp_acc will be modified.  
> &ensp;&ensp;&ensp;&ensp;4. The last_used_angles/last_used_joint_speed/last_used_joint_acc will not be modified.  
>   
> :param paths: cartesian path list  
> &ensp;&ensp;&ensp;&ensp;1. Specify arc radius: [[x, y, z, roll, pitch, yaw, radius], ....]  
> &ensp;&ensp;&ensp;&ensp;2. Do not specify arc radius (radius=0): [[x, y, z, roll, pitch, yaw], ....]  
> &ensp;&ensp;&ensp;&ensp;3. If you want to plan the continuous motion,set radius>0.  
>   
> :param is_radian: roll/pitch/yaw of paths are in radians or not, default is self.default_is_radian  
> :param times: repeat times, 0 is infinite loop, default is 1  
> :param first_pause_time: sleep time at first, purpose is to cache the commands and plan continuous motion, default is 0.1s  
> :param repeat_pause_time: interval between repeated movements, unit: (s)second  
> :param automatic_calibration: automatic calibration or not, default is True  
> :param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed  
> :param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc  
> :param mvtime: 0, reserved  
> :param wait: whether to wait for the arm to complete, default is False


#### def __move_circle__(self, pose1, pose2, percent, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, is_tool_coord=False, is_axis_angle=False, **kwargs):

> The motion calculates the trajectory of the space circle according to the three-point coordinates.  
> The three-point coordinates are (current starting point, pose1, pose2).  
>   
> :param pose1: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
> :param pose2: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
> :param percent: the percentage of arc length and circumference of the movement  
> :param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed  
> :param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc  
> :param mvtime: 0, reserved  
> :param is_radian: roll/pitch/yaw value is radians or not, default is self.default_is_radian  
> :param wait: whether to wait for the arm to complete, default is False  
> :param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True  
> :param is_tool_coord: is tool coord or not, default is False, only available if firmware_version >= 1.11.100  
> :param is_axis_angle: is axis angle or not, default is False, only available if firmware_version >= 1.11.100  
> :param kwargs: reserved  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code < 0: the last_used_tcp_speed/last_used_tcp_acc will not be modified  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code >= 0: the last_used_tcp_speed/last_used_tcp_acc will be modified


#### def __move_gohome__(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):

> Move to go home (Back to zero), the API will modify self.last_used_position and self.last_used_angles value  
> Warnning: without limit detection  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. The API will change self.last_used_position value into [201.5, 0, 140.5, -180, 0, 0]  
> &ensp;&ensp;&ensp;&ensp;2. The API will change self.last_used_angles value into [0, 0, 0, 0, 0, 0, 0]  
> &ensp;&ensp;&ensp;&ensp;3. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.move_gohome(wait=True)  
> &ensp;&ensp;&ensp;&ensp;4. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc  
>   
> :param speed: gohome speed (unit: rad/s if is_radian is True else °/s), default is 50 °/s  
> :param mvacc: gohome acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is 5000 °/s^2  
> :param mvtime: reserved  
> :param is_radian: the speed and acceleration are in radians or not, default is self.default_is_radian  
> :param wait: whether to wait for the arm to complete, default is False  
> :param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __open_bio_gripper__(self, speed=0, wait=True, timeout=5, **kwargs):

> Open the bio gripper  
>   
> :param speed: speed value, default is 0 (not set the speed)  
> :param wait: whether to wait for the bio gripper motion complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 5, only available if wait=True  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __open_lite6_gripper__(self, sync=True):

> Open the gripper of Lite6 series robotic arms  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.10.0  
> &ensp;&ensp;&ensp;&ensp;2. this API can only be used on Lite6 series robotic arms  
>   
> :param sync: whether to execute in the motion queue, set to False to execute immediately(default is True)  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __playback_trajectory__(self, times=1, filename=None, wait=True, double_speed=1, **kwargs):

> Playback trajectory  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
>   
> :param times: Number of playbacks,  
> &ensp;&ensp;&ensp;&ensp;1. Only valid when the current position of the arm is the end position of the trajectory, otherwise it will only be played once.  
> :param filename: The name of the trajectory to play back  
> &ensp;&ensp;&ensp;&ensp;1. If filename is None, you need to manually call the `load_trajectory` to load the trajectory.  
> :param wait: whether to wait for the arm to complete, default is False  
> :param double_speed: double speed, only support 1/2/4, default is 1, only available if version > 1.2.11  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __read_coil_bits__(self, addr, quantity):

> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Read Coils (0x01)  
>   
> :param addr: the starting address of the register to be read  
> :param quantity: number of registers  
> :return: tuple((code, bits)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)


#### def __read_holding_registers__(self, addr, quantity, is_signed=False):

> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Read Holding Registers (0x03)  
>   
> :param addr: the starting address of the register to be read  
> :param quantity: number of registers  
> :param is_signed: whether to convert the read register value into a signed form  
> :return: tuple((code, bits)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)


#### def __read_input_bits__(self, addr, quantity):

> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Read Discrete Inputs (0x02)  
>   
> :param addr: the starting address of the register to be read  
> :param quantity: number of registers  
> :return: tuple((code, bits)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)


#### def __read_input_registers__(self, addr, quantity, is_signed=False):

> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Read Input Registers (0x04)  
>   
> :param addr: the starting address of the register to be read  
> :param quantity: number of registers  
> :param is_signed: whether to convert the read register value into a signed form  
> :return: tuple((code, bits)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)


#### def __register_cmdnum_changed_callback__(self, callback=None):

> Register the cmdnum changed callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"cmdnum": cmdnum  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False


#### def __register_connect_changed_callback__(self, callback=None):

> Register the connect status changed callback  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"connected": connected,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"reported": reported,  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False


#### def __register_count_changed_callback__(self, callback=None):

> Register the counter value changed callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"count": counter value  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False


#### def __register_error_warn_changed_callback__(self, callback=None):

> Register the error code or warn code changed callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"error_code": error_code,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"warn_code": warn_code,  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False


#### def __register_feedback_callback__(self, callback=None):

> Register the callback of feedback  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.1.0  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data: bytes data  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;data[0:2]: transaction id, (Big-endian conversion to unsigned 16-bit integer data), command ID corresponding to the feedback, consistent with issued instructions  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: this can be used to distinguish which instruction the feedback belongs to  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;data[4:6]: feedback_length, feedback_length == len(data) - 6, (Big-endian conversion to unsigned 16-bit integer data)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;data[8]: feedback type  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: the motion task starts executing  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2: the motion task execution ends or motion task is discarded(usually when the distance is too close to be planned)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4: the non-motion task is triggered  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;data[9]: feedback funcode, command code corresponding to feedback, consistent with issued instructions  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: this can be used to distinguish what instruction the feedback belongs to  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;data[10:12]: feedback taskid, (Big-endian conversion to unsigned 16-bit integer data)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;data[12]: feedback code, execution status code, generally only meaningful when the feedback type is end, normally 0, 2 means discarded  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;data[13:21]: feedback us, (Big-endian conversion to unsigned 64-bit integer data), time when feedback triggers (microseconds)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: this time is the corresponding controller system time when the feedback is triggered  
> :return: True/False


#### def __register_iden_progress_changed_callback__(self, callback=None):

> Register the Identification progress value changed callback, only available if enable_report is True  
>   
> :param callback:   
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"progress": progress value  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False


#### def __register_mode_changed_callback__(self, callback=None):

> Register the mode changed callback, only available if enable_report is True and the connect way is socket  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"mode": mode,  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False


#### def __register_mtable_mtbrake_changed_callback__(self, callback=None):

> Register the motor enable states or motor brake states changed callback, only available if enable_report is True and the connect way is socket  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"mtable": [motor-1-motion-enable, motor-2-motion-enable, ...],  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"mtbrake": [motor-1-brake-enable, motor-1-brake-enable,...],  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False


#### def __register_report_callback__(self, callback=None, report_cartesian=True, report_joints=True, report_state=True, report_error_code=True, report_warn_code=True, report_mtable=True, report_mtbrake=True, report_cmd_num=True):

> Register the report callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'cartesian': [], # if report_cartesian is True  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'joints': [], # if report_joints is True  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'error_code': 0, # if report_error_code is True  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'warn_code': 0, # if report_warn_code is True  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'state': state, # if report_state is True  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'mtbrake': mtbrake, # if report_mtbrake is True, and available if enable_report is True and the connect way is socket  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'mtable': mtable, # if report_mtable is True, and available if enable_report is True and the connect way is socket  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;'cmdnum': cmdnum, # if report_cmd_num is True  
> &ensp;&ensp;&ensp;&ensp;}  
> :param report_cartesian: report cartesian or not, default is True  
> :param report_joints: report joints or not, default is True  
> :param report_state: report state or not, default is True  
> :param report_error_code: report error or not, default is True  
> :param report_warn_code: report warn or not, default is True  
> :param report_mtable: report motor enable states or not, default is True  
> :param report_mtbrake: report motor brake states or not, default is True  
> :param report_cmd_num: report cmdnum or not, default is True  
> :return: True/False


#### def __register_report_location_callback__(self, callback=None, report_cartesian=True, report_joints=True):

> Register the report location callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"cartesian": [x, y, z, roll, pitch, yaw], ## if report_cartesian is True  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"joints": [angle-1, angle-2, angle-3, angle-4, angle-5, angle-6, angle-7], ## if report_joints is True  
> &ensp;&ensp;&ensp;&ensp;}  
> :param report_cartesian: report or not, True/False, default is True  
> :param report_joints: report or not, True/False, default is True  
> :return: True/False


#### def __register_state_changed_callback__(self, callback=None):

> Register the state status changed callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"state": state,  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False


#### def __register_temperature_changed_callback__(self, callback=None):

> Register the temperature changed callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"temperatures": [servo-1-temperature, ...., servo-7-temperature]  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False


#### def __release_cmdnum_changed_callback__(self, callback=None):

> Release the cmdnum changed callback  
>   
> :param callback:  
> :return: True/False


#### def __release_connect_changed_callback__(self, callback=None):

> Release the connect changed callback  
>   
> :param callback:  
> :return: True/False


#### def __release_count_changed_callback__(self, callback=None):

> Release the counter value changed callback  
>   
> :param callback:  
> :return: True/False


#### def __release_error_warn_changed_callback__(self, callback=None):

> Release the error warn changed callback  
>   
> :param callback:  
> :return: True/False


#### def __release_feedback_callback__(self, callback=None):

> Release the callback of feedback  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.1.0  
>   
> :param callback:  
> :return: True/False


#### def __release_iden_progress_changed_callback__(self, callback=None):

> Release the Identification progress value changed callback  
>   
> :param callback:  
> :return: True/False


#### def __release_mode_changed_callback__(self, callback=None):

> Release the mode changed callback  
>   
> :param callback:  
> :return: True/False


#### def __release_mtable_mtbrake_changed_callback__(self, callback=None):

> Release the motor enable states or motor brake states changed callback  
>   
> :param callback:  
> :return: True/False


#### def __release_report_callback__(self, callback=None):

> Release the report callback  
>   
> :param callback:  
> :return: True/False


#### def __release_report_location_callback__(self, callback=None):

> Release the location report callback  
>   
> :param callback:  
> :return: True/False


#### def __release_state_changed_callback__(self, callback=None):

> Release the state changed callback  
>   
> :param callback:  
> :return: True/False


#### def __release_temperature_changed_callback__(self, callback=None):

> Release the temperature changed callback  
>   
> :param callback:  
> :return: True/False


#### def __reset__(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):

> Reset the xArm  
> Warnning: without limit detection  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. If there are errors or warnings, this interface will clear the warnings and errors.  
> &ensp;&ensp;&ensp;&ensp;2. If not ready, the api will auto enable motion and set state  
> &ensp;&ensp;&ensp;&ensp;3. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc  
>   
> :param speed: reset speed (unit: rad/s if is_radian is True else °/s), default is 50 °/s  
> :param mvacc: reset acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is 5000 °/s^2  
> :param mvtime: reserved  
> :param is_radian: the speed and acceleration are in radians or not, default is self.default_is_radian  
> :param wait: whether to wait for the arm to complete, default is False  
> :param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True


#### def __robotiq_close__(self, speed=255, force=255, wait=True, timeout=5, **kwargs):

> Close the robotiq gripper  
>   
> :param speed: gripper speed between 0 and 255  
> :param force: gripper force between 0 and 255  
> :param wait: whether to wait for the robotiq motion to complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 3, only available if wait=True  
>   
> :return: tuple((code, robotiq_response))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;robotiq_response: See the robotiq documentation


#### def __robotiq_get_status__(self, number_of_registers=3):

> Reading the status of robotiq gripper  
>   
> :param number_of_registers: number of registers, 1/2/3, default is 3  
> &ensp;&ensp;&ensp;&ensp;number_of_registers=1: reading the content of register 0x07D0  
> &ensp;&ensp;&ensp;&ensp;number_of_registers=2: reading the content of register 0x07D0/0x07D1  
> &ensp;&ensp;&ensp;&ensp;number_of_registers=3: reading the content of register 0x07D0/0x07D1/0x07D2  
> &ensp;&ensp;&ensp;&ensp;  
> &ensp;&ensp;&ensp;&ensp;Note:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;register 0x07D0: Register GRIPPER STATUS  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;register 0x07D1: Register FAULT STATUS and register POSITION REQUEST ECHO  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;register 0x07D2: Register POSITION and register CURRENT  
> :return: tuple((code, robotiq_response))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;robotiq_response: See the robotiq documentation


#### def __robotiq_open__(self, speed=255, force=255, wait=True, timeout=5, **kwargs):

> Open the robotiq gripper  
>   
> :param speed: gripper speed between 0 and 255  
> :param force: gripper force between 0 and 255  
> :param wait: whether to wait for the robotiq motion to complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 5, only available if wait=True  
>   
> :return: tuple((code, robotiq_response))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;robotiq_response: See the robotiq documentation


#### def __robotiq_reset__(self):

> Reset the robotiq gripper (clear previous activation if any)  
>   
> :return: tuple((code, robotiq_response))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;robotiq_response: See the robotiq documentation


#### def __robotiq_set_activate__(self, wait=True, timeout=3):

> If not already activated. Activate the robotiq gripper  
>   
> :param wait: whether to wait for the robotiq activate complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 3, only available if wait=True  
>   
> :return: tuple((code, robotiq_response))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;robotiq_response: See the robotiq documentation


#### def __robotiq_set_position__(self, pos, speed=255, force=255, wait=True, timeout=5, **kwargs):

> Go to the position with determined speed and force.  
>   
> :param pos: position of the gripper. Integer between 0 and 255. 0 being the open position and 255 being the close position.  
> :param speed: gripper speed between 0 and 255  
> :param force: gripper force between 0 and 255  
> :param wait: whether to wait for the robotion motion complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 5, only available if wait=True  
>   
> :return: tuple((code, robotiq_response))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;robotiq_response: See the robotiq documentation


#### def __run_blockly_app__(self, path, **kwargs):

> Run the app generated by xArmStudio software  
> :param path: app path


#### def __run_gcode_app__(self, path, **kwargs):

> Run gcode project file by xArmStudio software  
> :param path: gcode file path  
>   
> :return: code, only when code is 0, the returned result is correct.


#### def __run_gcode_file__(self, path, **kwargs):

> Run the gcode file  
> :param path: gcode file path


#### def __save_conf__(self):

> Save config  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;2. The clean_conf interface can restore system default settings  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __save_record_trajectory__(self, filename, wait=True, timeout=5, **kwargs):

> Save the trajectory you just recorded  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
>   
> :param filename: The name to save  
> &ensp;&ensp;&ensp;&ensp;1. Only strings consisting of English or numbers are supported, and the length is no more than 50.  
> &ensp;&ensp;&ensp;&ensp;2. The trajectory is saved in the controller box.  
> &ensp;&ensp;&ensp;&ensp;3. This action will overwrite the trajectory with the same name  
> &ensp;&ensp;&ensp;&ensp;4. Empty the trajectory in memory after saving, so repeated calls will cause the recorded trajectory to be covered by an empty trajectory.  
> :param wait: Whether to wait for saving, default is True  
> :param timeout: Timeout waiting for saving to complete  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __send_cmd_sync__(self, command=None):

> Send cmd and wait (only waiting the cmd response, not waiting for the movement)  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Some command depends on self.default_is_radian  
>   
> :param command:  
> &ensp;&ensp;&ensp;&ensp;'G1': 'set_position(MoveLine): G1 X{x} Y{y} Z{z} A{roll} B{pitch} C{yaw} F{speed} Q{acc} T{mvtime}'  
> &ensp;&ensp;&ensp;&ensp;'G2': 'move_circle: G2 X{x1} Y{y1} Z{z1} A{roll1} B{pitch1} C{yaw1} I{x2} J{y2} K{z2} L{roll2} M{pitch2} N{yaw2} F{speed} Q{acc} T{mvtime}'  
> &ensp;&ensp;&ensp;&ensp;'G4': 'set_pause_time: G4 T{second}'  
> &ensp;&ensp;&ensp;&ensp;'G7': 'set_servo_angle: G7 I{servo_1} J{servo_2} K{servo_3} L{servo_4} M{servo_5} N{servo_6} O{servo_7} F{speed} Q{acc} T{mvtime}'  
> &ensp;&ensp;&ensp;&ensp;'G8': 'move_gohome: G8 F{speed} Q{acc} T{mvtime}'  
> &ensp;&ensp;&ensp;&ensp;'G9': 'set_position(MoveArcLine): G9 X{x} Y{y} Z{z} A{roll} B{pitch} C{yaw} R{radius} F{speed} Q{acc} T{mvtime}'  
> &ensp;&ensp;&ensp;&ensp;'G11': 'set_servo_angle_j: G11 I{servo_1} J{servo_2} K{servo_3} L{servo_4} M{servo_5} N{servo_6} O{servo_7} F{speed} Q{acc} T{mvtime}'  
> &ensp;&ensp;&ensp;&ensp;'H1': 'get_version: H1'  
> &ensp;&ensp;&ensp;&ensp;'H11': 'motion_enable: H11 I{servo_id} V{enable}'  
> &ensp;&ensp;&ensp;&ensp;'H12': 'set_state: H12 V{state}'  
> &ensp;&ensp;&ensp;&ensp;'H13': 'get_state: H13'  
> &ensp;&ensp;&ensp;&ensp;'H14': 'get_cmdnum: H14'  
> &ensp;&ensp;&ensp;&ensp;'H15': 'get_err_warn_code: H15'  
> &ensp;&ensp;&ensp;&ensp;'H16': 'clean_error: H16'  
> &ensp;&ensp;&ensp;&ensp;'H17': 'clean_warn: H17'  
> &ensp;&ensp;&ensp;&ensp;'H18': 'set_servo_attach/set_servo_detach: H18 I{servo_id} V{1: enable(detach), 0: disable(attach)}'  
> &ensp;&ensp;&ensp;&ensp;'H19': 'set_mode: H19 V{mode}'  
> &ensp;&ensp;&ensp;&ensp;'H31': 'set_tcp_jerk: H31 V{jerk(mm/s^3)}'  
> &ensp;&ensp;&ensp;&ensp;'H32': 'set_tcp_maxacc: H32 V{maxacc(mm/s^2)}'  
> &ensp;&ensp;&ensp;&ensp;'H33': 'set_joint_jerk: H33 V{jerk(°/s^3 or rad/s^3)}'  
> &ensp;&ensp;&ensp;&ensp;'H34': 'set_joint_maxacc: H34 {maxacc(°/s^2 or rad/s^2)}'  
> &ensp;&ensp;&ensp;&ensp;'H35': 'set_tcp_offset: H35 X{x} Y{y} Z{z} A{roll} B{pitch} C{yaw}'  
> &ensp;&ensp;&ensp;&ensp;'H36': 'set_tcp_load: H36 I{weight} J{center_x} K{center_y} L{center_z}'  
> &ensp;&ensp;&ensp;&ensp;'H37': 'set_collision_sensitivity: H37 V{sensitivity}'  
> &ensp;&ensp;&ensp;&ensp;'H38': 'set_teach_sensitivity: H38 V{sensitivity}'  
> &ensp;&ensp;&ensp;&ensp;'H39': 'clean_conf: H39'  
> &ensp;&ensp;&ensp;&ensp;'H40': 'save_conf: H40'  
> &ensp;&ensp;&ensp;&ensp;'H41': 'get_position: H41'  
> &ensp;&ensp;&ensp;&ensp;'H42': 'get_servo_angle: H42'  
> &ensp;&ensp;&ensp;&ensp;'H43': 'get_inverse_kinematics: H43 X{x} Y{y} Z{z} A{roll} B{pitch} C{yaw}'  
> &ensp;&ensp;&ensp;&ensp;'H44': 'get_forward_kinematics: H44 I{servo_1} J{servo_2} K{servo_3} L{servo_4} M{servo_5} N{servo_6} O{servo_7}'  
> &ensp;&ensp;&ensp;&ensp;'H45': 'is_joint_limit: H45 I{servo_1} J{servo_2} K{servo_3} L{servo_4} M{servo_5} N{servo_6} O{servo_7}'  
> &ensp;&ensp;&ensp;&ensp;'H46': 'is_tcp_limit: H46 X{x} Y{y} Z{z} A{roll} B{pitch} C{yaw}'  
> &ensp;&ensp;&ensp;&ensp;'H51': 'set_gravity_direction: H51 X{x} Y{y} Z{z}'  
> &ensp;&ensp;&ensp;&ensp;'H106': 'get_servo_debug_msg: H106'  
> &ensp;&ensp;&ensp;&ensp;'M116': 'set_gripper_enable: M116 V{enable}'  
> &ensp;&ensp;&ensp;&ensp;'M117': 'set_gripper_mode: M117 V{mode}'  
> &ensp;&ensp;&ensp;&ensp;'M119': 'get_gripper_position: M119'  
> &ensp;&ensp;&ensp;&ensp;'M120': 'set_gripper_position: M120 V{pos}'  
> &ensp;&ensp;&ensp;&ensp;'M121': 'set_gripper_speed: M116 V{speed}'  
> &ensp;&ensp;&ensp;&ensp;'M125': 'get_gripper_err_code: M125'  
> &ensp;&ensp;&ensp;&ensp;'M126': 'clean_gripper_error: M126'  
> &ensp;&ensp;&ensp;&ensp;'M131': 'get_tgpio_digital: M131'  
> &ensp;&ensp;&ensp;&ensp;'M132': 'set_tgpio_digital: M132 I{ionum} V{value}'  
> &ensp;&ensp;&ensp;&ensp;'M133': 'get_tgpio_analog, default ionum=0: M133 I{ionum=0}'  
> &ensp;&ensp;&ensp;&ensp;'M134': 'get_tgpio_analog, default ionum=1: M134 I{ionum=1}'  
> &ensp;&ensp;&ensp;&ensp;'C131': 'get_cgpio_digital: C131'  
> &ensp;&ensp;&ensp;&ensp;'C132': 'get_cgpio_analog, default ionum=0: C132 I{ionum=0}'  
> &ensp;&ensp;&ensp;&ensp;'C133': 'get_cgpio_analog, default ionum=1: C133 I{ionum=1}'  
> &ensp;&ensp;&ensp;&ensp;'C134': 'set_cgpio_digital: C134 I{ionum} V{value}'  
> &ensp;&ensp;&ensp;&ensp;'C135': 'set_cgpio_analog, default ionum=0: C135 I{ionum=0} V{value}'  
> &ensp;&ensp;&ensp;&ensp;'C136': 'set_cgpio_analog, default ionum=1: C136 I{ionum=1} V{value}'  
> &ensp;&ensp;&ensp;&ensp;'C137': 'set_cgpio_digital_input_function: C137 I{ionum} V{fun}'  
> &ensp;&ensp;&ensp;&ensp;'C138': 'set_cgpio_digital_output_function: C138 I{ionum} V{fun}'  
> &ensp;&ensp;&ensp;&ensp;'C139': 'get_cgpio_state: C139'  
> :return: code or tuple((code, ...))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __send_hex_cmd__(self, datas, **kwargs):

> Hexadecimal communication protocol instruction  
>   
> :param datas: Hexadecimal data_list  
> :param timeout: timeout: wait timeout, seconds, default is 10s.  
> :return : Hexadecimal data_list or code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)


#### def __set_allow_approx_motion__(self, on_off):

> Settings allow to avoid overspeed near some singularities using approximate solutions  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.9.0  
>   
> :param on_off: allow or not, True means allow, default is False  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_baud_checkset_enable__(self, enable):

> Enable auto checkset the baudrate of the end IO board or not  
> Note:  
> &ensp;&ensp;&ensp;&ensp;only available in the API of gripper/bio/robotiq/linear_motor.  
> &ensp;&ensp;&ensp;&ensp;  
> :param enable: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_bio_gripper_control_mode__(self, mode):

> Set the bio gripper control mode  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Only available in the new version of BIO Gripper  
>   
> :param mode: mode  
> &ensp;&ensp;&ensp;&ensp;0: bio gripper opening and closing mode  
> &ensp;&ensp;&ensp;&ensp;1: position loop mode  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_bio_gripper_enable__(self, enable=True, wait=True, timeout=3):

> If not already enabled. Enable the bio gripper  
>   
> :param enable: enable or not  
> :param wait: whether to wait for the bio gripper enable complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 3, only available if wait=True  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_bio_gripper_force__(self, force):

> Set the bio gripper force  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Only available in the new version of BIO Gripper  
>   
> :param force: gripper force between 10 and 100  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_bio_gripper_g2_position__(self, pos, speed=2000, force=100, wait=True, timeout=5, **kwargs):

> Set the position of BIO Gripper G2  
>   
> :param pos: gripper pos between 71 and 150, (unit: mm)  
> :param speed: gripper speed between 500 and 4500, default is 2000, (unit: pulse/s)  
> :param force: gripper force between 1 and 100, default is 100  
> :param wait: whether to wait for the robotiq motion to complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 5, only available if wait=True  
>   
> :return: tuple((code, robotiq_response))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;robotiq_response: See the robotiq documentation


#### def __set_bio_gripper_speed__(self, speed):

> Set the speed of the bio gripper  
>   
> :param speed: speed  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_cartesian_velo_continuous__(self, on_off):

> Set cartesian motion velocity continuous  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.9.0  
>   
> :param on_off: continuous or not, True means continuous, default is False  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_cgpio_analog__(self, ionum, value, sync=True):

> Set the analog value of the specified Controller GPIO  
>   
> :param ionum: 0 or 1  
> :param value: value  
> :param sync: whether to execute in the motion queue, set to False to execute immediately(default is True)  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_cgpio_analog_with_xyz__(self, ionum, value, xyz, fault_tolerance_radius):

> Set the analog value of the specified Controller GPIO when the robot has reached the specified xyz position             
>   
> :param ionum: 0 ~ 1  
> :param value: value  
> :param xyz: position xyz, as [x, y, z]  
> :param fault_tolerance_radius: fault tolerance radius  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_cgpio_digital__(self, ionum, value, delay_sec=None, sync=True):

> Set the digital value of the specified Controller GPIO  
>   
> :param ionum: 0~7(CO0~CO7), 8~15(DO0~DO7)  
> :param value: value  
> :param delay_sec: delay effective time from the current start, in seconds, default is None(effective immediately)  
> :param sync: whether to execute in the motion queue, set to False to execute immediately(default is True)  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
> &ensp;&ensp;&ensp;&ensp;2. only available if delay_sec <= 0  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_cgpio_digital_input_function__(self, ionum, fun):

> Set the digital input functional mode of the Controller GPIO  
> :param ionum: 0~7(CI0~CI7), 8~15(DI0~DI7)  
> :param fun: functional mode  
> &ensp;&ensp;&ensp;&ensp;0: general input  
> &ensp;&ensp;&ensp;&ensp;1: external emergency stop  
> &ensp;&ensp;&ensp;&ensp;2: protection reset  
> &ensp;&ensp;&ensp;&ensp;11: offline task  
> &ensp;&ensp;&ensp;&ensp;12: teaching mode  
> &ensp;&ensp;&ensp;&ensp;13: reduced mode  
> &ensp;&ensp;&ensp;&ensp;14: enable arm  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_cgpio_digital_output_function__(self, ionum, fun):

> Set the digital output functional mode of the specified Controller GPIO  
> :param ionum: 0~7(CO0~CO7), 8~15(DO0~DO7)  
> :param fun: functionnal mode  
> &ensp;&ensp;&ensp;&ensp;0: general output  
> &ensp;&ensp;&ensp;&ensp;1: emergency stop  
> &ensp;&ensp;&ensp;&ensp;2: in motion  
> &ensp;&ensp;&ensp;&ensp;11: has error  
> &ensp;&ensp;&ensp;&ensp;12: has warn  
> &ensp;&ensp;&ensp;&ensp;13: in collision  
> &ensp;&ensp;&ensp;&ensp;14: in teaching  
> &ensp;&ensp;&ensp;&ensp;15: in offline task  
> &ensp;&ensp;&ensp;&ensp;16: in reduced mode  
> &ensp;&ensp;&ensp;&ensp;17: is enabled  
> &ensp;&ensp;&ensp;&ensp;18: emergency stop is pressed  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_cgpio_digital_with_xyz__(self, ionum, value, xyz, fault_tolerance_radius):

> Set the digital value of the specified Controller GPIO when the robot has reached the specified xyz position             
>   
> :param ionum: 0 ~ 15  
> :param value: value  
> :param xyz: position xyz, as [x, y, z]  
> :param fault_tolerance_radius: fault tolerance radius  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_checkset_default_baud__(self, type_, baud):

> Set the checkset baud value  
>   
> :param type_: checkset type  
> &ensp;&ensp;&ensp;&ensp;1: xarm gripper  
> &ensp;&ensp;&ensp;&ensp;2: bio gripper  
> &ensp;&ensp;&ensp;&ensp;3: robotiq gripper  
> &ensp;&ensp;&ensp;&ensp;4: linear motor  
> :param baud: checkset baud value, less than or equal to 0 means disable checkset  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_cmd_mat_history_num__(self, num):

> Set cmd mat history num  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :param num: history num  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_collision_rebound__(self, on):

> Set the collision rebound,turn on/off collision rebound  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.11 or above  
>   
> :param on: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_collision_sensitivity__(self, value, wait=True):

> Set the sensitivity of collision  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Do not use if not required  
> &ensp;&ensp;&ensp;&ensp;2. If not saved, it will be lost after reboot  
> &ensp;&ensp;&ensp;&ensp;3. The save_conf interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;4. The clean_conf interface can restore system default settings  
>   
> :param value: sensitivity value, 0~5  
> :param wait: reversed  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_collision_tool_model__(self, tool_type, *args, **kwargs):

> Set the geometric model of the end effector for self collision detection  
> &ensp;  
> :param tool_type: the geometric model type  
> &ensp;&ensp;&ensp;&ensp;0: No end effector, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;1: xArm Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;2: xArm Vacuum Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;3: xArm Bio Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;4: Robotiq-2F-85 Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;5: Robotiq-2F-140 Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;7: Lite Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;8: Lite Vacuum Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;9: xArm Gripper G2, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;10: PGC-140-50 of the DH-ROBOTICS, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;11: RH56DFX-2L of the INSPIRE-ROBOTS, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;12: RH56DFX-2R of the INSPIRE-ROBOTS, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;13: xArm Bio Gripper G2, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;21: Cylinder, need additional parameters radius, height  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;self.set_collision_tool_model(21, radius=45, height=137)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param radius: the radius of cylinder, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param height: the height of cylinder, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param x_offset: offset in the x direction, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param y_offset: offset in the y direction, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param z_offset: offset in the z direction, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;22: Cuboid, need additional parameters x, y, z  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;self.set_collision_tool_model(22, x=234, y=323, z=23)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param x: the length of the cuboid in the x coordinate direction, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param y: the length of the cuboid in the y coordinate direction, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param z: the length of the cuboid in the z coordinate direction, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param x_offset: offset in the x direction, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param y_offset: offset in the y direction, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param z_offset: offset in the z direction, (unit: mm)  
> :param args: additional parameters  
> :param kwargs: additional parameters  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_control_modbus_baudrate__(self, baud):

> Set the modbus baudrate of the control box  
>   
> :param baud: 4800/9600/19200/38400/57600/115200/230400/460800/921600/1000000/1500000/2000000/2500000  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_counter_increase__(self, val=1):

> Set counter plus value, only support plus 1  
>   
> :param val: reversed  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_counter_reset__(self):

> Reset counter value  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_dh_params__(self, dh_params, flag=0):

> Set the DH parameters  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.0.0  
> &ensp;&ensp;&ensp;&ensp;2. this interface is only provided for users who need to use external DH parameters, ordinary users should not try to modify DH parameters.  
>   
> :param dh_params: DH parameters  
> :param flag:   
> &ensp;&ensp;&ensp;&ensp;0: Use the set DH parameters, but do not write to the configuration file  
> &ensp;&ensp;&ensp;&ensp;1: Use the set DH parameters and write to the configuration file  
> &ensp;&ensp;&ensp;&ensp;2: Use the set DH parameters and delete the DH parameters of the configuration file  
> &ensp;&ensp;&ensp;&ensp;3: Use the default DH parameters, but will not delete the DH parameters of the configuration file  
> &ensp;&ensp;&ensp;&ensp;4: Use the default DH parameters and delete the DH parameters of the configuration file  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_fdb_mat_history_num__(self, num):

> Set fdb mat history num  
> Note:  
> &ensp;&ensp;&ensp;&ensp;Only available if firmware_version >= 2.3.0  
>   
> :param num: history num  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_feedback_type__(self, feedback_type):

> Set the feedback type  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.1.0  
> &ensp;&ensp;&ensp;&ensp;2. only works in position mode  
> &ensp;&ensp;&ensp;&ensp;3. the setting will only affect subsequent tasks and will not affect previously cached tasks  
> &ensp;&ensp;&ensp;&ensp;4. only valid for the current connection  
>   
> :param feedback_type:  
> &ensp;&ensp;&ensp;&ensp;0: disable feedback  
> &ensp;&ensp;&ensp;&ensp;1: feedback when the motion task starts executing  
> &ensp;&ensp;&ensp;&ensp;2: feedback when the motion task execution ends or motion task is discarded(usually when the distance is too close to be planned)  
> &ensp;&ensp;&ensp;&ensp;4: feedback when the non-motion task is triggered  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_fence_mode__(self, on):

> Set the fence mode,turn on/off fense mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.11 or above  
>   
> :param on: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_ft_admittance_ctrl_threshold__(self, thresholds):

> Set the reaction thresholds in each direction under the admittance control mode of the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.110  
>   
> :param thresholds: thresholds, [x(N), y(N), z(N), Rx(Nm), Ry(Nm), Rz(Nm)]  
> &ensp;&ensp;&ensp;&ensp;x: [0.1, 50] (N)  
> &ensp;&ensp;&ensp;&ensp;y: [0.1, 50] (N)  
> &ensp;&ensp;&ensp;&ensp;z: [0.1, 50] (N)  
> &ensp;&ensp;&ensp;&ensp;Rx: [0.01, 2] (Nm)  
> &ensp;&ensp;&ensp;&ensp;Ry: [0.01, 2] (Nm)  
> &ensp;&ensp;&ensp;&ensp;Rz: [0.01, 2] (Nm)  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_ft_collision_detection__(self, on_off):

> Set whether to enable collision detection with the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
> &ensp;&ensp;&ensp;&ensp;3. the Six-axis Force Torque Sensor needs to be enabled and set force mode  
>   
> :param on_off: enable or not  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_ft_collision_reb_distance__(self, distances, is_radian=None):

> Set the rebound distance of the collision rebound with the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103  
>   
> :param distances: collision rebound distance, [x(mm), y(mm), z(mm), Rx(° or rad), Ry(° or rad), Rz(° or rad)]  
> &ensp;&ensp;&ensp;&ensp;x: [2, 500] (mm)  
> &ensp;&ensp;&ensp;&ensp;y: [2, 500] (mm)  
> &ensp;&ensp;&ensp;&ensp;z: [2, 500] (mm)  
> &ensp;&ensp;&ensp;&ensp;Rx: [0.2, 50] (°)  
> &ensp;&ensp;&ensp;&ensp;Ry: [0.2, 50] (°)  
> &ensp;&ensp;&ensp;&ensp;Rz: [0.2, 50] (°)  
> :param is_radian: the value of distance (only Rx/Ry/Rz) is in radians or not, default is self.default_is_radian  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_ft_collision_rebound__(self, on_off):

> Set whether to enable collision rebound with the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103  
>   
> :param on_off: enable or not  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_ft_collision_threshold__(self, thresholds):

> Set the thresholds of the collision detection with the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.6.103  
>   
> :param thresholds: collision detection thresholds, [x(N), y(N), z(N), Rx(Nm), Ry(Nm), Rz(Nm)]  
> &ensp;&ensp;&ensp;&ensp;x: [5, 200] (N)  
> &ensp;&ensp;&ensp;&ensp;y: [5, 200] (N)  
> &ensp;&ensp;&ensp;&ensp;z: [5, 200] (N)  
> &ensp;&ensp;&ensp;&ensp;Rx: [0.1, 4] (Nm)  
> &ensp;&ensp;&ensp;&ensp;Ry: [0.1, 4] (Nm)  
> &ensp;&ensp;&ensp;&ensp;Rz: [0.1, 4] (Nm)  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_ft_sensor_admittance_parameters__(self, coord=None, c_axis=None, M=None, K=None, B=None, **kwargs):

> Set the parameters of admittance control through the Six-axis Force Torque Sensor.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
> &ensp;&ensp;&ensp;&ensp;3. parameters coord and c_axis must be specified at the same time, either as an integer(only coord) and array or None (not set)  
> &ensp;&ensp;&ensp;&ensp;4. parameters M, K, and B must be specified at the same time, either as an array or None (not set)  
> &ensp;&ensp;&ensp;&ensp;5. supports multiple parameter combinations and sequences  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;- set_ft_sensor_admittance_parameters(coord, c_axis)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;- set_ft_sensor_admittance_parameters(M, K, B)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;- set_ft_sensor_admittance_parameters(coord, c_axis, M, K, B)  
>   
> :param coord: task frame (0: base frame. 1: tool frame)  
> :param c_axis: a 6d vector of 0s and 1s. 1 means that robot will be admittance in the corresponding axis of the task frame.  
> :param M: 6d vector, mass. (kg)  
> :param K: 6d vector, stiffness coefficient.  
> :param B: 6d vector, damping coefficient.  
> &ensp;&ensp;&ensp;&ensp;Note: the value is set to 2*sqrt(M*K) in controller.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_ft_sensor_enable__(self, on_off):

> Used for enabling and disabling the use of the Six-axis Force Torque Sensor measurements in the controller.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :param on_off: enable or disable F/T data sampling.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_ft_sensor_force_parameters__(self, coord=None, c_axis=None, f_ref=None, limits=None, kp=None, ki=None, kd=None, xe_limit=None, **kwargs):

> Set the parameters of force control through the Six-axis Force Torque Sensor.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
> &ensp;&ensp;&ensp;&ensp;3. parameters coord, c_axis, f_ref and limits must be specified at the same time, either as an integer(only coord) and array or None (not set)  
> &ensp;&ensp;&ensp;&ensp;4. parameters kp, ki, kd, and xe_limit must be specified at the same time, either as an array or None (not set)  
> &ensp;&ensp;&ensp;&ensp;5. supports multiple parameter combinations and sequences  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;- set_ft_sensor_force_parameters(coord, c_axis, f_ref, limits)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;- set_ft_sensor_force_parameters(kp, ki, kd, xe_limit)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;- set_ft_sensor_force_parameters(coord, c_axis, f_ref, limits, kp, ki, kd, xe_limit)  
>   
> :param coord: task frame (0: base frame. 1: tool frame)  
> :param c_axis: a 6d vector of 0s and 1s. 1 means that robot will be compliant in the corresponding axis of the task frame.  
> :param f_ref: 6d vector, the forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque.  
> :param limits: 6d vector, for compliant axes, these values are the maximum allowed tcp speed along/about the axis.  
> :param kp: 6d vector, proportional gain.  
> :param ki: 6d vector, integral gain.  
> :param kd: 6d vector, differential gain.  
> :param xe_limit: 6d vector. for compliant axes, these values are the maximum allowed tcp speed along/about the axis. mm/s  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_ft_sensor_load_offset__(self, iden_result_list, association_setting_tcp_load=False, **kwargs):

> Write the load offset parameters identified by the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
> &ensp;&ensp;&ensp;&ensp;3. starting from SDK v1.11.0, the centroid unit is millimeters (originally meters)  
>   
> :param iden_result_list:  [mass(kg), x_centroid(mm), y_centroid(mm), z_centroid(mm), Fx_offset, Fy_offset, Fz_offset, Tx_offset, Ty_offset, Tz_ffset]  
> :param association_setting_tcp_load: whether to convert the parameter to the corresponding tcp load and set, default is False  
> &ensp;&ensp;&ensp;&ensp;Note: If True, the value of tcp load will be modified  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_ft_sensor_mode__(self, mode, **kwargs):

> Set robot to be controlled in force mode. (Through the Six-axis Force Torque Sensor)  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :param app_code: force mode.  
> &ensp;&ensp;&ensp;&ensp;0: non-force mode  
> &ensp;&ensp;&ensp;&ensp;1: admittance control  
> &ensp;&ensp;&ensp;&ensp;2: force control  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_ft_sensor_zero__(self):

> Set the current state to the zero point of the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_gravity_direction__(self, direction, wait=True):

> Set the gravity direction for proper torque compensation and collision detection.  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Use only if necessary. Incorrect settings may affect torque compensation.  
> &ensp;&ensp;&ensp;&ensp;2. Changes are not saved automatically. Call save_conf() to save the settings,   
> &ensp;&ensp;&ensp;otherwise, they will be lost after a reboot.  
> &ensp;&ensp;&ensp;&ensp;3. Use clean_conf() to restore the system default settings.  
> &ensp;&ensp;&ensp;&ensp;4. The clean_conf interface can restore system default settings  
>   
> :param direction: Gravity direction vector [x, y, z], e.g., [0, 0, -1] for a floor-mounted arm.  
> :param wait: Whether to wait for the robotic arm to stop or clear all previous queued commands before applying the setting.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_gripper_enable__(self, enable, **kwargs):

> Set the gripper enable  
>   
> :param enable: enable or not  
> &ensp;&ensp;&ensp;&ensp;Note: such as code = arm.set_gripper_enable(True)  #turn on the Gripper  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_gripper_g2_position__(self, pos, speed=100, force=50, wait=False, timeout=None, **kwargs):

> Set the position of the xArm Gripper G2  
>   
> :param pos: gripper pos between 0 and 84, (unit: mm)  
> :param speed: gripper speed between 15 and 225, default is 100, (unit: mm/s)  
> :param force: gripper force between 1 and 100, default is 50  
> :param wait: whether to wait for the bio gripper motion complete, default is False  
> :param timeout: maximum waiting time(unit: second), default is 10s, only valid if wait is True  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_gripper_mode__(self, mode, **kwargs):

> Set the gripper mode  
>   
> :param mode: 0: location mode  
> &ensp;&ensp;&ensp;&ensp;Note: such as code = arm.set_gripper_mode(0)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_gripper_position__(self, pos, wait=False, speed=None, auto_enable=False, timeout=None, **kwargs):

> Set the gripper position  
>   
> :param pos: pos  
> :param wait: wait or not, default is False  
> :param speed: speed, unit:r/min  
> :param auto_enable: auto enable or not, default is False  
> :param timeout: wait time, unit:second, default is 10s  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_gripper_speed__(self, speed, **kwargs):

> Set the gripper speed  
>   
> :param speed:  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_initial_point__(self, point):

> Set the initial point  
>   
> :param point: initial point, [J1, J2, ..., J7]  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_joint_jerk__(self, jerk, is_radian=None):

> Set the jerk of Joint space  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Do not use if not required  
> &ensp;&ensp;&ensp;&ensp;2. If not saved, it will be lost after reboot  
> &ensp;&ensp;&ensp;&ensp;3. The save_conf interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;4. The clean_conf interface can restore system default settings  
>   
> :param jerk: jerk (°/s^3 or rad/s^3)  
> :param is_radian: the jerk in radians or not, default is self.default_is_radian  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_joint_maxacc__(self, acc, is_radian=None):

> Set the max acceleration of Joint space  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Do not use if not required  
> &ensp;&ensp;&ensp;&ensp;2. If not saved, it will be lost after reboot  
> &ensp;&ensp;&ensp;&ensp;3. The save_conf interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;4. The clean_conf interface can restore system default settings  
>   
> :param acc: max acceleration (°/s^2 or rad/s^2)  
> :param is_radian: the jerk in radians or not, default is self.default_is_radian  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_linear_motor_back_origin__(self, wait=True, **kwargs):

> Set the linear motor go back to the origin position  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
> &ensp;&ensp;&ensp;&ensp;2. only useful when powering on for the first time  
> &ensp;&ensp;&ensp;&ensp;3. this operation must be performed at the first power-on  
> &ensp;&ensp;&ensp;&ensp;  
> :param wait: wait to motion finish or not, default is True  
> :param kwargs:  
> &ensp;&ensp;&ensp;&ensp;auto_enable: enable after back to origin or not, default is True  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_linear_motor_enable__(self, enable):

> Set the linear motor enable/disable  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :param enable: enable or not  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_linear_motor_pos__(self, pos, speed=None, wait=True, timeout=100, **kwargs):

> Set the position of the linear motor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :param pos: position. Integer between 0 and 700/1000/1500mm.  
> &ensp;&ensp;&ensp;&ensp;If SN start with AL1300 the position range is 0~700mm.  
> &ensp;&ensp;&ensp;&ensp;If SN start with AL1301 the position range is 0~1000mm.  
> &ensp;&ensp;&ensp;&ensp;If SN start with AL1302 the position range is 0~1500mm.  
> :param speed: speed of the linear motor. Integer between 1 and 1000mm/s. default is not set  
> :param wait: wait to motion finish or not, default is True  
> :param timeout: wait timeout, seconds, default is 100s.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_linear_motor_speed__(self, speed):

> Set the speed of the linear motor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :param speed: Integer between 1 and 1000mm/s.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_linear_motor_stop__(self):

> Set the linear motor to stop  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_linear_spd_limit_factor__(self, factor):

> Set linear speed limit factor (default is 1.2)  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.3.0  
> &ensp;&ensp;&ensp;&ensp;2. only available in mode 1  
>   
> :param factor: speed limit factor  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_mode__(self, mode=0, detection_param=0):

> Set the xArm mode  
>   
> :param mode: default is 0  
> &ensp;&ensp;&ensp;&ensp;0: position control mode  
> &ensp;&ensp;&ensp;&ensp;1: servo motion mode  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: the use of the set_servo_angle_j interface must first be set to this mode  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: the use of the set_servo_cartesian interface must first be set to this mode  
> &ensp;&ensp;&ensp;&ensp;2: joint teaching mode  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: use this mode to ensure that the arm has been identified and the control box and arm used for identification are one-to-one.  
> &ensp;&ensp;&ensp;&ensp;3: cartesian teaching mode (invalid)  
> &ensp;&ensp;&ensp;&ensp;4: joint velocity control mode  
> &ensp;&ensp;&ensp;&ensp;5: cartesian velocity control mode  
> &ensp;&ensp;&ensp;&ensp;6: joint online trajectory planning mode   
> &ensp;&ensp;&ensp;&ensp;7: cartesian online trajectory planning mode   
> :param detection_param: Teaching detection parameters, default is 0  
> &ensp;&ensp;&ensp;&ensp;0: Turn on motion detection   
> &ensp;&ensp;&ensp;&ensp;1: Turn off motion detection  
> &ensp;&ensp;&ensp;&ensp;Note:  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.10.1  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. only available if set_mode(2)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_mount_direction__(self, base_tilt_deg, rotation_deg, is_radian=None):

> Set the mount direction  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Do not use if not required  
> &ensp;&ensp;&ensp;&ensp;2. If not saved, it will be lost after reboot  
> &ensp;&ensp;&ensp;&ensp;3. The save_conf interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;4. The clean_conf interface can restore system default settings  
>   
> :param base_tilt_deg: tilt degree  
> :param rotation_deg: rotation degree  
> :param is_radian: the base_tilt_deg/rotation_deg in radians or not, default is self.default_is_radian  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_only_check_type__(self, only_check_type=0):

> Set the motion process detection type (valid for all motion interfaces of the current SDK instance)  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.11.100  
> &ensp;&ensp;&ensp;&ensp;2. This interface is a global configuration item of the current SDK, and affects all motion-related interfaces  
> &ensp;&ensp;&ensp;&ensp;3. Generally, you only need to call when you don't want to move the robotic arm and only check whether some paths will have self-collision/angle-limit/cartesian-limit/overspeed.  
> &ensp;&ensp;&ensp;&ensp;4. Currently only self-collision/angle-limit/cartesian-limit/overspeed are detected  
> &ensp;&ensp;&ensp;&ensp;5. If only_check_type is set to be greater than 0, and the return value of calling the motion interface is not 0, you can view arm.only_check_result to view the specific error code  
>   
> Example: (Common scenarios, here is an example of the set_position interface)  
> &ensp;&ensp;&ensp;&ensp;1. Check whether the process from point A to point B is normal (no self-collision and overspeed triggered)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1.1 Move to point A  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;arm.set_only_check_type(0)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code = arm.set_position(A)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1.2 Check if the process from point A to point B is normal (no self-collision and overspeed triggered)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;arm.set_only_check_type(1)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code = arm.set_position(B)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;# If code is not equal to 0, it means that the path does not pass. You can check the specific error code through arm.only_check_result  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;arm.set_only_check_type(0)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;  
> &ensp;&ensp;&ensp;&ensp;2. Check whether the process from point A to point B, C, and D to point E is normal (no self-collision and overspeed are triggered)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2.1 Move to point A  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;arm.set_only_check_type(0)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code = arm.set_position(A)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2.2 Check whether the process of point A passing through points B, C, D to point E is normal (no self-collision and overspeed are triggered)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;arm.set_only_check_type(3)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code = arm.set_position(B)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;# If code is not equal to 0, it means that the path does not pass. You can check the specific error code through arm.only_check_result  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code = arm.set_position(C)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;# If code is not equal to 0, it means that the path does not pass. You can check the specific error code through arm.only_check_result  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code = arm.set_position(D)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;# If code is not equal to 0, it means that the path does not pass. You can check the specific error code through arm.only_check_result  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code = arm.set_position(E)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;# If code is not equal to 0, it means that the path does not pass. You can check the specific error code through arm.only_check_result  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;arm.set_only_check_type(0)  
>   
> :param only_check_type: Motion Detection Type  
> &ensp;&ensp;&ensp;&ensp;only_check_type == 0: Restore the original function of the motion interface, it will move, the default is 0  
> &ensp;&ensp;&ensp;&ensp;only_check_type == 1: Only check the self-collision without moving, take the actual state of the manipulator as the initial planned path, and check whether the path has self-collision (the intermediate state will be updated at this time)  
> &ensp;&ensp;&ensp;&ensp;only_check_type == 2: Only check the self-collision without moving, use the intermediate state as the starting planning path, check whether the path has self-collision (the intermediate state will be updated at this time), and restore the intermediate state to the actual state after the end  
> &ensp;&ensp;&ensp;&ensp;only_check_type == 3: Only check the self-collision without moving, use the intermediate state as the starting planning path, and check whether the path has self-collision (the intermediate state will be updated at this time)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_pause_time__(self, sltime, wait=False):

> Set the arm pause time, xArm will pause sltime second  
>   
> :param sltime: sleep time,unit:(s)second  
> :param wait: wait or not, default is False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_position__(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None, speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None, wait=False, timeout=None, **kwargs):

> Set the cartesian position, the API will modify self.last_used_position value  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. If it is xArm5, ensure that the current robotic arm has a roll value of 180° or π rad and has a roll value of 0 before calling this interface.  
> &ensp;&ensp;&ensp;&ensp;2. If it is xArm5, roll must be set to 180° or π rad, pitch must be set to 0  
> &ensp;&ensp;&ensp;&ensp;3. If the parameter(roll/pitch/yaw) you are passing is an radian unit, be sure to set the parameter is_radian to True.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_position(x=300, y=0, z=200, roll=-3.14, pitch=0, yaw=0, is_radian=True)  
> &ensp;&ensp;&ensp;&ensp;4. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_position(x=300, y=0, z=200, roll=180, pitch=0, yaw=0, is_radian=False, wait=True)  
> &ensp;&ensp;&ensp;&ensp;5. This interface is only used in the base coordinate system.  
>   
> :param x: cartesian position x, (unit: mm), default is self.last_used_position[0]  
> :param y: cartesian position y, (unit: mm), default is self.last_used_position[1]  
> :param z: cartesian position z, (unit: mm), default is self.last_used_position[2]  
> :param roll: rotate around the X axis, (unit: rad if is_radian is True else °), default is self.last_used_position[3]  
> :param pitch: rotate around the Y axis, (unit: rad if is_radian is True else °), default is self.last_used_position[4]  
> :param yaw: rotate around the Z axis, (unit: rad if is_radian is True else °), default is self.last_used_position[5]  
> :param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine  
> &ensp;&ensp;&ensp;&ensp;MoveLine: Linear motion  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_position(..., radius=None)  
> &ensp;&ensp;&ensp;&ensp;MoveArcLine: Linear arc motion with interpolation  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_position(..., radius=0)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: Need to set radius>=0  
> :param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed  
> :param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc  
> :param mvtime: 0, reserved  
> :param relative: relative move or not  
> :param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian  
> :param wait: whether to wait for the arm to complete, default is False  
> :param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True  
> :param kwargs: extra parameters  
> &ensp;&ensp;&ensp;&ensp;:param motion_type: motion planning type, default is 0  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;motion_type == 0: default, linear planning  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;motion_type == 1: prioritize linear planning, and turn to IK for joint planning when linear planning is not possible  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;motion_type == 2: direct transfer to IK using joint planning  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.11.100  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. when motion_type is 1 or 2, linear motion cannot be guaranteed  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. once IK is transferred to joint planning, the given Cartesian velocity and acceleration are converted into joint velocity and acceleration according to the percentage  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;speed = speed / max_tcp_speed * max_joint_speed  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;mvacc = mvacc / max_tcp_acc * max_joint_acc  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4. if there is no suitable IK, a C40 error will be triggered  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code < 0: the last_used_position/last_used_tcp_speed/last_used_tcp_acc will not be modified  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code >= 0: the last_used_position/last_used_tcp_speed/last_used_tcp_acc will be modified


#### def __set_position_aa__(self, axis_angle_pose, speed=None, mvacc=None, mvtime=None, is_radian=None, is_tool_coord=False, relative=False, wait=False, timeout=None, radius=None, **kwargs):

> Set the pose represented by the axis angle pose  
>   
> :param axis_angle_pose: the axis angle pose, [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]  
> :param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed  
> :param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc  
> :param mvtime: 0, reserved   
> :param is_radian: the rx/ry/rz of axis_angle_pose in radians or not, default is self.default_is_radian  
> :param is_tool_coord: is tool coordinate or not, if it is True, the relative parameter is no longer valid  
> :param relative: relative move or not  
> :param wait: whether to wait for the arm to complete, default is False  
> :param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True  
> :param radius: move radius, if radius is None or radius less than 0, will MoveLineAA, else MoveArcLineAA  
> &ensp;&ensp;&ensp;&ensp;only available if firmware_version >= 1.11.100  
> &ensp;&ensp;&ensp;&ensp;MoveLineAA: Linear motion  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_position_aa(..., radius=None)  
> &ensp;&ensp;&ensp;&ensp;MoveArcLineAA: Linear arc motion with interpolation  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_position_aa(..., radius=0)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: Need to set radius>=0  
> :param kwargs: extra parameters  
> &ensp;&ensp;&ensp;&ensp;:param motion_type: motion planning type, default is 0  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;motion_type == 0: default, linear planning  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;motion_type == 1: prioritize linear planning, and turn to IK for joint planning when linear planning is not possible  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;motion_type == 2: direct transfer to IK using joint planning  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.11.100  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. when motion_type is 1 or 2, linear motion cannot be guaranteed  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. once IK is transferred to joint planning, the given Cartesian velocity and acceleration are converted into joint velocity and acceleration according to the percentage  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;speed = speed / max_tcp_speed * max_joint_speed  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;mvacc = mvacc / max_tcp_acc * max_joint_acc  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4. if there is no suitable IK, a C40 error will be triggered  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_reduced_joint_range__(self, joint_range, is_radian=None):

> Set the joint range of the reduced mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.11 or above  
> &ensp;&ensp;&ensp;&ensp;2. Only reset the reduced mode to take effect (`set_reduced_mode(True)`)  
>   
> :param joint_range: [joint-1-min, joint-1-max, ..., joint-7-min, joint-7-max]  
> :param is_radian: the param joint_range are in radians or not, default is self.default_is_radian  
> :return:


#### def __set_reduced_max_joint_speed__(self, speed, is_radian=None):

> Set the maximum joint speed of the reduced mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
> &ensp;&ensp;&ensp;&ensp;2. Only reset the reduced mode to take effect (`set_reduced_mode(True)`)  
>   
> :param speed: speed (°/s or rad/s)  
> :param is_radian: the speed is in radians or not, default is self.default_is_radian  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_reduced_max_tcp_speed__(self, speed):

> Set the maximum tcp speed of the reduced mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
> &ensp;&ensp;&ensp;&ensp;2. Only reset the reduced mode to take effect (`set_reduced_mode(True)`)  
>   
> :param speed: speed (mm/s)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_reduced_mode__(self, on):

> Turn on/off reduced mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
>   
> :param on: True/False  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;such as:Turn on the reduced mode : code=arm.set_reduced_mode(True)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_reduced_tcp_boundary__(self, boundary):

> Set the boundary of the safety boundary mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
> &ensp;&ensp;&ensp;&ensp;2. Only reset the reduced mode to take effect (`set_reduced_mode(True)`)  
>   
> :param boundary: [x_max, x_min, y_max, y_min, z_max, z_min]  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_report_tau_or_i__(self, tau_or_i=0):

> Set the reported torque or electric current  
>   
> :param tau_or_i:   
> &ensp;&ensp;&ensp;&ensp;0: torque  
> &ensp;&ensp;&ensp;&ensp;1: electric current  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_self_collision_detection__(self, on_off):

> Set whether to enable self-collision detection   
>   
> :param on_off: enable or not  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_servo_angle__(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None, wait=False, timeout=None, radius=None, **kwargs):

> Set the servo angle, the API will modify self.last_used_angles value  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. If the parameter angle you are passing is an radian unit, be sure to set the parameter is_radian to True.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_servo_angle(servo_id=1, angle=1.57, is_radian=True)  
> &ensp;&ensp;&ensp;&ensp;2. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_servo_angle(servo_id=1, angle=45, is_radian=False,wait=True)  
> &ensp;&ensp;&ensp;&ensp;3. This interface is only used in the base coordinate system.  
>   
> :param servo_id: 1-(Number of axes), None(8)  
> &ensp;&ensp;&ensp;&ensp;1. 1-(Number of axes) indicates the corresponding joint, the parameter angle should be a numeric value  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_servo_angle(servo_id=1, angle=45, is_radian=False)  
> &ensp;&ensp;&ensp;&ensp;2. None(8) means all joints, default is None, the parameter angle should be a list of values whose length is the number of joints  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_servo_angle(angle=[30, -45, 0, 0, 0, 0, 0], is_radian=False)  
> :param angle: angle or angle list, (unit: rad if is_radian is True else °)  
> &ensp;&ensp;&ensp;&ensp;1. If servo_id is 1-(Number of axes), angle should be a numeric value  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_servo_angle(servo_id=1, angle=45, is_radian=False)  
> &ensp;&ensp;&ensp;&ensp;2. If servo_id is None or 8, angle should be a list of values whose length is the number of joints  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;like [axis-1, axis-2, axis-3, axis-3, axis-4, axis-5, axis-6, axis-7]  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_servo_angle(angle=[30, -45, 0, 0, 0, 0, 0], is_radian=False)  
> :param speed: move speed (unit: rad/s if is_radian is True else °/s), default is self.last_used_joint_speed  
> :param mvacc: move acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is self.last_used_joint_acc  
> :param mvtime: 0, reserved  
> :param relative: relative move or not  
> :param is_radian: the angle in radians or not, default is self.default_is_radian  
> :param wait: whether to wait for the arm to complete, default is False  
> :param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True  
> :param radius: move radius, if radius is None or radius less than 0, will MoveJoint, else MoveArcJoint  
> &ensp;&ensp;&ensp;&ensp;Note: Only available if version > 1.5.20  
> &ensp;&ensp;&ensp;&ensp;Note: The blending radius cannot be greater than the track length.  
> &ensp;&ensp;&ensp;&ensp;MoveJoint: joint motion  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_servo_angle(..., radius=None)  
> &ensp;&ensp;&ensp;&ensp;MoveArcJoint: joint fusion motion with interpolation  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_servo_angle(..., radius=0)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: Need to set radius>=0  
> :param kwargs: reserved  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code < 0: the last_used_angles/last_used_joint_speed/last_used_joint_acc will not be modified  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code >= 0: the last_used_angles/last_used_joint_speed/last_used_joint_acc will be modified


#### def __set_servo_angle_j__(self, angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):

> Set the servo angle, execute only the last instruction, need to be set to servo motion mode(self.set_mode(1))  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc  
> &ensp;&ensp;&ensp;&ensp;2. This interface is only used in the base coordinate system.  
>   
> :param angles: angle list, (unit: rad if is_radian is True else °)  
> :param speed: speed, reserved  
> :param mvacc: acceleration, reserved  
> :param mvtime: 0, reserved  
> :param is_radian: the angles in radians or not, default is self.default_is_radian  
> :param kwargs: reserved  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_servo_attach__(self, servo_id=None):

> Attach the servo  
>   
> :param servo_id: 1-(Number of axes), 8, if servo_id is 8, will attach all servo  
> &ensp;&ensp;&ensp;&ensp;1. 1-(Number of axes): attach only one joint  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: arm.set_servo_attach(servo_id=1)  
> &ensp;&ensp;&ensp;&ensp;2: 8: attach all joints  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: arm.set_servo_attach(servo_id=8)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_servo_cartesian__(self, mvpose, speed=None, mvacc=None, mvtime=0, is_radian=None, is_tool_coord=False, **kwargs):

> Set the servo cartesian, execute only the last instruction, need to be set to servo motion mode(self.set_mode(1))  
>   
> :param mvpose: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]  
> :param speed: move speed (mm/s), reserved  
> :param mvacc: move acceleration (mm/s^2), reserved  
> :param mvtime: 0, reserved  
> :param is_radian: the roll/pitch/yaw of mvpose in radians or not, default is self.default_is_radian  
> :param is_tool_coord: is tool coordinate or not  
> :param kwargs: reserved  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_servo_cartesian_aa__(self, axis_angle_pose, speed=None, mvacc=None, is_radian=None, is_tool_coord=False, relative=False, **kwargs):

> Set the servo cartesian represented by the axis angle pose, execute only the last instruction, need to be set to servo motion mode(self.set_mode(1))  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.4.7  
>   
> :param axis_angle_pose: the axis angle pose, [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]  
> :param speed: move speed (mm/s), reserved  
> :param mvacc: move acceleration (mm/s^2), reserved  
> :param is_radian: the rx/ry/rz of axis_angle_pose in radians or not, default is self.default_is_radian  
> :param is_tool_coord: is tool coordinate or not  
> :param relative: relative move or not  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_servo_detach__(self, servo_id=None):

> Detach the servo, be sure to do protective work before unlocking to avoid injury or damage.  
>   
> :param servo_id: 1-(Number of axes), 8, if servo_id is 8, will detach all servo  
> &ensp;&ensp;&ensp;&ensp;1. 1-(Number of axes): detach only one joint  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: arm.set_servo_detach(servo_id=1)  
> &ensp;&ensp;&ensp;&ensp;2: 8: detach all joints, please  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: arm.set_servo_detach(servo_id=8)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_simulation_robot__(self, on_off):

> Set the simulation robot  
>   
> :param on_off: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_state__(self, state=0):

> Set the xArm state  
>   
> :param state: default is 0  
> &ensp;&ensp;&ensp;&ensp;0: motion state  
> &ensp;&ensp;&ensp;&ensp;3: pause state  
> &ensp;&ensp;&ensp;&ensp;4: stop state  
> &ensp;&ensp;&ensp;&ensp;6: deceleration stop state  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_tcp_jerk__(self, jerk):

> Set the translational jerk of Cartesian space  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Do not use if not required  
> &ensp;&ensp;&ensp;&ensp;2. If not saved, it will be lost after reboot  
> &ensp;&ensp;&ensp;&ensp;3. The save_conf interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;4. The clean_conf interface can restore system default settings  
>   
> :param jerk: jerk (mm/s^3)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_tcp_load__(self, weight, center_of_gravity, wait=False, **kwargs):

> Set the end load of xArm  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Do not use if not required  
> &ensp;&ensp;&ensp;&ensp;2. If not saved, it will be lost after reboot  
> &ensp;&ensp;&ensp;&ensp;3. The save_conf interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;4. The clean_conf interface can restore system default settings  
>   
> :param weight: load weight (unit: kg)  
> :param center_of_gravity: load center of gravity, such as [x(mm), y(mm), z(mm)]  
> :param wait: whether to wait for the command to be executed or the the robotic arm to stop  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_tcp_maxacc__(self, acc):

> Set the max translational acceleration of Cartesian space  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Do not use if not required  
> &ensp;&ensp;&ensp;&ensp;2. If not saved, it will be lost after reboot  
> &ensp;&ensp;&ensp;&ensp;3. The save_conf interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;4. The clean_conf interface can restore system default settings  
>   
> :param acc: max acceleration (mm/s^2)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_tcp_offset__(self, offset, is_radian=None, wait=True, **kwargs):

> Set the tool coordinate system offset at the end  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Do not use if not required  
> &ensp;&ensp;&ensp;&ensp;2. If not saved and you want to revert to the last saved value, please reset the offset by set_tcp_offset([0, 0, 0, 0, 0, 0])  
> &ensp;&ensp;&ensp;&ensp;3. If not saved, it will be lost after reboot  
> &ensp;&ensp;&ensp;&ensp;4. The save_conf interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;5. The clean_conf interface can restore system default settings  
>   
> :param offset: [x, y, z, roll, pitch, yaw]  
> :param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian  
> :param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_teach_sensitivity__(self, value, wait=True):

> Set the sensitivity of drag and teach  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Do not use if not required  
> &ensp;&ensp;&ensp;&ensp;2. If not saved, it will be lost after reboot  
> &ensp;&ensp;&ensp;&ensp;3. The save_conf interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;4. The clean_conf interface can restore system default settings  
>   
> :param value: sensitivity value, 1~5  
> :param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_tgpio_digital__(self, ionum, value, delay_sec=None, sync=True):

> Set the digital value of the specified Tool GPIO  
>   
> :param ionum: 0 or 1  
> :param value: value  
> :param delay_sec: delay effective time from the current start, in seconds, default is None(effective immediately)  
> :param sync: whether to execute in the motion queue, set to False to execute immediately(default is True)  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
> &ensp;&ensp;&ensp;&ensp;2. only available if delay_sec <= 0  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_tgpio_digital_with_xyz__(self, ionum, value, xyz, fault_tolerance_radius):

> Set the digital value of the specified Tool GPIO when the robot has reached the specified xyz position             
>   
> :param ionum: 0 or 1  
> :param value: value  
> :param xyz: position xyz, as [x, y, z]  
> :param fault_tolerance_radius: fault tolerance radius  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_tgpio_modbus_baudrate__(self, baud):

> Set the modbus baudrate of the tool gpio  
>   
> :param baud: 4800/9600/19200/38400/57600/115200/230400/460800/921600/1000000/1500000/2000000/2500000  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_tgpio_modbus_timeout__(self, timeout, is_transparent_transmission=False, **kwargs):

> Set the modbus timeout of the tool gpio  
>   
> :param timeout: timeout, milliseconds  
> :param is_transparent_transmission: whether the set timeout is the timeout of transparent transmission  
> &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.11.0  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_tgpio_modbus_use_503_port__(self, use_503_port=True):



#### def __set_timeout__(self, timeout):

> Set the timeout of cmd response  
>   
> :param timeout: seconds


#### def __set_tool_position__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, radius=None, **kwargs):

> Movement relative to the tool coordinate system  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface is moving relative to the current tool coordinate system  
> &ensp;&ensp;&ensp;&ensp;2. The tool coordinate system is not fixed and varies with position.  
> &ensp;&ensp;&ensp;&ensp;3. This interface is only used in the tool coordinate system.  
>   
>   
> :param x: the x coordinate relative to the current tool coordinate system, (unit: mm), default is 0  
> :param y: the y coordinate relative to the current tool coordinate system, (unit: mm), default is 0  
> :param z: the z coordinate relative to the current tool coordinate system, (unit: mm), default is 0  
> :param roll: the rotate around the X axis relative to the current tool coordinate system, (unit: rad if is_radian is True else °), default is 0  
> :param pitch: the rotate around the Y axis relative to the current tool coordinate system, (unit: rad if is_radian is True else °), default is 0  
> :param yaw: the rotate around the Z axis relative to the current tool coordinate system, (unit: rad if is_radian is True else °), default is 0  
> :param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed  
> :param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc  
> :param mvtime: 0, reserved  
> :param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian  
> :param wait: whether to wait for the arm to complete, default is False  
> :param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True  
> :param radius: move radius, if radius is None or radius less than 0, will MoveToolLine, else MoveToolArcLine  
> &ensp;&ensp;&ensp;&ensp;only available if firmware_version >= 1.11.100  
> &ensp;&ensp;&ensp;&ensp;MoveToolLine: Linear motion  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_tool_position(..., radius=None)  
> &ensp;&ensp;&ensp;&ensp;MoveToolArcLine: Linear arc motion with interpolation  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code = arm.set_tool_position(..., radius=0)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: Need to set radius>=0  
> :param kwargs: extra parameters  
> &ensp;&ensp;&ensp;&ensp;:param motion_type: motion planning type, default is 0  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;motion_type == 0: default, linear planning  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;motion_type == 1: prioritize linear planning, and turn to IK for joint planning when linear planning is not possible  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;motion_type == 2: direct transfer to IK using joint planning  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.11.100  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. when motion_type is 1 or 2, linear motion cannot be guaranteed  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. once IK is transferred to joint planning, the given Cartesian velocity and acceleration are converted into joint velocity and acceleration according to the percentage  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;speed = speed / max_tcp_speed * max_joint_speed  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;mvacc = mvacc / max_tcp_acc * max_joint_acc  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4. if there is no suitable IK, a C40 error will be triggered  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code < 0: the last_used_tcp_speed/last_used_tcp_acc will not be modified  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;code >= 0: the last_used_tcp_speed/last_used_tcp_acc will be modified


#### def __set_vacuum_gripper__(self, on, wait=False, timeout=3, delay_sec=None, sync=True, hardware_version=1):

> Set the Vacuum Gripper ON/OFF  
>   
> :param on: open or not  
> &ensp;&ensp;&ensp;&ensp;on=True: equivalent to calling `set_tgpio_digital(0, 1)` and `set_tgpio_digital(1, 0)`  
> &ensp;&ensp;&ensp;&ensp;on=False: equivalent to calling `set_tgpio_digital(0, 0)` and `set_tgpio_digital(1, 1)`  
> :param wait: wait the object picked by the vacuum gripper or not, default is False  
> :param timeout: wait time, unit:second, default is 3s  
> :param delay_sec: delay effective time from the current start, in seconds, default is None(effective immediately)  
> :param sync: whether to execute in the motion queue, set to False to execute immediately(default is True)  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
> &ensp;&ensp;&ensp;&ensp;2. only available if delay_sec <= 0  
> :param hardware_version: hardware version  
> &ensp;&ensp;&ensp;&ensp;1: Plug-in Connection, default  
> &ensp;&ensp;&ensp;&ensp;2: Contact Connection  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __set_world_offset__(self, offset, is_radian=None, wait=True):

> Set the base coordinate offset  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.11 or above  
>   
> :param offset: [x, y, z, roll, pitch, yaw]  
> :param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian  
> :param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __start_record_trajectory__(self):

> Start trajectory recording, only in teach mode, so you need to set joint teaching mode before.  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
> &ensp;&ensp;&ensp;&ensp;2. set joint teaching mode: set_mode(2);set_state(0)  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __stop_lite6_gripper__(self, sync=True):

> Stop the gripper of Lite6 series robotic arms  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.10.0  
> &ensp;&ensp;&ensp;&ensp;2. this API can only be used on Lite6 series robotic arms  
>   
> :param sync: whether to execute in the motion queue, set to False to execute immediately(default is True)  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.4.101  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __stop_record_trajectory__(self, filename=None, **kwargs):

> Stop trajectory recording  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
>   
> :param filename: The name to save  
> &ensp;&ensp;&ensp;&ensp;1. Only strings consisting of English or numbers are supported, and the length is no more than 50.  
> &ensp;&ensp;&ensp;&ensp;2. The trajectory is saved in the controller box.  
> &ensp;&ensp;&ensp;&ensp;3. If the filename is None, just stop recording, do not save, you need to manually call `save_record_trajectory` save before changing the mode. otherwise it will be lost  
> &ensp;&ensp;&ensp;&ensp;4. This action will overwrite the trajectory with the same name  
> &ensp;&ensp;&ensp;&ensp;5. Empty the trajectory in memory after saving  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __system_control__(self, value=1):

> Control the xArm controller system  
>   
> :param value: 1: shutdown, 2: reboot  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __vc_set_cartesian_velocity__(self, speeds, is_radian=None, is_tool_coord=False, duration=-1, **kwargs):

> Cartesian velocity control, need to be set to cartesian velocity control mode(self.set_mode(5))  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.6.9  
> &ensp;&ensp;&ensp;&ensp;  
> :param speeds: [spd_x, spd_y, spd_z, spd_rx, spd_ry, spd_rz]  
> :param is_radian: the spd_rx/spd_ry/spd_rz in radians or not, default is self.default_is_radian  
> :param is_tool_coord: is tool coordinate or not, default is False  
> :param duration: the maximum duration of the speed, over this time will automatically set the speed to 0  
> &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0  
> &ensp;&ensp;&ensp;&ensp;duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained  
> &ensp;&ensp;&ensp;&ensp;duration == 0: Always effective, will not stop automatically  
> &ensp;&ensp;&ensp;&ensp;duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __vc_set_joint_velocity__(self, speeds, is_radian=None, is_sync=True, duration=-1, **kwargs):

> Joint velocity control, need to be set to joint velocity control mode(self.set_mode(4))  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.6.9  
>   
> :param speeds: [spd_J1, spd_J2, ..., spd_J7]  
> :param is_radian: the spd_Jx in radians or not, default is self.default_is_radian  
> :param is_sync: whether all joints accelerate and decelerate synchronously, default is True  
> :param duration: The duration of this speed command, over this time will automatically set the speed to 0  
> &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.8.0  
> &ensp;&ensp;&ensp;&ensp;duration > 0: seconds  
> &ensp;&ensp;&ensp;&ensp;duration == 0: Always effective, will not stop automatically  
> &ensp;&ensp;&ensp;&ensp;duration < 0: default value, only used to be compatible with the old protocol, equivalent to 0  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.


#### def __write_and_read_holding_registers__(self, r_addr, r_quantity, w_addr, w_regs, is_signed=False):

> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Write and Read Holding Registers (0x17)  
>   
> :param r_addr: the starting address of the register to be read  
> :param r_quantity: number of registers to read  
> :param w_addr: the starting address of the register to be written  
> :param w_regs: array of values to write  
> :param is_signed: whether to convert the read register value into a signed form  
> :return: tuple((code, regs)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)


#### def __write_multiple_coil_bits__(self, addr, bits):

> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Write Multiple Coils (0x0F)  
>   
> :param addr: the starting address of the register to be written  
> :param bits: array of values to write  
> :return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)


#### def __write_multiple_holding_registers__(self, addr, regs):

> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Write Multiple Holding Registers (0x10)  
>   
> :param addr: the starting address of the register to be written  
> :param regs: array of values to write  
> :return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)


#### def __write_single_coil_bit__(self, addr, bit_val):

> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Write Single Coil (0x05)  
>   
> :param addr: register address  
> :param bit_val: the value to write (0/1)  
> :return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)


#### def __write_single_holding_register__(self, addr, reg_val):

> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Write Single Holding Register (0x06)  
>   
> :param addr: register address  
> :param bit_val: the value to write  
> :return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)
