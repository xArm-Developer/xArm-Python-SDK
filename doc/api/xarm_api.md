xArm-Python-SDK API Documentation (V1.13.16): class XArmAPI in module xarm.wrapper.xarm_api

## class __XArmAPI__
****************************************

>   

****************************************
### __Methods__
****************************************
#### __init__
__\__init__\__ = <function XArmAPI.__init__>
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




#### calibrate_tcp_coordinate_offset
__calibrate_tcp_coordinate_offset__ = <function XArmAPI.calibrate_tcp_coordinate_offset>
> Four-point method to calibrate tool coordinate system position offset  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.6.9  
>   
> :param four_points: a list of four teaching coordinate positions [x, y, z, roll, pitch, yaw]  
> :param is_radian: the roll/pitch/yaw value of the each point in radians or not, default is self.default_is_radian  
> :return: tuple((code, xyz_offset)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;xyz_offset: calculated xyz(mm) TCP offset, [x, y, z]  




#### calibrate_tcp_orientation_offset
__calibrate_tcp_orientation_offset__ = <function XArmAPI.calibrate_tcp_orientation_offset>
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




#### calibrate_user_coordinate_offset
__calibrate_user_coordinate_offset__ = <function XArmAPI.calibrate_user_coordinate_offset>
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




#### calibrate_user_orientation_offset
__calibrate_user_orientation_offset__ = <function XArmAPI.calibrate_user_orientation_offset>
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




#### check_verification
__check_verification__ = <function XArmAPI.check_verification>
> check verification  
>   
> :return: tuple((code, status)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status:  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: verified  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;other: not verified  




#### clean_bio_gripper_error
__clean_bio_gripper_error__ = <function XArmAPI.clean_bio_gripper_error>
> Clean the error code of the bio gripper  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### clean_conf
__clean_conf__ = <function XArmAPI.clean_conf>
> Clean current config and restore system default settings  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface will clear the current settings and restore to the original settings (system default settings)  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### clean_error
__clean_error__ = <function XArmAPI.clean_error>
> Clean the error, need to be manually enabled motion(arm.motion_enable(True)) and set state(arm.set_state(state=0))after clean error  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### clean_gripper_error
__clean_gripper_error__ = <function XArmAPI.clean_gripper_error>
> Clean the gripper error  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### clean_linear_track_error
__clean_linear_track_error__ = <function XArmAPI.clean_linear_track_error>
> Clean the linear track error  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### clean_warn
__clean_warn__ = <function XArmAPI.clean_warn>
> Clean the warn  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### close_bio_gripper
__close_bio_gripper__ = <function XArmAPI.close_bio_gripper>
> Close the bio gripper  
>   
> :param speed: speed value, default is 0 (not set the speed)  
> :param wait: whether to wait for the bio gripper motion complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 5, only available if wait=True  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### close_lite6_gripper
__close_lite6_gripper__ = <function XArmAPI.close_lite6_gripper>
> Close the gripper of Lite6 series robotic arms  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.10.0  
> &ensp;&ensp;&ensp;&ensp;2. this API can only be used on Lite6 series robotic arms  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### config_cgpio_reset_when_stop
__config_cgpio_reset_when_stop__ = <function XArmAPI.config_cgpio_reset_when_stop>
> Config the Controller GPIO reset the digital output when the robot is in stop state  
>   
> :param on_off: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### config_force_control
__config_force_control__ = <function XArmAPI.config_force_control>
> Set force control parameters through the Six-axis Force Torque Sensor.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :param coord:  task frame. 0: base frame. 1: tool frame.  
> :param c_axis: a 6d vector of 0s and 1s. 1 means that robot will be compliant in the corresponding axis of the task frame.  
> :param f_ref:  the forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque.  
> :param limits:  for compliant axes, these values are the maximum allowed tcp speed along/about the axis.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### config_tgpio_reset_when_stop
__config_tgpio_reset_when_stop__ = <function XArmAPI.config_tgpio_reset_when_stop>
> Config the Tool GPIO reset the digital output when the robot is in stop state  
>   
> :param on_off: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### connect
__connect__ = <function XArmAPI.connect>
> Connect to xArm  
>   
> :param port: port name or the ip address, default is the value when initializing an instance  
> :param baudrate: baudrate, only available in serial way, default is the value when initializing an instance  
> :param timeout: timeout, only available in serial way, default is the value when initializing an instance  
> :param axis: number of axes, required only when using a serial port connection, default is 7  




#### delete_blockly_app
__delete_blockly_app__ = <function XArmAPI.delete_blockly_app>
> Delete blockly app  
>   
> :param name: blockly app name  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### delete_trajectory
__delete_trajectory__ = <function XArmAPI.delete_trajectory>
> Delete trajectory  
>   
> :param name: trajectory name  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### disconnect
__disconnect__ = <function XArmAPI.disconnect>
> Disconnect  




#### emergency_stop
__emergency_stop__ = <function XArmAPI.emergency_stop>
> Emergency stop (set_state(4) -> motion_enable(True) -> set_state(0))  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface does not automatically clear the error. If there is an error, you need to handle it according to the error code.  




#### ft_sensor_app_get
__ft_sensor_app_get__ = <function XArmAPI.ft_sensor_app_get>
> Get force mode  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :return: tuple((code, app_code))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;app_code:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: non-force mode  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: impedance control mode  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2: force control mode  




#### ft_sensor_app_set
__ft_sensor_app_set__ = <function XArmAPI.ft_sensor_app_set>
> Set robot to be controlled in force mode. (Through the Six-axis Force Torque Sensor)  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :param app_code: force mode.  
> &ensp;&ensp;&ensp;&ensp;0: non-force mode  
> &ensp;&ensp;&ensp;&ensp;1: impendance control  
> &ensp;&ensp;&ensp;&ensp;2: force control  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### ft_sensor_cali_load
__ft_sensor_cali_load__ = <function XArmAPI.ft_sensor_cali_load>
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




#### ft_sensor_enable
__ft_sensor_enable__ = <function XArmAPI.ft_sensor_enable>
> Used for enabling and disabling the use of the Six-axis Force Torque Sensor measurements in the controller.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :param on_off: enable or disable F/T data sampling.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### ft_sensor_iden_load
__ft_sensor_iden_load__ = <function XArmAPI.ft_sensor_iden_load>
> Identification the tcp load with the the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
> &ensp;&ensp;&ensp;&ensp;3. starting from SDK v1.11.0, the centroid unit is millimeters (originally meters)  
>   
> :return: tuple((code, load)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;load:  [mass(kg), x_centroid(mm), y_centroid(mm), z_centroid(mm), Fx_offset, Fy_offset, Fz_offset, Tx_offset, Ty_offset, Tz_ffset]  




#### ft_sensor_set_zero
__ft_sensor_set_zero__ = <function XArmAPI.ft_sensor_set_zero>
> Set the current state to the zero point of the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_allow_approx_motion
__get_allow_approx_motion__ = <function XArmAPI.get_allow_approx_motion>
> Obtain whether to enable approximate solutions to avoid certain singularities  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.9.0  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_base_board_version
__get_base_board_version__ = <function XArmAPI.get_base_board_version>
> &ensp;Get base board version  
>   
> :param board_id: int  
> :return: : (code, version)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_bio_gripper_error
__get_bio_gripper_error__ = <function XArmAPI.get_bio_gripper_error>
> Get the error code of the bio gripper  
>   
> :return: tuple((code, error_code))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;error_code: See the [Bio Gripper Error Code Documentation](./xarm_api_code.md#bio-gripper-error-code) for details.  




#### get_bio_gripper_status
__get_bio_gripper_status__ = <function XArmAPI.get_bio_gripper_status>
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




#### get_cgpio_analog
__get_cgpio_analog__ = <function XArmAPI.get_cgpio_analog>
> Get the analog value of the specified Controller GPIO  
> :param ionum: 0 or 1 or None(both 0 and 1), default is None  
> :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_cgpio_digital
__get_cgpio_digital__ = <function XArmAPI.get_cgpio_digital>
> Get the digital value of the specified Controller GPIO  
>   
> :param ionum: 0~15 or None(both 0~15), default is None  
> :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_cgpio_state
__get_cgpio_state__ = <function XArmAPI.get_cgpio_state>
> Get the state of the Controller GPIO  
> :return: code, states  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;states: [...]  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[0]: contorller gpio module state  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[0] == 0: normal  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[0] == 1：wrong  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[0] == 6：communication failure  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[1]: controller gpio module error code  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[1] == 0: normal  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;states[1] != 0：error code  
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




#### get_checkset_default_baud
__get_checkset_default_baud__ = <function XArmAPI.get_checkset_default_baud>
> Get the checkset baud value  
>   
> :param type_: checkset type  
> &ensp;&ensp;&ensp;&ensp;1: xarm gripper  
> &ensp;&ensp;&ensp;&ensp;2: bio gripper  
> &ensp;&ensp;&ensp;&ensp;3: robotiq gripper  
> &ensp;&ensp;&ensp;&ensp;4: linear track  
> :return: tuple((code, baud))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;baud: the checkset baud value  




#### get_cmdnum
__get_cmdnum__ = <function XArmAPI.get_cmdnum>
> Get the cmd count in cache  
> :return: tuple((code, cmd_num)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_dh_params
__get_dh_params__ = <function XArmAPI.get_dh_params>
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




#### get_err_warn_code
__get_err_warn_code__ = <function XArmAPI.get_err_warn_code>
> Get the controller error and warn code  
>   
> :param show: show the detail info if True  
> :param lang: show language, en/cn, degault is en, only available if show is True  
> :return: tuple((code, [error_code, warn_code])), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;error_code: See the [Controller Error Code Documentation](./xarm_api_code.md#controller-error-code) for details.  
> &ensp;&ensp;&ensp;&ensp;warn_code: See the [Controller Error Code Documentation](./xarm_api_code.md#controller-warn-code) for details.  




#### get_forward_kinematics
__get_forward_kinematics__ = <function XArmAPI.get_forward_kinematics>
> Get forward kinematics  
>   
> :param angles: [angle-1, angle-2, ..., angle-n], n is the number of axes of the arm  
> :param input_is_radian: the param angles value is in radians or not, default is self.default_is_radian  
> :param return_is_radian: the returned value is in radians or not, default is self.default_is_radian  
> :return: tuple((code, pose)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;pose: [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)] or []  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: the roll/pitch/yaw value is radians if return_is_radian is True, else °  




#### get_ft_sensor_config
__get_ft_sensor_config__ = <function XArmAPI.get_ft_sensor_config>
> Get the config of the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
> &ensp;&ensp;&ensp;&ensp;  
> :return: tuple((code, config))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;config: [...], the config of the Six-axis Force Torque Sensor, only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[0] ft_app_status: force mode  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: non-force mode  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: impendance control  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2: force control  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[1] ft_is_started: ft sensor is enable or not  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[2] ft_type: ft sensor type  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[3] ft_id: ft sensor id  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[4] ft_freq: ft sensor frequency  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[5] ft_mass: load mass  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[6] ft_dir_bias: reversed  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[7] ft_centroid: [x_centroid，y_centroid，z_centroid]  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[8] ft_zero: [Fx_offset，Fy_offset，Fz_offset，Tx_offset，Ty_offset，Tz_ffset]  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[9] imp_coord: task frame of impendance control mode.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: base frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: tool frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[10] imp_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[11] M: mass. (kg)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[12] K: stiffness coefficient.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[13] B: damping coefficient. invalid.   Note: the value is set to 2*sqrt(M*K) in controller.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[14] f_coord: task frame of force control mode.   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: base frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: tool frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[15] f_c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[16] f_ref:  the forces/torques the robot will apply to its environment. The robot adjusts its position along/about compliant axis in order to achieve the specified force/torque.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[17] f_limits: reversed.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[18] kp: proportional gain  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[19] ki: integral gain.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[20] kd: differential gain.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;[21] xe_limit: 6d vector. for compliant axes, these values are the maximum allowed tcp speed along/about the axis. mm/s  




#### get_ft_sensor_data
__get_ft_sensor_data__ = <function XArmAPI.get_ft_sensor_data>
> Get the data of the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :return: tuple((code, exe_ft))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;ft_data: only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: The external force detection value of the Six-axis Force Torque Sensor after filtering, load and offset compensation  




#### get_ft_sensor_error
__get_ft_sensor_error__ = <function XArmAPI.get_ft_sensor_error>
> Get the error code of the Six-axis Force Torque Sensor  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :return: tuple((code, error))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;error: See the [Six-axis Force Torque Sensor Error Code Documentation](./xarm_api_code.md#six-axis-force-torque-sensor-error-code) for details.  




#### get_gripper_err_code
__get_gripper_err_code__ = <function XArmAPI.get_gripper_err_code>
> Get the gripper error code  
>   
> :return: tuple((code, err_code)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;err_code: See the [Gripper Error Code Documentation](./xarm_api_code.md#gripper-error-code) for details.  




#### get_gripper_position
__get_gripper_position__ = <function XArmAPI.get_gripper_position>
> Get the gripper position  
>   
> :return: tuple((code, pos)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_gripper_version
__get_gripper_version__ = <function XArmAPI.get_gripper_version>
> Get gripper version, only for debug  
>   
> :return: (code, version)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_harmonic_type
__get_harmonic_type__ = <function XArmAPI.get_harmonic_type>
> Get harmonic type, only for debug  
>   
> :return: (code, type)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_hd_types
__get_hd_types__ = <function XArmAPI.get_hd_types>
> Get harmonic types, only for debug  
>   
> :return: (code, types)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_initial_point
__get_initial_point__ = <function XArmAPI.get_initial_point>
> Get the initial point from studio  
>   
> :return: tuple((code, point)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;point: initial point, [J1, J2, ..., J7]  




#### get_inverse_kinematics
__get_inverse_kinematics__ = <function XArmAPI.get_inverse_kinematics>
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




#### get_is_moving
__get_is_moving__ = <function XArmAPI.get_is_moving>
> Check xArm is moving or not  
> :return: True/False  




#### get_joint_states
__get_joint_states__ = <function XArmAPI.get_joint_states>
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




#### get_joints_torque
__get_joints_torque__ = <function XArmAPI.get_joints_torque>
> Get joints torque  
>   
> :return: tuple((code, joints_torque))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;joints_torque: joints torque  




#### get_linear_track_error
__get_linear_track_error__ = <function XArmAPI.get_linear_track_error>
> Get the error code of the linear track  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, error)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;error: See the [Linear Motor Error Code Documentation](./xarm_api_code.md#linear-motor-error-code) for details.  




#### get_linear_track_is_enabled
__get_linear_track_is_enabled__ = <function XArmAPI.get_linear_track_is_enabled>
> Get the linear track is enabled or not  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, status)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: linear track is not enabled  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: linear track is enabled  




#### get_linear_track_on_zero
__get_linear_track_on_zero__ = <function XArmAPI.get_linear_track_on_zero>
> Get the linear track is on zero positon or not  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, status)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: linear track is not on zero  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: linear track is on zero  




#### get_linear_track_pos
__get_linear_track_pos__ = <function XArmAPI.get_linear_track_pos>
> Get the pos of the linear track  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, position)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;position: position  




#### get_linear_track_registers
__get_linear_track_registers__ = <function XArmAPI.get_linear_track_registers>
> Get the status of the linear track  
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




#### get_linear_track_sci
__get_linear_track_sci__ = <function XArmAPI.get_linear_track_sci>
> Get the sci1 value of the linear track  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, sci1)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_linear_track_sco
__get_linear_track_sco__ = <function XArmAPI.get_linear_track_sco>
> Get the sco value of the linear track  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, sco)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;sco: [sco0, sco1]  




#### get_linear_track_status
__get_linear_track_status__ = <function XArmAPI.get_linear_track_status>
> Get the status of the linear track  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: tuple((code, status)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;status: status  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x00: motion finish  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x01: in motion  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;status & 0x02: has stop  




#### get_mount_direction
__get_mount_direction__ = <function XArmAPI.get_mount_direction>
> Get the mount degrees from studio  
>   
> :return: tuple((code, degrees)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;degrees: mount degrees, [tilt angle, rotate angle]  




#### get_pose_offset
__get_pose_offset__ = <function XArmAPI.get_pose_offset>
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




#### get_position
__get_position__ = <function XArmAPI.get_position>
> Get the cartesian position  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. If the value(roll/pitch/yaw) you want to return is an radian unit, please set the parameter is_radian to True  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: code, pos = arm.get_position(is_radian=True)  
>   
> :param is_radian: the returned value (only roll/pitch/yaw) is in radians or not, default is self.default_is_radian  
> :return: tuple((code, [x, y, z, roll, pitch, yaw])), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_position_aa
__get_position_aa__ = <function XArmAPI.get_position_aa>
> Get the pose represented by the axis angle pose  
>   
> :param is_radian: the returned value (only rx/ry/rz) is in radians or not, default is self.default_is_radian  
> :return: tuple((code, [x, y, z, rx, ry, rz])), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_reduced_mode
__get_reduced_mode__ = <function XArmAPI.get_reduced_mode>
> Get reduced mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
>   
> :return: tuple((code, mode))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;mode: 0 or 1, 1 means that the reduced mode is turned on. 0 means that the reduced mode is not turned on  




#### get_reduced_states
__get_reduced_states__ = <function XArmAPI.get_reduced_states>
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




#### get_report_tau_or_i
__get_report_tau_or_i__ = <function XArmAPI.get_report_tau_or_i>
> Get the reported torque or electric current  
>   
> :return: tuple((code, tau_or_i))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;tau_or_i:   
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: torque  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: electric current  




#### get_robot_sn
__get_robot_sn__ = <function XArmAPI.get_robot_sn>
> Get the xArm sn  
>   
> :return: tuple((code, sn)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_safe_level
__get_safe_level__ = <function XArmAPI.get_safe_level>
> Get safe level  
>   
> :return: tuple((code, safe_level))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;safe_level: safe level  




#### get_servo_angle
__get_servo_angle__ = <function XArmAPI.get_servo_angle>
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




#### get_servo_debug_msg
__get_servo_debug_msg__ = <function XArmAPI.get_servo_debug_msg>
> Get the servo debug msg, used only for debugging  
>   
> :param show: show the detail info if True  
> :param lang: language, en/cn, default is en  
> :return: tuple((code, servo_info_list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_servo_version
__get_servo_version__ = <function XArmAPI.get_servo_version>
> Get servo version, only for debug  
>   
> :param servo_id: servo id(1~7)  
> :return: (code, version)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_state
__get_state__ = <function XArmAPI.get_state>
> Get state  
>   
> :return: tuple((code, state)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;state:  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: in motion  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2: sleeping  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3: suspended  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4: stopping  




#### get_tgpio_analog
__get_tgpio_analog__ = <function XArmAPI.get_tgpio_analog>
> Get the analog value of the specified Tool GPIO  
> :param ionum: 0 or 1 or None(both 0 and 1), default is None  
> :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_tgpio_digital
__get_tgpio_digital__ = <function XArmAPI.get_tgpio_digital>
> Get the digital value of the specified Tool GPIO  
>   
> :param ionum: 0 or 1 or None(both 0 and 1), default is None  
> :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_tgpio_modbus_baudrate
__get_tgpio_modbus_baudrate__ = <function XArmAPI.get_tgpio_modbus_baudrate>
> Get the modbus baudrate of the tool gpio  
>   
> :return: tuple((code, baudrate)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;baudrate: the modbus baudrate of the tool gpio  




#### get_tgpio_output_digital
__get_tgpio_output_digital__ = <function XArmAPI.get_tgpio_output_digital>
> Get the digital value of the specified Tool GPIO output  
>   
> :param ionum: 0 or 1 or 2 or 3 or 4 or None(both 0 and 1 and 2 and 3 and 4), default is None  
> :return: tuple((code, value or value list)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_tgpio_version
__get_tgpio_version__ = <function XArmAPI.get_tgpio_version>
> Get tool gpio version, only for debug  
>   
> :return: (code, version)  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### get_trajectories
__get_trajectories__ = <function XArmAPI.get_trajectories>
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




#### get_trajectory_rw_status
__get_trajectory_rw_status__ = <function XArmAPI.get_trajectory_rw_status>
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




#### get_vacuum_gripper
__get_vacuum_gripper__ = <function XArmAPI.get_vacuum_gripper>
> Get vacuum gripper state  
>   
> :return: tuple((code, state)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;state: suction cup state  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;0: suction cup is off  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1: suction cup is on  




#### get_version
__get_version__ = <function XArmAPI.get_version>
> Get the xArm firmware version  
>   
> :return: tuple((code, version)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### getset_tgpio_modbus_data
__getset_tgpio_modbus_data__ = <function XArmAPI.getset_tgpio_modbus_data>
> Send the modbus data to the tool gpio  
>   
> :param datas: data_list  
> :param min_res_len: the minimum length of modbus response data. Used to check the data length, if not specified, no check  
> :param host_id: host_id, default is 9 (TGPIO_HOST_ID)  
> &ensp;&ensp;&ensp;&ensp;9: END RS485  
> &ensp;&ensp;&ensp;&ensp;10: CONTROLLER RS485  
> :param is_transparent_transmission: whether to choose transparent transmission, default is False  
> &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.11.0  
> :param use_503_port: whether to use port 503 for communication, default is False  
> &ensp;&ensp;&ensp;&ensp;Note: if it is True, it will connect to 503 port for communication when it is used for the first time, which is generally only useful for transparent transmission.  
> &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.11.0  
>   
> :return: tuple((code, modbus_response))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;modbus_response: modbus response data  




#### iden_joint_friction
__iden_joint_friction__ = <function XArmAPI.iden_joint_friction>
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




#### iden_tcp_load
__iden_tcp_load__ = <function XArmAPI.iden_tcp_load>
> Identification the tcp load with current  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :param estimated_mass: estimated mass  
> &ensp;&ensp;&ensp;&ensp;Note: this parameter is only available on the lite6 model manipulator, and this parameter must be specified for the lite6 model manipulator  
> :return: tuple((code, load)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;load:  [mass，x_centroid，y_centroid，z_centroid]  




#### is_joint_limit
__is_joint_limit__ = <function XArmAPI.is_joint_limit>
> Check the joint angle is in limit  
>   
> :param joint: [angle-1, angle-2, ..., angle-n], n is the number of axes of the arm  
> :param is_radian: angle value is radians or not, default is self.default_is_radian  
> :return: tuple((code, limit)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;limit: True/False/None, limit or not, or failed  




#### is_tcp_limit
__is_tcp_limit__ = <function XArmAPI.is_tcp_limit>
> Check the tcp pose is in limit  
>   
> :param pose: [x, y, z, roll, pitch, yaw]  
> :param is_radian: roll/pitch/yaw value is radians or not, default is self.default_is_radian  
> :return: tuple((code, limit)), only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;limit: True/False/None, limit or not, or failed  




#### load_trajectory
__load_trajectory__ = <function XArmAPI.load_trajectory>
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




#### mask_write_holding_register
__mask_write_holding_register__ = <function XArmAPI.mask_write_holding_register>
> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Mask Write Holding Register (0x16)  
>   
> :param addr: register address  
> :param and_mask: mask to be AND with  
> :param or_mask: mask to be OR with  
> :return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)  




#### motion_enable
__motion_enable__ = <function XArmAPI.motion_enable>
> Motion enable  
>   
> :param enable:True/False  
> :param servo_id: 1-(Number of axes), None(8)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### move_arc_lines
__move_arc_lines__ = <function XArmAPI.move_arc_lines>
> Continuous linear motion with interpolation.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. If an error occurs, it will return early.  
> &ensp;&ensp;&ensp;&ensp;2. If the emergency_stop interface is called actively, it will return early.  
> &ensp;&ensp;&ensp;&ensp;3. The last_used_position/last_used_tcp_speed/last_used_tcp_acc will be modified.  
> &ensp;&ensp;&ensp;&ensp;4. The last_used_angles/last_used_joint_speed/last_used_joint_acc will not be modified.  
>   
> :param paths: cartesian path list  
> &ensp;&ensp;&ensp;&ensp;1. Specify arc radius： [[x, y, z, roll, pitch, yaw, radius], ....]  
> &ensp;&ensp;&ensp;&ensp;2. Do not specify arc radius (radius=0)： [[x, y, z, roll, pitch, yaw], ....]  
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




#### move_circle
__move_circle__ = <function XArmAPI.move_circle>
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




#### move_gohome
__move_gohome__ = <function XArmAPI.move_gohome>
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




#### open_bio_gripper
__open_bio_gripper__ = <function XArmAPI.open_bio_gripper>
> Open the bio gripper  
>   
> :param speed: speed value, default is 0 (not set the speed)  
> :param wait: whether to wait for the bio gripper motion complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 5, only available if wait=True  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### open_lite6_gripper
__open_lite6_gripper__ = <function XArmAPI.open_lite6_gripper>
> Open the gripper of Lite6 series robotic arms  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.10.0  
> &ensp;&ensp;&ensp;&ensp;2. this API can only be used on Lite6 series robotic arms  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### playback_trajectory
__playback_trajectory__ = <function XArmAPI.playback_trajectory>
> Playback trajectory  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
>   
> :param times: Number of playbacks,  
> &ensp;&ensp;&ensp;&ensp;1. Only valid when the current position of the arm is the end position of the track, otherwise it will only be played once.  
> :param filename: The name of the trajectory to play back  
> &ensp;&ensp;&ensp;&ensp;1. If filename is None, you need to manually call the `load_trajectory` to load the trajectory.  
> :param wait: whether to wait for the arm to complete, default is False  
> :param double_speed: double speed, only support 1/2/4, default is 1, only available if version > 1.2.11  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### read_coil_bits
__read_coil_bits__ = <function XArmAPI.read_coil_bits>
> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Read Coils (0x01)  
>   
> :param addr: the starting address of the register to be read  
> :param quantity: number of registers  
> :return: tuple((code, bits)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)  




#### read_holding_registers
__read_holding_registers__ = <function XArmAPI.read_holding_registers>
> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Read Holding Registers (0x03)  
>   
> :param addr: the starting address of the register to be read  
> :param quantity: number of registers  
> :param is_signed: whether to convert the read register value into a signed form  
> :return: tuple((code, bits)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)  




#### read_input_bits
__read_input_bits__ = <function XArmAPI.read_input_bits>
> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Read Discrete Inputs (0x02)  
>   
> :param addr: the starting address of the register to be read  
> :param quantity: number of registers  
> :return: tuple((code, bits)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)  




#### read_input_registers
__read_input_registers__ = <function XArmAPI.read_input_registers>
> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Read Input Registers (0x04)  
>   
> :param addr: the starting address of the register to be read  
> :param quantity: number of registers  
> :param is_signed: whether to convert the read register value into a signed form  
> :return: tuple((code, bits)) only when code is 0, the returned result is correct.  
> &ensp;&ensp;&ensp;&ensp;code:  See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)  




#### register_cmdnum_changed_callback
__register_cmdnum_changed_callback__ = <function XArmAPI.register_cmdnum_changed_callback>
> Register the cmdnum changed callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"cmdnum": cmdnum  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False  




#### register_connect_changed_callback
__register_connect_changed_callback__ = <function XArmAPI.register_connect_changed_callback>
> Register the connect status changed callback  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"connected": connected,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"reported": reported,  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False  




#### register_count_changed_callback
__register_count_changed_callback__ = <function XArmAPI.register_count_changed_callback>
> Register the counter value changed callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"count": counter value  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False  




#### register_error_warn_changed_callback
__register_error_warn_changed_callback__ = <function XArmAPI.register_error_warn_changed_callback>
> Register the error code or warn code changed callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"error_code": error_code,  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"warn_code": warn_code,  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False  




#### register_feedback_callback
__register_feedback_callback__ = <function XArmAPI.register_feedback_callback>
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




#### register_iden_progress_changed_callback
__register_iden_progress_changed_callback__ = <function XArmAPI.register_iden_progress_changed_callback>
> Register the Identification progress value changed callback, only available if enable_report is True  
>   
> :param callback:   
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"progress": progress value  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False  




#### register_mode_changed_callback
__register_mode_changed_callback__ = <function XArmAPI.register_mode_changed_callback>
> Register the mode changed callback, only available if enable_report is True and the connect way is socket  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"mode": mode,  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False  




#### register_mtable_mtbrake_changed_callback
__register_mtable_mtbrake_changed_callback__ = <function XArmAPI.register_mtable_mtbrake_changed_callback>
> Register the motor enable states or motor brake states changed callback, only available if enable_report is True and the connect way is socket  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"mtable": [motor-1-motion-enable, motor-2-motion-enable, ...],  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"mtbrake": [motor-1-brake-enable, motor-1-brake-enable,...],  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False  




#### register_report_callback
__register_report_callback__ = <function XArmAPI.register_report_callback>
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




#### register_report_location_callback
__register_report_location_callback__ = <function XArmAPI.register_report_location_callback>
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




#### register_state_changed_callback
__register_state_changed_callback__ = <function XArmAPI.register_state_changed_callback>
> Register the state status changed callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"state": state,  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False  




#### register_temperature_changed_callback
__register_temperature_changed_callback__ = <function XArmAPI.register_temperature_changed_callback>
> Register the temperature changed callback, only available if enable_report is True  
>   
> :param callback:  
> &ensp;&ensp;&ensp;&ensp;callback data:  
> &ensp;&ensp;&ensp;&ensp;{  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;"temperatures": [servo-1-temperature, ...., servo-7-temperature]  
> &ensp;&ensp;&ensp;&ensp;}  
> :return: True/False  




#### release_cmdnum_changed_callback
__release_cmdnum_changed_callback__ = <function XArmAPI.release_cmdnum_changed_callback>
> Release the cmdnum changed callback  
>   
> :param callback:  
> :return: True/False  




#### release_connect_changed_callback
__release_connect_changed_callback__ = <function XArmAPI.release_connect_changed_callback>
> Release the connect changed callback  
>   
> :param callback:  
> :return: True/False  




#### release_count_changed_callback
__release_count_changed_callback__ = <function XArmAPI.release_count_changed_callback>
> Release the counter value changed callback  
>   
> :param callback:  
> :return: True/False  




#### release_error_warn_changed_callback
__release_error_warn_changed_callback__ = <function XArmAPI.release_error_warn_changed_callback>
> Release the error warn changed callback  
>   
> :param callback:  
> :return: True/False  




#### release_feedback_callback
__release_feedback_callback__ = <function XArmAPI.release_feedback_callback>
> Release the callback of feedback  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 2.1.0  
>   
> :param callback:  
> :return: True/False  




#### release_iden_progress_changed_callback
__release_iden_progress_changed_callback__ = <function XArmAPI.release_iden_progress_changed_callback>
> Release the Identification progress value changed callback  
>   
> :param callback:  
> :return: True/False  




#### release_mode_changed_callback
__release_mode_changed_callback__ = <function XArmAPI.release_mode_changed_callback>
> Release the mode changed callback  
>   
> :param callback:  
> :return: True/False  




#### release_mtable_mtbrake_changed_callback
__release_mtable_mtbrake_changed_callback__ = <function XArmAPI.release_mtable_mtbrake_changed_callback>
> Release the motor enable states or motor brake states changed callback  
>   
> :param callback:  
> :return: True/False  




#### release_report_callback
__release_report_callback__ = <function XArmAPI.release_report_callback>
> Release the report callback  
>   
> :param callback:  
> :return: True/False  




#### release_report_location_callback
__release_report_location_callback__ = <function XArmAPI.release_report_location_callback>
> Release the location report callback  
>   
> :param callback:  
> :return: True/False  




#### release_state_changed_callback
__release_state_changed_callback__ = <function XArmAPI.release_state_changed_callback>
> Release the state changed callback  
>   
> :param callback:  
> :return: True/False  




#### release_temperature_changed_callback
__release_temperature_changed_callback__ = <function XArmAPI.release_temperature_changed_callback>
> Release the temperature changed callback  
>   
> :param callback:  
> :return: True/False  




#### reset
__reset__ = <function XArmAPI.reset>
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




#### robotiq_close
__robotiq_close__ = <function XArmAPI.robotiq_close>
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




#### robotiq_get_status
__robotiq_get_status__ = <function XArmAPI.robotiq_get_status>
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




#### robotiq_open
__robotiq_open__ = <function XArmAPI.robotiq_open>
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




#### robotiq_reset
__robotiq_reset__ = <function XArmAPI.robotiq_reset>
> Reset the robotiq gripper (clear previous activation if any)  
>   
> :return: tuple((code, robotiq_response))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;robotiq_response: See the robotiq documentation  




#### robotiq_set_activate
__robotiq_set_activate__ = <function XArmAPI.robotiq_set_activate>
> If not already activated. Activate the robotiq gripper  
>   
> :param wait: whether to wait for the robotiq activate complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 3, only available if wait=True  
>   
> :return: tuple((code, robotiq_response))  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;robotiq_response: See the robotiq documentation  




#### robotiq_set_position
__robotiq_set_position__ = <function XArmAPI.robotiq_set_position>
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




#### run_blockly_app
__run_blockly_app__ = <function XArmAPI.run_blockly_app>
> Run the app generated by xArmStudio software  
> :param path: app path  




#### run_gcode_file
__run_gcode_file__ = <function XArmAPI.run_gcode_file>
> Run the gcode file  
> :param path: gcode file path  




#### save_conf
__save_conf__ = <function XArmAPI.save_conf>
> Save config  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;2. The clean_conf interface can restore system default settings  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### save_record_trajectory
__save_record_trajectory__ = <function XArmAPI.save_record_trajectory>
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




#### send_cmd_sync
__send_cmd_sync__ = <function XArmAPI.send_cmd_sync>
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




#### send_hex_cmd
__send_hex_cmd__ = <function XArmAPI.send_hex_cmd>
> Hexadecimal communication protocol instruction  
>   
> :param datas: Hexadecimal data_list  
> :param timeout: timeout: wait timeout, seconds, default is 10s.  
> :return : Hexadecimal data_list or code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)  




#### set_allow_approx_motion
__set_allow_approx_motion__ = <function XArmAPI.set_allow_approx_motion>
> Settings allow to avoid overspeed near some singularities using approximate solutions  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.9.0  
>   
> :param on_off: allow or not, True means allow, default is False  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_baud_checkset_enable
__set_baud_checkset_enable__ = <function XArmAPI.set_baud_checkset_enable>
> Enable auto checkset the baudrate of the end IO board or not  
> Note:  
> &ensp;&ensp;&ensp;&ensp;only available in the API of gripper/bio/robotiq/linear_track.  
> &ensp;&ensp;&ensp;&ensp;  
> :param enable: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_bio_gripper_enable
__set_bio_gripper_enable__ = <function XArmAPI.set_bio_gripper_enable>
> If not already enabled. Enable the bio gripper  
>   
> :param enable: enable or not  
> :param wait: whether to wait for the bio gripper enable complete, default is True  
> :param timeout: maximum waiting time(unit: second), default is 3, only available if wait=True  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_bio_gripper_speed
__set_bio_gripper_speed__ = <function XArmAPI.set_bio_gripper_speed>
> Set the speed of the bio gripper  
>   
> :param speed: speed  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_cartesian_velo_continuous
__set_cartesian_velo_continuous__ = <function XArmAPI.set_cartesian_velo_continuous>
> Set cartesian motion velocity continuous  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.9.0  
>   
> :param on_off: continuous or not, True means continuous, default is False  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_cgpio_analog
__set_cgpio_analog__ = <function XArmAPI.set_cgpio_analog>
> Set the analog value of the specified Controller GPIO  
>   
> :param ionum: 0 or 1  
> :param value: value  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_cgpio_analog_with_xyz
__set_cgpio_analog_with_xyz__ = <function XArmAPI.set_cgpio_analog_with_xyz>
> Set the analog value of the specified Controller GPIO when the robot has reached the specified xyz position             
>   
> :param ionum: 0 ~ 1  
> :param value: value  
> :param xyz: position xyz, as [x, y, z]  
> :param fault_tolerance_radius: fault tolerance radius  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_cgpio_digital
__set_cgpio_digital__ = <function XArmAPI.set_cgpio_digital>
> Set the digital value of the specified Controller GPIO  
>   
> :param ionum: 0~15  
> :param value: value  
> :param delay_sec: delay effective time from the current start, in seconds, default is None(effective immediately)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_cgpio_digital_input_function
__set_cgpio_digital_input_function__ = <function XArmAPI.set_cgpio_digital_input_function>
> Set the digital input functional mode of the Controller GPIO  
> :param ionum: 0~15  
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




#### set_cgpio_digital_output_function
__set_cgpio_digital_output_function__ = <function XArmAPI.set_cgpio_digital_output_function>
> Set the digital output functional mode of the specified Controller GPIO  
> :param ionum: 0~15  
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




#### set_cgpio_digital_with_xyz
__set_cgpio_digital_with_xyz__ = <function XArmAPI.set_cgpio_digital_with_xyz>
> Set the digital value of the specified Controller GPIO when the robot has reached the specified xyz position             
>   
> :param ionum: 0 ~ 15  
> :param value: value  
> :param xyz: position xyz, as [x, y, z]  
> :param fault_tolerance_radius: fault tolerance radius  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_checkset_default_baud
__set_checkset_default_baud__ = <function XArmAPI.set_checkset_default_baud>
> Set the checkset baud value  
>   
> :param type_: checkset type  
> &ensp;&ensp;&ensp;&ensp;1: xarm gripper  
> &ensp;&ensp;&ensp;&ensp;2: bio gripper  
> &ensp;&ensp;&ensp;&ensp;3: robotiq gripper  
> &ensp;&ensp;&ensp;&ensp;4: linear track  
> :param baud: checkset baud value, less than or equal to 0 means disable checkset  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_collision_rebound
__set_collision_rebound__ = <function XArmAPI.set_collision_rebound>
> Set the collision rebound,turn on/off collision rebound  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.11 or above  
>   
> :param on: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_collision_sensitivity
__set_collision_sensitivity__ = <function XArmAPI.set_collision_sensitivity>
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




#### set_collision_tool_model
__set_collision_tool_model__ = <function XArmAPI.set_collision_tool_model>
> Set the geometric model of the end effector for self collision detection  
> &ensp;  
> :param tool_type: the geometric model type  
> &ensp;&ensp;&ensp;&ensp;0: No end effector, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;1: xArm Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;2: xArm Vacuum Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;3: xArm Bio Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;4: Robotiq-2F-85 Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;5: Robotiq-2F-140 Gripper, no additional parameters required  
> &ensp;&ensp;&ensp;&ensp;21: Cylinder, need additional parameters radius, height  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;self.set_collision_tool_model(21, radius=45, height=137)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param radius: the radius of cylinder, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param height: the height of cylinder, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;22: Cuboid, need additional parameters x, y, z  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;self.set_collision_tool_model(22, x=234, y=323, z=23)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param x: the length of the cuboid in the x coordinate direction, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param y: the length of the cuboid in the y coordinate direction, (unit: mm)  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;:param z: the length of the cuboid in the z coordinate direction, (unit: mm)  
> :param args: additional parameters  
> :param kwargs: additional parameters  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_counter_increase
__set_counter_increase__ = <function XArmAPI.set_counter_increase>
> Set counter plus value, only support plus 1  
>   
> :param val: reversed  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_counter_reset
__set_counter_reset__ = <function XArmAPI.set_counter_reset>
> Reset counter value  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_dh_params
__set_dh_params__ = <function XArmAPI.set_dh_params>
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




#### set_feedback_type
__set_feedback_type__ = <function XArmAPI.set_feedback_type>
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




#### set_fence_mode
__set_fence_mode__ = <function XArmAPI.set_fence_mode>
> Set the fence mode,turn on/off fense mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.11 or above  
>   
> :param on: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_force_control_pid
__set_force_control_pid__ = <function XArmAPI.set_force_control_pid>
> Set force control pid parameters through the Six-axis Force Torque Sensor.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :param kp: proportional gain.  
> :param ki: integral gain.  
> :param kd: differential gain.  
> :param xe_limit: 6d vector. for compliant axes, these values are the maximum allowed tcp speed along/about the axis. mm/s  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_gravity_direction
__set_gravity_direction__ = <function XArmAPI.set_gravity_direction>
> Set the direction of gravity  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. Do not use if not required  
> &ensp;&ensp;&ensp;&ensp;2. If not saved, it will be lost after reboot  
> &ensp;&ensp;&ensp;&ensp;3. The save_conf interface can record the current settings and will not be lost after the restart.  
> &ensp;&ensp;&ensp;&ensp;4. The clean_conf interface can restore system default settings  
>   
> :param direction: direction of gravity, such as [x(mm), y(mm), z(mm)]  
> :param wait: whether to wait for the robotic arm to stop or all previous queue commands to be executed or cleared before setting  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_gripper_enable
__set_gripper_enable__ = <function XArmAPI.set_gripper_enable>
> Set the gripper enable  
>   
> :param enable: enable or not  
> &ensp;Note： such as code = arm.set_gripper_enable(True)  #turn on the Gripper  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_gripper_mode
__set_gripper_mode__ = <function XArmAPI.set_gripper_mode>
> Set the gripper mode  
>   
> :param mode: 0: location mode  
> &ensp;Note： such as code = arm.set_gripper_mode(0)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_gripper_position
__set_gripper_position__ = <function XArmAPI.set_gripper_position>
> Set the gripper position  
>   
> :param pos: pos  
> :param wait: wait or not, default is False  
> :param speed: speed,unit:r/min  
> :param auto_enable: auto enable or not, default is False  
> :param timeout: wait time, unit:second, default is 10s  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_gripper_speed
__set_gripper_speed__ = <function XArmAPI.set_gripper_speed>
> Set the gripper speed  
>   
> :param speed:  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_impedance
__set_impedance__ = <function XArmAPI.set_impedance>
> Set all parameters of impedance control through the Six-axis Force Torque Sensor.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :param coord: task frame. 0: base frame. 1: tool frame.  
> :param c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.  
> :param M: mass. (kg)  
> :param K: stiffness coefficient.  
> :param B: damping coefficient. invalid.  
> &ensp;&ensp;&ensp;&ensp;Note: the value is set to 2*sqrt(M*K) in controller.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_impedance_config
__set_impedance_config__ = <function XArmAPI.set_impedance_config>
> Set impedance control parameters of impedance control through the Six-axis Force Torque Sensor.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :param coord: task frame. 0: base frame. 1: tool frame.  
> :param c_axis: a 6d vector of 0s and 1s. 1 means that robot will be impedance in the corresponding axis of the task frame.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_impedance_mbk
__set_impedance_mbk__ = <function XArmAPI.set_impedance_mbk>
> Set mbk parameters of impedance control through the Six-axis Force Torque Sensor.  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.3  
> &ensp;&ensp;&ensp;&ensp;2. the Six-axis Force Torque Sensor is required (the third party is not currently supported)  
>   
> :param M: mass. (kg)  
> :param K: stiffness coefficient.  
> :param B: damping coefficient. invalid.  
> &ensp;&ensp;&ensp;&ensp;Note: the value is set to 2*sqrt(M*K) in controller.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_initial_point
__set_initial_point__ = <function XArmAPI.set_initial_point>
> Set the initial point  
>   
> :param point: initial point, [J1, J2, ..., J7]  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_joint_jerk
__set_joint_jerk__ = <function XArmAPI.set_joint_jerk>
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




#### set_joint_maxacc
__set_joint_maxacc__ = <function XArmAPI.set_joint_maxacc>
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




#### set_linear_track_back_origin
__set_linear_track_back_origin__ = <function XArmAPI.set_linear_track_back_origin>
> Set the linear track go back to the origin position  
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




#### set_linear_track_enable
__set_linear_track_enable__ = <function XArmAPI.set_linear_track_enable>
> Set the linear track enable/disable  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :param enable: enable or not  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_linear_track_pos
__set_linear_track_pos__ = <function XArmAPI.set_linear_track_pos>
> Set the position of the linear track  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :param pos: position. Integer between 0 and 700/1000/1500mm.  
> &ensp;&ensp;&ensp;&ensp;If SN start with AL1300 the position range is 0~700mm.  
> &ensp;&ensp;&ensp;&ensp;If SN start with AL1301 the position range is 0~1000mm.  
> &ensp;&ensp;&ensp;&ensp;If SN start with AL1302 the position range is 0~1500mm.  
> :param speed: speed of the linear track. Integer between 1 and 1000mm/s. default is not set  
> :param wait: wait to motion finish or not, default is True  
> :param timeout: wait timeout, seconds, default is 100s.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_linear_track_speed
__set_linear_track_speed__ = <function XArmAPI.set_linear_track_speed>
> Set the speed of the linear track  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :param speed: Integer between 1 and 1000mm/s.  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_linear_track_stop
__set_linear_track_stop__ = <function XArmAPI.set_linear_track_stop>
> Set the linear track to stop  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.8.0  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_mode
__set_mode__ = <function XArmAPI.set_mode>
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




#### set_mount_direction
__set_mount_direction__ = <function XArmAPI.set_mount_direction>
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




#### set_only_check_type
__set_only_check_type__ = <function XArmAPI.set_only_check_type>
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




#### set_pause_time
__set_pause_time__ = <function XArmAPI.set_pause_time>
> Set the arm pause time, xArm will pause sltime second  
>   
> :param sltime: sleep time,unit:(s)second  
> :param wait: wait or not, default is False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_position
__set_position__ = <function XArmAPI.set_position>
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




#### set_position_aa
__set_position_aa__ = <function XArmAPI.set_position_aa>
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




#### set_reduced_joint_range
__set_reduced_joint_range__ = <function XArmAPI.set_reduced_joint_range>
> Set the joint range of the reduced mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.11 or above  
> &ensp;&ensp;&ensp;&ensp;2. Only reset the reduced mode to take effect (`set_reduced_mode(True)`)  
>   
> :param joint_range: [joint-1-min, joint-1-max, ..., joint-7-min, joint-7-max]  
> :param is_radian: the param joint_range are in radians or not, default is self.default_is_radian  
> :return:  




#### set_reduced_max_joint_speed
__set_reduced_max_joint_speed__ = <function XArmAPI.set_reduced_max_joint_speed>
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




#### set_reduced_max_tcp_speed
__set_reduced_max_tcp_speed__ = <function XArmAPI.set_reduced_max_tcp_speed>
> Set the maximum tcp speed of the reduced mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
> &ensp;&ensp;&ensp;&ensp;2. Only reset the reduced mode to take effect (`set_reduced_mode(True)`)  
>   
> :param speed: speed (mm/s)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_reduced_mode
__set_reduced_mode__ = <function XArmAPI.set_reduced_mode>
> Turn on/off reduced mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
>   
> :param on: True/False  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;such as:Turn on the reduced mode : code=arm.set_reduced_mode(True)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_reduced_tcp_boundary
__set_reduced_tcp_boundary__ = <function XArmAPI.set_reduced_tcp_boundary>
> Set the boundary of the safety boundary mode  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
> &ensp;&ensp;&ensp;&ensp;2. Only reset the reduced mode to take effect (`set_reduced_mode(True)`)  
>   
> :param boundary: [x_max, x_min, y_max, y_min, z_max, z_min]  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_report_tau_or_i
__set_report_tau_or_i__ = <function XArmAPI.set_report_tau_or_i>
> Set the reported torque or electric current  
>   
> :param tau_or_i:   
> &ensp;&ensp;&ensp;&ensp;0: torque  
> &ensp;&ensp;&ensp;&ensp;1: electric current  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_safe_level
__set_safe_level__ = <function XArmAPI.set_safe_level>
> Set safe level,  
>   
> :param level: safe level, default is 4  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_self_collision_detection
__set_self_collision_detection__ = <function XArmAPI.set_self_collision_detection>
> Set whether to enable self-collision detection   
>   
> :param on_off: enable or not  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_servo_angle
__set_servo_angle__ = <function XArmAPI.set_servo_angle>
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




#### set_servo_angle_j
__set_servo_angle_j__ = <function XArmAPI.set_servo_angle_j>
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




#### set_servo_attach
__set_servo_attach__ = <function XArmAPI.set_servo_attach>
> Attach the servo  
>   
> :param servo_id: 1-(Number of axes), 8, if servo_id is 8, will attach all servo  
> &ensp;&ensp;&ensp;&ensp;1. 1-(Number of axes): attach only one joint  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: arm.set_servo_attach(servo_id=1)  
> &ensp;&ensp;&ensp;&ensp;2: 8: attach all joints  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: arm.set_servo_attach(servo_id=8)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_servo_cartesian
__set_servo_cartesian__ = <function XArmAPI.set_servo_cartesian>
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




#### set_servo_cartesian_aa
__set_servo_cartesian_aa__ = <function XArmAPI.set_servo_cartesian_aa>
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




#### set_servo_detach
__set_servo_detach__ = <function XArmAPI.set_servo_detach>
> Detach the servo, be sure to do protective work before unlocking to avoid injury or damage.  
>   
> :param servo_id: 1-(Number of axes), 8, if servo_id is 8, will detach all servo  
> &ensp;&ensp;&ensp;&ensp;1. 1-(Number of axes): detach only one joint  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: arm.set_servo_detach(servo_id=1)  
> &ensp;&ensp;&ensp;&ensp;2: 8: detach all joints, please  
> &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;ex: arm.set_servo_detach(servo_id=8)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_simulation_robot
__set_simulation_robot__ = <function XArmAPI.set_simulation_robot>
> Set the simulation robot  
>   
> :param on_off: True/False  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_state
__set_state__ = <function XArmAPI.set_state>
> Set the xArm state  
>   
> :param state: default is 0  
> &ensp;&ensp;&ensp;&ensp;0: sport state  
> &ensp;&ensp;&ensp;&ensp;3: pause state  
> &ensp;&ensp;&ensp;&ensp;4: stop state  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_tcp_jerk
__set_tcp_jerk__ = <function XArmAPI.set_tcp_jerk>
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




#### set_tcp_load
__set_tcp_load__ = <function XArmAPI.set_tcp_load>
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




#### set_tcp_maxacc
__set_tcp_maxacc__ = <function XArmAPI.set_tcp_maxacc>
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




#### set_tcp_offset
__set_tcp_offset__ = <function XArmAPI.set_tcp_offset>
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




#### set_teach_sensitivity
__set_teach_sensitivity__ = <function XArmAPI.set_teach_sensitivity>
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




#### set_tgpio_digital
__set_tgpio_digital__ = <function XArmAPI.set_tgpio_digital>
> Set the digital value of the specified Tool GPIO  
>   
> :param ionum: 0 or 1  
> :param value: value  
> :param delay_sec: delay effective time from the current start, in seconds, default is None(effective immediately)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_tgpio_digital_with_xyz
__set_tgpio_digital_with_xyz__ = <function XArmAPI.set_tgpio_digital_with_xyz>
> Set the digital value of the specified Tool GPIO when the robot has reached the specified xyz position             
>   
> :param ionum: 0 or 1  
> :param value: value  
> :param xyz: position xyz, as [x, y, z]  
> :param fault_tolerance_radius: fault tolerance radius  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_tgpio_modbus_baudrate
__set_tgpio_modbus_baudrate__ = <function XArmAPI.set_tgpio_modbus_baudrate>
> Set the modbus baudrate of the tool gpio  
>   
> :param baud: 4800/9600/19200/38400/57600/115200/230400/460800/921600/1000000/1500000/2000000/2500000  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_tgpio_modbus_timeout
__set_tgpio_modbus_timeout__ = <function XArmAPI.set_tgpio_modbus_timeout>
> Set the modbus timeout of the tool gpio  
>   
> :param timeout: timeout, milliseconds  
> :param is_transparent_transmission: whether the set timeout is the timeout of transparent transmission  
> &ensp;&ensp;&ensp;&ensp;Note: only available if firmware_version >= 1.11.0  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_timeout
__set_timeout__ = <function XArmAPI.set_timeout>
> Set the timeout of cmd response  
>   
> :param timeout: seconds  




#### set_tool_position
__set_tool_position__ = <function XArmAPI.set_tool_position>
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




#### set_vacuum_gripper
__set_vacuum_gripper__ = <function XArmAPI.set_vacuum_gripper>
> Set vacuum gripper state  
>   
> :param on: open or not  
> &ensp;&ensp;&ensp;&ensp;on=True: equivalent to calling `set_tgpio_digital(0, 1)` and `set_tgpio_digital(1, 0)`  
> &ensp;&ensp;&ensp;&ensp;on=False: equivalent to calling `set_tgpio_digital(0, 0)` and `set_tgpio_digital(1, 1)`  
> :param wait: wait or not, default is False  
> :param timeout: wait time, unit:second, default is 3s  
> :param delay_sec: delay effective time from the current start, in seconds, default is None(effective immediately)  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### set_world_offset
__set_world_offset__ = <function XArmAPI.set_world_offset>
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




#### start_record_trajectory
__start_record_trajectory__ = <function XArmAPI.start_record_trajectory>
> Start trajectory recording, only in teach mode, so you need to set joint teaching mode before.  
>   
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. This interface relies on Firmware 1.2.0 or above  
> &ensp;&ensp;&ensp;&ensp;2. set joint teaching mode: set_mode(2);set_state(0)  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### stop_lite6_gripper
__stop_lite6_gripper__ = <function XArmAPI.stop_lite6_gripper>
> Stop the gripper of Lite6 series robotic arms  
> Note:  
> &ensp;&ensp;&ensp;&ensp;1. only available if firmware_version >= 1.10.0  
> &ensp;&ensp;&ensp;&ensp;2. this API can only be used on Lite6 series robotic arms  
>   
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### stop_record_trajectory
__stop_record_trajectory__ = <function XArmAPI.stop_record_trajectory>
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




#### system_control
__system_control__ = <function XArmAPI.system_control>
> Control the xArm controller system  
>   
> :param value: 1: shutdown, 2: reboot  
> :return: code  
> &ensp;&ensp;&ensp;&ensp;code: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  




#### vc_set_cartesian_velocity
__vc_set_cartesian_velocity__ = <function XArmAPI.vc_set_cartesian_velocity>
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




#### vc_set_joint_velocity
__vc_set_joint_velocity__ = <function XArmAPI.vc_set_joint_velocity>
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




#### write_and_read_holding_registers
__write_and_read_holding_registers__ = <function XArmAPI.write_and_read_holding_registers>
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




#### write_multiple_coil_bits
__write_multiple_coil_bits__ = <function XArmAPI.write_multiple_coil_bits>
> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Write Multiple Coils (0x0F)  
>   
> :param addr: the starting address of the register to be written  
> :param bits: array of values to write  
> :return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)  




#### write_multiple_holding_registers
__write_multiple_holding_registers__ = <function XArmAPI.write_multiple_holding_registers>
> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Write Multiple Holding Registers (0x10)  
>   
> :param addr: the starting address of the register to be written  
> :param regs: array of values to write  
> :return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)  




#### write_single_coil_bit
__write_single_coil_bit__ = <function XArmAPI.write_single_coil_bit>
> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Write Single Coil (0x05)  
>   
> :param addr: register address  
> :param bit_val: the value to write (0/1)  
> :return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)  




#### write_single_holding_register
__write_single_holding_register__ = <function XArmAPI.write_single_holding_register>
> ([Standard Modbus TCP](../UF_ModbusTCP_Manual.md)) Write Single Holding Register (0x06)  
>   
> :param addr: register address  
> :param bit_val: the value to write  
> :return: See the [API Code Documentation](./xarm_api_code.md#api-code) for details.  
> &ensp;&ensp;&ensp;&ensp;Note: code 129~144 means modbus tcp exception, the actual modbus tcp exception code is (code-0x80), refer to [Standard Modbus TCP](../UF_ModbusTCP_Manual.md)  



