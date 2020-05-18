xArm-Python-SDK API Documentation: class XArmAPI in module xarm.wrapper.xarm_api

## class __XArmAPI__
****************************************

### __Attributes__
****************************************
#### __angles__
```
Servo angles
Note:
    1. If self.default_is_radian is True, the returned value is in radians

:return: [angle1(° or rad), angle2(° or rad), ..., anglen7(° or rad)]
```

#### __arm__

#### __axis__
```
Axis number, only available in socket way and enable_report is True and report_type is 'rich'
```

#### __cmd_num__
```
Number of command caches in the controller
```

#### __collision_sensitivity__
```
The sensitivity value of collision, only available in socket way and  enable_report is True and report_type is 'rich'

:return: 0~5
```

#### __connected__
```
Connection status
```

#### __core__
```
Core layer API, set only for advanced developers, please do not use
Ex:
    self.core.move_line(...)
    self.core.move_lineb(...)
    self.core.move_joint(...)
    ...
```

#### __default_is_radian__
```
The default unit is radians or not
```

#### __device_type__
```
Device type, only available in socket way and  enable_report is True and report_type is 'rich'
```

#### __error_code__
```
Controller error code. See Chapter 7 of the xArm User Manual for details.
```

#### __gpio_reset_config__
```
The gpio reset enable config
:return: [cgpio_reset_enable, tgpio_reset_enable]
```

#### __gravity_direction__
```
gravity direction, only available in socket way and enable_report is True and report_type is 'rich'
:return:
```

#### __has_err_warn__
```
Contorller have an error or warning or not

:return: True/False
```

#### __has_error__
```
Controller have an error or not
```

#### __has_warn__
```
Controller have an warnning or not
```

#### __joint_acc_limit__
```
Joint acceleration limit, only available in socket way and enable_report is True and report_type is 'rich'
Note:
    1. If self.default_is_radian is True, the returned value is in radians

:return: [min_joint_acc(°/s^2 or rad/s^2), max_joint_acc(°/s^2 or rad/s^2)]
```

#### __joint_jerk__
```
Joint jerk
Note:
    1. If self.default_is_radian is True, the returned value is in radians

:return: jerk (°/s^3 or rad/s^3)
```

#### __joint_speed_limit__
```
Joint speed limit,  only available in socket way and enable_report is True and report_type is 'rich'
Note:
    1. If self.default_is_radian is True, the returned value is in radians

:return: [min_joint_speed(°/s or rad/s), max_joint_speed(°/s or rad/s)]
```

#### __joints_torque__
```
Joints torque, only available in socket way and  enable_report is True and report_type is 'rich'

:return: [joint-1, ....]
```

#### __last_used_angles__
```
The last used servo angles, default value of parameter angle of interface set_servo_angle
Note:
    1. If self.default_is_radian is True, the returned value is in radians
    2. self.set_servo_angle(servo_id=1, angle=75) <==> self.set_servo_angle(angle=[75] + self.last_used_angles[1:])
    3. self.set_servo_angle(servo_id=5, angle=30) <==> self.set_servo_angle(angle=self.last_used_angles[:4] + [30] + self.last_used_angles[5:])

:return: [angle1(° or rad), angle2(° or rad), ..., angle7(° or rad)]
```

#### __last_used_joint_acc__
```
The last used joint acceleration, default value of parameter mvacc of interface set_servo_angle
Note:
    1. If self.default_is_radian is True, the returned value is in radians

:return: acceleration (°/s^2 or rad/s^2)
```

#### __last_used_joint_speed__
```
The last used joint speed, default value of parameter speed of interface set_servo_angle
Note:
    1. If self.default_is_radian is True, the returned value is in radians

:return: speed (°/s or rad/s)
```

#### __last_used_position__
```
The last used cartesion position, default value of parameter x/y/z/roll/pitch/yaw of interface set_position
Note:
    1. If self.default_is_radian is True, the returned value (only roll/pitch/yaw) is in radians
    2. self.set_position(x=300) <==> self.set_position(x=300, *last_used_position[1:])
    2. self.set_position(roll=-180) <==> self.set_position(x=self.last_used_position[:3], roll=-180, *self.last_used_position[4:])

:return: [x(mm), y(mm), z(mm), roll(° or rad), pitch(° or rad), yaw(° or rad)]
```

#### __last_used_tcp_acc__
```
The last used cartesion acceleration, default value of parameter mvacc of interface set_position/move_circle

:return: acceleration (mm/s^2)
```

#### __last_used_tcp_speed__
```
The last used cartesion speed, default value of parameter speed of interface set_position/move_circle

:return: speed (mm/s)
```

#### __master_id__
```
Master id, only available in socket way and enable_report is True and report_type is 'rich'
```

#### __mode__
```
xArm mode，only available in socket way and  enable_report is True

:return:
    0: position control mode
    1: servo motion mode
    2: joint teaching mode
    3: cartesian teaching mode (invalid)
```

#### __motor_brake_states__
```
Motor brake state list, only available in socket way and  enable_report is True and report_type is 'rich'
Note:
    For a robot with a number of axes n, only the first n states are valid, and the latter are reserved.

:return: [motor-1-brake-state, ..., motor-7-brake-state, reserved]
    motor-{i}-brake-state:
        0: enable
        1: disable
```

#### __motor_enable_states__
```
Motor enable state list, only available in socket way and  enable_report is True and report_type is 'rich'
Note:
    For a robot with a number of axes n, only the first n states are valid, and the latter are reserved.

:return: [motor-1-enable-state, ..., motor-7-enable-state, reserved]
    motor-{i}-enable-state:
        0: disable
        1: enable
```

#### __position__
```
Cartesion position
Note:
    1. If self.default_is_radian is True, the returned value (only roll/pitch/yaw) is in radians

return: [x(mm), y(mm), z(mm), roll(° or rad), pitch(° or rad), yaw(° or rad)]
```

#### __realtime_joint_speeds__
```
The real time speed of joint motion, only available if version > 1.2.11

:return: [joint-1-speed(°/s or rad/s), ...., joint-7-speed(°/s or rad/s)]
```

#### __realtime_tcp_speed__
```
The real time speed of tcp motion, only available if version > 1.2.11

:return: real time speed (mm/s)
```

#### __slave_id__
```
Slave id, only available in socket way and enable_report is True and report_type is 'rich'
```

#### __sn__
```
xArm sn
```

#### __state__
```
xArm state

:return:
    1: in motion
    2: sleeping
    3: suspended
    4: stopping
```

#### __tcp_acc_limit__
```
Tcp acceleration limit, only available in socket way and enable_report is True and report_type is 'rich' 

:return: [min_tcp_acc(mm/s^2), max_tcp_acc(mm/s^2)]
```

#### __tcp_jerk__
```
Tcp jerk

:return: jerk (mm/s^3)
```

#### __tcp_load__
```
xArm tcp load, only available in socket way and  enable_report is True and report_type is 'rich'

:return: [weight, center of gravity]
    such as: [weight(kg), [x(mm), y(mm), z(mm)]]
```

#### __tcp_offset__
```
Cartesion position offset, only available in socket way and enable_report is True
Note:
    1. If self.default_is_radian is True, the returned value(roll_offset/pitch_offset/yaw_offset) is in radians

:return: [x_offset(mm), y_offset(mm), z_offset(mm), roll_offset(° or rad), pitch_offset(° or rad), yaw_offset(° or rad)]
```

#### __tcp_speed_limit__
```
Tcp speed limit, only available in socket way and enable_report is True and report_type is 'rich'

:return: [min_tcp_speed(mm/s), max_tcp_speed(mm/s)]
```

#### __teach_sensitivity__
```
The sensitivity value of drag and teach, only available in socket way and  enable_report is True and report_type is 'rich'

:return: 1~5
```

#### __temperatures__
```
Motor temperature, only available if version > 1.2.11

:return: [motor-1-temperature, ..., motor-7-temperature]
```

#### __version__
```
xArm version
```

#### __version_number__
```
Frimware version number

:return: (major_version_number, minor_version_number, revision_version_number)
```

#### __warn_code__
```
Controller warn code. See Chapter 7 of the xArm User Manual for details.
```

#### __world_offset__
```
Base coordinate offset, only available if version > 1.2.11

Note:
    1. If self.default_is_radian is True, the returned value(roll_offset/pitch_offset/yaw_offset) is in radians

:return: [x_offset(mm), y_offset(mm), z_offset(mm), roll_offset(° or rad), pitch_offset(° or rad), yaw_offset(° or rad)]
```

****************************************
### __Methods__
****************************************
#### def __\__init__\__(self, port=None, is_radian=False, do_not_open=False, **kwargs):

```
The API wrapper of xArm
Note: Orientation of attitude angle
    roll: rotate around the X axis
    pitch: rotate around the Y axis
    yaw: rotate around the Z axis

:param port: ip-address(such as '192.168.1.185')
    Note: this parameter is required if parameter do_not_open is False
:param is_radian: set the default unit is radians or not, default is False
    Note: (aim of design)
        1. Default value for unified interface parameters
        2: Unification of the external unit system
        3. For compatibility with previous interfaces
    Note: the conversion of degree (°) and radians (rad)
        * 1 rad == 57.29577951308232 °
        * 1 ° == 0.017453292519943295 rad
        * 1 rad/s == 57.29577951308232 °/s
        * 1 °/s == 0.017453292519943295 rad/s
        * 1 rad/s^2 == 57.29577951308232 °/s^2
        * 1 °/s^2 == 0.017453292519943295 rad/s^2
        * 1 rad/s^3 == 57.29577951308232 °/s^3
        * 1 °/s^3 == 0.017453292519943295 rad/s^3
    Note: This parameter determines the value of the property self.default_is_radian
    Note: This parameter determines the default value of the interface with the is_radian/input_is_radian/return_is_radian parameter
       The list of affected interfaces is as follows:
            1. method: get_position
            2. method: set_position
            3. method: get_servo_angle
            4. method: set_servo_angle
            5. method: set_servo_angle_j
            6. method: move_gohome
            7. method: reset
            8. method: set_tcp_offset
            9. method: set_joint_jerk
            10. method: set_joint_maxacc
            11. method: get_inverse_kinematics
            12. method: get_forward_kinematics
            13. method: is_tcp_limit
            14. method: is_joint_limit
            15. method: get_params
            16: method: move_arc_lines
            17: method: move_circle
            18: method: set_servo_cartesian
    Note: This parameter determines the default return type for some interfaces (such as the position, velocity, and acceleration associated with the return angle arc).
        The affected attributes are as follows:
            1. property: position
            2. property: last_used_position
            3. property: angles
            4. property: last_used_angles
            5. property: last_used_joint_speed
            6. property: last_used_joint_acc
            7. property: tcp_offset
:param do_not_open: do not open, default is False, if true, you need to manually call the connect interface.
:param kwargs: keyword parameters, generally do not need to set
    axis: number of axes, required only when using a serial port connection, default is 7
    baudrate: serial baudrate, invalid, reserved.
    timeout: serial timeout, invalid, reserved.
    filters: serial port filters, invalid, reserved.
    check_tcp_limit: check the tcp param value out of limit or not, default is True
        Note: only check the param roll/pitch/yaw of the interface `set_position`/`move_arc_lines`
    check_joint_limit: check the joint           param value out of limit or not, default is True
        Note: only check the param angle of the interface `set_servo_angle` and the param angles of the interface `set_servo_angle_j`
    check_cmdnum_limit: check the cmdnum out of limit or not, default is True
        Note: only available in the interface `set_position`/`set_servo_angle`/`move_circle`/`move_arc_lines`
    max_cmdnum: max cmdnum, default is 256
        Note: only available in the param `check_cmdnum_limit` is True
        Note: only available in the interface `set_position`/`set_servo_angle`/`move_circle`/`move_arc_lines`
    check_is_ready: check if the arm is in motion, default is True
        Note: only available in the interface `set_position`/`set_servo_angle`/`set_servo_angle_j`/`set_servo_cartesian`/`move_circle`/`move_gohome`/`move_arc_lines`
```

#### def __check_verification__(self):

```
check verification

:return: tuple((code, status)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    status:
        0: verified
        other: not verified
```

#### def __clean_conf__(self):

```
Clean current config and restore system default settings
Note:
    1. This interface will clear the current settings and restore to the original settings (system default settings)

:return: code
    code: See the API code documentation for details.
```

#### def __clean_error__(self):

```
Clean the error, need to be manually enabled motion(arm.motion_enable(True)) and set state(arm.set_state(state=0))after clean error

:return: code
    code: See the API code documentation for details.
```

#### def __clean_gripper_error__(self, **kwargs):

```
Clean the gripper error

:return: code
    code: See the Gripper code documentation for details.
```

#### def __clean_warn__(self):

```
Clean the warn

:return: code
    code: See the API code documentation for details.
```

#### def __config_cgpio_reset_when_stop__(self, on_off):

```
Config the Controller GPIO reset the digital output when the robot is in stop state

:param on_off: True/False
:return: code
    code: See the API code documentation for details.
```

#### def __config_tgpio_reset_when_stop__(self, on_off):

```
Config the Tool GPIO reset the digital output when the robot is in stop state

:param on_off: True/False
:return: code
    code: See the API code documentation for details.
```

#### def __connect__(self, port=None, baudrate=None, timeout=None, axis=None):

```
Connect to xArm

:param port: port name or the ip address, default is the value when initializing an instance
:param baudrate: baudrate, only available in serial way, default is the value when initializing an instance
:param timeout: timeout, only available in serial way, default is the value when initializing an instance
:param axis: number of axes, required only when using a serial port connection, default is 7
```

#### def __disconnect__(self):

```
Disconnect
```

#### def __emergency_stop__(self):

```
Emergency stop (set_state(4) -> motion_enable(True) -> set_state(0))
Note:
    1. This interface does not automatically clear the error. If there is an error, you need to handle it according to the error code.
```

#### def __get_cgpio_analog__(self, ionum=None):

```
Get the analog value of the specified Controller GPIO
:param ionum: 0 or 1 or None(both 0 and 1), default is None
:return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

#### def __get_cgpio_digital__(self, ionum=None):

```
Get the digital value of the specified Controller GPIO

:param ionum: 0~7 or None(both 0~7), default is None
:return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

#### def __get_cgpio_state__(self):

```
Get the state of the Controller GPIO
:return: code, states
    code: See the API code documentation for details.
    states: [...]
        states[0]: contorller gpio module state
            states[0] == 0: normal
            states[0] == 1：wrong
            states[0] == 6：communication failure
        states[1]: controller gpio module error code
            states[1] == 0: normal
            states[1] != 0：error code
        states[2]: digital input functional gpio state
            Note: digital-i-input functional gpio state = states[2] >> i & 0x01
        states[3]: digital input configuring gpio state
            Note: digital-i-input configuring gpio state = states[3] >> i & 0x01
        states[4]: digital output functional gpio state
            Note: digital-i-output functional gpio state = states[4] >> i & 0x01
        states[5]: digital output configuring gpio state
            Note: digital-i-output configuring gpio state = states[5] >> i & 0x01
        states[6]: analog-0 input value
        states[7]: analog-1 input value
        states[8]: analog-0 output value
        states[9]: analog-1 output value
        states[10]: digital input functional info, [digital-0-input-functional-mode, ... digital-7-input-functional-mode]
        states[11]: digital output functional info, [digital-0-output-functional-mode, ... digital-7-output-functional-mode]
```

#### def __get_cmdnum__(self):

```
Get the cmd count in cache
:return: tuple((code, cmd_num)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

#### def __get_err_warn_code__(self, show=False, lang='en'):

```
Get the controller error and warn code

:param show: show the detail info if True
:param lang: show language, en/cn, degault is en, only available if show is True
:return: tuple((code, [error_code, warn_code])), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    error_code: See Chapter 7 of the xArm User Manual for details.
    warn_code: See Chapter 7 of the xArm User Manual for details.
```

#### def __get_forward_kinematics__(self, angles, input_is_radian=None, return_is_radian=None):

```
Get forward kinematics

:param angles: [angle-1, angle-2, ..., angle-n], n is the number of axes of the arm
:param input_is_radian: the param angles value is in radians or not, default is self.default_is_radian
:param return_is_radian: the returned value is in radians or not, default is self.default_is_radian
:return: tuple((code, pose)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    pose: [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)] or []
        Note: the roll/pitch/yaw value is radians if return_is_radian is True, else °
```

#### def __get_gripper_err_code__(self, **kwargs):

```
Get the gripper error code

:return: tuple((code, err_code)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    err_code: See the Gripper code documentation for details.
```

#### def __get_gripper_position__(self, **kwargs):

```
Get the gripper position

:return: tuple((code, pos)), only when code is 0, the returned result is correct.
    code: See the Gripper code documentation for details.
```

#### def __get_gripper_version__(self):

```
Get gripper version, only for debug

:return: (code, version)
    code: See the API code documentation for details.
```

#### def __get_harmonic_type__(self, servo_id=1):

```
Get harmonic type, only for debu

:return: (code, type)
    code: See the API code documentation for details.
```

#### def __get_hd_types__(self):

```
Get harmonic types, only for debug

:return: (code, types)
    code: See the API code documentation for details.
```

#### def __get_inverse_kinematics__(self, pose, input_is_radian=None, return_is_radian=None):

```
Get inverse kinematics

:param pose: [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
    Note: the roll/pitch/yaw unit is radian if input_is_radian is True, else °
:param input_is_radian: the param pose value(only roll/pitch/yaw) is in radians or not, default is self.default_is_radian
:param return_is_radian: the returned value is in radians or not, default is self.default_is_radian
:return: tuple((code, angles)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    angles: [angle-1(rad or °), angle-2, ..., angle-(Number of axes)] or []
        Note: the returned angle value is radians if return_is_radian is True, else °
```

#### def __get_is_moving__(self):

```
Check xArm is moving or not
:return: True/False
```

#### def __get_pose_offset__(self, pose1, pose2, orient_type_in=0, orient_type_out=0, is_radian=None):

```
Calculate the pose offset of two given points

:param pose1: [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]
:param pose2: [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]
:param orient_type_in: input attitude notation, 0 is RPY(roll/pitch/yaw) (default), 1 is axis angle(rx/ry/rz)
:param orient_type_out: notation of output attitude, 0 is RPY (default), 1 is axis angle
:param is_radian: the roll/rx/pitch/ry/yaw/rz of pose1/pose2/return_pose is radian or not
:return: tuple((code, pose)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    pose: [x(mm), y(mm), z(mm), roll/rx(rad or °), pitch/ry(rad or °), yaw/rz(rad or °)]
```

#### def __get_position__(self, is_radian=None):

```
Get the cartesian position
Note:
    1. If the value(roll/pitch/yaw) you want to return is an radian unit, please set the parameter is_radian to True
        ex: code, pos = arm.get_position(is_radian=True)

:param is_radian: the returned value (only roll/pitch/yaw) is in radians or not, default is self.default_is_radian
:return: tuple((code, [x, y, z, roll, pitch, yaw])), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

#### def __get_position_aa__(self, is_radian=None):

```
Get the pose represented by the axis angle pose

:param is_radian: the returned value (only rx/ry/rz) is in radians or not, default is self.default_is_radian
:return: tuple((code, [x, y, z, rx, ry, rz])), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

#### def __get_reduced_mode__(self):

```
Get reduced mode

Note:
    1. This interface relies on Firmware 1.2.0 or above

:return: tuple((code, mode))
    code: See the API code documentation for details.
    mode: 0 or 1, 1 means that the reduced mode is turned on. 0 means that the reduced mode is not turned on
```

#### def __get_reduced_states__(self, is_radian=None):

```
Get states of the reduced mode

Note:
    1. This interface relies on Firmware 1.2.0 or above

:param is_radian: the max_joint_speed of the states is in radians or not, default is self.default_is_radian
:return: tuple((code, states))
    code: See the API code documentation for details.
    states: [....]
        if version > 1.2.11:
            states: [
                reduced_mode_is_on,
                [reduced_x_max, reduced_x_min, reduced_y_max, reduced_y_min, reduced_z_max, reduced_z_min],
                reduced_max_tcp_speed,
                reduced_max_joint_speed,
                joint_ranges([joint-1-min, joint-1-max, ..., joint-7-min, joint-7-max]),
                safety_boundary_is_on,
                collision_rebound_is_on,
            ]`
        if version <= 1.2.11:
            states: [
                reduced_mode_is_on,
                [reduced_x_max, reduced_x_min, reduced_y_max, reduced_y_min, reduced_z_max, reduced_z_min],
                reduced_max_tcp_speed,
                reduced_max_joint_speed,
            ]`
```

#### def __get_robot_sn__(self):

```
Get the xArm sn

:return: tuple((code, sn)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

#### def __get_servo_angle__(self, servo_id=None, is_radian=None):

```
Get the servo angle
Note:
    1. If the value you want to return is an radian unit, please set the parameter is_radian to True
        ex: code, angles = arm.get_servo_angle(is_radian=True)
    2. If you want to return only the angle of a single joint, please set the parameter servo_id
        ex: code, angle = arm.get_servo_angle(servo_id=2)
    3. This interface is only used in the base coordinate system.

:param servo_id: 1-(Number of axes), None(8), default is None
:param is_radian: the returned value is in radians or not, default is self.default_is_radian
:return: tuple((code, angle list if servo_id is None or 8 else angle)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

#### def __get_servo_debug_msg__(self, show=False, lang='en'):

```
Get the servo debug msg, used only for debugging

:param show: show the detail info if True
:param lang: language, en/cn, default is en
:return: tuple((code, servo_info_list)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

#### def __get_servo_version__(self, servo_id=1):

```
Get servo version, only for debug

:param servo_id: servo id(1~7)
:return: (code, version)
    code: See the API code documentation for details.
```

#### def __get_state__(self):

```
Get state

:return: tuple((code, state)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    state:
        1: in motion
        2: sleeping
        3: suspended
        4: stopping
```

#### def __get_tgpio_analog__(self, ionum=None):

```
Get the analog value of the specified Tool GPIO
:param ionum: 0 or 1 or None(both 0 and 1), default is None
:return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

#### def __get_tgpio_digital__(self, ionum=None):

```
Get the digital value of the specified Tool GPIO

:param ionum: 0 or 1 or None(both 0 and 1), default is None
:return: tuple((code, value or value list)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

#### def __get_tgpio_version__(self):

```
Get tool gpio version, only for debug

:return: (code, version)
    code: See the API code documentation for details.
```

#### def __get_trajectories__(self):

```
get the trajectories

Note:
    1. This interface relies on xArmStudio 1.2.0 or above
    2. This interface relies on Firmware 1.2.0 or above

:return: tuple((code, trajectories))
    code: See the API code documentation for details.
    trajectories: [{
        'name': name, # The name of the trajectory
        'duration': duration, # The duration of the trajectory (seconds)
    }]
```

#### def __get_trajectory_rw_status__(self):

```
Get trajectory read/write status

:return: (code, status)
    code: See the API code documentation for details.
    status:
        0: no read/write
        1: loading
        2: load success
        3: load failed
        4: saving
        5: save success
        6: save failed
```

#### def __get_vacuum_gripper__(self):

```
Get vacuum gripper state

:return: tuple((code, state)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    state: suction cup state
        0: suction cup is off
        1: suction cup is on
```

#### def __get_version__(self):

```
Get the xArm firmware version

:return: tuple((code, version)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
```

#### def __is_joint_limit__(self, joint, is_radian=None):

```
Check the joint angle is in limit

:param joint: [angle-1, angle-2, ..., angle-n], n is the number of axes of the arm
:param is_radian: angle value is radians or not, default is self.default_is_radian
:return: tuple((code, limit)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    limit: True/False/None, limit or not, or failed
```

#### def __is_tcp_limit__(self, pose, is_radian=None):

```
Check the tcp pose is in limit

:param pose: [x, y, z, roll, pitch, yaw]
:param is_radian: roll/pitch/yaw value is radians or not, default is self.default_is_radian
:return: tuple((code, limit)), only when code is 0, the returned result is correct.
    code: See the API code documentation for details.
    limit: True/False/None, limit or not, or failed
```

#### def __load_trajectory__(self, filename, wait=True, timeout=10):

```
Load the trajectory

Note:
    1. This interface relies on Firmware 1.2.0 or above

:param filename: The name of the trajectory to load
:param wait: Whether to wait for loading, default is True
:param timeout: Timeout waiting for loading to complete
:return: code
    code: See the API code documentation for details.
```

#### def __motion_enable__(self, enable=True, servo_id=None):

```
Motion enable

:param enable:True/False
:param servo_id: 1-(Number of axes), None(8)
:return: code
    code: See the API code documentation for details.
```

#### def __move_arc_lines__(self, paths, is_radian=None, times=1, first_pause_time=0.1, repeat_pause_time=0, automatic_calibration=True, speed=None, mvacc=None, mvtime=None, wait=False):

```
Continuous linear motion with interpolation.
Note:
    1. If an error occurs, it will return early.
    2. If the emergency_stop interface is called actively, it will return early.
    3. The last_used_position/last_used_tcp_speed/last_used_tcp_acc will be modified.
    4. The last_used_angles/last_used_joint_speed/last_used_joint_acc will not be modified.

:param paths: cartesian path list
    1. Specify arc radius： [[x, y, z, roll, pitch, yaw, radius], ....]
    2. Do not specify arc radius (radius=0)： [[x, y, z, roll, pitch, yaw], ....]
    3. If you want to plan the continuous motion,set radius>0.

:param is_radian: roll/pitch/yaw of paths are in radians or not, default is self.default_is_radian
:param times: repeat times, 0 is infinite loop, default is 1
:param first_pause_time: sleep time at first, purpose is to cache the commands and plan continuous motion, default is 0.1s
:param repeat_pause_time: interval between repeated movements, unit: (s)second
:param automatic_calibration: automatic calibration or not, default is True
:param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
:param mvtime: 0, reserved
:param wait: whether to wait for the arm to complete, default is False
```

#### def __move_circle__(self, pose1, pose2, percent, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):

```
The motion calculates the trajectory of the space circle according to the three-point coordinates.
The three-point coordinates are (current starting point, pose1, pose2).

:param pose1: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
:param pose2: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
:param percent: the percentage of arc length and circumference of the movement
:param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
:param mvtime: 0, reserved
:param is_radian: roll/pitch/yaw value is radians or not, default is self.default_is_radian
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True
:param kwargs: reserved
:return: code
    code: See the API code documentation for details.
        code < 0: the last_used_tcp_speed/last_used_tcp_acc will not be modified
        code >= 0: the last_used_tcp_speed/last_used_tcp_acc will be modified
```

#### def __move_gohome__(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):

```
Move to go home (Back to zero), the API will modify self.last_used_position and self.last_used_angles value
Warnning: without limit detection
Note:
    1. The API will change self.last_used_position value into [201.5, 0, 140.5, -180, 0, 0]
    2. The API will change self.last_used_angles value into [0, 0, 0, 0, 0, 0, 0]
    3. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
        ex: code = arm.move_gohome(wait=True)
    4. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc

:param speed: gohome speed (unit: rad/s if is_radian is True else °/s), default is 50 °/s
:param mvacc: gohome acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is 5000 °/s^2
:param mvtime: reserved
:param is_radian: the speed and acceleration are in radians or not, default is self.default_is_radian
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True
:return: code
    code: See the API code documentation for details.
```

#### def __playback_trajectory__(self, times=1, filename=None, wait=True, double_speed=1):

```
Playback trajectory

Note:
    1. This interface relies on Firmware 1.2.0 or above

:param times: Number of playbacks,
    1. Only valid when the current position of the arm is the end position of the track, otherwise it will only be played once.
:param filename: The name of the trajectory to play back
    1. If filename is None, you need to manually call the `load_trajectory` to load the trajectory.
:param wait: whether to wait for the arm to complete, default is False
:param double_speed: double speed, only support 1/2/4, default is 1, only available if version > 1.2.11
:return: code
    code: See the API code documentation for details.
```

#### def __register_cmdnum_changed_callback__(self, callback=None):

```
Register the cmdnum changed callback, only available if enable_report is True

:param callback:
    callback data:
    {
        "cmdnum": cmdnum
    }
:return: True/False
```

#### def __register_connect_changed_callback__(self, callback=None):

```
Register the connect status changed callback

:param callback:
    callback data:
    {
        "connected": connected,
        "reported": reported,
    }
:return: True/False
```

#### def __register_count_changed_callback__(self, callback=None):

```
Register the counter value changed callback, only available if enable_report is True

:param callback:
    callback data:
    {
        "count": counter value
    }
:return: True/False
```

#### def __register_error_warn_changed_callback__(self, callback=None):

```
Register the error code or warn code changed callback, only available if enable_report is True

:param callback:
    callback data:
    {
        "error_code": error_code,
        "warn_code": warn_code,
    }
:return: True/False
```

#### def __register_mode_changed_callback__(self, callback=None):

```
Register the mode changed callback, only available if enable_report is True and the connect way is socket

:param callback:
    callback data:
    {
        "mode": mode,
    }
:return: True/False
```

#### def __register_mtable_mtbrake_changed_callback__(self, callback=None):

```
Register the motor enable states or motor brake states changed callback, only available if enable_report is True and the connect way is socket

:param callback:
    callback data:
    {
        "mtable": [motor-1-motion-enable, motor-2-motion-enable, ...],
        "mtbrake": [motor-1-brake-enable, motor-1-brake-enable,...],
    }
:return: True/False
```

#### def __register_report_callback__(self, callback=None, report_cartesian=True, report_joints=True, report_state=True, report_error_code=True, report_warn_code=True, report_mtable=True, report_mtbrake=True, report_cmd_num=True):

```
Register the report callback, only available if enable_report is True

:param callback:
    callback data:
    {
        'cartesian': [], # if report_cartesian is True
        'joints': [], # if report_joints is True
        'error_code': 0, # if report_error_code is True
        'warn_code': 0, # if report_warn_code is True
        'state': state, # if report_state is True
        'mtbrake': mtbrake, # if report_mtbrake is True, and available if enable_report is True and the connect way is socket
        'mtable': mtable, # if report_mtable is True, and available if enable_report is True and the connect way is socket
        'cmdnum': cmdnum, # if report_cmd_num is True
    }
:param report_cartesian: report cartesian or not, default is True
:param report_joints: report joints or not, default is True
:param report_state: report state or not, default is True
:param report_error_code: report error or not, default is True
:param report_warn_code: report warn or not, default is True
:param report_mtable: report motor enable states or not, default is True
:param report_mtbrake: report motor brake states or not, default is True
:param report_cmd_num: report cmdnum or not, default is True
:return: True/False
```

#### def __register_report_location_callback__(self, callback=None, report_cartesian=True, report_joints=True):

```
Register the report location callback, only available if enable_report is True

:param callback:
    callback data:
    {
        "cartesian": [x, y, z, roll, pitch, yaw], ## if report_cartesian is True
        "joints": [angle-1, angle-2, angle-3, angle-4, angle-5, angle-6, angle-7], ## if report_joints is True
    }
:param report_cartesian: report or not, True/False, default is True
:param report_joints: report or not, True/False, default is True
:return: True/False
```

#### def __register_state_changed_callback__(self, callback=None):

```
Register the state status changed callback, only available if enable_report is True

:param callback:
    callback data:
    {
        "state": state,
    }
:return: True/False
```

#### def __register_temperature_changed_callback__(self, callback=None):

```
Register the temperature changed callback, only available if enable_report is True

:param callback:
    callback data:
    {
        "temperatures": [servo-1-temperature, ...., servo-7-temperature]
    }
:return: True/False
```

#### def __release_cmdnum_changed_callback__(self, callback=None):

```
Release the cmdnum changed callback

:param callback:
:return: True/False
```

#### def __release_connect_changed_callback__(self, callback=None):

```
Release the connect changed callback

:param callback:
:return: True/False
```

#### def __release_count_changed_callback__(self, callback=None):

```
Release the counter value changed callback

:param callback:
:return: True/False
```

#### def __release_error_warn_changed_callback__(self, callback=None):

```
Release the error warn changed callback

:param callback:
:return: True/False
```

#### def __release_mode_changed_callback__(self, callback=None):

```
Release the mode changed callback

:param callback:
:return: True/False
```

#### def __release_mtable_mtbrake_changed_callback__(self, callback=None):

```
Release the motor enable states or motor brake states changed callback

:param callback:
:return: True/False
```

#### def __release_report_callback__(self, callback=None):

```
Release the report callback

:param callback:
:return: True/False
```

#### def __release_report_location_callback__(self, callback=None):

```
Release the location report callback

:param callback:
:return: True/False
```

#### def __release_state_changed_callback__(self, callback=None):

```
Release the state changed callback

:param callback:
:return: True/False
```

#### def __release_temperature_changed_callback__(self, callback=None):

```
Release the temperature changed callback

:param callback:
:return: True/False
```

#### def __reset__(self, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None):

```
Reset the xArm
Warnning: without limit detection
Note:
    1. The API will change self.last_used_position value into [201.5, 0, 140.5, -180, 0, 0]
    2. The API will change self.last_used_angles value into [0, 0, 0, 0, 0, 0, 0]
    3. If there are errors or warnings, this interface will clear the warnings and errors.
    4. If not ready, the api will auto enable motion and set state
    5. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc

:param speed: reset speed (unit: rad/s if is_radian is True else °/s), default is 50 °/s
:param mvacc: reset acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is 5000 °/s^2
:param mvtime: reserved
:param is_radian: the speed and acceleration are in radians or not, default is self.default_is_radian
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True
```

#### def __run_blockly_app__(self, path, **kwargs):

```
Run the app generated by xArmStudio software
:param path: app path
```

#### def __run_gcode_file__(self, path, **kwargs):

```
Run the gcode file
:param path: gcode file path
```

#### def __save_conf__(self):

```
Save config
Note:
    1. This interface can record the current settings and will not be lost after the restart.
    2. The clean_conf interface can restore system default settings

:return: code
    code: See the API code documentation for details.
```

#### def __save_record_trajectory__(self, filename, wait=True, timeout=2):

```
Save the trajectory you just recorded

Note:
    1. This interface relies on Firmware 1.2.0 or above

:param filename: The name to save
    1. Only strings consisting of English or numbers are supported, and the length is no more than 50.
    2. The trajectory is saved in the controller box.
    3. This action will overwrite the trajectory with the same name
    4. Empty the trajectory in memory after saving, so repeated calls will cause the recorded trajectory to be covered by an empty trajectory.
:param wait: Whether to wait for saving, default is True
:param timeout: Timeout waiting for saving to complete
:return: code
    code: See the API code documentation for details.
```

#### def __send_cmd_sync__(self, command=None):

```
Send cmd and wait (only waiting the cmd response, not waiting for the movement)
Note:
    1. Some command depends on self.default_is_radian

:param command:
    'G1': 'set_position(MoveLine): G1 X{x} Y{y} Z{z} A{roll} B{pitch} C{yaw} F{speed} Q{acc} T{mvtime}'
    'G2': 'move_circle: G2 X{x1} Y{y1} Z{z1} A{roll1} B{pitch1} C{yaw1} I{x2} J{y2} K{z2} L{roll2} M{pitch2} N{yaw2} F{speed} Q{acc} T{mvtime}'
    'G4': 'set_pause_time: G4 T{second}'
    'G7': 'set_servo_angle: G7 I{servo_1} J{servo_2} K{servo_3} L{servo_4} M{servo_5} N{servo_6} O{servo_7} F{speed} Q{acc} T{mvtime}'
    'G8': 'move_gohome: G8 F{speed} Q{acc} T{mvtime}'
    'G9': 'set_position(MoveArcLine): G9 X{x} Y{y} Z{z} A{roll} B{pitch} C{yaw} R{radius} F{speed} Q{acc} T{mvtime}'
    'G11': 'set_servo_angle_j: G11 I{servo_1} J{servo_2} K{servo_3} L{servo_4} M{servo_5} N{servo_6} O{servo_7} F{speed} Q{acc} T{mvtime}'
    'H1': 'get_version: H1'
    'H11': 'motion_enable: H11 I{servo_id} V{enable}'
    'H12': 'set_state: H12 V{state}'
    'H13': 'get_state: H13'
    'H14': 'get_cmdnum: H14'
    'H15': 'get_err_warn_code: H15'
    'H16': 'clean_error: H16'
    'H17': 'clean_warn: H17'
    'H18': 'set_servo_attach/set_servo_detach: H18 I{servo_id} V{1: enable(detach), 0: disable(attach)}'
    'H19': 'set_mode: H19 V{mode}'
    'H31': 'set_tcp_jerk: H31 V{jerk(mm/s^3)}'
    'H32': 'set_tcp_maxacc: H32 V{maxacc(mm/s^2)}'
    'H33': 'set_joint_jerk: H33 V{jerk(°/s^3 or rad/s^3)}'
    'H34': 'set_joint_maxacc: H34 {maxacc(°/s^2 or rad/s^2)}'
    'H35': 'set_tcp_offset: H35 X{x} Y{y} Z{z} A{roll} B{pitch} C{yaw}'
    'H36': 'set_tcp_load: H36 I{weight} J{center_x} K{center_y} L{center_z}'
    'H37': 'set_collision_sensitivity: H37 V{sensitivity}'
    'H38': 'set_teach_sensitivity: H38 V{sensitivity}'
    'H39': 'clean_conf: H39'
    'H40': 'save_conf: H40'
    'H41': 'get_position: H41'
    'H42': 'get_servo_angle: H42'
    'H43': 'get_inverse_kinematics: H43 X{x} Y{y} Z{z} A{roll} B{pitch} C{yaw}'
    'H44': 'get_forward_kinematics: H44 I{servo_1} J{servo_2} K{servo_3} L{servo_4} M{servo_5} N{servo_6} O{servo_7}'
    'H45': 'is_joint_limit: H45 I{servo_1} J{servo_2} K{servo_3} L{servo_4} M{servo_5} N{servo_6} O{servo_7}'
    'H46': 'is_tcp_limit: H46 X{x} Y{y} Z{z} A{roll} B{pitch} C{yaw}'
    'H51': 'set_gravity_direction: H51 X{x} Y{y} Z{z}'
    'H106': 'get_servo_debug_msg: H106'
    'M116': 'set_gripper_enable: M116 V{enable}'
    'M117': 'set_gripper_mode: M117 V{mode}'
    'M119': 'get_gripper_position: M119'
    'M120': 'set_gripper_position: M120 V{pos}'
    'M121': 'set_gripper_speed: M116 V{speed}'
    'M125': 'get_gripper_err_code: M125'
    'M126': 'clean_gripper_error: M126'
    'M131': 'get_tgpio_digital: M131'
    'M132': 'set_tgpio_digital: M132 I{ionum} V{value}'
    'M133': 'get_tgpio_analog, default ionum=0: M133 I{ionum=0}'
    'M134': 'get_tgpio_analog, default ionum=1: M134 I{ionum=1}'
    'C131': 'get_cgpio_digital: C131'
    'C132': 'get_cgpio_analog, default ionum=0: C132 I{ionum=0}'
    'C133': 'get_cgpio_analog, default ionum=1: C133 I{ionum=1}'
    'C134': 'set_cgpio_digital: C134 I{ionum} V{value}'
    'C135': 'set_cgpio_analog, default ionum=0: C135 I{ionum=0} V{value}'
    'C136': 'set_cgpio_analog, default ionum=1: C136 I{ionum=1} V{value}'
    'C137': 'set_cgpio_digital_input_function: C137 I{ionum} V{fun}'
    'C138': 'set_cgpio_digital_output_function: C138 I{ionum} V{fun}'
    'C139': 'get_cgpio_state: C139'
:return: code or tuple((code, ...))
    code: See the API code documentation for details.
```

#### def __set_cgpio_analog__(self, ionum, value):

```
Set the analog value of the specified Controller GPIO

:param ionum: 0 or 1
:param value: value
:return: code
    code: See the API code documentation for details.
```

#### def __set_cgpio_digital__(self, ionum, value, delay_sec=None):

```
Set the digital value of the specified Controller GPIO

:param ionum: 0~7
:param value: value
:param delay_sec: delay effective time from the current start, in seconds, default is None(effective immediately)
:return: code
    code: See the API code documentation for details.
```

#### def __set_cgpio_digital_input_function__(self, ionum, fun):

```
Set the digital input functional mode of the Controller GPIO
:param ionum: 0~7
:param fun: functional mode
    0: general input
    1: external emergency stop
    2: reversed, protection reset
    3: reversed, reduced mode
    4: reversed, operating mode
    5: reversed, three-state switching signal
    11: offline task
    12: teaching mode
:return: code
    code: See the API code documentation for details.
```

#### def __set_cgpio_digital_output_function__(self, ionum, fun):

```
Set the digital output functional mode of the specified Controller GPIO
:param ionum: 0~7
:param fun: functionnal mode
    0: general output
    1: emergency stop
    2: in motion
    11: has error
    12: has warn
    13: in collision
    14: in teaching
    15: in offline task
:return: code
    code: See the API code documentation for details.
```

#### def __set_cgpio_digital_with_xyz__(self, ionum, value, xyz, fault_tolerance_radius):

```
Set the digital value of the specified Controller GPIO when the robot has reached the specified xyz position           

:param ionum: 0 ~ 7
:param value: value
:param xyz: position xyz, as [x, y, z]
:param fault_tolerance_radius: fault tolerance radius
:return: code
    code: See the API code documentation for details.
```

#### def __set_collision_rebound__(self, on):

```
Set the collision rebound,turn on/off collision rebound

Note:
    1. This interface relies on Firmware 1.2.11 or above

:param on: True/False
:return: code
    code: See the API code documentation for details.
```

#### def __set_collision_sensitivity__(self, value):

```
Set the sensitivity of collision

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param value: sensitivity value, 0~5
:return: code
    code: See the API code documentation for details.
```

#### def __set_counter_increase__(self, val=1):

```
Set counter plus value, only support plus 1

:param val: reversed
:return: code
    code: See the API code documentation for details.
```

#### def __set_counter_reset__(self):

```
Reset counter value

:return: code
    code: See the API code documentation for details.
```

#### def __set_fence_mode__(self, on):

```
Set the fence mode,turn on/off fense mode

Note:
    1. This interface relies on Firmware 1.2.11 or above

:param on: True/False
:return: code
    code: See the API code documentation for details.
```

#### def __set_gravity_direction__(self, direction):

```
Set the direction of gravity

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param direction: direction of gravity, such as [x(mm), y(mm), z(mm)]
:return: code
    code: See the API code documentation for details.
```

#### def __set_gripper_enable__(self, enable, **kwargs):

```
Set the gripper enable

:param enable: enable or not
 Note： such as code = arm.set_gripper_enable(True)  #turn on the Gripper
:return: code
    code: See the Gripper code documentation for details.
```

#### def __set_gripper_mode__(self, mode, **kwargs):

```
Set the gripper mode

:param mode: 0: location mode
 Note： such as code = rm.set_gripper_mode(0)
:return: code
    code: See the Gripper code documentation for details.
```

#### def __set_gripper_position__(self, pos, wait=False, speed=None, auto_enable=False, timeout=None, **kwargs):

```
Set the gripper position

:param pos: pos
:param wait: wait or not, default is False
:param speed: speed,unit:r/min
:param auto_enable: auto enable or not, default is False
:param timeout: wait time, unit:second, default is 10s
:return: code
    code: See the Gripper code documentation for details.
```

#### def __set_gripper_speed__(self, speed, **kwargs):

```
Set the gripper speed

:param speed:
:return: code
    code: See the Gripper code documentation for details.
```

#### def __set_joint_jerk__(self, jerk, is_radian=None):

```
Set the jerk of Joint space
Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param jerk: jerk (°/s^3 or rad/s^3)
:param is_radian: the jerk in radians or not, default is self.default_is_radian
:return: code
    code: See the API code documentation for details.
```

#### def __set_joint_maxacc__(self, acc, is_radian=None):

```
Set the max acceleration of Joint space

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param acc: max acceleration (°/s^2 or rad/s^2)
:param is_radian: the jerk in radians or not, default is self.default_is_radian
:return: code
    code: See the API code documentation for details.
```

#### def __set_mode__(self, mode=0):

```
Set the xArm mode

:param mode: default is 0
    0: position control mode
    1: servo motion mode
        Note: the use of the set_servo_angle_j interface must first be set to this mode
        Note: the use of the set_servo_cartesian interface must first be set to this mode
    2: joint teaching mode
        Note: use this mode to ensure that the arm has been identified and the control box and arm used for identification are one-to-one.
    3: cartesian teaching mode (invalid)
:return: code
    code: See the API code documentation for details.
```

#### def __set_mount_direction__(self, base_tilt_deg, rotation_deg, is_radian=None):

```
Set the mount direction

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param base_tilt_deg: tilt degree
:param rotation_deg: rotation degree
:param is_radian: the base_tilt_deg/rotation_deg in radians or not, default is self.default_is_radian
:return: code
    code: See the API code documentation for details.
```

#### def __set_pause_time__(self, sltime, wait=False):

```
Set the arm pause time, xArm will pause sltime second

:param sltime: sleep time,unit:(s)second
:param wait: wait or not, default is False
:return: code
    code: See the API code documentation for details.
```

#### def __set_position__(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None, speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None, wait=False, timeout=None, **kwargs):

```
Set the cartesian position, the API will modify self.last_used_position value
Note:
    1. If it is xArm5, ensure that the current robotic arm has a roll value of 180° or π rad and has a roll value of 0 before calling this interface.
    2. If it is xArm5, roll must be set to 180° or π rad, pitch must be set to 0
    3. If the parameter(roll/pitch/yaw) you are passing is an radian unit, be sure to set the parameter is_radian to True.
        ex: code = arm.set_position(x=300, y=0, z=200, roll=-3.14, pitch=0, yaw=0, is_radian=True)
    4. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
        ex: code = arm.set_position(x=300, y=0, z=200, roll=180, pitch=0, yaw=0, is_radian=False, wait=True)
    5. This interface is only used in the base coordinate system.

:param x: cartesian position x, (unit: mm), default is self.last_used_position[0]
:param y: cartesian position y, (unit: mm), default is self.last_used_position[1]
:param z: cartesian position z, (unit: mm), default is self.last_used_position[2]
:param roll: rotate around the X axis, (unit: rad if is_radian is True else °), default is self.last_used_position[3]
:param pitch: rotate around the Y axis, (unit: rad if is_radian is True else °), default is self.last_used_position[4]
:param yaw: rotate around the Z axis, (unit: rad if is_radian is True else °), default is self.last_used_position[5]
:param radius: move radius, if radius is None or radius less than 0, will MoveLine, else MoveArcLine
    MoveLine: Linear motion
        ex: code = arm.set_position(..., radius=None)
    MoveArcLine: Linear arc motion with interpolation
        ex: code = arm.set_position(..., radius=0)
        Note: Need to set radius>=0
:param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
:param mvtime: 0, reserved
:param relative: relative move or not
:param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True
:param kwargs: reserved
:return: code
    code: See the API code documentation for details.
        code < 0: the last_used_position/last_used_tcp_speed/last_used_tcp_acc will not be modified
        code >= 0: the last_used_position/last_used_tcp_speed/last_used_tcp_acc will be modified
```

#### def __set_position_aa__(self, axis_angle_pose, speed=None, mvacc=None, mvtime=None, is_radian=None, is_tool_coord=False, relative=False, wait=False, timeout=None, **kwargs):

```
Set the pose represented by the axis angle pose

:param axis_angle_pose: the axis angle pose, [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]
:param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
:param mvtime: 0, reserved 
:param is_radian: the rx/ry/rz of axis_angle_pose in radians or not, default is self.default_is_radian
:param is_tool_coord: is tool coordinate or not
:param relative: relative move or not
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True
:return: code
    code: See the API code documentation for details.
```

#### def __set_reduced_joint_range__(self, joint_range, is_radian=None):

```
Set the joint range of the reduced mode

Note:
    1. This interface relies on Firmware 1.2.11 or above
    2. Only reset the reduced mode to take effect (`set_reduced_mode(True)`)

:param joint_range: [joint-1-min, joint-1-max, ..., joint-7-min, joint-7-max]
:param is_radian: the param joint_range are in radians or not, default is self.default_is_radian
:return:
```

#### def __set_reduced_max_joint_speed__(self, speed, is_radian=None):

```
Set the maximum joint speed of the reduced mode

Note:
    1. This interface relies on Firmware 1.2.0 or above
    2. Only reset the reduced mode to take effect (`set_reduced_mode(True)`)

:param speed: speed (°/s or rad/s)
:param is_radian: the speed is in radians or not, default is self.default_is_radian
:return: code
    code: See the API code documentation for details.
```

#### def __set_reduced_max_tcp_speed__(self, speed):

```
Set the maximum tcp speed of the reduced mode

Note:
    1. This interface relies on Firmware 1.2.0 or above
    2. Only reset the reduced mode to take effect (`set_reduced_mode(True)`)

:param speed: speed (mm/s)
:return: code
    code: See the API code documentation for details.
```

#### def __set_reduced_mode__(self, on):

```
Turn on/off reduced mode

Note:
    1. This interface relies on Firmware 1.2.0 or above

:param on: True/False
           such as:Turn on the reduced mode : code=arm.set_reduced_mode(True)
:return: code
    code: See the API code documentation for details.
```

#### def __set_reduced_tcp_boundary__(self, boundary):

```
Set the boundary of the safety boundary mode

Note:
    1. This interface relies on Firmware 1.2.0 or above
    2. Only reset the reduced mode to take effect (`set_reduced_mode(True)`)

:param boundary: [x_max, x_min, y_max, y_min, z_max, z_min]
:return: code
    code: See the API code documentation for details.
```

#### def __set_servo_angle__(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None, relative=False, is_radian=None, wait=False, timeout=None, **kwargs):

```
Set the servo angle, the API will modify self.last_used_angles value
Note:
    1. If the parameter angle you are passing is an radian unit, be sure to set the parameter is_radian to True.
        ex: code = arm.set_servo_angle(servo_id=1, angle=1.57, is_radian=True)
    2. If you want to wait for the robot to complete this action and then return, please set the parameter wait to True.
        ex: code = arm.set_servo_angle(servo_id=1, angle=45, is_radian=False,wait=True)
    3. This interface is only used in the base coordinate system.

:param servo_id: 1-(Number of axes), None(8)
    1. 1-(Number of axes) indicates the corresponding joint, the parameter angle should be a numeric value
        ex: code = arm.set_servo_angle(servo_id=1, angle=45, is_radian=False)
    2. None(8) means all joints, default is None, the parameter angle should be a list of values whose length is the number of joints
        ex: code = arm.set_servo_angle(angle=[30, -45, 0, 0, 0, 0, 0], is_radian=False)
:param angle: angle or angle list, (unit: rad if is_radian is True else °)
    1. If servo_id is 1-(Number of axes), angle should be a numeric value
        ex: code = arm.set_servo_angle(servo_id=1, angle=45, is_radian=False)
    2. If servo_id is None or 8, angle should be a list of values whose length is the number of joints
        like [axis-1, axis-2, axis-3, axis-3, axis-4, axis-5, axis-6, axis-7]
        ex: code = arm.set_servo_angle(angle=[30, -45, 0, 0, 0, 0, 0], is_radian=False)
:param speed: move speed (unit: rad/s if is_radian is True else °/s), default is self.last_used_joint_speed
:param mvacc: move acceleration (unit: rad/s^2 if is_radian is True else °/s^2), default is self.last_used_joint_acc
:param mvtime: 0, reserved
:param relative: relative move or not
:param is_radian: the angle in radians or not, default is self.default_is_radian
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True
:param kwargs: reserved
:return: code
    code: See the API code documentation for details.
        code < 0: the last_used_angles/last_used_joint_speed/last_used_joint_acc will not be modified
        code >= 0: the last_used_angles/last_used_joint_speed/last_used_joint_acc will be modified
```

#### def __set_servo_angle_j__(self, angles, speed=None, mvacc=None, mvtime=None, is_radian=None, **kwargs):

```
Set the servo angle, execute only the last instruction, need to be set to servo motion mode(self.set_mode(1))
Note:
    1. This interface does not modify the value of last_used_angles/last_used_joint_speed/last_used_joint_acc
    2. This interface is only used in the base coordinate system.

:param angles: angle list, (unit: rad if is_radian is True else °)
:param speed: speed, reserved
:param mvacc: acceleration, reserved
:param mvtime: 0, reserved
:param is_radian: the angles in radians or not, defalut is self.default_is_radian
:param kwargs: reserved
:return: code
    code: See the API code documentation for details.
```

#### def __set_servo_attach__(self, servo_id=None):

```
Attach the servo

:param servo_id: 1-(Number of axes), 8, if servo_id is 8, will attach all servo
    1. 1-(Number of axes): attach only one joint
        ex: arm.set_servo_attach(servo_id=1)
    2: 8: attach all joints
        ex: arm.set_servo_attach(servo_id=8)
:return: code
    code: See the API code documentation for details.
```

#### def __set_servo_cartesian__(self, mvpose, speed=None, mvacc=None, mvtime=0, is_radian=None, is_tool_coord=False, **kwargs):

```
Set the servo cartesian, execute only the last instruction, need to be set to servo motion mode(self.set_mode(1))
Note:
    1. only available if firmware_version >= 1.4.0
    2. This interface is only used in the base coordinate system.

:param mvpose: cartesian position, [x(mm), y(mm), z(mm), roll(rad or °), pitch(rad or °), yaw(rad or °)]
:param speed: move speed (mm/s), reserved
:param mvacc: move acceleration (mm/s^2), reserved
:param mvtime: 0, reserved
:param is_radian: the roll/pitch/yaw of mvpose in radians or not, default is self.default_is_radian
:param is_tool_coord: is tool coordinate or not
:param kwargs: reserved
:return: code
    code: See the API code documentation for details.
```

#### def __set_servo_cartesian_aa__(self, axis_angle_pose, speed=None, mvacc=None, is_radian=None, is_tool_coord=False, relative=False, **kwargs):

```
Set the servo cartesian represented by the axis angle pose, execute only the last instruction, need to be set to servo motion mode(self.set_mode(1))
Note:
    1. only available if firmware_version >= 1.4.7

:param axis_angle_pose: the axis angle pose, [x(mm), y(mm), z(mm), rx(rad or °), ry(rad or °), rz(rad or °)]
:param speed: move speed (mm/s), reserved
:param mvacc: move acceleration (mm/s^2), reserved
:param is_radian: the rx/ry/rz of axis_angle_pose in radians or not, default is self.default_is_radian
:param is_tool_coord: is tool coordinate or not
:param relative: relative move or not
:return: code
    code: See the API code documentation for details.
```

#### def __set_servo_detach__(self, servo_id=None):

```
Detach the servo, be sure to do protective work before unlocking to avoid injury or damage.

:param servo_id: 1-(Number of axes), 8, if servo_id is 8, will detach all servo
    1. 1-(Number of axes): detach only one joint
        ex: arm.set_servo_detach(servo_id=1)
    2: 8: detach all joints, please
        ex: arm.set_servo_detach(servo_id=8)
:return: code
    code: See the API code documentation for details.
```

#### def __set_state__(self, state=0):

```
Set the xArm state

:param state: default is 0
    0: sport state
    3: pause state
    4: stop state
:return: code
    code: See the API code documentation for details.
```

#### def __set_tcp_jerk__(self, jerk):

```
Set the translational jerk of Cartesian space
Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param jerk: jerk (mm/s^3)
:return: code
    code: See the API code documentation for details.
```

#### def __set_tcp_load__(self, weight, center_of_gravity):

```
Set the end load of xArm

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param weight: load weight (unit: kg)
:param center_of_gravity: load center of gravity, such as [x(mm), y(mm), z(mm)]
:return: code
    code: See the API code documentation for details.
```

#### def __set_tcp_maxacc__(self, acc):

```
Set the max translational acceleration of Cartesian space
Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param acc: max acceleration (mm/s^2)
:return: code
    code: See the API code documentation for details.
```

#### def __set_tcp_offset__(self, offset, is_radian=None):

```
Set the tool coordinate system offset at the end
Note:
    1. Do not use if not required
    2. If not saved and you want to revert to the last saved value, please reset the offset by set_tcp_offset([0, 0, 0, 0, 0, 0])
    3. If not saved, it will be lost after reboot
    4. The save_conf interface can record the current settings and will not be lost after the restart.
    5. The clean_conf interface can restore system default settings

:param offset: [x, y, z, roll, pitch, yaw]
:param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian
:return: code
    code: See the API code documentation for details.
```

#### def __set_teach_sensitivity__(self, value):

```
Set the sensitivity of drag and teach

Note:
    1. Do not use if not required
    2. If not saved, it will be lost after reboot
    3. The save_conf interface can record the current settings and will not be lost after the restart.
    4. The clean_conf interface can restore system default settings

:param value: sensitivity value, 1~5
:return: code
    code: See the API code documentation for details.
```

#### def __set_tgpio_digital__(self, ionum, value, delay_sec=None):

```
Set the digital value of the specified Tool GPIO

:param ionum: 0 or 1
:param value: value
:param delay_sec: delay effective time from the current start, in seconds, default is None(effective immediately)
:return: code
    code: See the API code documentation for details.
```

#### def __set_tgpio_digital_with_xyz__(self, ionum, value, xyz, fault_tolerance_radius):

```
Set the digital value of the specified Tool GPIO when the robot has reached the specified xyz position           

:param ionum: 0 or 1
:param value: value
:param xyz: position xyz, as [x, y, z]
:param fault_tolerance_radius: fault tolerance radius
:return: code
    code: See the API code documentation for details.
```

#### def __set_tool_position__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, speed=None, mvacc=None, mvtime=None, is_radian=None, wait=False, timeout=None, **kwargs):

```
Movement relative to the tool coordinate system
Note:
    1. This interface is moving relative to the current tool coordinate system
    2. The tool coordinate system is not fixed and varies with position.
    3. This interface is only used in the tool coordinate system.


:param x: the x coordinate relative to the current tool coordinate system, (unit: mm), default is 0
:param y: the y coordinate relative to the current tool coordinate system, (unit: mm), default is 0
:param z: the z coordinate relative to the current tool coordinate system, (unit: mm), default is 0
:param roll: the rotate around the X axis relative to the current tool coordinate system, (unit: rad if is_radian is True else °), default is 0
:param pitch: the rotate around the Y axis relative to the current tool coordinate system, (unit: rad if is_radian is True else °), default is 0
:param yaw: the rotate around the Z axis relative to the current tool coordinate system, (unit: rad if is_radian is True else °), default is 0
:param speed: move speed (mm/s, rad/s), default is self.last_used_tcp_speed
:param mvacc: move acceleration (mm/s^2, rad/s^2), default is self.last_used_tcp_acc
:param mvtime: 0, reserved
:param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian
:param wait: whether to wait for the arm to complete, default is False
:param timeout: maximum waiting time(unit: second), default is None(no timeout), only valid if wait is True
:param kwargs: reserved
:return: code
    code: See the API code documentation for details.
        code < 0: the last_used_tcp_speed/last_used_tcp_acc will not be modified
        code >= 0: the last_used_tcp_speed/last_used_tcp_acc will be modified
```

#### def __set_vacuum_gripper__(self, on, wait=False, timeout=3, delay_sec=None):

```
Set vacuum gripper state

:param on: open or not
    on=True: equivalent to calling `set_tgpio_digital(0, 1)` and `set_tgpio_digital(1, 0)`
    on=False: equivalent to calling `set_tgpio_digital(0, 0)` and `set_tgpio_digital(1, 1)`
:param wait: wait or not, default is False
:param timeout: wait time, unit:second, default is 3s
:param delay_sec: delay effective time from the current start, in seconds, default is None(effective immediately)
:return: code
    code: See the API code documentation for details.
```

#### def __set_world_offset__(self, offset, is_radian=None):

```
Set the base coordinate offset

Note:
    1. This interface relies on Firmware 1.2.11 or above

:param offset: [x, y, z, roll, pitch, yaw]
:param is_radian: the roll/pitch/yaw in radians or not, default is self.default_is_radian
:return: code
    code: See the API code documentation for details.
```

#### def __shutdown_system__(self, value=1):

```
Shutdown the xArm controller system

:param value: 1: remote shutdown
:return: code
    code: See the API code documentation for details.
```

#### def __start_record_trajectory__(self):

```
Start trajectory recording, only in teach mode, so you need to set joint teaching mode before.

Note:
    1. This interface relies on Firmware 1.2.0 or above
    2. set joint teaching mode: set_mode(2);set_state(0)

:return: code
    code: See the API code documentation for details.
```

#### def __stop_record_trajectory__(self, filename=None):

```
Stop trajectory recording

Note:
    1. This interface relies on Firmware 1.2.0 or above

:param filename: The name to save
    1. Only strings consisting of English or numbers are supported, and the length is no more than 50.
    2. The trajectory is saved in the controller box.
    3. If the filename is None, just stop recording, do not save, you need to manually call `save_record_trajectory` save before changing the mode. otherwise it will be lost
    4. This action will overwrite the trajectory with the same name
    5. Empty the trajectory in memory after saving
:return: code
    code: See the API code documentation for details.
```
