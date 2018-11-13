xArm-Python-SDK API Documentation: class XArmAPI in module xarm.wrapper.xarm_api

## class __XArmAPI__
****************************************

### __descriptors__
****************************************
#### __angles__
Servo angles
:return: [angle1(radian), angle2(radian), angle3(radian), angle4(radian), angle5(radian), angle6(radian), angle7(radian)]

#### __axis__
Axis number, only available in socket way and enable_report is True and report_type is 'rich'

#### __cmd_num__
Number of command caches

#### __connected__
Connection status

#### __device_type__
Device type, only available in socket way and  enable_report is True and report_type is 'rich'

#### __error_code__
Error code

#### __has_err_warn__
xArm has error warn or not
:return: True/False

#### __maable__
Servo enable state, only available in socket way and enable_report is True
:return: [servo-1, servo-2, servo-3, servo-4, servo-5, servo-6, servo-7, reserved]

#### __master_id__
Master id, only available in socket way and enable_report is True and report_type is 'rich'

#### __mtbrake__
Servo brake state, only available in socket way and enable_report is True
:return: [servo-1, servo-2, servo-3, servo-4, servo-5, servo-6, servo-7, reserved]

#### __position__
Cartesion position
return: [x(mm), y(mm), z(mm), roll(radian), yaw(radian), pitch(radian)]

#### __position_offset__
Cartesion position offset, only available in socket way and enable_report is True 
:return: [x_offset(mm), y_offset(mm), z_offset(mm), roll_offset(radian), yaw_offset(radian), pitch_offset(radian)]

#### __slave_id__
Slave id, only available in socket way and enable_report is True and report_type is 'rich'

#### __state__
xArm state
:return: 0: in motion, 1: sleeping, 2: suspended, 3: stopping

#### __version__
xArm version

#### __warn_code__
Warn code

****************************************
### __methods__
****************************************
#### def __\__init__\__(self, port=None, baudrate=921600, timeout=None, filters=None, enable_heartbeat=True, enable_report=True, report_type='normal', do_not_open=False, limit_velo=None, limit_acc=None, limit_angle_velo=None, limit_angle_acc=None):

```
The API wrapper of xArm
:param port: port name(such as 'COM5'/'/dev/ttyUSB0') or ip-address(such as '192.168.1.185')
:param baudrate: baudrate, only available in serial way
:param timeout: timeout, only available in serial way
:param filters: filters, reserved.
:param enable_heartbeat: whether to enable heartbeat, default is True, only available in socket way
:param enable_report: whether to enable report, default is True
:param report_type: report type('normal'/'real'/'rich'), only available in socket way
:param do_not_open: do not open, default is False
:param limit_velo: limit velo, default is [0, 1000] mm/s
:param limit_acc: limit acc, default is [0, 100000] mm/s^2
:param limit_angle_velo: limit angle velo, default is [1, 180] °/s
:param limit_angle_acc: limit angle acc, default is [1, 100000] °/s^2
```

#### def __clean_conf__(self):

```
Clean config
:return: code
```

#### def __clean_error__(self):

```
Clean the error, need to be manually enabled motion and set state after clean error
:return: code
```

#### def __clean_gripper_error__(self):

```
Clean the gripper error
:return: code
```

#### def __clean_warn__(self):

```
Clean the warn
:return: code
```

#### def __connect__(self, port=None, baudrate=None, timeout=None):

```
Connect to xArm
:param port: port name or the ip address
:param baudrate: baudrate, only available in serial way
:param timeout: timeout, only available in serial way
```

#### def __disconnect__(self):

```
Disconnect
```

#### def __emergency_stop__(self):

```
Emergency stop
```

#### def __get_cmdnum__(self):

```
Get the cmd count in cache
:return: tuple((code, cmd num)), only when code is 0, the returned result is correct.
```

#### def __get_err_warn_code__(self):

```
Get the error and warn code
:return: tuple((code, [error_code, warn_code])), only when code is 0, the returned result is correct.
```

#### def __get_fk__(self, angles, is_radian=True):

```
Positive kinematics, do not use, just for debugging
:param angles: 
:param is_radian: 
:return: tuple((code, pose)), only when code is 0, the returned result is correct.
```

#### def __get_gripper_err_code__(self):

```
Get the gripper error code
:return: tuple((code, err_code)), only when code is 0, the returned result is correct.
```

#### def __get_gripper_position__(self):

```
Get the gripper position
:return: tuple((code, pos)), only when code is 0, the returned result is correct.
```

#### def __get_ik__(self, pose, is_radian=True):

```
Inverse kinematics, do not use, just for debugging
:param pose: 
:param is_radian:
:return: tuple((code, angles)), only when code is 0, the returned result is correct.
```

#### def __get_is_moving__(self):

```
Check xArm is moving or not
:return: True/False
```

#### def __get_params__(self):


#### def __get_position__(self, is_radian=True):

```
Get the cartesian position
:param is_radian: roll/yaw/pitch value is radian or not
:return: tuple((code, [x, y, z, roll, yaw, pitch])), only when code is 0, the returned result is correct.
```

#### def __get_servo_angle__(self, servo_id=None, is_radian=True):

```
Get the servo angle
:param servo_id: 1-7, None(8), default is None
:param is_radian: return radian or not
:return: tuple((code, angle list if servo_id is None or 8 else angle)), only when code is 0, the returned result is correct.
```

#### def __get_state__(self):

```
Get state
:return: tuple((code, state)), only when code is 0, the returned result is correct.
    state:
        1: moving
        2: sleeping
        3: suspended
        4: stopping
```

#### def __get_version__(self):

```
Get the xArm version
:return: tuple((code, version)), only when code is 0, the returned result is correct.
```

#### def __is_joint_limit__(self, joint, is_radian=True):

```
Check the joint is in limit
:param joint: angle list
:param is_radian: angle value is radian or not
:return: tuple((code, limit)), only when code is 0, the returned result is correct.
```

#### def __is_tcp_limit__(self, pose, is_radian=True):

```
Check the tcp pose is in limit
:param pose: [x, y, z, roll, yaw, pitch]
:param is_radian: roll/yaw/pitch value is radian or not
:return: tuple((code, limit)), only when code is 0, the returned result is correct.
```

#### def __motion_enable__(self, enable=True, servo_id=None):

```
Motion enable
:param enable: 
:param servo_id: 1-7, None(8)
:return: code
```

#### def __move_gohome__(self, speed=None, mvacc=None, mvtime=None, is_radian=True, wait=False, timeout=None):

```
Move to go home (Back to zero)
:param speed: reserved
:param mvacc: reserved
:param mvtime: reserved
:param is_radian: reserved
:param wait: if True will wait the robot stop
:param timeout: second，default is 10s
:return: code
```

#### def __register_cmdnum_changed_callback__(self, callback=None):

```
Register the cmdnum changed callback, only available if enable_report is True
:param callback: 
    callback data:
    {
        "cmdnum": cmdnum
    }
:return:
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
:return:
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
:return:
```

#### def __register_maable_mtbrake_changed_callback__(self, callback=None):

```
Register the maable or mtbrake status changed callback, only available if enable_report is True and the connect way is socket
:param callback: 
    callback data:
    {
        "maable": [axis-1-motion-enable, axis-2-motion-enable, ...],
        "mtbrake": [axis-1-brake-enable, axis-1-brake-enable,...],
    }
:return:
```

#### def __register_report_callback__(self, callback=None, report_cartesian=True, report_joints=True, report_state=True, report_error_code=True, report_warn_code=True, report_maable=True, report_mtbrake=True, report_cmd_num=True):

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
        'maable': maable, # if report_maable is True, and available if enable_report is True and the connect way is socket
        'cmdnum': cmdnum, # if report_cmd_num is True
    }
:param report_cartesian: report cartesian or not, default is True
:param report_joints: report joints or not, default is True
:param report_state: report state or not, default is True
:param report_error_code: report error or not, default is True
:param report_warn_code: report warn or not, default is True
:param report_maable: report maable or not, default is True
:param report_mtbrake: report mtbrake or not, default is True
:param report_cmd_num: report cmdnum or not, default is True
:return:
```

#### def __register_report_location_callback__(self, callback=None, report_cartesian=True, report_joints=True):

```
Register the report location callback, only available if enable_report is True
:param callback: 
    callback data:
    {
        "cartesian": [x, y, z, roll, yaw, pitch], ## if report_cartesian is True
        "joints": [angle-1, angle-2, angle-3, angle-4, angle-5, angle-6, angle-7], ## if report_joints is True
    }
:param report_cartesian: report or not, True/False, default is True
:param report_joints: report or not, True/False, default is True
:return:
```

#### def __register_state_changed_callback__(self, callback=None):

```
Register the state status changed callback, only available if enable_report is True
:param callback:
    callback data:
    {
        "state": state,
    }
:return:
```

#### def __release_cmdnum_changed_callback__(self, callback=None):

```
Release the cmdnum changed callback
:param callback: 
:return:
```

#### def __release_connect_changed_callback__(self, callback=None):

```
Release the connect changed callback
:param callback: 
:return:
```

#### def __release_error_warn_changed_callback__(self, callback=None):

```
Release the error warn changed callback
:param callback: 
:return:
```

#### def __release_maable_mtbrake_changed_callback__(self, callback=None):

```
Release the maable or mtbrake changed callback
:param callback: 
:return:
```

#### def __release_report_callback__(self, callback=None):

```
Release the report callback
:param callback: 
:return:
```

#### def __release_report_location_callback__(self, callback=None):

```
Release the location report callback
:param callback: 
:return:
```

#### def __release_state_changed_callback__(self, callback=None):

```
Release the state changed callback
:param callback: 
:return:
```

#### def __reset__(self, speed=None, is_radian=False, wait=False, timeout=None):

```
Reset the xArm (motion enable -> set state -> back to zero)
:param speed: reserved
:param is_radian: reserved
:param wait: if True will wait the robot stop
:param timeout: second，default is 10s
```

#### def __save_conf__(self):

```
Save config
:return: code
```

#### def __send_cmd_sync__(self, command=None):

```
Send cmd and wait (only waiting the cmd response, not waiting for the movement)
:param command:
```

#### def __set_gripper_enable__(self, enable):

```
Set the gripper enable
:param enable: 
:return: code
```

#### def __set_gripper_mode__(self, mode):

```
Set the gripper mode
:param mode: 1: location mode, 2: speed mode (no use), 3: torque mode (no use)
:return: code
```

#### def __set_gripper_position__(self, pos, wait=False, speed=None, auto_enable=False, timeout=None):

```
Set the gripper position
:param pos: pos
:param wait: wait or not
:param speed: speed
:param auto_enable: auto enable or not
:param timeout: timeout
:return: code
```

#### def __set_gripper_speed__(self, speed):

```
Set the gripper speed
:param speed: 
:return: code
```

#### def __set_joint_jerk__(self, jerk):

```
Set joint jerk, do not use, just for debugging
:param jerk: 
:return: code
```

#### def __set_joint_maxacc__(self, acc):

```
Set joint maxacc, do not use, just for debugging
:param acc: 
:return: code
```

#### def __set_params__(self, **kwargs):


#### def __set_position__(self, x=None, y=None, z=None, roll=None, yaw=None, pitch=None, radius=None, speed=None, mvacc=None, mvtime=None, relative=False, is_radian=True, wait=False, timeout=None, **kwargs):

```
Set the cartesian position
:param x: cartesian position x, (unit: mm)
:param y: cartesian position y, (unit: mm)
:param z: cartesian position z, (unit: mm)
:param roll: cartesian roll, (unit: radian if is_radian is True else °)
:param yaw: cartesian yaw, (unit: radian if is_radian is True else °)
:param pitch: cartesian pitch, (unit: radian if is_radian is True else °)
:param radius: move radius, if radius is None, will move line, else move arc line
:param speed: move speed (mm/s, radian/s)
:param mvacc: move acc (mm/s^2, radian/s^2)
:param mvtime: 0, reserved
:param relative: relative move or not
:param is_radian: roll/yaw/pitch value is radian or not 
:param wait: if True will wait the robot stop
:param timeout: second，default is 10s
:param kwargs: reserved
:return: code
```

#### def __set_servo_angle__(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None, relative=False, is_radian=True, wait=False, timeout=None, **kwargs):

```
Set the servo angle
:param servo_id: 1-7, None(8)
:param angle: angle or angle list, (unit: radian if is_radian is True else °)
:param speed: move speed (unit: radian/s if is_radian is True else °/s)
:param mvacc: move acc (unit: radian/s^2 if is_radian is True else °/s^2)
:param mvtime: 0, reserved
:param relative: relative move or not
:param is_radian: angle value is radian or not
:param wait: if True will wait the robot stop
:param timeout: second，default is 10s
:param kwargs: reserved
:return: code
```

#### def __set_servo_attach__(self, servo_id=None):

```
Attach the servo
:param servo_id: 1-7, 8, if servo_id is 8, will attach all servo
:return: code
```

#### def __set_servo_detach__(self, servo_id=None):

```
Detach the servo
:param servo_id: 1-7, 8, if servo_id is 8, will detach all servo
:return: code
```

#### def __set_sleep_time__(self, sltime, wait=False):

```
Set the sleep time, xArm will sleep sltime second
:param sltime: sleep second
:param wait: wait or not
:return: code
```

#### def __set_state__(self, state=0):

```
Set the xArm state
:param state: default is 0
    0: sport state
    3: pause state
    4: stop state
:return: code
```

#### def __set_tcp_jerk__(self, jerk):

```
Set tcp jerk, do not use, just for debugging
:param jerk: 
:return: code
```

#### def __set_tcp_maxacc__(self, acc):

```
Set tcp maxacc, do not use, just for debugging
:param acc: 
:return: code
```

#### def __set_tcp_offset__(self, offset):

```
Set tcp offset, do not use, just for debugging
:param offset: 
:return: code
```
