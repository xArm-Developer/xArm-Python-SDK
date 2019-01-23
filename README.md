# xArm-Python-SDK

## Overview
xArm Python SDK

## Update summary for 0.1.1
- Support GPIO interface

## Update Summary for 0.1.0
- Compatible with 5, 6, and 7 axis robot arms

## Update Summary for 0.0.13
- Supports trajectory repeat motion interface with circular interpolation, supports repetition times, calibration
- Increase joint limits, attitude angle limits and cmd cache limits
- Exchange the parameters of the attitude angle yaw and the attitude angle pitch, but retain the parameter position

## Update Summary for 0.0.12
- By specifying the value of the default parameter of the interface in the instantiation parameter, using angle or radians
- Unify all interfaces by default using angle or radians
- More interface routines
- More detailed interface documentation
- Richer interface
- Set interface alias
- Modified the default values of the default parameters of some interfaces, but support the use of parameters to be compatible when instantiating

## Update Summary for 0.0.11
- Support serial port and TCP connection
- Support parameter to enable reporting
- Support callback register and release
- Support Gripper setting
- Support servo setting (Some interfaces are limited to professional debugging)
- Unified return value
- Snaps an exception and returns the specified return value


## Caution
- There is currently no collision detection, so try not to be close during use.
- During use, people should stay away from the robot arm to avoid accidental injury or damage to other items by the robot arm.
- Protect the arm before use.
- Before you exercise, please make sure you don't encounter obstacles.
- Protect the arm before unlocking the motor.

## Installation
Install is not necessary, you can run examples without installation.
- download

  ``` git clone git@github.com:xArm-Developer/xArm-Python-SDK.git```
- install

  ``` python setup.py install ```

## Doc
- #### [API](doc/api/xarm_api.md)
- #### [API Code](doc/api/xarm_api_code.md)

## Example
- #### [xArm](example/wrapper/)
- ##### 0001-connect-with-serial --> [xarm5](example/wrapper/xarm5/0001-connect_with_serial.py) --- [xarm6](example/wrapper/xarm6/0001-connect_with_serial.py) --- [xarm7](example/wrapper/xarm7/0001-connect_with_serial.py)
- ##### 0002-connect_with_socket --> [xarm5](example/wrapper/xarm5/0002-connect_with_socket.py) --- [xarm6](example/wrapper/xarm6/0002-connect_with_socket.py) --- [xarm7](example/wrapper/xarm7/0002-connect_with_socket.py)
- ##### 0003-event_register --> [xarm5](example/wrapper/xarm5/0003-event_register.py) --- [xarm6](example/wrapper/xarm6/0003-event_register.py) --- [xarm7](example/wrapper/xarm7/0003-event_register.py)
- ##### 0004-handle_error_warn --> [xarm5](example/wrapper/xarm5/0004-handle_error_warn.py) --- [xarm6](example/wrapper/xarm6/0004-handle_error_warn.py) --- [xarm7](example/wrapper/xarm7/0004-handle_error_warn.py)
- ##### 0005-get_property --> [xarm5](example/wrapper/xarm5/0005-get_property.py) --- [xarm6](example/wrapper/xarm6/0005-get_property.py) --- [xarm7](example/wrapper/xarm7/0005-get_property.py)
- ##### 0006-get --> [xarm5](example/wrapper/xarm5/0006-get.py) --- [xarm6](example/wrapper/xarm6/0006-get.py) --- [xarm7](example/wrapper/xarm7/0006-get.py)
- ##### 0007-servo_attach_detach --> [xarm5](example/wrapper/xarm5/0007-servo_attach_detach.py) --- [xarm6](example/wrapper/xarm6/0007-servo_attach_detach.py) --- [xarm7](example/wrapper/xarm7/0007-servo_attach_detach.py)
- ##### 0008-send_cmd --> [xarm5](example/wrapper/xarm5/0008-send_cmd.py) --- [xarm6](example/wrapper/xarm6/0008-send_cmd.py) --- [xarm7](example/wrapper/xarm7/0008-send_cmd.py)
- ##### 0009-set_gripper--> [xarm5](example/wrapper/xarm5/0009-set_gripper.py) --- [xarm6](example/wrapper/xarm6/0009-set_gripper.py) --- [xarm7](example/wrapper/xarm7/0009-set_gripper.py)
- ##### 0101-move_line --> [xarm5](example/wrapper/xarm5/0101-move_line.py) --- [xarm6](example/wrapper/xarm6/0101-move_line.py) --- [xarm7](example/wrapper/xarm7/0101-move_line.py)
- ##### 0102-move_line --> [xarm5](example/wrapper/xarm5/0102-move_line.py) --- [xarm6](example/wrapper/xarm6/0102-move_line.py) --- [xarm7](example/wrapper/xarm7/0102-move_line.py)
- ##### 0103-move_line --> [xarm5](example/wrapper/xarm5/0103-move_line.py) --- [xarm6](example/wrapper/xarm6/0103-move_line.py) --- [xarm7](example/wrapper/xarm7/0103-move_line.py)
- ##### 0104-move_line --> [xarm5](example/wrapper/xarm5/0104-move_line.py) --- [xarm6](example/wrapper/xarm6/0104-move_line.py) --- [xarm7](example/wrapper/xarm7/0104-move_line.py)
- ##### 0105-relative_move_line --> [xarm5](example/wrapper/xarm5/0105-relative_move_line.py) --- [xarm6](example/wrapper/xarm6/0105-relative_move_line.py) --- [xarm7](example/wrapper/xarm7/0105-relative_move_line.py)
- ##### 0106-move_arc_line --> [xarm5](example/wrapper/xarm5/0106-move_arc_line.py) --- [xarm6](example/wrapper/xarm6/0106-move_arc_line.py) --- [xarm7](example/wrapper/xarm7/0106-move_arc_line.py)
- ##### 0107-move_arc_line --> [xarm5](example/wrapper/xarm5/0107-move_arc_line.py) --- [xarm6](example/wrapper/xarm6/0107-move_arc_line.py) --- [xarm7](example/wrapper/xarm7/0107-move_arc_line.py)
- ##### 0201-move_joint --> [xarm5](example/wrapper/xarm5/0201-move_joint.py) --- [xarm6](example/wrapper/xarm6/0201-move_joint.py) --- [xarm7](example/wrapper/xarm7/0201-move_joint.py)
- ##### 0202-move_joint --> [xarm5](example/wrapper/xarm5/0202-move_joint.py) --- [xarm6](example/wrapper/xarm6/0202-move_joint.py) --- [xarm7](example/wrapper/xarm7/0202-move_joint.py)
- ##### 0203-move_joint --> [xarm5](example/wrapper/xarm5/0203-move_joint.py) --- [xarm6](example/wrapper/xarm6/0203-move_joint.py) --- [xarm7](example/wrapper/xarm7/0203-move_joint.py)
- ##### 0204-move_joint --> [xarm5](example/wrapper/xarm5/0204-move_joint.py) --- [xarm6](example/wrapper/xarm6/0204-move_joint.py) --- [xarm7](example/wrapper/xarm7/0204-move_joint.py)
- ##### 0205-move_joint --> [xarm5](example/wrapper/xarm5/0205-move_joint.py) --- [xarm6](example/wrapper/xarm6/0205-move_joint.py) --- [xarm7](example/wrapper/xarm7/0205-move_joint.py)


- #### Import
  ```
  from xarm.wrapper import XArmAPI
  arm = XArmAPI('COM5')
  arm = XArmAPI('192.168.1.113')
  arm = XArmAPI('192.168.1.113', enable_report=True)
  arm = XArmAPI('192.168.1.113', do_not_open=False)
  arm = XArmAPI('192.168.1.113', report='rich')
  arm = XArmAPI('192.168.1.113', is_radian=False)
  ```
- #### Connect/Disconnect
  ```
  arm.connect(...)
  arm.disconnect()
  ```
- #### Move
  ```
  arm.reset(...)
  arm.set_position(...)
  arm.set_servo_angle(...)
  arm.set_servo_angle_j(...)
  arm.move_gohome(...)
- #### Set
  ```
  arm.set_servo_attach(...)
  arm.set_servo_detach(...)
  arm.set_state(...)
  arm.set_mode(...)
  arm.motion_enable(...)
  arm.set_pause_time(...)
  ```
- #### Get
  ```
  arm.get_version()
  arm.get_state()
  arm.get_is_moving()
  arm.get_cmdnum()
  arm.get_err_warn_code()
  arm.get_position(...)
  arm.get_servo_angle(...)
  ```
- #### Gripper
  ```
  arm.set_gripper_enable(...)
  arm.set_gripper_speed(...)
  arm.set_gripper_position(...)
  arm.get_gripper_position()
  ```
- #### Register/Release
  ```
  arm.register_report_callback(...)
  arm.register_report_location_callback(...)
  arm.register_connect_changed_callback(callback)
  arm.register_state_changed_callback(callback)
  arm.register_maable_mtbrake_changed_callback(callback)
  arm.register_error_warn_changed_callback(callback)
  arm.register_cmdnum_changed_callback(callback)
  arm.release_report_callback(callback)
  arm.release_report_location_callback(callback)
  arm.release_connect_changed_callback(callback)
  arm.release_state_changed_callback(callback)
  arm.release_maable_mtbrake_changed_callback(callback)
  arm.release_error_warn_changed_callback(callback)
  arm.release_cmdnum_changed_callback(callback)
  ```
- #### Property
  ```
  arm.connected
  arm.default_is_radian
  arm.version
  arm.position
  arm.last_used_position
  arm.tcp_speed_limit
  arm.tcp_acc_limit
  arm.last_used_tcp_speed
  arm.last_used_tcp_acc
  arm.angles
  arm.joint_speed_limit
  arm.joint_acc_limit
  arm.last_used_angles
  arm.last_used_joint_speed
  arm.last_used_joint_acc
  arm.position_offset
  arm.state
  arm.motor_brake_states
  arm.motor_enable_states
  arm.has_err_warn
  arm.has_error
  arm.has_warn
  arm.error_code
  arm.warn_code
  arm.cmd_num
  arm.device_type
  arm.axis
  ```

