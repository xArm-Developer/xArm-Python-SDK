# xArm-Python-SDK

## Overview
xArm Python SDK

## Caution
- There is currently no collision detection, so try not to be close during use.
- During use, people should stay away from the robot arm to avoid accidental injury or damage to other items by the robot arm.
- Protect the arm before use.
- Before you exercise, please make sure you don't encounter obstacles.
- Protect the arm before unlocking the motor.

## Installation
Install is not necessary, you can run examples without installation.Only Python3 is supported.
- download

  ``` git clone https://github.com/xArm-Developer/xArm-Python-SDK.git```

- install

  ``` python setup.py install ```

## Doc
- #### [API Document](doc/api/xarm_api.md)

- #### [API Code Document](doc/api/xarm_api_code.md)

## Update Summary

- > ### 0.3.0

  - Modify clip and Tool GPIO communication protocol, compatible with standard modbus
  - Modify the interface name and parameters of the tool GPIO, note that the value of the parameter ionum is changed from [1, 2] to [0, 1]

- >### 0.2.1

  - Added GPIO example
  - Compatible with old reporting protocols using new reporting protocols
  - New tools to convert xArmStudio's app code into Python code


- >### 0.2.0

  - Support torque detection
  - Support drag teaching mode

- >### 0.1.1

  - Support GPIO interface

- >### 0.1.0

  - Compatible with 5, 6, and 7 axis robot arms

- >### 0.0.13

  - Supports trajectory repeat motion interface with circular interpolation, supports repetition times, calibration
  - Increase joint limits, attitude angle limits and cmd cache limits
  - Exchange the parameters of the attitude angle yaw and the attitude angle pitch, but retain the parameter position

- >### 0.0.12

  - By specifying the value of the default parameter of the interface in the instantiation parameter, using angle or radians
  - Unify all interfaces by default using angle or radians
  - More interface routines
  - More detailed interface documentation
  - Richer interface
  - Set interface alias
  - Modified the default values of the default parameters of some interfaces, but support the use of parameters to be compatible when instantiating

- >### 0.0.11

  - Support serial port and TCP connection
  - Support parameter to enable reporting
  - Support callback register and release
  - Support Gripper setting
  - Support servo setting (Some interfaces are limited to professional debugging)
  - Unified return value
  - Snaps an exception and returns the specified return value

## Example
- #### [xArm](example/wrapper/)

- ##### 0002-connect_with_socket --> [xarm5](example/wrapper/xarm5/0002-connect_with_socket.py) --- [xarm6](example/wrapper/xarm6/0002-connect_with_socket.py) --- [xarm7](example/wrapper/xarm7/0002-connect_with_socket.py)

- ##### 0003-event_register --> [xarm5](example/wrapper/xarm5/0003-event_register.py) --- [xarm6](example/wrapper/xarm6/0003-event_register.py) --- [xarm7](example/wrapper/xarm7/0003-event_register.py)

- ##### 0005-get_property --> [xarm5](example/wrapper/xarm5/0005-get_property.py) --- [xarm6](example/wrapper/xarm6/0005-get_property.py) --- [xarm7](example/wrapper/xarm7/0005-get_property.py)

- ##### 0006-get --> [xarm5](example/wrapper/xarm5/0006-get.py) --- [xarm6](example/wrapper/xarm6/0006-get.py) --- [xarm7](example/wrapper/xarm7/0006-get.py)

- ##### 0007-servo_attach_detach --> [xarm5](example/wrapper/xarm5/0007-servo_attach_detach.py) --- [xarm6](example/wrapper/xarm6/0007-servo_attach_detach.py) --- [xarm7](example/wrapper/xarm7/0007-servo_attach_detach.py)

- ##### 0008-send_cmd --> [xarm5](example/wrapper/xarm5/0008-send_cmd.py) --- [xarm6](example/wrapper/xarm6/0008-send_cmd.py) --- [xarm7](example/wrapper/xarm7/0008-send_cmd.py)

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

- ##### 0301-move_circle --> [xarm5](example/wrapper/xarm5/0301-move_circle.py) -- [xarm6](example/wrapper/xarm6/0301-move_circle.py) --- [xarm7](example/wrapper/xarm7/0301-move_circle.py)

- [1001-get_tgpio_digital](example/wrapper/common/1001-get_tgpio_digital.py)

- [1002-get_tgpio_analog](example/wrapper/common/1002-get_tgpio_analog.py)

- [1003-set_tgpio_digital](example/wrapper/common/1003-set_tgpio_digital.py)

- [1004-set_gripper](example/wrapper/common/1004-set_gripper.py)

- [1005-get_cgpio_digital_analog](example/wrapper/common/1005-get_cgpio_digital_analog.py)

- [1006-set_cgpio_dialog_analog](example/wrapper/common/1006-set_cgpio_dialog_analog.py)

- [1007-set_cgpio_input_output_function](example/wrapper/common/1007-set_cgpio_input_output_function.py)

- [1008-get_cgpio_state](example/wrapper/common/1008-get_cgpio_state.py)

- [blockly_to_python](example/wrapper/tool/blockly_to_python.py)


- #### Import
  ```python
  from xarm.wrapper import XArmAPI
  arm = XArmAPI('COM5')
  arm = XArmAPI('192.168.1.113')
  arm = XArmAPI('192.168.1.113', enable_report=True)
  arm = XArmAPI('192.168.1.113', do_not_open=False)
  arm = XArmAPI('192.168.1.113', report='rich')
  arm = XArmAPI('192.168.1.113', is_radian=False)
  ```
- #### Connect/Disconnect
  ```python
  arm.connect(...)
  arm.disconnect()
  ```
- #### Move
  ```python
  arm.reset(...)
  arm.set_position(...)
  arm.set_servo_angle(...)
  arm.set_servo_angle_j(...)
  arm.move_gohome(...)
  arm.move_circle(...)
  arm.emergency_stop()
  ```
- #### Set
  ```python
  arm.set_servo_attach(...)
  arm.set_servo_detach(...)
  arm.set_state(...)
  arm.set_mode(...)
  arm.motion_enable(...)
  arm.set_pause_time(...)
  ```
- #### Get
  ```python
  arm.get_version()
  arm.get_state()
  arm.get_is_moving()
  arm.get_cmdnum()
  arm.get_err_warn_code()
  arm.get_position(...)
  arm.get_servo_angle(...)
  ```
- #### Setting
```python
  arm.set_tcp_offset(...)
  arm.set_tcp_jerk(...)
  arm.set_tcp_maxacc(...)
  arm.set_joint_jerk(...)
  arm.set_joint_maxacc(...)
  arm.set_tcp_load(...)
  arm.set_collision_sensitivity(...)
  arm.set_teach_sensitivity(...)
  arm.set_gravity_direction(...)
  arm.clean_conf()
  arm.save_conf()
```
- #### Gripper
  ```python
  arm.set_gripper_enable(...)
  arm.set_gripper_mode(...)
  arm.set_gripper_speed(...)
  arm.set_gripper_position(...)
  arm.get_gripper_position()
  arm.get_gripper_err_code()
  arm.clean_gripper_error()
  ```
- #### GPIO
  ```python
  # Tool GPIO
  arm.get_tgpio_digital(...)
  arm.set_tgpio_digital(...)
  arm.get_tgpio_analog(...)
  
  # Controller GPIO
  arm.get_cgpio_digital(...)
  arm.get_cgpio_analog(...)
  arm.set_cgpio_digital(...)
  arm.set_cgpio_analog(...)
  arm.set_cgpio_digital_input_function(...)
  arm.set_cgpio_digital_output_function(...)
  arm.get_cgpio_state()
  ```
- #### Other
```python
  arm.set_pause_time(...)
  arm.shutdown_system(...)
  arm.clean_error()
  arm.clean_warn()
```
- #### Register/Release
  ```python
  arm.register_report_callback(...)
  arm.register_report_location_callback(...)
  arm.register_connect_changed_callback(callback)
  arm.register_state_changed_callback(callback)
  arm.register_mode_changed_callback(callback)
  arm.register_mtable_mtbrake_changed_callback(callback)
  arm.register_error_warn_changed_callback(callback)
  arm.register_cmdnum_changed_callback(callback)
  arm.release_report_callback(callback)
  arm.release_report_location_callback(callback)
  arm.release_connect_changed_callback(callback)
  arm.release_state_changed_callback(callback)
  arm.release_mode_changed_callback(callback)
  arm.release_mtable_mtbrake_changed_callback(callback)
  arm.release_error_warn_changed_callback(callback)
  arm.release_cmdnum_changed_callback(callback)
  ```
- #### Property
  ```python
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
  arm.tcp_offset
  arm.state
  arm.mode
  arm.joints_torque
  arm.tcp_load
  arm.collision_sensitivity
  arm.teach_sensitivity
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
  arm.gravity_direction
  ```

