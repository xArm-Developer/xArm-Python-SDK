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

  ```bash
  git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
  ```

- install

  ```bash
  python setup.py install
  ```

## Doc
- #### [API Document](doc/api/xarm_api.md)

- #### [API Code Document](doc/api/xarm_api_code.md)

## Update Summary

- > ### 1.0.0

  - Support for the latest firmware 1.0.0, compatible with old firmware
  - Support mount angle setting interface
  - Support controller IO interface
  - Modify clip and Tool GPIO communication protocol, compatible with standard modbus
  - __Modify the interface name and parameters of the Tool GPIO, note that the value of the parameter ionum is changed from [1, 2] to [0, 1]__
  - Support for blocky code conversion and operation of xArmStudio1.0.0

- >### [More](ReleaseNotes.md)


## [Example](example/wrapper/)

__Note: Before running the example, please modify the ip value in the [robot.conf](example/wrapper/robot.conf) file to the robot arm you want to control.__

- #### [0000-template](example/wrapper/common/0000-template.py)

- ##### [0001-event_register](example/wrapper/common/0001-event_register.py)

- ##### [0002-get_property](example/wrapper/common/0002-get_property.py)

- ##### [0003-api_get](example/wrapper/common/0003-api_get.py)

- ##### [0004-servo_attach_detach](example/wrapper/common/0004-servo_attach_detach.py)

- ##### [1001-move_line](example/wrapper/common/1001-move_line.py)

- #####  [1002-move_line](example/wrapper/common/1002-move_line.py)

- #####  [1003-relative_move_line](example/wrapper/common/1003-relative_move_line.py)

- #####  [1004-move_arc_line](example/wrapper/common/1004-move_arc_line.py)

- #####  [1005-move_arc_line](example/wrapper/common/1005-move_arc_line.py)

- ##### 2001-move_joint --> [xarm5](example/wrapper/xarm5/2001-move_joint.py) --- [xarm6](example/wrapper/xarm6/2001-move_joint.py) --- [xarm7](example/wrapper/xarm7/2001-move_joint.py)

- ##### 2002-move_joint --> [xarm5](example/wrapper/xarm5/2002-move_joint.py) --- [xarm6](example/wrapper/xarm6/2002-move_joint.py) --- [xarm7](example/wrapper/xarm7/2002-move_joint.py)

- ##### 2003-move_joint --> [xarm5](example/wrapper/xarm5/2003-move_joint.py) --- [xarm6](example/wrapper/xarm6/2003-move_joint.py) --- [xarm7](example/wrapper/xarm7/2003-move_joint.py)

- ##### 2004-move_joint --> [xarm5](example/wrapper/xarm5/2004-move_joint.py) --- [xarm6](example/wrapper/xarm6/2004-move_joint.py) --- [xarm7](example/wrapper/xarm7/2004-move_joint.py)

- #####  [3001-move_circle](example/wrapper/common/3001-move_circle.py)

- ##### [5001-get_tgpio_digital](example/wrapper/common/5001-get_tgpio_digital.py)

- ##### [5002-get_tgpio_analog](example/wrapper/common/5002-get_tgpio_analog.py)

- ##### [5003-set_tgpio_digital](example/wrapper/common/5003-set_tgpio_digital.py)

- ##### [5004-set_gripper](example/wrapper/common/5004-set_gripper.py)

- ##### [5005-get_cgpio_digital_analog](example/wrapper/common/5005-get_cgpio_digital_analog.py)

- ##### [5006-set_cgpio_dialog_analog](example/wrapper/common/5006-set_cgpio_dialog_analog.py)

- ##### [5007-set_cgpio_input_output_function](example/wrapper/common/5007-set_cgpio_input_output_function.py)

- ##### [5008-get_cgpio_state](example/wrapper/common/5008-get_cgpio_state.py)

- ##### [blockly_to_python](example/wrapper/tool/blockly_to_python.py)


- #### Import
  ```python
  from xarm.wrapper import XArmAPI
  arm = XArmAPI('COM5')
  arm = XArmAPI('192.168.1.113')
  arm = XArmAPI('192.168.1.113', do_not_open=False)
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

