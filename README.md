# xArm-Python-SDK
 
## Overview
xArm Python SDK

## Update Summary for 0.0.5
- Support serial port and TCP connection
- Support parameter to enable reporting
- Support callback register and release
- Support Gripper setting
- Support servo setting (Some interfaces are limited to professional debugging)

## Caution
- There is currently no collision detection, so try not to be close during use.
- During use, people should stay away from the robot arm to avoid accidental injury or damage to other items by the robot arm.
- Protect the arm before use.
- Before you exercise, please make sure you don't encounter obstacles.
- Protect the arm before unlocking the motor.

## Installation
Install is not necessary, you can run examples without installation.
- download
  
  ``` git clone git@github.com:uArm-Developer/xArm-Python-SDK.git```
- install

  ``` python setup.py install ```

## Doc
- [API](doc/api/xarm_api.md)

## Example
- [xArm](example/wrapper/)

- Import 
  ```
  from xarm.wrapper import XArmAPI
  xarm = XArmAPI('COM5')
  xarm = XArmAPI('192.168.1.185')
  xarm = XArmAPI('192.168.1.185', enable_report=False)
  xarm = XArmAPI('192.168.1.185', do_not_open=False)
  ```
- Connect/Disconnect
  ```
  xarm.connect(...)
  xarm.disconnect()
  ```
- Move
  ```
  xarm.reset(...)
  xarm.set_position(...)
  xarm.set_servo_angle(...)
- Set
  ```
  xarm.set_servo_attach(...)
  xarm.set_servo_detach(...)
  xarm.set_state(...)
  xarm.motion_enable(...)
  ```
- Get
  ```
  xarm.get_version()
  xarm.get_state()
  xarm.get_is_moving()
  xarm.get_cmdnum()
  xarm.get_err_warn_code()
  xarm.get_position(...)
  xarm.get_servo_angle(...)
  ```
- Gripper
  ```
  xarm.set_gripper_enable(...)
  xarm.set_gripper_speed(...)
  xarm.set_gripper_position(...)
  ```
- Property
  ```
  xarm.connected
  xarm.version
  xarm.position
  xarm.angles
  xarm.state
  xarm.error_code
  xarm.warn_code
  xarm.cmd_num
  ```

