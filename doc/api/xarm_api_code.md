# xArmSDK API code description
<!-- TOC --> 
[Contents](#xarmsdk-api-code-description)
- [xArmSDK API code description](#xarmsdk-api-code-description)
  - [API Code](#api-code)
  - [Controller Warn Code](#controller-warn-code)
  - [Controller Error Code](#controller-error-code)
  - [Servo Error Code](#servo-error-code)
  - [Gripper Error Code](#gripper-error-code)
  - [Bio Gripper Error Code](#bio-gripper-error-code)
  - [Linear Motor Error Code](#linear-motor-error-code)
  - [Six-axis Force Torque Sensor Error Code](#six-axis-force-torque-sensor-error-code)


## API Code
- -12: run blockly app exception
- -11: convert blockly app to pythen exception
- -9: emergency stop
- -8: out of range
- -7: joint angle limit
- -6: cartesian pos limit
- -5: revesed, no use
- -4: command is not exist
- -3: revesed, no use
- -2: xArm is not ready, may be the motion is not enable or not set state
- -1: xArm is disconnect or not connect
- 0: success
- 1: there are errors that have not been cleared
- 2: there are warnings that have not been cleared
- 3: get response timeout
- 4: tcp reply length error
- 5: tcp reply number error
- 6: tcp protocol flag error
- 7: tcp reply command and send command do not match
- 8: send command error, may be network exception
- 9: state is not ready to move
- 10: the result is invalid
- 11: other error
- 12: parameter error
- 20: host id error
- 21: modbus baudrate not supported
- 22: modbus baudrate not correct
- 23: modbus reply length error
- 31: trajectory read/write failed
- 32: trajectory read/write timeout
- 33: playback trajectory timeout
- 34: playback trajectory failed
- 41: wait to set suction cup timeout
- 80: linear track has error
- 81: linear track sci is low
- 82: linear track is not init
- 100: wait finish timeout
- 101: too many consecutive failed tests
- 102: end effector has error
- 103: end effector is not enabled
- 129: (standard modbus tcp)illegal/unsupported function code
- 120: (standard modbus tcp)illegal target address
- 131: (standard modbus tcp)exection of requested data

## Controller Warn Code
- 11: uxbux que is full
- 12: parameter error
- 13: the instruction does not exist
- 14: command has no solution
- 15: modbus cmd full

## Controller Error Code
- 1: The Emergency Stop Button is pushed
- 2: The Emergency IO of the Control Box is triggered
- 3: The Emergency Stop Button of the Three-state Switch is pressed
- 10: Servo motor error
- 11: Servo motor 1 error
- 12: Servo motor 2 error
- 13: Servo motor 3 error
- 14: Servo motor 4 error
- 15: Servo motor 5 error
- 16: Servo motor 6 error
- 17: Servo motor 7 error
- 18: Force Torque Sensor Communication Error
- 19: End Module Communication Error
- 21: Kinematic Error
- 22: Self-Collision Error
- 23: Joints Angle Exceed Limit
- 24: Speed Exceeds Limit
- 25: Planning Error
- 26: Linux RT Error
- 27: Command Reply Error
- 28: End Module Communication Error
- 29: Other Errors
- 30: Feedback Speed Exceeds limit
- 31: Collision Caused Abnormal Current
- 32: Three-point drawing circle calculation error
- 33: Controller GPIO error
- 34: Recording Timeout
- 35: Safety Boundary Limit
- 36: The number of delay commands exceeds the limit
- 37: Abnormal movement in Manual Mode
- 38: Abnormal Joint Angle
- 39: Abnormal Communication Between Master and Slave IC of Power Board
- 40: No IK available
- 50: Six-axis Force Torque Sensor read error
- 51: Six-axis Force Torque Sensor set mode error
- 52: Six-axis Force Torque Sensor set zero error
- 53: Six-axis Force Torque Sensor is overloaded or the reading exceeds the limit
- 110: Robot Arm Base Board Communication Error
- 111: Control Box External 485 Device Communication Error

## Servo Error Code

- 10: Current Detection Error
- 11: Joint Current Overlimit
- 12: Joint Speed Overlimit
- 14: Position Command Overlimit
- 15: Joints Overheat
- 16: Encoder Initialization Error
- 17: Single Ring Encoder Error
- 18: Multi-turn Encoder Error
- 19: Low Battery Voltage
- 20: Driver IC Hardware Error
- 21: Driver IC Initialization Error
- 22: Encoder Configuration Error
- 23: Large Motor Position Deviation
- 26: Joint N Positive Overrun
- 27: Joint N Negative Overrun
- 28: Joint Commands Error
- 33: Drive Overloaded
- 34: Motor Overload
- 35: Motor Type Error
- 36: Driver Type Error
- 39: Joint Voltage Overload
- 40: Joint Voltage Insufficient
- 49: EEPROM Read and Write Error
- 52: Motor Angle Initialization Error

## Gripper Error Code

- 9: Gripper Current Detection Error
- 11: Gripper Current Overlimit
- 12: Gripper Speed Overlimit
- 14: Gripper Position Command Overlimit
- 15: Gripper EEPROM Read and Write Error
- 20: Gripper Driver IC Hardware Error
- 21: Gripper Driver IC Initialization Error
- 23: Gripper Large Motor Position Deviation
- 25: Gripper Command Over Software Limit
- 26: Gripper Feedback Position Software Limit
- 33: Gripper Drive Overloaded
- 34: Gripper Motor Overload
- 36: Gripper Driver Type Error

## Bio Gripper Error Code

- 11: BIO Gripper Current Overlimit
- 12: The object slipped from the BIO Gripper

## Linear Motor Error Code

- 10: Linear Motor Current Detection Error
- 11: Linear Motor Current Overlimit
- 12: Linear Motor Speed Overlimit
- 13: Linear Motor Large Motor Position Deviation
- 14: Linear Motor Position Command Overlimit
- 20: Linear Motor Driver IC Hardware Error
- 21: Linear Motor Driver IC Initialization Error
- 25: Linear Motor Command Over Software Limit
- 26: Linear Motor Feedback Position Software Limit
- 33: Linear Motor Drive Overloaded
- 34: Linear Motor Motor Overload
- 35: Linear Motor type error
- 36: Linear Motor Driver Type Error
- 39: Linear Motor over voltage
- 40: Linear Moter undervoltage
- 49: Linear Motor EEPROM Read and Write Error
  
## Six-axis Force Torque Sensor Error Code

- 64: Six-axis Force Torque Sensor Communication Failure
- 65: The Data Collected by the Six-axis Force Torque Sensor is Abnormal
- 66: Six-axis Force Torque Sensor X-direction Torque Exceeds Limit
- 67: Six-axis Force Torque Sensor Y-direction Torque Exceeds Limit
- 68: Six-axis Force Torque Sensor Z-direction Torque Exceeds Limitrection
- 69: Six-axis Force Torque Sensor Tx Torque Exceeds Limit
- 70: Six-axis Force Torque Sensor Ty direction Torque Exceeds Limit
- 71: Six-axis Force Torque Sensor Tz direction Torque Exceeds Limit
- 73: Six-axis Force Torque Sensor Failed to Initialize
