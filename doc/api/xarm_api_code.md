# xArmPythonSDK API code description

## API return value status code
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
- 9: reversed, no use
- 10: reversed, no use
- 11: other error

## Controller warning code
- 11: uxbux que is full
- 12: parameter error
- 13: the instruction does not exist
- 14: command has no solution

## Controller error code
- 10: Servo motor error
- 11: Servo motor 1 error
- 12: Servo motor 2 error
- 13: Servo motor 3 error
- 14: Servo motor 4 error
- 15: Servo motor 5 error
- 16: Servo motor 6 error
- 17: Servo motor 7 error
- 19: Tool GPIO error
- 21: Kinematic Error
- 22: Collision Error
- 23: Joints Angle Exceed Limit
- 24: Speed Exceeds Limit
- 25: Planning Error
- 26: Linux RT Error
- 27: Command Reply Error
- 28: Gripper error
- 29: Other Errors
- 30: Teach Mode Current Abnormality
- 31: Collision Caused Abnormal Current
- 33: Controller GPIO error

