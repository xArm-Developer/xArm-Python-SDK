# xArmPythonSDK API code description

## API return value status code
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
- 10: servo error
- 11: servo-1 error
- 12: servo-2 error
- 13: servo-3 error
- 14: servo-4 error
- 15: servo-5 error
- 16: servo-6 error
- 17: servo-7 error
- 21: inverse kinematics
- 22: collision limit
- 23: angle limit
- 24: angle speed limit
- 25: planning error
- 26: rtlinux set timing error
- 27: reply command failed
- 28: gripper error
- 29: other error
