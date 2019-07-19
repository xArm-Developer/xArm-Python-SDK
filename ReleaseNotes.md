# xArm-Python-SDK Release Notes

## Update Summary

- > ### 1.1.0

  - Modify the claw interface
  - Modify error code and error message
  - Support for blocky code conversion and operation of xArmStudio1.1.0

- > ### 1.0.0

  - Support for the latest firmware 1.0.0, compatible with old firmware
  - Support mount angle setting interface
  - Support controller IO interface
  - Modify clip and Tool GPIO communication protocol, compatible with standard modbus
  - Modify the interface name and parameters of the Tool GPIO, note that the value of the parameter ionum is changed from [1, 2] to [0, 1]
  - Support for blocky code conversion and operation of xArmStudio1.0.0

- > ### 0.2.1

  - Added GPIO example
  - Compatible with old reporting protocols using new reporting protocols
  - New tools to convert xArmStudio's app code into Python code

- > ### 0.2.0

  - Support torque detection
  - Support drag teaching mode

- > ### 0.1.1

  - Support GPIO interface

- > ### 0.1.0

  - Compatible with 5, 6, and 7 axis robot arms

- > ### 0.0.13

  - Supports trajectory repeat motion interface with circular interpolation, supports repetition times, calibration
  - Increase joint limits, attitude angle limits and cmd cache limits
  - Exchange the parameters of the attitude angle yaw and the attitude angle pitch, but retain the parameter position

- > ### 0.0.12

  - By specifying the value of the default parameter of the interface in the instantiation parameter, using angle or radians
  - Unify all interfaces by default using angle or radians
  - More interface routines
  - More detailed interface documentation
  - Richer interface
  - Set interface alias
  - Modified the default values of the default parameters of some interfaces, but support the use of parameters to be compatible when instantiating

- > ### 0.0.11

  - Support serial port and TCP connection
  - Support parameter to enable reporting
  - Support callback register and release
  - Support Gripper setting
  - Support servo setting (Some interfaces are limited to professional debugging)
  - Unified return value
  - Snaps an exception and returns the specified return value

