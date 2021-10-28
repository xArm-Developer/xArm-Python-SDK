# xArm-Python-SDK Release Notes

## Update Summary

- > ### 1.8.4
  - Support the Six-axis Force Torque Sensor (not a third party)
  - Add threads to handle callbacks
  - Modify the reporting processing logic and optimize the processing of sticky packets
  - Fixed frequent switching of the pause state causing the program to hang
  - Fix the program hangs when setting the mechanical claw position in speed mode
  - Fix relative movement in unsynchronized position

- > ### 1.8.0

  - Support for blocky code conversion and operation of xArmStudio-1.8.0
  - The Velocity interface supports the duration parameter (requires firmware 1.8.0 or higher)
  - Added identification interface (current identification and torque identification)
  - Support linear track interface
  - Support calling some studio APIs

- > ### 1.6.9

  - Support for blocky code conversion and operation of xArmStudio1.6.9
  - Support velocity control
  - Support calibrate tcp offset and user offset

- - > ### 1.6.5

  - Support for blocky code conversion and operation of xArmStudio1.6.5

- > ### 1.6.0

  - Support the xArm BIO gripper, Robotiq 2F-85 gripper and Robotiq 2F-140 gripper
  - Support position detection trigger the controller analog IO
  - Support self-collision model parameter setting
  - Support Modbus communication of end tools
  - Supports TCP timeout for setting instructions
  - Support joint motion with circular interpolation
  - Support for blocky code conversion and operation of xArmStudio1.6.0
  - Optimize logic, enhance API security, Fix several bugs

- > ### 1.5.0

  - The new parameter of `set_servo_cartisian` interface is used to support servo cartisian movement of tool coordinate system
  - Support delayed trigger digital IO
  - Support position detection trigger digital IO
  - Support configure the stop state to automatically reset IO signal
  - Support motion commands based on axis angle
  - Support to calculate the offset between two points
  - Support for blocky code conversion and operation of xArmStudio1.5.0

- > ### 1.4.0
  - Support servo cartesian interface
  - Support for blocky code conversion and operation of xArmStudio1.4.0

- > ### 1.3.0

  - Added several attributes
  - Support tool coordinate system movement
  - Support joint range limitation, collision rebound setting
  - Support user coordinate system setting
  - Support the status of the air pump
  - Added counter interface
  - Support for blocky code conversion and operation of xArmStudio1.3.0

- > ### 1.2.0

  - Fix the parameters of the control box GPIO analog signal
  - Support for more Gcode commands
  - Support trajectory recording
  - Support for reduced mode settings
  - Optimize some interfaces and bug fixes
  - Support for blocky code conversion and operation of xArmStudio1.2.0

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

