# xArm-Python-SDK Release Notes

## Update Summary

- > ### 1.17.0 
  - Change some API names

- > ### 1.16.0
  - Added parameter to support get raw data of the Six-axis Force Torque Sensor
  - Added an interface to control xArm Gripper G2
  - Optimize the interface for controlling BIO Gripper G2
  - Extend the get_joint_states interface

- > ### 1.15.0
  - Added the Six-axis Force Torque Sensor collision detection related interfaces
  - Added support for the new version of BIO Gripper control interface

- > ### 1.14.7
  - Fixed the problem that single joint or direction movement fails in some cases (valid from firmware 2.5.0)
  - Added standard ModbusTcp client and Gcode client
  - GPIO control supports immediate execution
  - Support new version of vacuum gripper
  - Support the studio-2.5.0 blockly project conversion to python

- > ### 1.13.30
  - Supports obtaining unsaved trajectory recording duration
  - Fix the abnormal path of running blockly program in some cases
  - Fix the return format of getting C23 and C38 errors
  - Supports obtaining identification status

- > ### 1.13.19
  - Friction identification supports xarm7_mirror model
  - Fix the abnormal return value of blockly conversion robotiq related interface

- > ### 1.13.0
  - Compatible with the standard Modbus TCP protocol, providing part of the standard Modbus TCP protocol interface

- > ### 1.12.2
  - Support partial Task feedback (requires firmware version greater than or equal to v2.1.0)

- > ### 1.11.6
  - Correct the ambiguity that the `set_position_aa` interface is true when both relative and is_tool_coord are true. After the correction, when is_tool_coord is true, relative is invalid (previously is_tool_coord was invalid when relative was true)

- > ### 1.11.5
  - Optimization pause time is too long (wait=true)
  - Add common motion api (Enabled after firmware version 1.11.100)
  - The Cartesian motion-related interface adds the motion_type parameter to determine the planning method (Enabled after firmware version 1.11.100)

- > ### 1.11.0
  - Support transparent transmission
    - 240: `set_tgpio_modbus_timeout(..., is_transparent_transmission=True)`
    - 241: `getset_tgpio_modbus_data(..., is_transparent_transmission=True)`
  - Modified the centroid unit of the `ft_sensor_iden_load` and `ft_sensor_cali_load` interfaces to millimeters (originally meters)

- > ### 1.10.0
  - Use monotonic time
  - Fix several bugs

- > ### 1.9.10
  - Use monotonic time
  - Fix several bugs

- > ### 1.9.10
  - Support Lite6 Model
  - Fix several bugs

- > ### 1.9.0
  - Support friction parameter identification interface
  - Support relative motion
  - Support xarm6-type11 firmware
  - Repair time-consuming interface (identification) failure due to heartbeat mechanism
  - Fix several bugs

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
  - Support linear motor interface
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

