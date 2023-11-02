# UFACTORY ModbusTCP User Instructions

## Notice:
- __Communication complies with standard Modbus TCP protocol__
- __Data transmission uses Big-endian method, for example transmission order of data 0x1234 is: 0x12, 0x34__ 
- __Different register addresses hold different specific contents, details refer to the appendix__
- __Make sure to access the pre-defined register address(refer to appendix)，or there will be exception in response__

## Supported Modbus TCP function codes:
- #### Coil register(1 bit)
    - __0x01__: Read multiple coil registers，each bit in responded data section represents the value of each one register:
        ```C++
        // sample request and response / exception
        // Request: Read consecutive 16 registers starting from address 0x0000
        00 01 00 00 00 06 01 01 00 00 00 10
        // Response: every bit in the Received data (0xF7 0x00 here as an example) represents the corresponding register value in order
        00 01 00 00 00 05 01 01 02 F7 00
        // Exception: XX is the exception code
        00 01 00 00 00 03 01 81 XX
        ```
    - __0x05__: Write single coil, according to Modbus protocol, specified data can only be 0xFF00 or 0x0000，for wriring 1 or 0 to the register.
        ```C++
        // sample request and response / exception
        // Request: Write 1 to register address 0x0002 (0xFF00 for writing 1, 0x0000 for writing 0)
        00 01 00 00 00 06 01 05 00 02 FF 00 
        // Response:
        00 01 00 00 00 06 01 05 00 02 FF 00
        // Exception: XX is the exception code
        00 01 00 00 00 03 01 85 XX
        ```
    - __0x0F__: Write multiple coil registers
        ```C++
        // sample request and response / exception
        // Request: write 3 registers starting from address 0x0002, 0x07 in binary form is for writing all 1s in the three registers.
        00 01 00 00 00 08 01 0F 00 02 00 03 01 07
        // Response:
        00 01 00 00 00 06 01 0F 00 02 00 03
        // Exception: XX is the exception code
        00 01 00 00 00 03 01 8F XX
        ```
- #### Discrete input register (1 bit)
    - __0x02__: Read multiple discrete input registers
        ```C++
        // sample request and response / exception
        // Request: Read 16 consecutive registers from address 0x0000
        00 01 00 00 00 06 01 02 00 00 00 10
        // Response:: every bit in the Received data (0xFF 0x00 here as an example) represents the corresponding register value in order
        00 01 00 00 00 05 01 02 02 FF 00
        // Exception: XX is the exception code
        00 01 00 00 00 03 01 82 XX
        ```
- #### Holding register (16 bit)
    - __0x03__: Read multiple holding registers
        ```C++
        // sample request and response / exception
        // Request: read 2 consecutive holding registers starting from address 0x0003
        00 01 00 00 00 06 01 03 00 03 00 02
        // Response:: Received values of the two registers are (00 05) and (00 06) as an example here
        00 01 00 00 00 07 01 03 04 00 05 00 06
        // Exception: XX is the exception code
        00 01 00 00 00 03 01 83 XX
        ```
    - __0x06__: Write single holding register
        ```C++
        // sample request and response / exception
        // Request: Write 0x0006 to register address 0x0020
        00 01 00 00 00 06 01 06 00 20 00 06
        // Response:
        00 01 00 00 00 06 01 06 00 20 00 06
        // Exception: XX is the exception code
        00 01 00 00 00 03 01 86 XX
        ```
    - __0x10__: Write multiple holding registers
        ```C++
        // sample request and response / exception
        // Request: Write consecutive 2 registers from address 0x0003
        // the written values are (04 D2) and (0D 80), with corresponding decimal values of 1234 and 3456
        00 01 00 00 00 0B 01 10 00 03 00 02 04 04 D2 0D 80
        // Response:
        00 01 00 00 00 06 01 10 00 03 00 02
        // Exception: XX is the exception code
        00 01 00 00 00 03 01 90 XX
        ```
    - __0x16__: Mask write single holding register
        ```C++
        // sample request and response / exception
        // Request: write to register address of 0x0000 with mask.
        // "AND" operation mask is (00 0F)，"OR" operation mask is (0F 00)
        // If the value before this operation is "val", then after the mask opeation it becomes: (val & 0x000F) | (0x0F00 & ~0x000F)
        00 04 00 00 00 08 01 16 00 00 00 0F 0F 00
        // Response:
        00 04 00 00 00 08 01 16 00 00 00 0F 0F 00
        // Exception: XX is the exception code
        00 01 00 00 00 03 01 96 XX
        ```
    - __0x17__: Read and Write multiple holding registers
        ```C++
        // sample request and response / exception
        // Request: Write 2 registers from address 0x0020, and read 2 registers from address 0x0003
        // values to be written are (00 06) and (00 04)
        00 01 00 00 00 0F 01 17 00 03 00 02 00 20 00 02 04 00 06 00 04 
        // Response:  Received values of the two registers are (04 D1) and (0D 7F) as an example here  
        00 01 00 00 00 07 01 17 04 04 D1 0D 7F 
        // Exception: XX is the exception code
        00 01 00 00 00 03 01 97 XX
        ```
- #### Input register (16 bit)
    - 0x04: Read multiple input registers
        ```C++
        // sample request and response / exception
        // Request: read 2 consecutive holding registers from address 0x0003
        00 01 00 00 00 06 01 04 00 03 00 02
        // Response: Received 2 values are (00 0E) and (00 13) as an example
        00 01 00 00 00 07 01 04 04 00 0E 00 13
        // Exception: XX is the exception code
        00 01 00 00 00 03 01 84 XX
        ```

# Exception code explanation
- __0x01__: Illegal/Unsuppported function code
- __0x02__: Illegal target address
- __0x03__: Exception of requested data

# Appendix
- #### Coil Registers (single bit access, READ/WRITE)
    | Address(Dec) | Address(Hex) |  Explanation  |
    |   ----  |   ----  |   ---- |
    | 0 ~ 31 | 0x00 ~ 0x1F | 32 controller Digital Output (Now only 16 effective) |
    | 32 ~ 39 | 0x20 ~ 0x27 | 8 tool Digital Output (Now only 2 effective) |
    | 40 ~ 127 | 0x28 ~ 0x7F | Reserved |
    | 128 ~ 134 | 0x80 ~ 0x86 | joint (J1-J7) brake states |
    | 135 ~ 141 | 0x87 ~ 0x8D | joint (J1-J7) enable states |
    | 142 | 0x8E | Reduced mode (0: OFF, 1: ON) |
    | 143 | 0x8F | Digital Fence (0: OFF, 1: ON) |
    | 144 | 0x90 | isPaused (0: False, 1: True) |
    | 145 | 0x91 | isStopped (0: False, 1: True) |
    | 146 ~ 159 | 0x92 ~ 0x9F | Robot Mode (14 bits for mode 0-13 respectively, 0: not in this mode, 1: in this mode) |
    | 160 ~ 255 | 0xA0 ~ 0xFF | Reserved |
    | 256 ~ 511 | 0x100 ~ 0x1FF | General purpose, user defined |

- #### Discrete Input Registers (single bit access, READ only)
    | Address(Dec) | Address(Hex) |  Explanation  |
    |   ----  |   ----  |   ---- |
    | 0 ~ 31 | 0x00 ~ 0x1F | 32 controller Digital Input (Now only 16 effective) |
    | 32 ~ 39 | 0x20 ~ 0x27 | 8 tool Digital Input (Now only 2 effective) |
    | 40 ~ 127 | 0x28 ~ 0x7F | Reserved |

- #### Holding Registers (16 bit access, READ/WRITE)
    | Address(Dec) | Address(Hex) |  Explanation  |
    |   ----  |   ----  |   ---- |
    | 0 ~ 1 | 0x00 ~ 0x01 | 32 controller Digital Outputs (Now only 16 effective), each bit correspond to one IO in order |
    | 2 | 0x02 | 8 tool Digital Outputs (Now only 2 effective), each bit correspond to one IO in order |
    | 3 ~ 6  | 0x03 ~ 0x06 | 4 controller analog outputs (now only 2 effective), it is 1000 times the real value |
    | 7 ~ 10  | 0x07 ~ 0x0A | 4 tool analog outputs (currently NOT EFFECTIVE), it is 1000 times the real value  |
    | 11 ~ 31  | 0x0B ~ 0x1F | Reserved |
    | 32 | 0x20 | Robot Mode |
    | 33 | 0x21 | Robot State |
    | 34 ~ 47 | 0x22 ~ 0x2F | Reserved |
    | 48 ~ 63 | 0x30 ~ 0x3F | Offline (Blockly) Task (__only effective by writing multiple (max 16) holding registers to address 0x30 via function code 0x10, each register value correspond to one Blockly project with specific naming convention, for example: value 1 for project '00001', 12 for project '00012', projects will be executed automatically in order__) |
    | 64 ~ 255 | 0x40 ~ 0xFF | Reserved |
    | 256 ~ 511 | 0x100 ~ 0x1FF | General purpose, user defined |

- #### Input Registers (16 bit access, READ only)
    | Address(Dec) | Address(Hex) |  Explanation  |
    |   ----  |   ----  |   ---- |
    | 0 ~ 1 | 0x00 ~ 0x01 | 32 controller Digital Inputs (Now only 16 effective) |
    | 2 | 0x02 | 8 tool Digital Inputs (Now only 2 effective) |
    | 3 ~ 6  | 0x03 ~ 0x06 | 4 controller analog inputs (now only 2 effective), it is 1000 times the real value |
    | 7 ~ 10  | 0x07 ~ 0x0A | 4 tool analog inputs (now only 2 effective), it is 1000 times the real value |
    | 11 ~ 31  | 0x0B ~ 0x1F | Reserved |
    | 32 | 0x20 | Robot Error code |
    | 33 | 0x21 | Robot Warning code |
    | 34 ~ 35 | 0x22 ~ 0x23 | Counter value (0x22 stores the higher 16-bit, 0x23 stores the lower 16-bit) |
    | 36 ~ 63 | 0x23 ~ 0x3F | Reserved |
    | 64 ~ 72 | 0x40 ~ 0x48 | Current TCP coordinate of x/y/z/roll/pitch/yaw/rx/ry/rz values, register values are 10 times the real numbers (unit: mm, degree) |
    | 73 ~ 76 | 0x49 ~ 0x4C | TCP payload mass(1000x)/center_x(10x)/center_y(10x)/center_z(10x) (unit: kg, mm) |
    | 77 ~ 82 | 0x4D ~ 0x52 | TCP Offset, register values are 10 times the real numbers(unit: mm, degree) |
    | 83 ~ 88 | 0x53 ~ 0x58 | User/world coordinate offset, register values are 10 times the real numbers(unit: mm, degree) |
    | 89 ~ 95 | 0x59 ~ 0x5F | joint (J1-J7) angles, register values are 10 times the real numbers(unit: degree) |
    | 86 ~ 102 | 0x60 ~ 0x66 | joint (J1-J7) temperature (unit: degree Celsius) |
    | 103 ~ 109 | 0x67 ~ 0x6D | joint (J1-J7) speed, register values are 10 times the real numbers(unit: degree/s) |
    | 110 | 0x6E | Robot TCP linear speed, register values are 10 times the real numbers(unit: mm/s) |
    | 111 ~ 127 | 0x6F ~ 0x7F | Reserved |
