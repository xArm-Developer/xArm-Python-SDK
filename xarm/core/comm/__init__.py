try:
    from .serial_port import SerialPort
except:
    import warnings
    warnings.warn('serial module is not found, if you want to connect to xArm with serial, please `pip install pyserial==3.4`')
    SerialPort = object
from .socket_port import SocketPort
