try:
    from .serial_port import SerialPort
except:
    SerialPort = None
from .socket_port import SocketPort
