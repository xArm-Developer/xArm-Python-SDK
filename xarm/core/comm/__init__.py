try:
    from .serial_port import SerialPort
except ImportError:
    SerialPort = None
from .socket_port import SocketPort
