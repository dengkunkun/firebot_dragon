import rclpy
from rclpy.node import Node
import inspect
import os

class Colors:
    """ANSI escape sequences for colored output"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    ERROR = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ColorLogger:
    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()

    def _get_caller_info(self):
        """Get the file name and line number of the caller"""
        caller_frame = inspect.currentframe().f_back.f_back  # Go back two frames to get the caller
        filename = os.path.basename(caller_frame.f_code.co_filename)
        lineno = caller_frame.f_lineno
        return f"[{filename}:{lineno}]"

    def debug(self, msg: str):
        caller_info = self._get_caller_info()
        self._logger.debug(f"{Colors.OKBLUE}{caller_info} {msg}{Colors.ENDC}")

    def info(self, msg: str):
        caller_info = self._get_caller_info()
        self._logger.info(f"{Colors.OKGREEN}{caller_info} {msg}{Colors.ENDC}")

    def warn(self, msg: str):
        caller_info = self._get_caller_info()
        self._logger.warn(f"{Colors.WARNING}{caller_info} {msg}{Colors.ENDC}")

    def error(self, msg: str):
        caller_info = self._get_caller_info()
        self._logger.error(f"{Colors.ERROR}{caller_info} {msg}{Colors.ENDC}")

    def fatal(self, msg: str):
        caller_info = self._get_caller_info()
        self._logger.fatal(f"{Colors.BOLD}{Colors.ERROR}{caller_info} {msg}{Colors.ENDC}")