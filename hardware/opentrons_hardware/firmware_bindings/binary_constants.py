"""Constants for binary format to rear-panel.

This file is used as a source for code generation, which does not run in a venv
by default. Please do not unconditionally import things outside the python standard
library.
"""
from enum import Enum, unique


@unique
class BinaryMessageId(int, Enum):
    """USB Binary message ID."""

    echo = 0x00
    ack = 0x01
    ack_failed = 0x02
    device_info_request = 0x03
    device_info_response = 0x04
    enter_bootloader_request = 0x05
    enter_bootloader_response = 0x06
