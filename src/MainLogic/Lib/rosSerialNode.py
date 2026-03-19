"""Compatibility shim. Use MainLogic.core.serial_node instead."""

from MainLogic.core.serial_node import SerialNode, SerialProcess, main, start_serial_process

__all__ = ["SerialNode", "SerialProcess", "main", "start_serial_process"]