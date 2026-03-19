"""Compatibility shim. Use MainLogic.core.ros_bridge_node instead."""

from MainLogic.core.ros_bridge_node import RosBridgeNodeInstance, rosBridgeNode

__all__ = ["rosBridgeNode", "RosBridgeNodeInstance"]
