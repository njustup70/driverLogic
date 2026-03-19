"""Compatibility shim. Use MainLogic.core.tf_manager instead."""

from MainLogic.core.tf_manager import TFManager, TFManagerInstance, move_to

__all__ = ["TFManager", "TFManagerInstance", "move_to"]
