"""MPC implementation library.

只放算法实现，不依赖 MainLogic、ROS、TF 或串口。
MainLogic 通过 MainLogic.core.mpc 适配层使用本库。

路径生成可以单独导入，不会顺带加载 do_mpc/casadi。
"""

from .path_generate import SplinePlanner

__all__ = ["MPCPathFollower", "MPC_DT", "MPC_TYPE", "SplinePlanner"]


def __getattr__(name):
    if name in {"MPCPathFollower", "MPC_DT", "MPC_TYPE"}:
        from . import mpc
        return getattr(mpc, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
