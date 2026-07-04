import math
from dataclasses import dataclass

import numpy as np

from MainLogic.Lib.odomVec import Odom, SE3


MIN_ODIN_MAP_POSE_COUNT = 3
FIELD_FRAME_CAR_POSE_PREFIX = "field_frame_car_pose"
ODIN_FRAME_LIDAR_POSE_PREFIX = "odin_frame_lidar_pose"
ODIN_MAP_TO_FIELD_SCALE = 1.0


# ============================================================
# 原始 Odin 标定参数
# ============================================================

ODIN_MAP_DATA = {
    "map_1": {
        "field_frame_car_pose1": (0.312/2, 0.28/2, -3.14159/2),
        "field_frame_car_pose2": (0.28/2, 6-0.312/2, 3.14159),
        "field_frame_car_pose3": (12-0.312/2, 0.28/2, 3.14159/2),
        "field_frame_car_pose4": (12-0.28/2, 0.312/2, 0.0),
        # "field_frame_car_pose5": (9.45-0.314/2, 6-0.28/2, 3.14159/2),
        "odin_frame_lidar_pose1": (12.9473, 1.5453, 0.0079, -0.0021,0.9969,0.0786),
        "odin_frame_lidar_pose2": (12.2404, -3.9216, 0.0001,0.0111,0.6506,0.7594),
        "odin_frame_lidar_pose3": (1.6892, 3.3907, 0.0108,-0.0029,-0.0811,0.9966),
        "odin_frame_lidar_pose4": (1.5393, 3.2042, -0.0030,-0.0024,-0.7591,0.6510),
        # "odin_frame_lidar_pose5": (3.3818, -2.6656, -0.0031,-0.0060,-0.0699,0.9975),
    },
    "map_2": {
        "field_frame_car_pose1": (0.0, 0.0, 0.0),
        "field_frame_car_pose2": (2.0, 0.0, 0.0),
        "field_frame_car_pose3": (0.0, 2.0, 0.0),
        "field_frame_car_pose4": (2.0, 2.0, 0.0),
        "field_frame_car_pose5": (2.0, 2.0, 0.0),
        "odin_frame_lidar_pose1": (0.0, 0.0, 0.0),
        "odin_frame_lidar_pose2": (2.0, 0.0, 0.0),
        "odin_frame_lidar_pose3": (0.0, 2.0, 0.0),
        "odin_frame_lidar_pose4": (2.0, 2.0, 0.0),
        "odin_frame_lidar_pose5": (2.0, 2.0, 0.0),
    },
}


@dataclass(frozen=True, slots=True)
class OdinMapParam:
    """一张 Odin 地图的标定参数。"""

    # 场地坐标系下的车体(base_link)位姿。
    field_frame_car_poses: tuple[Odom, ...]

    # Odin 坐标系下的雷达位姿。
    odin_frame_lidar_poses: tuple[Odom, ...]

    @property
    def pose_count(self) -> int:
        return len(self.field_frame_car_poses)

    def __getattr__(self, name: str) -> Odom:
        for prefix, poses in (
            (FIELD_FRAME_CAR_POSE_PREFIX, self.field_frame_car_poses),
            (ODIN_FRAME_LIDAR_POSE_PREFIX, self.odin_frame_lidar_poses),
        ):
            if not name.startswith(prefix):
                continue

            pose_index_text = name[len(prefix):]
            if not pose_index_text.isdecimal():
                continue

            pose_index = int(pose_index_text) - 1
            if 0 <= pose_index < len(poses):
                return poses[pose_index]

        raise AttributeError(name)


@dataclass(frozen=True, slots=True)
class OdinMapAnalyzeResult:
    """Odin 地图标定参数解析后的变换结果。"""

    map_param: OdinMapParam
    scale: float
    odin_to_field: Odom
    odin_to_field_matrix: np.ndarray
    base_to_odin: Odom

    @property
    def odin_to_base(self) -> Odom:
        return self.base_to_odin.inverse()


def _to_pose(value: tuple[float, ...]) -> Odom:
    """把 (x, y)、(x, y, yaw) 或 (x, y, qx, qy, qz, qw) 转换为 Odom。"""
    if len(value) == 2:
        return Odom(
            float(value[0]),
            float(value[1]),
            0.0,
        )

    if len(value) == 3:
        return Odom(
            float(value[0]),
            float(value[1]),
            float(value[2]),
        )

    if len(value) == 6:
        return Odom(
            float(value[0]),
            float(value[1]),
            Odom.quaternion_to_yaw(
                float(value[2]),
                float(value[3]),
                float(value[4]),
                float(value[5]),
            ),
        )

    raise ValueError(
        f"\033[31m[Odin参数错误] "
        f"参数长度必须为 2、3 或 6，实际长度为 {len(value)}"
        f"\033[0m"
    )


def _collect_pose_values(
    values: dict[str, tuple[float, ...]],
    prefix: str,
    map_num: str,
) -> tuple[tuple[int, ...], tuple[tuple[float, ...], ...]]:
    plural_key = f"{prefix}s"
    has_plural_key = plural_key in values
    numbered_poses = []

    for key, value in values.items():
        if not key.startswith(prefix):
            continue

        pose_index_text = key[len(prefix):]
        if not pose_index_text.isdecimal():
            continue

        pose_index = int(pose_index_text)
        if pose_index <= 0:
            raise ValueError(
                f"\033[31m[Odin参数错误] {map_num}.{key} 编号必须从 1 开始"
                f"\033[0m"
            )
        numbered_poses.append((pose_index, value))

    if has_plural_key and numbered_poses:
        raise ValueError(
            f"\033[31m[Odin参数错误] {map_num} 不能同时配置 "
            f"{plural_key} 和 {prefix}1/{prefix}2... 两种格式"
            f"\033[0m"
        )

    if has_plural_key:
        pose_values = tuple(values[plural_key])
        pose_indices = tuple(range(1, len(pose_values) + 1))
        return pose_indices, pose_values

    numbered_poses.sort(key=lambda item: item[0])
    pose_indices = tuple(pose_index for pose_index, _ in numbered_poses)
    pose_values = tuple(pose_value for _, pose_value in numbered_poses)

    if pose_indices:
        expected_indices = tuple(range(1, pose_indices[-1] + 1))
        if pose_indices != expected_indices:
            raise ValueError(
                f"\033[31m[Odin参数错误] {map_num}.{prefix} 编号必须连续，"
                f"当前编号：{pose_indices}，期望编号：{expected_indices}"
                f"\033[0m"
            )

    return pose_indices, pose_values


def get_odin_map_param(
    map_num: str = "map_1",
) -> OdinMapParam:
    """读取指定地图并生成 OdinMapParam 实例。"""
    try:
        values = ODIN_MAP_DATA[map_num]
    except KeyError as exc:
        raise KeyError(
            f"\033[31m[Odin参数错误] 不存在地图 {map_num}，"
            f"可用地图：{', '.join(ODIN_MAP_DATA)}"
            f"\033[0m"
        ) from exc

    field_pose_indices, field_pose_values = _collect_pose_values(
        values,
        FIELD_FRAME_CAR_POSE_PREFIX,
        map_num,
    )
    lidar_pose_indices, lidar_pose_values = _collect_pose_values(
        values,
        ODIN_FRAME_LIDAR_POSE_PREFIX,
        map_num,
    )

    if field_pose_indices != lidar_pose_indices:
        raise ValueError(
            f"\033[31m[Odin参数错误] {map_num} 的场地车体位姿编号和 "
            f"Odin 雷达位姿编号必须一一对应，"
            f"当前 field 编号：{field_pose_indices}，"
            f"当前 odin 编号：{lidar_pose_indices}"
            f"\033[0m"
        )

    if len(field_pose_values) < MIN_ODIN_MAP_POSE_COUNT:
        raise ValueError(
            f"\033[31m[Odin参数错误] {map_num} 至少需要 "
            f"{MIN_ODIN_MAP_POSE_COUNT} 组 field/odin 成对位姿，"
            f"当前只有 {len(field_pose_values)} 组"
            f"\033[0m"
        )

    return OdinMapParam(
        field_frame_car_poses=tuple(_to_pose(value) for value in field_pose_values),
        odin_frame_lidar_poses=tuple(_to_pose(value) for value in lidar_pose_values),
    )


def _build_odin_map_analyze_result(
    map_param: OdinMapParam,
    odin_to_field_result: np.ndarray,
    base_to_odin_yaw: float,
) -> OdinMapAnalyzeResult:
    xy_slice = slice(0, 2)
    matrix_xy = (xy_slice, xy_slice)

    odin_x_axis_field, odin_to_field_xy, base_to_odin_xy = np.split(
        np.asarray(odin_to_field_result, dtype=np.float64),
        (2, 4),
    )
    scale = float(np.linalg.norm(odin_x_axis_field))
    odin_to_field = Odom.from_array(
        (*odin_to_field_xy, float(np.angle(complex(*odin_x_axis_field))))
    )
    odin_to_field_matrix = np.eye(3, dtype=np.float64)
    odin_to_field_matrix[matrix_xy] = np.column_stack(
        (
            odin_x_axis_field,
            SE3.from_odom(Odom(0.0, 0.0, math.pi / 2.0)).matrix[matrix_xy]
            @ odin_x_axis_field,
        )
    )
    odin_to_field_matrix[xy_slice, 2] = odin_to_field_xy

    return OdinMapAnalyzeResult(
        map_param=map_param,
        scale=scale,
        odin_to_field=odin_to_field,
        odin_to_field_matrix=odin_to_field_matrix,
        base_to_odin=Odom.from_array((*base_to_odin_xy, base_to_odin_yaw)),
    )


def _force_unit_odin_to_field_scale(
    odin_to_field_result: np.ndarray,
    field_frame_car_pose_list: tuple[Odom, ...],
    odin_frame_lidar_pose_list: tuple[Odom, ...],
    right_angle_matrix: np.ndarray,
    matrix_xy,
    matrix_translation,
) -> np.ndarray:
    odin_x_axis_field = np.asarray(odin_to_field_result[:2], dtype=np.float64)
    raw_scale = float(np.linalg.norm(odin_x_axis_field))
    if raw_scale == 0.0:
        raise ValueError(
            "\033[31m[Odin参数错误] 无法归一化 Odin 到场地坐标系的方向向量，"
            "原始缩放为 0\033[0m"
        )

    odin_x_axis_field = odin_x_axis_field / raw_scale * ODIN_MAP_TO_FIELD_SCALE
    odin_to_field_rotation = np.column_stack(
        (
            odin_x_axis_field,
            right_angle_matrix @ odin_x_axis_field,
        )
    )

    translation_slice = slice(0, 2)
    base_offset_slice = slice(2, 4)
    solve_matrix = np.zeros((len(field_frame_car_pose_list) * 2, 4), dtype=np.float64)
    solve_value = np.zeros(len(field_frame_car_pose_list) * 2, dtype=np.float64)

    for pose_index, (field_frame_car_pose, odin_frame_lidar_pose) in enumerate(
        zip(field_frame_car_pose_list, odin_frame_lidar_pose_list)
    ):
        pose_row_slice = slice(pose_index * 2, pose_index * 2 + 2)
        field_to_base_matrix = SE3.from_odom(field_frame_car_pose).matrix
        odin_to_lidar_matrix = SE3.from_odom(odin_frame_lidar_pose).matrix
        odin_to_lidar_xy = odin_to_lidar_matrix[matrix_translation]

        solve_matrix[pose_row_slice, translation_slice] = np.eye(2, dtype=np.float64)
        solve_matrix[pose_row_slice, base_offset_slice] = -field_to_base_matrix[
            matrix_xy
        ]
        solve_value[pose_row_slice] = (
            field_to_base_matrix[matrix_translation]
            - odin_to_field_rotation @ odin_to_lidar_xy
        )

    translation_and_base_offset, *_ = np.linalg.lstsq(
        solve_matrix,
        solve_value,
        rcond=None,
    )
    odin_to_field_xy, base_to_odin_xy = np.split(
        np.asarray(translation_and_base_offset, dtype=np.float64),
        (2,),
    )
    return np.asarray(
        (*odin_x_axis_field, *odin_to_field_xy, *base_to_odin_xy),
        dtype=np.float64,
    )


def analyze_odin_map_param(map_num: str = "map_1") -> OdinMapAnalyzeResult:
    """解析 Odin 地图标定参数，求出 Odin 坐标系到场地坐标系的变换。"""
    xy_slice = slice(0, 2)
    odin_axis_slice = slice(0, 2)
    translation_slice = slice(2, 4)
    base_offset_slice = slice(4, 6)
    matrix_xy = (xy_slice, xy_slice)
    matrix_translation = (xy_slice, 3)

    map_param = get_odin_map_param(map_num)

    field_frame_car_pose_list = map_param.field_frame_car_poses
    odin_frame_lidar_pose_list = map_param.odin_frame_lidar_poses

    odin_to_field_matrix = np.zeros(
        (len(field_frame_car_pose_list) * 2, 6),
        dtype=np.float64,
    )
    odin_to_field_value = np.zeros(
        len(field_frame_car_pose_list) * 2,
        dtype=np.float64,
    )
    right_angle_matrix = SE3.from_odom(Odom(0.0, 0.0, math.pi / 2.0)).matrix[
        matrix_xy
    ]

    for pose_index, (field_frame_car_pose, odin_frame_lidar_pose) in enumerate(
        zip(field_frame_car_pose_list, odin_frame_lidar_pose_list)
    ):
        pose_row_slice = slice(pose_index * 2, pose_index * 2 + 2)
        field_to_base_matrix = SE3.from_odom(field_frame_car_pose).matrix
        odin_to_lidar_matrix = SE3.from_odom(odin_frame_lidar_pose).matrix
        odin_to_lidar_xy = odin_to_lidar_matrix[matrix_translation]

        odin_to_field_matrix[pose_row_slice, odin_axis_slice] = np.column_stack(
            (odin_to_lidar_xy, right_angle_matrix @ odin_to_lidar_xy)
        )
        odin_to_field_matrix[pose_row_slice, translation_slice] = np.eye(
            2,
            dtype=np.float64,
        )
        odin_to_field_matrix[pose_row_slice, base_offset_slice] = (
            -field_to_base_matrix[matrix_xy]
        )
        odin_to_field_value[pose_row_slice] = field_to_base_matrix[
            matrix_translation
        ]

    odin_to_field_result, *_ = np.linalg.lstsq(
        odin_to_field_matrix,
        odin_to_field_value,
        rcond=None,
    )
    odin_to_field_result = _force_unit_odin_to_field_scale(
        odin_to_field_result,
        field_frame_car_pose_list,
        odin_frame_lidar_pose_list,
        right_angle_matrix,
        matrix_xy,
        matrix_translation,
    )
    odin_to_field_analyze_result = _build_odin_map_analyze_result(
        map_param,
        odin_to_field_result,
        0.0,
    )

    base_to_odin_matrix = np.zeros((2, 2), dtype=np.float64)
    for field_frame_car_pose, odin_frame_lidar_pose in zip(
        field_frame_car_pose_list,
        odin_frame_lidar_pose_list,
    ):
        base_to_odin_yaw = (
            odin_to_field_analyze_result.odin_to_field.yaw
            + odin_frame_lidar_pose.yaw
            - field_frame_car_pose.yaw
        )
        base_to_odin_matrix += SE3.from_odom(
            Odom(0.0, 0.0, base_to_odin_yaw)
        ).matrix[matrix_xy]

    base_to_odin_se3_matrix = np.eye(4, dtype=np.float64)
    base_to_odin_se3_matrix[matrix_xy] = base_to_odin_matrix
    return _build_odin_map_analyze_result(
        map_param,
        odin_to_field_result,
        SE3(base_to_odin_se3_matrix).to_odom().yaw,
    )
