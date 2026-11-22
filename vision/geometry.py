"""雷达坐标到图像的投影。

雷达系：x 右、y 前、z 上，原点在雷达。
相机系：x 右、y 下、z 前，原点在相机（OpenCV）。

安装角写在雷达系里，旋转顺序 R = Rz(yaw) @ Ry(roll) @ Rx(pitch)：
正 yaw 表示雷达朝向相对相机逆时针偏（往左看）；
正 pitch 表示雷达光轴抬头，投影点在画面上移（v 变小）。
然后再做轴映射：X = x' + tx，Y = -z' + ty，Z = y' + tz。
"""

from __future__ import annotations

import math

import numpy as np

from .config import FusionConfig


def radar_rotation(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    yaw, pitch, roll = (math.radians(v) for v in (yaw_deg, pitch_deg, roll_deg))
    cz, sz = math.cos(yaw), math.sin(yaw)
    cy, sy = math.cos(roll), math.sin(roll)
    cx, sx = math.cos(pitch), math.sin(pitch)
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    return rz @ ry @ rx


def radar_to_camera(x: float, y: float, z: float, cfg: FusionConfig) -> np.ndarray:
    rotated = radar_rotation(cfg.yaw_deg, cfg.pitch_deg, cfg.roll_deg) @ np.array(
        [x, y, z], dtype=float
    )
    # 轴映射单独做，避免把“雷达朝前”和“相机朝前”写成同一个轴。
    return np.array(
        [
            rotated[0] + cfg.tx_m,
            -rotated[2] + cfg.ty_m,
            rotated[1] + cfg.tz_m,
        ],
        dtype=float,
    )


def project_radar(x: float, y: float, z: float, cfg: FusionConfig):
    """返回 (u, v, ok)。Z 太小或在相机后面时 ok=False。"""
    cam = radar_to_camera(x, y, z, cfg)
    if cam[2] <= 0.15:
        return 0.0, 0.0, False
    u = cfg.fx * cam[0] / cam[2] + cfg.cx
    v = cfg.fy * cam[1] / cam[2] + cfg.cy
    return float(u), float(v), True


def camera_azimuth(u: float, cfg: FusionConfig) -> float:
    """图像列对应的相机水平角，右为正，单位弧度。与距离无关。"""
    return math.atan((u - cfg.cx) / cfg.fx)


def xy_from_range_and_pixel(range_m: float, u: float, z: float, cfg: FusionConfig):
    """用雷达斜距（水平）和视觉列坐标反解雷达平面位置。

    在当前外参下搜索方位角，使投影的 u 与检测一致。
    这样融合测量和 project_radar 使用同一套外参，不会各写一套公式。
    """
    radius = max(float(range_m), 0.05)
    lo, hi = -1.35, 1.35

    def column_error(azimuth: float) -> float:
        uu, _vv, ok = project_radar(
            radius * math.sin(azimuth), radius * math.cos(azimuth), z, cfg
        )
        if not ok:
            return -1e6 if azimuth < 0 else 1e6
        return uu - u

    for _ in range(24):
        mid = 0.5 * (lo + hi)
        if column_error(mid) < 0.0:
            lo = mid
        else:
            hi = mid
    azimuth = 0.5 * (lo + hi)
    return radius * math.sin(azimuth), radius * math.cos(azimuth)


def suggest_yaw_deg(radar_x: float, radar_y: float, u: float, cfg: FusionConfig) -> float:
    """人站在画面中心附近时，给 yaw 一个初值。

    忽略平移和俯仰，只适合 tx、tz 接近 0 的粗标。
    公式来自 yaw=0 附近：相机水平角 = 雷达方位角 - yaw。
    """
    radar_az = math.atan2(radar_x, radar_y)
    cam_az = camera_azimuth(u, cfg)
    return math.degrees(radar_az - cam_az)


def point_box_distance(u: float, v: float, x1: float, y1: float, x2: float, y2: float):
    dx = 0.0 if x1 <= u <= x2 else (x1 - u if u < x1 else u - x2)
    dy = 0.0 if y1 <= v <= y2 else (y1 - v if v < y1 else v - y2)
    return dx, dy


def inside_pixel_gate(u: float, v: float, x1: float, y1: float, x2: float, y2: float, cfg: FusionConfig) -> bool:
    dx, dy = point_box_distance(u, v, x1, y1, x2, y2)
    gx = max(cfg.assoc_gate_px_x, 1.0)
    gy = max(cfg.assoc_gate_px_y, 1.0)
    return (dx / gx) ** 2 + (dy / gy) ** 2 <= 1.0


def covariance_along_ray(x: float, y: float, sigma_cross: float, sigma_along: float) -> np.ndarray:
    """沿目标方向和横向分别给方差，再转回 x/y。"""
    radius = math.hypot(x, y)
    if radius < 1e-4:
        return np.diag([sigma_cross ** 2, sigma_along ** 2])
    along = np.array([x / radius, y / radius])
    cross = np.array([-y / radius, x / radius])
    return (sigma_along ** 2) * np.outer(along, along) + (sigma_cross ** 2) * np.outer(cross, cross)
