"""融合参数。缺省值只能让程序跑起来，上机前要按 docs/融合调参.md 标定。"""

from __future__ import annotations

import json
from dataclasses import dataclass, fields, asdict
from pathlib import Path


@dataclass
class FusionConfig:
    # 相机内参。calibrated=false 时服务会告警，投影只适合看趋势。
    calibrated: bool = False
    width: int = 640
    height: int = 480
    fx: float = 600.0
    fy: float = 600.0
    cx: float = 320.0
    cy: float = 240.0

    # 雷达原点在相机坐标系的平移（米）。相机系：x 右、y 下、z 前。
    tx_m: float = 0.0
    ty_m: float = 0.0
    tz_m: float = 0.0
    # 雷达安装角（度），定义见 geometry.radar_rotation。
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0

    # 二维雷达没有高度。投影用的身体点 = 瞄准高度 - 雷达安装高度。
    radar_mount_height_m: float = 0.8
    person_aim_height_m: float = 1.05
    person_height_m: float = 1.70

    # 正值表示：同一姿态下，雷达时间戳比相机时间戳更晚（雷达数据偏旧）。
    radar_time_offset_s: float = 0.0
    camera_latency_s: float = 0.0
    max_time_skew_s: float = 0.18
    max_interp_gap_s: float = 0.25
    interp_match_m: float = 1.2
    radar_buffer_s: float = 1.5

    # 关联门限。垂直方向放宽，因为身高假设误差主要体现在 v。
    assoc_gate_px_x: float = 70.0
    assoc_gate_px_y: float = 140.0
    mahalanobis_gate: float = 9.21
    single_sensor_penalty: float = 1.5
    radar_gate_m: float = 1.6
    min_range_m: float = 0.3
    max_range_m: float = 18.0

    # 噪声（米、米/秒²）。雷达横向比径向差，所以 sigma_x 更大。
    radar_sigma_x_m: float = 0.40
    radar_sigma_y_m: float = 0.18
    fused_sigma_cross_m: float = 0.10
    fused_sigma_along_m: float = 0.18
    vision_sigma_cross_m: float = 0.15
    vision_sigma_along_m: float = 1.6
    process_noise_accel: float = 2.5
    max_predict_dt_s: float = 0.4

    min_hits: int = 3
    max_coast_s: float = 0.45
    tentative_ttl_s: float = 0.70
    require_vision_for_confirm: bool = True

    # 检测。允许名单，不是排除名单：植物、车和其他类都不会进来。
    allowed_class_names: tuple = ("person",)
    allowed_class_ids: tuple = (0,)
    det_conf: float = 0.35
    det_iou: float = 0.45
    det_input_size: int = 640
    detector_backend: str = "auto"  # auto | rknn | onnx | none
    rknn_model: str = "models/yolov8n.rknn"
    onnx_model: str = "models/yolov8n.onnx"
    # RK3588 三核。RKNNLite.NPU_CORE_0_1_2 == 7。
    rknn_core_mask: int = 7

    camera_source: str = "0"
    max_fps: float = 15.0
    web_host: str = "0.0.0.0"
    web_port: int = 8081
    jpeg_quality: int = 70

    radar_mode: str = "ld2450"  # ld2450 | jsonl | none
    radar_serial_port: str = "/dev/ttyUSB0"
    radar_baud: int = 256000
    radar_udp_port: int = 8090
    radar_json_unit: str = "m"

    @property
    def aim_z_m(self) -> float:
        """雷达坐标系里用于投影的身体点高度（向上为正）。"""
        return self.person_aim_height_m - self.radar_mount_height_m


def _coerce(field_type, value):
    # 有 from __future__ import annotations 时，dataclass 里的类型是字符串。
    name = field_type if isinstance(field_type, str) else getattr(field_type, "__name__", "")
    origin = getattr(field_type, "__origin__", None)
    if origin is tuple or field_type is tuple or name in ("tuple", "Tuple"):
        return tuple(value)
    if field_type is float or name == "float":
        return float(value)
    if field_type is int or name == "int":
        return int(value)
    if field_type is bool or name == "bool":
        return bool(value)
    if field_type is str or name == "str":
        return str(value)
    return value


def config_from_dict(data: dict) -> FusionConfig:
    known = {item.name for item in fields(FusionConfig)}
    unknown = sorted(set(data) - known)
    if unknown:
        raise ValueError("未知配置项: " + ", ".join(unknown))
    kwargs = {}
    types = {item.name: item.type for item in fields(FusionConfig)}
    for key, value in data.items():
        kwargs[key] = _coerce(types[key], value)
    return FusionConfig(**kwargs)


def load_config(path: str | Path) -> FusionConfig:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    payload.pop("_comment", None)
    return config_from_dict(payload)


def dump_defaults() -> dict:
    raw = asdict(FusionConfig())
    for key, value in list(raw.items()):
        if isinstance(value, tuple):
            raw[key] = list(value)
    return raw
