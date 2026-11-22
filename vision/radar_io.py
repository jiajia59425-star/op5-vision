"""雷达输入。

LD2450 上报帧（协议 V1.03）：
AA FF 03 00 + 3 个目标 × 8 字节 + 55 CC，共 30 字节，默认波特率 256000。
坐标单位 mm，符号位与普通 int16 相反：最高位 1 为正、0 为负。
速度单位 cm/s，正值表示靠近，负值表示远离。

其他雷达走 JSON 行，避免把私有协议写死：
{"t": 12.3, "targets": [{"x": 0.2, "y": 3.1, "vx": 0.0, "vy": -0.4, "id": 1}]}
x 右、y 前，单位米。若 unit 为 "mm" 则自动换算。
"""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass, field


@dataclass
class RadarTarget:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    track_id: int | None = None
    speed_mps: float | None = None
    resolution_m: float | None = None

    def moved(self, dt: float) -> "RadarTarget":
        return RadarTarget(
            self.x + self.vx * dt,
            self.y + self.vy * dt,
            self.vx,
            self.vy,
            self.track_id,
            self.speed_mps,
            self.resolution_m,
        )


@dataclass
class RadarScan:
    t: float
    targets: list[RadarTarget] = field(default_factory=list)


def _decode_signed(low: int, high: int) -> int:
    """LD2450：bit15=1 为正，bit15=0 为负，低 15 位是幅值。"""
    magnitude = ((high & 0x7F) << 8) | low
    if high & 0x80:
        return magnitude
    return -magnitude


def velocity_from_radial(x_m: float, y_m: float, speed_mps: float) -> tuple[float, float]:
    """正速度=靠近，所以距离变化率是 -speed。"""
    radius = math.hypot(x_m, y_m)
    if radius < 1e-6:
        return 0.0, 0.0
    range_rate = -speed_mps
    return range_rate * x_m / radius, range_rate * y_m / radius


def parse_ld2450_frame(frame: bytes, t: float) -> RadarScan:
    if len(frame) != 30 or frame[:4] != b"\xAA\xFF\x03\x00" or frame[-2:] != b"\x55\xCC":
        raise ValueError("不是 LD2450 目标帧")
    targets = []
    for index in range(3):
        block = frame[4 + index * 8 : 12 + index * 8]
        x_mm = _decode_signed(block[0], block[1])
        y_mm = _decode_signed(block[2], block[3])
        speed_cm = _decode_signed(block[4], block[5])
        resolution_mm = block[6] | (block[7] << 8)
        if x_mm == 0 and y_mm == 0:
            continue
        x_m = x_mm / 1000.0
        y_m = y_mm / 1000.0
        speed_mps = speed_cm / 100.0
        vx, vy = velocity_from_radial(x_m, y_m, speed_mps)
        targets.append(
            RadarTarget(
                x=x_m,
                y=y_m,
                vx=vx,
                vy=vy,
                track_id=index + 1,
                speed_mps=speed_mps,
                resolution_m=resolution_mm / 1000.0,
            )
        )
    return RadarScan(t=t, targets=targets)


class LD2450Parser:
    """从串口字节流里切出 30 字节上报帧。配置帧（FD FC FB FA）会被跳过。"""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes, t: float) -> list[RadarScan]:
        self._buf.extend(data)
        scans = []
        while True:
            start = self._buf.find(b"\xAA\xFF\x03\x00")
            if start < 0:
                # 保留可能跨包的帧头残片。
                if len(self._buf) > 3:
                    del self._buf[:-3]
                break
            if start > 0:
                del self._buf[:start]
            if len(self._buf) < 30:
                break
            frame = bytes(self._buf[:30])
            del self._buf[:30]
            if frame[-2:] != b"\x55\xCC":
                continue
            scans.append(parse_ld2450_frame(frame, t))
        return scans


def parse_radar_json(line: str, fallback_t: float, default_unit: str = "m") -> RadarScan:
    payload = json.loads(line)
    unit = str(payload.get("unit", default_unit))
    scale = 0.001 if unit == "mm" else 1.0
    stamp = payload.get("t", payload.get("timestamp"))
    # Unix 时间不能和 time.monotonic 混用，调用方看到 None 就用接收时刻。
    if stamp is None or float(stamp) > 1e8:
        t = fallback_t
    else:
        t = float(stamp)
    targets = []
    for item in payload.get("targets") or []:
        x = float(item["x"]) * scale
        y = float(item["y"]) * scale
        vx = float(item.get("vx", 0.0)) * scale
        vy = float(item.get("vy", 0.0)) * scale
        track_id = item.get("id", item.get("track_id"))
        targets.append(
            RadarTarget(
                x=x,
                y=y,
                vx=vx,
                vy=vy,
                track_id=None if track_id is None else int(track_id),
            )
        )
    return RadarScan(t=t, targets=targets)


class RadarTimeline:
    """按场景时间插值雷达目标。不能拿“最新一帧”去配当前图像。"""

    def __init__(self, max_age_s: float = 1.5):
        self.max_age_s = max_age_s
        self.scans: deque[RadarScan] = deque()

    def add(self, scan: RadarScan) -> None:
        self.scans.append(scan)
        while self.scans and scan.t - self.scans[0].t > self.max_age_s:
            self.scans.popleft()

    def at(
        self,
        t_scene: float,
        radar_time_offset_s: float,
        max_skew_s: float,
        max_gap_s: float,
        match_gate_m: float,
    ) -> list[RadarTarget]:
        if not self.scans:
            return []
        corrected = [(scan.t - radar_time_offset_s, scan) for scan in self.scans]
        earlier = [item for item in corrected if item[0] <= t_scene]
        later = [item for item in corrected if item[0] >= t_scene]
        if earlier and later:
            t0, left = earlier[-1]
            t1, right = later[0]
            if t1 - t0 <= 1e-6:
                return [tgt.moved(0.0) for tgt in left.targets]
            if t1 - t0 <= max_gap_s:
                return _interpolate(left.targets, right.targets, t0, t1, t_scene, match_gate_m)
        nearest_t, nearest = min(corrected, key=lambda item: abs(item[0] - t_scene))
        dt = t_scene - nearest_t
        if abs(dt) > max_skew_s:
            return []
        return [tgt.moved(dt) for tgt in nearest.targets]


def _match_targets(left: list[RadarTarget], right: list[RadarTarget], gate_m: float):
    pairs = []
    for i, a in enumerate(left):
        for j, b in enumerate(right):
            if a.track_id is not None and a.track_id == b.track_id:
                cost = -1.0
            else:
                cost = math.hypot(a.x - b.x, a.y - b.y)
                if cost > gate_m:
                    continue
            pairs.append((cost, i, j))
    pairs.sort(key=lambda item: item[0])
    used_l, used_r = set(), set()
    matched = []
    for _cost, i, j in pairs:
        if i in used_l or j in used_r:
            continue
        used_l.add(i)
        used_r.add(j)
        matched.append((i, j))
    return matched


def _interpolate(left, right, t0, t1, t_query, gate_m):
    alpha = (t_query - t0) / (t1 - t0)
    matched = _match_targets(left, right, gate_m)
    output = []
    for i, j in matched:
        a, b = left[i], right[j]
        dt = t1 - t0
        vx = (b.x - a.x) / dt
        vy = (b.y - a.y) / dt
        output.append(
            RadarTarget(
                x=(1.0 - alpha) * a.x + alpha * b.x,
                y=(1.0 - alpha) * a.y + alpha * b.y,
                vx=vx,
                vy=vy,
                track_id=a.track_id if a.track_id is not None else b.track_id,
                speed_mps=a.speed_mps,
                resolution_m=a.resolution_m,
            )
        )
    return output
