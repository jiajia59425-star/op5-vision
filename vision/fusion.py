"""雷达 + 行人视觉融合。

不稳定通常来自三件事，这里都按可单测的方式处理，而不是只调一个阈值：

1. 坐标。雷达 x/y 不能当成图像像素，也不能当成相机的 x/y。
   先用外参变到相机系再投影，关联发生在图像上；
   更新航迹时用雷达的距离、视觉的水平角，两者在雷达平面合成。
2. 时间。调用方要传入已经对齐到相机时刻的雷达目标
   （RadarTimeline.at）。本模块不读取“最新雷达帧”。
3. 关联。投影点必须落在检测框的门里，航迹更新还要过马氏距离门，
   并且雷达、视觉、航迹都是一对一。对不上的雷达点不能把行人拽走。

非 person 检测在入口丢掉，植物不会建航迹。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import FusionConfig
from .detector import Detection, filter_people
from .geometry import covariance_along_ray, inside_pixel_gate, project_radar, xy_from_range_and_pixel
from .radar_io import RadarTarget


@dataclass
class FusionTrack:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    p: np.ndarray
    hits: int
    seen_person: bool
    born_t: float
    last_meas_t: float
    score: float
    source: str
    u: float = 0.0
    v: float = 0.0
    _confirmed_flag: bool = False

    @property
    def confirmed(self) -> bool:
        return self.hits >= 1 and self.seen_person and self._confirmed_flag

    def range_m(self) -> float:
        return math.hypot(self.x, self.y)

    def bearing_deg(self) -> float:
        return math.degrees(math.atan2(self.x, self.y))

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "x_m": round(self.x, 3),
            "y_m": round(self.y, 3),
            "vx_mps": round(self.vx, 3),
            "vy_mps": round(self.vy, 3),
            "range_m": round(self.range_m(), 3),
            "bearing_deg": round(self.bearing_deg(), 2),
            "source": self.source,
            "hits": self.hits,
            "score": round(self.score, 3),
            "u": None if self.u != self.u else round(self.u, 1),
            "v": None if self.v != self.v else round(self.v, 1),
            "class_name": "person",
        }


@dataclass
class _Meas:
    x: float
    y: float
    r: np.ndarray
    kind: str
    score: float
    radar_index: int | None = None
    det_index: int | None = None
    # 视觉测量要带框，更新前再用投影点做一次门控。
    x1: float = 0.0
    y1: float = 0.0
    x2: float = 0.0
    y2: float = 0.0
    has_box: bool = False


class FusionEngine:
    def __init__(self, cfg: FusionConfig):
        self.cfg = cfg
        self.tracks: list[FusionTrack] = []
        self._next_id = 1
        self.last_debug: dict = {}

    def update(self, t: float, detections: list[Detection], radars: list[RadarTarget]) -> list[FusionTrack]:
        cfg = self.cfg
        people, dropped = filter_people(detections, cfg)
        radars = [item for item in radars if _range_ok(item, cfg)]
        self._predict(t)
        pairs, free_radar, free_vision = _pair_sensors(radars, people, cfg)
        measurements = [_fused_meas(radars[i], people[j], cfg) for i, j in pairs]
        radar_meas = [_radar_meas(radars[i], cfg, i) for i in free_radar]
        vision_meas = [_vision_meas(people[j], cfg, j) for j in free_vision]
        vision_meas = [item for item in vision_meas if item is not None]
        assigned = _assign(self.tracks, measurements, radar_meas, vision_meas, cfg)
        used_tracks = set()
        used_fused = set()
        used_radar = set()
        used_vision = set()
        for track_index, meas in assigned:
            self._apply(self.tracks[track_index], meas, t)
            used_tracks.add(track_index)
            if meas.kind == "fused":
                used_fused.add(id(meas))
            elif meas.kind == "radar":
                used_radar.add(meas.radar_index)
            elif meas.kind == "vision":
                used_vision.add(meas.det_index)
        for meas in measurements:
            if id(meas) in used_fused:
                continue
            self._spawn(meas, t)
        for meas in radar_meas:
            if meas.radar_index in used_radar:
                continue
            self._spawn(meas, t)
        for meas in vision_meas:
            if meas.det_index in used_vision:
                continue
            self._spawn(meas, t)
        self._prune(t)
        self._project_tracks()
        confirmed = [track for track in self.tracks if track._confirmed_flag]
        self.last_debug = {
            "dropped_non_person": dropped,
            "radar_targets": len(radars),
            "vision_people": len(people),
            "fused_pairs": len(pairs),
            "confirmed": len(confirmed),
            "tentative": len(self.tracks) - len(confirmed),
        }
        return confirmed

    def _predict(self, t: float) -> None:
        q = self.cfg.process_noise_accel
        limit = self.cfg.max_predict_dt_s
        for track in self.tracks:
            # 用独立的预测时刻，避免同一帧重复积分。
            if not hasattr(track, "_pred_t"):
                track._pred_t = track.last_meas_t
            step = min(max(t - track._pred_t, 0.0), limit)
            track._pred_t = t
            if step <= 1e-6:
                continue
            state = np.array([track.x, track.y, track.vx, track.vy], dtype=float)
            state, cov = _kf_predict(state, track.p, step, q)
            track.x, track.y, track.vx, track.vy = (float(v) for v in state)
            track.p = cov

    def _apply(self, track: FusionTrack, meas: _Meas, t: float) -> None:
        state = np.array([track.x, track.y, track.vx, track.vy], dtype=float)
        state, cov = _kf_update(state, track.p, np.array([meas.x, meas.y]), meas.r)
        track.x, track.y, track.vx, track.vy = (float(v) for v in state)
        track.p = cov
        track.hits += 1
        track.last_meas_t = t
        track.score = meas.score
        track.source = meas.kind
        if meas.kind in ("fused", "vision"):
            track.seen_person = True
        self._mark_confirmed(track)

    def _spawn(self, meas: _Meas, t: float) -> None:
        if any(math.hypot(track.x - meas.x, track.y - meas.y) < 0.45 for track in self.tracks):
            return
        cov = np.diag([0.6 ** 2, 0.6 ** 2, 1.5 ** 2, 1.5 ** 2])
        track = FusionTrack(
            id=self._next_id,
            x=meas.x,
            y=meas.y,
            vx=0.0,
            vy=0.0,
            p=cov,
            hits=1,
            seen_person=meas.kind in ("fused", "vision"),
            born_t=t,
            last_meas_t=t,
            score=meas.score,
            source=meas.kind,
        )
        track._pred_t = t
        self._next_id += 1
        self._mark_confirmed(track)
        self.tracks.append(track)

    def _mark_confirmed(self, track: FusionTrack) -> None:
        if track.hits < self.cfg.min_hits:
            track._confirmed_flag = False
            return
        if self.cfg.require_vision_for_confirm and not track.seen_person:
            track._confirmed_flag = False
            return
        track._confirmed_flag = True

    def _prune(self, t: float) -> None:
        kept = []
        for track in self.tracks:
            age_since = t - track.last_meas_t
            if age_since > self.cfg.max_coast_s:
                continue
            # 一直没有被人眼确认的雷达杂波，到时就删，避免占着关联。
            if not track.seen_person and t - track.born_t > self.cfg.tentative_ttl_s:
                continue
            kept.append(track)
        self.tracks = kept

    def _project_tracks(self) -> None:
        z = self.cfg.aim_z_m
        for track in self.tracks:
            u, v, ok = project_radar(track.x, track.y, z, self.cfg)
            track.u = u if ok else float("nan")
            track.v = v if ok else float("nan")


def _range_ok(target: RadarTarget, cfg: FusionConfig) -> bool:
    radius = math.hypot(target.x, target.y)
    if target.y <= 0.0:
        return False
    return cfg.min_range_m <= radius <= cfg.max_range_m


def _pair_sensors(radars: list[RadarTarget], people: list[Detection], cfg: FusionConfig):
    candidates = []
    z = cfg.aim_z_m
    for i, radar in enumerate(radars):
        u, v, ok = project_radar(radar.x, radar.y, z, cfg)
        if not ok:
            continue
        for j, det in enumerate(people):
            if inside_pixel_gate(u, v, det.x1, det.y1, det.x2, det.y2, cfg):
                dx, dy = det.cx - u, det.cy - v
                candidates.append((dx * dx + dy * dy, i, j))
    candidates.sort(key=lambda item: item[0])
    used_r, used_d = set(), set()
    pairs = []
    for _cost, i, j in candidates:
        if i in used_r or j in used_d:
            continue
        used_r.add(i)
        used_d.add(j)
        pairs.append((i, j))
    free_radar = [i for i in range(len(radars)) if i not in used_r]
    free_vision = [j for j in range(len(people)) if j not in used_d]
    return pairs, free_radar, free_vision


def _fused_meas(radar: RadarTarget, det: Detection, cfg: FusionConfig) -> _Meas:
    radius = math.hypot(radar.x, radar.y)
    # 距离信雷达，水平位置信视觉。外参已经包含在反投影里。
    x, y = xy_from_range_and_pixel(radius, det.cx, cfg.aim_z_m, cfg)
    cov = covariance_along_ray(x, y, cfg.fused_sigma_cross_m, cfg.fused_sigma_along_m)
    return _Meas(x, y, cov, "fused", det.score, None, None, det.x1, det.y1, det.x2, det.y2, True)


def _radar_meas(radar: RadarTarget, cfg: FusionConfig, index: int) -> _Meas:
    cov = np.diag([cfg.radar_sigma_x_m ** 2, cfg.radar_sigma_y_m ** 2])
    return _Meas(radar.x, radar.y, cov, "radar", 0.2, index, None)


def _vision_meas(det: Detection, cfg: FusionConfig, index: int):
    # 没有雷达时用框高估距离，沿径向方差很大，主要约束横向。
    z_cam = _range_from_bbox(det, cfg)
    x, y = xy_from_range_and_pixel(z_cam, det.cx, cfg.aim_z_m, cfg)
    cov = covariance_along_ray(x, y, cfg.vision_sigma_cross_m, cfg.vision_sigma_along_m)
    return _Meas(x, y, cov, "vision", det.score * 0.6, None, index, det.x1, det.y1, det.x2, det.y2, True)


def _range_from_bbox(det: Detection, cfg: FusionConfig) -> float:
    # Z ≈ fy * 身高 / 像素高。人被切到画面边缘时框高不可信，后面方差已经放大。
    rough = cfg.fy * cfg.person_height_m / det.height
    return float(np.clip(rough, cfg.min_range_m, cfg.max_range_m))


def _vision_projects_into(track: FusionTrack, meas: _Meas, cfg: FusionConfig) -> bool:
    if not meas.has_box:
        return True
    u, v, ok = project_radar(track.x, track.y, cfg.aim_z_m, cfg)
    if not ok:
        return False
    return inside_pixel_gate(u, v, meas.x1, meas.y1, meas.x2, meas.y2, cfg)


def _mahalanobis(track: FusionTrack, meas: _Meas) -> float:
    diff = np.array([meas.x - track.x, meas.y - track.y])
    cov = track.p[:2, :2] + meas.r
    try:
        solved = np.linalg.solve(cov, diff)
    except np.linalg.LinAlgError:
        return 1e9
    return float(diff @ solved)


def _assign(tracks, fused, radar_meas, vision_meas, cfg: FusionConfig):
    candidates = []
    for ti, track in enumerate(tracks):
        for meas in fused:
            dist = _mahalanobis(track, meas)
            if dist <= cfg.mahalanobis_gate:
                candidates.append((dist, ti, meas))
        for meas in radar_meas:
            dist = _mahalanobis(track, meas) + cfg.single_sensor_penalty
            if (
                dist <= cfg.mahalanobis_gate + cfg.single_sensor_penalty
                and _euclidean(track, meas) <= cfg.radar_gate_m
            ):
                candidates.append((dist, ti, meas))
        for meas in vision_meas:
            if not _vision_projects_into(track, meas, cfg):
                continue
            dist = _mahalanobis(track, meas) + cfg.single_sensor_penalty
            if dist <= cfg.mahalanobis_gate + cfg.single_sensor_penalty:
                candidates.append((dist, ti, meas))
    candidates.sort(key=lambda item: item[0])
    used_t = set()
    used_m = set()
    assigned = []
    for _cost, ti, meas in candidates:
        marker = id(meas)
        if ti in used_t or marker in used_m:
            continue
        used_t.add(ti)
        used_m.add(marker)
        assigned.append((ti, meas))
    return assigned


def _euclidean(track: FusionTrack, meas: _Meas) -> float:
    return math.hypot(track.x - meas.x, track.y - meas.y)


def _kf_predict(state: np.ndarray, cov: np.ndarray, dt: float, sigma_a: float):
    f = np.array(
        [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt2 * dt2
    q = sigma_a ** 2
    process = q * np.array(
        [
            [dt4 / 4.0, 0.0, dt3 / 2.0, 0.0],
            [0.0, dt4 / 4.0, 0.0, dt3 / 2.0],
            [dt3 / 2.0, 0.0, dt2, 0.0],
            [0.0, dt3 / 2.0, 0.0, dt2],
        ]
    )
    state = f @ state
    cov = f @ cov @ f.T + process
    return state, 0.5 * (cov + cov.T)


def _kf_update(state: np.ndarray, cov: np.ndarray, z: np.ndarray, r: np.ndarray):
    h = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
    innovation = z - h @ state
    s = h @ cov @ h.T + r
    k = cov @ h.T @ np.linalg.inv(s)
    state = state + k @ innovation
    eye = np.eye(4)
    cov = (eye - k @ h) @ cov @ (eye - k @ h).T + k @ r @ k.T
    return state, 0.5 * (cov + cov.T)
