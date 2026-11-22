"""只输出 person。

COCO 里 person 是类别 0，potted plant 是 58。这里用允许名单：
名字非空时必须是 person；没有名字时才看类别号。
因此植物、车、以及名字标错成植物但 id 碰巧为 0 的框都不会进入跟踪。

RK3588 优先走 rknn-toolkit-lite2（NPU）。没有 rknn 模型时可以用 onnx + OpenCV DNN，
方便在没有 NPU 的机器上检查后处理。融合单元测试不加载模型。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import FusionConfig


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    class_name: str = ""
    class_id: int = -1

    @property
    def cx(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def height(self) -> float:
        return max(1.0, self.y2 - self.y1)


def is_person(class_id: int, class_name: str, cfg: FusionConfig) -> bool:
    allowed_names = {name.strip().lower() for name in cfg.allowed_class_names}
    name = (class_name or "").strip().lower()
    if name:
        return name in allowed_names
    return int(class_id) in set(cfg.allowed_class_ids)


def filter_people(detections: list[Detection], cfg: FusionConfig) -> tuple[list[Detection], int]:
    kept = []
    dropped = 0
    for det in detections:
        if is_person(det.class_id, det.class_name, cfg):
            kept.append(det)
        else:
            dropped += 1
    return kept, dropped


def _sigmoid(value: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(value, -40.0, 40.0)))


def _xywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
    out = np.empty_like(boxes)
    out[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
    out[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
    out[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.0
    out[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.0
    return out


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_thres: float) -> list[int]:
    if len(boxes) == 0:
        return []
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        area_i = max(0.0, boxes[i, 2] - boxes[i, 0]) * max(0.0, boxes[i, 3] - boxes[i, 1])
        area_r = np.maximum(0.0, boxes[rest, 2] - boxes[rest, 0]) * np.maximum(
            0.0, boxes[rest, 3] - boxes[rest, 1]
        )
        union = area_i + area_r - inter
        iou = np.where(union > 0, inter / union, 0.0)
        order = rest[iou <= iou_thres]
    return keep


def _scale_box(box, scale, pad_x, pad_y, width, height):
    x1 = (box[0] - pad_x) / scale
    y1 = (box[1] - pad_y) / scale
    x2 = (box[2] - pad_x) / scale
    y2 = (box[3] - pad_y) / scale
    x1 = float(np.clip(x1, 0, width - 1))
    y1 = float(np.clip(y1, 0, height - 1))
    x2 = float(np.clip(x2, 0, width - 1))
    y2 = float(np.clip(y2, 0, height - 1))
    return x1, y1, x2, y2


def _channels_first(arr: np.ndarray) -> np.ndarray:
    """把输出收成 (4+类别, 候选框)。

    常见是 (84, 8400) 或 (8400, 84)。候选框很少时不能靠“哪一边更长”，
    所以先认 5 / 84 / 85 这些通道数（4 个框参数 + 类别或单个分数）。
    """
    known = {5, 6, 84, 85}
    if arr.shape[0] in known and arr.shape[0] != arr.shape[1]:
        return arr
    if arr.shape[1] in known and arr.shape[0] != arr.shape[1]:
        return arr.T
    if arr.shape[0] <= arr.shape[1]:
        return arr
    return arr.T


def decode_yolov8_people(
    prediction,
    cfg: FusionConfig,
    scale: float,
    pad_x: float,
    pad_y: float,
    width: int,
    height: int,
) -> list[Detection]:
    """解析 YOLOv8 输出里的 person。

    支持两种常见 RKNN / ONNX 头：
    - (4+nc, N) 或 (N, 4+nc)：cx, cy, w, h + 各类别分数，person 在第 0 类
    - (N, 6)：x1, y1, x2, y2, score, class_id
    分数若明显是 logits（有负数或大于 1），会先做 sigmoid。
    """
    arr = np.asarray(prediction, dtype=np.float32)
    if arr.ndim == 3:
        arr = arr[0]
    if arr.ndim != 2:
        raise ValueError(f"无法识别的检测输出形状 {arr.shape}")

    if arr.shape[1] == 6 and arr.shape[0] != 6:
        boxes = arr[:, :4]
        scores = arr[:, 4]
        classes = arr[:, 5]
        if np.nanmax(scores) > 1.2 or np.nanmin(scores) < 0.0:
            scores = _sigmoid(scores)
        detections = []
        for box, score, class_id in zip(boxes, scores, classes):
            if float(score) < cfg.det_conf:
                continue
            if int(round(float(class_id))) not in set(cfg.allowed_class_ids):
                continue
            x1, y1, x2, y2 = _scale_box(box, scale, pad_x, pad_y, width, height)
            if x2 - x1 < 2 or y2 - y1 < 2:
                continue
            detections.append(Detection(x1, y1, x2, y2, float(score), "person", 0))
        return detections

    arr = _channels_first(arr)
    if arr.shape[0] < 5:
        raise ValueError(f"检测通道太少: {arr.shape}")
    boxes = _xywh_to_xyxy(arr[0:4].T)
    class_scores = arr[4:]
    person = class_scores[0].astype(np.float32)
    if np.nanmax(person) > 1.2 or np.nanmin(person) < 0.0:
        person = _sigmoid(person)
    # 只和 person 通道比阈值。其他类即使分数更高也不保留。
    mask = person >= cfg.det_conf
    boxes = boxes[mask]
    person = person[mask]
    keep = _nms(boxes, person, cfg.det_iou)
    detections = []
    for index in keep:
        x1, y1, x2, y2 = _scale_box(boxes[index], scale, pad_x, pad_y, width, height)
        if x2 - x1 < 2 or y2 - y1 < 2:
            continue
        detections.append(Detection(x1, y1, x2, y2, float(person[index]), "person", 0))
    return detections


def letterbox_params(width: int, height: int, input_size: int):
    scale = min(input_size / max(height, 1), input_size / max(width, 1))
    resized_w = int(round(width * scale))
    resized_h = int(round(height * scale))
    pad_x = (input_size - resized_w) / 2.0
    pad_y = (input_size - resized_h) / 2.0
    return scale, pad_x, pad_y, resized_w, resized_h


class PersonDetector:
    """auto：有 rknn 且能 import rknnlite 就用 NPU，否则尝试 onnx。"""

    def __init__(self, cfg: FusionConfig):
        self.cfg = cfg
        self.backend = "none"
        self.error = ""
        self._rknn = None
        self._net = None
        self._cv2 = None
        if cfg.detector_backend == "none":
            return
        self._open(cfg.detector_backend)

    def _open(self, preference: str):
        from pathlib import Path

        rknn_path = Path(self.cfg.rknn_model)
        onnx_path = Path(self.cfg.onnx_model)
        want_rknn = preference in ("auto", "rknn")
        want_onnx = preference in ("auto", "onnx")
        if want_rknn and rknn_path.is_file():
            try:
                from rknnlite.api import RKNNLite

                core = self.cfg.rknn_core_mask
                rknn = RKNNLite()
                if rknn.load_rknn(str(rknn_path)) != 0:
                    raise RuntimeError("load_rknn 失败")
                mask = getattr(RKNNLite, "NPU_CORE_0_1_2", core)
                if core != 7:
                    mask = core
                if rknn.init_runtime(core_mask=mask) != 0:
                    raise RuntimeError("init_runtime 失败")
                self._rknn = rknn
                self.backend = "rknn"
                return
            except Exception as exc:  # noqa: BLE001 - 设备上缺库时要落到 onnx
                self.error = f"RKNN 不可用: {exc}"
                if preference == "rknn":
                    return
        if want_onnx and onnx_path.is_file():
            try:
                import cv2

                net = cv2.dnn.readNetFromONNX(str(onnx_path))
                self._cv2 = cv2
                self._net = net
                self.backend = "onnx"
                return
            except Exception as exc:  # noqa: BLE001
                self.error = (self.error + " | " if self.error else "") + f"ONNX 不可用: {exc}"
        if not self.error:
            self.error = "没有可用的 person 模型（models/yolov8n.rknn 或 .onnx）"

    def detect(self, frame_bgr) -> list[Detection]:
        if self.backend == "none":
            return []
        import cv2

        self._cv2 = cv2
        cfg = self.cfg
        height, width = frame_bgr.shape[:2]
        scale, pad_x, pad_y, resized_w, resized_h = letterbox_params(width, height, cfg.det_input_size)
        resized = cv2.resize(frame_bgr, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((cfg.det_input_size, cfg.det_input_size, 3), 114, dtype=np.uint8)
        x0, y0 = int(round(pad_x)), int(round(pad_y))
        canvas[y0 : y0 + resized_h, x0 : x0 + resized_w] = resized
        if self.backend == "rknn":
            rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
            outputs = self._rknn.inference(inputs=[rgb])
            prediction = outputs[0]
        else:
            blob = cv2.dnn.blobFromImage(
                canvas, scalefactor=1.0 / 255.0, size=(cfg.det_input_size, cfg.det_input_size), swapRB=True, crop=False
            )
            self._net.setInput(blob)
            prediction = self._net.forward()
        return decode_yolov8_people(prediction, cfg, scale, pad_x, pad_y, width, height)

    def close(self):
        if self._rknn is not None:
            self._rknn.release()
            self._rknn = None
