"""在香橙派上提供和原来页面相同的 8081 入口。

/           预览页（原 index 用 iframe 打开这里）
/video      MJPEG
/tracks     已确认的行人融合航迹
/health     检测器、雷达、外参是否还在用缺省值

检测后端优先 RKNN（RK3588 NPU）。雷达默认按 LD2450 串口解析，
也可以改成 JSON 行，方便接别的模块。
"""

from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .config import FusionConfig, load_config
from .fusion import FusionEngine
from .radar_io import LD2450Parser, RadarTimeline, parse_radar_json

log = logging.getLogger("vision")


PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OP5 行人融合</title>
<style>
body{margin:0;background:#08101d;color:#e8eef8;font:14px sans-serif}
header{padding:12px 16px} img{width:100%;background:#000}
pre{font:12px/1.45 ui-monospace,monospace;padding:0 16px 20px;white-space:pre-wrap}
</style></head><body>
<header>只跟踪行人 · 雷达与视觉融合</header>
<img id="v" alt="融合画面" src="/video">
<pre id="t">等待航迹…</pre>
<script>
async function tick(){
  try{const r=await fetch('/tracks',{cache:'no-store'});const d=await r.json();
  document.getElementById('t').textContent=JSON.stringify(d,null,2)}catch(e){}
}
setInterval(tick,500);tick();
</script></body></html>
"""


class VisionApp:
    def __init__(self, cfg: FusionConfig):
        self.cfg = cfg
        self.engine = FusionEngine(cfg)
        self.timeline = RadarTimeline(cfg.radar_buffer_s)
        self.detector = None
        self.lock = threading.Lock()
        self.jpeg = b""
        self.payload = {"tracks": [], "debug": {}}
        self.health = {"detector": "starting", "radar": cfg.radar_mode, "calibrated": cfg.calibrated}
        self.running = True

    def start_workers(self):
        from .detector import PersonDetector

        self.detector = PersonDetector(self.cfg)
        self.health["detector"] = self.detector.backend
        self.health["detector_error"] = self.detector.error
        if not self.cfg.calibrated:
            log.warning("相机内参 calibrated=false，投影只适合看趋势。请先按 docs/融合调参.md 填写 fx/fy/cx/cy 和外参。")
        threading.Thread(target=self._radar_loop, name="radar", daemon=True).start()
        threading.Thread(target=self._camera_loop, name="camera", daemon=True).start()

    def _radar_loop(self):
        mode = self.cfg.radar_mode
        try:
            if mode == "ld2450":
                self._read_ld2450()
            elif mode == "jsonl":
                self._read_jsonl()
            else:
                self.health["radar"] = "disabled"
        except Exception as exc:  # noqa: BLE001
            self.health["radar_error"] = str(exc)
            log.exception("雷达读取停止")

    def _read_ld2450(self):
        import serial

        parser = LD2450Parser()
        port = serial.Serial(self.cfg.radar_serial_port, self.cfg.radar_baud, timeout=0.2)
        log.info("LD2450 %s @ %s", self.cfg.radar_serial_port, self.cfg.radar_baud)
        while self.running:
            data = port.read(512)
            if not data:
                continue
            for scan in parser.feed(data, time.monotonic()):
                self.timeline.add(scan)

    def _read_jsonl(self):
        import socket

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", self.cfg.radar_udp_port))
        sock.settimeout(0.5)
        log.info("雷达 JSON UDP :%s", self.cfg.radar_udp_port)
        while self.running:
            try:
                data, _addr = sock.recvfrom(65535)
            except TimeoutError:
                continue
            except socket.timeout:
                continue
            now = time.monotonic()
            for line in data.decode("utf-8", "ignore").splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    self.timeline.add(parse_radar_json(line, now, self.cfg.radar_json_unit))
                except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                    log.warning("丢弃雷达 JSON: %s", exc)

    def _camera_loop(self):
        try:
            import cv2
        except ImportError as exc:
            self.health["camera_error"] = f"没有 OpenCV: {exc}"
            return
        source = self.cfg.camera_source
        cap = cv2.VideoCapture(int(source) if str(source).isdigit() else source)
        if not cap.isOpened():
            self.health["camera_error"] = f"打不开相机 {source}"
            log.error(self.health["camera_error"])
            return
        period = 1.0 / max(self.cfg.max_fps, 1.0)
        while self.running:
            started = time.monotonic()
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            t_scene = time.monotonic() - self.cfg.camera_latency_s
            try:
                detections = self.detector.detect(frame) if self.detector else []
                radars = self.timeline.at(
                    t_scene,
                    self.cfg.radar_time_offset_s,
                    self.cfg.max_time_skew_s,
                    self.cfg.max_interp_gap_s,
                    self.cfg.interp_match_m,
                )
                tracks = self.engine.update(t_scene, detections, radars)
                self._draw(cv2, frame, detections, radars, tracks)
            except Exception as exc:  # noqa: BLE001 - 单帧失败不能把预览线程打死
                self.health["frame_error"] = str(exc)
                log.exception("处理帧失败")
                time.sleep(0.05)
                continue
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.cfg.jpeg_quality)])
            if ok:
                blob = encoded.tobytes()
                body = {
                    "t": round(t_scene, 3),
                    "tracks": [track.to_dict() for track in tracks],
                    "debug": self.engine.last_debug,
                    "class_filter": list(self.cfg.allowed_class_names),
                }
                with self.lock:
                    self.jpeg = blob
                    self.payload = body
            delay = period - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)

    def _draw(self, cv2, frame, detections, radars, tracks):
        for det in detections:
            p1 = (int(det.x1), int(det.y1))
            p2 = (int(det.x2), int(det.y2))
            cv2.rectangle(frame, p1, p2, (80, 220, 120), 2)
            cv2.putText(frame, f"person {det.score:.2f}", (p1[0], max(16, p1[1] - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 220, 120), 1)
        z = self.cfg.aim_z_m
        from .geometry import project_radar

        for radar in radars:
            u, v, ok = project_radar(radar.x, radar.y, z, self.cfg)
            if not ok:
                continue
            cv2.circle(frame, (int(u), int(v)), 5, (255, 220, 40), -1)
        for track in tracks:
            if track.u != track.u:
                continue
            cv2.circle(frame, (int(track.u), int(track.v)), 8, (40, 180, 255), 2)
            cv2.putText(
                frame,
                f"id{track.id} {track.range_m():.1f}m {track.source}",
                (int(track.u) + 8, int(track.v)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (40, 180, 255),
                1,
            )


def make_handler(app: VisionApp):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):
            return

        def _send(self, code, body, content_type):
            data = body if isinstance(body, bytes) else body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif path == "/tracks":
                with app.lock:
                    body = json.dumps(app.payload, ensure_ascii=False)
                self._send(200, body, "application/json; charset=utf-8")
            elif path == "/health":
                self._send(200, json.dumps(app.health, ensure_ascii=False), "application/json; charset=utf-8")
            elif path == "/video":
                self._stream()
            else:
                self._send(404, "not found", "text/plain; charset=utf-8")

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while app.running:
                    with app.lock:
                        blob = app.jpeg
                    if not blob:
                        time.sleep(0.05)
                        continue
                    chunk = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + blob + b"\r\n"
                    self.wfile.write(chunk)
                    time.sleep(1.0 / max(app.cfg.max_fps, 1.0))
            except (BrokenPipeError, ConnectionResetError):
                return

    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description="香橙派行人雷达视觉融合")
    parser.add_argument("--config", default="config/fusion.json")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    cfg = load_config(args.config)
    app = VisionApp(cfg)
    app.start_workers()
    server = ThreadingHTTPServer((cfg.web_host, cfg.web_port), make_handler(app))
    log.info("预览 http://%s:%s/  只检测 person", cfg.web_host, cfg.web_port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        app.running = False
        server.shutdown()


if __name__ == "__main__":
    main()
