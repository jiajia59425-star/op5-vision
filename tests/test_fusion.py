"""融合航迹：距离用雷达、横向用视觉，植物不建航迹，遮挡时 id 保持。"""

import math
import unittest

from vision.config import FusionConfig
from vision.detector import Detection
from vision.fusion import FusionEngine
from vision.geometry import project_radar
from vision.radar_io import RadarScan, RadarTarget, RadarTimeline


def model():
    return FusionConfig(
        calibrated=True,
        fx=600,
        fy=600,
        cx=320,
        cy=240,
        width=640,
        height=480,
        radar_mount_height_m=0.0,
        person_aim_height_m=0.0,
        person_height_m=1.7,
        min_hits=3,
        max_coast_s=0.45,
        tentative_ttl_s=0.7,
        radar_time_offset_s=0.2,
    )


def person_box(u, v, half_w=22, half_h=70, score=0.9):
    return Detection(u - half_w, v - half_h, u + half_w, v + half_h, score, "person", 0)


class FusionTests(unittest.TestCase):
    def test_fused_track_uses_vision_bearing_not_radar_angle(self):
        cfg = model()
        engine = FusionEngine(cfg)
        timeline = RadarTimeline()
        last = None
        ids = []
        for i in range(20):
            t = i * 0.1
            true_x = 0.4 * t
            true_y = 6.0
            u, v, ok = project_radar(true_x, true_y, 0.0, cfg)
            self.assertTrue(ok)
            plant = Detection(10, 10, 90, 180, 0.99, "potted plant", 58)
            # 雷达方位偏 0.7 米，时间戳晚 0.2 秒，但描述的是当前姿态。
            timeline.add(
                RadarScan(
                    t + 0.2,
                    [
                        RadarTarget(true_x + 0.7, true_y, vx=0.4, vy=0.0, track_id=1),
                        RadarTarget(-3.0, 4.0, track_id=7),
                    ],
                )
            )
            radars = timeline.at(t, cfg.radar_time_offset_s, cfg.max_time_skew_s, cfg.max_interp_gap_s, cfg.interp_match_m)
            tracks = engine.update(t, [person_box(u, v), plant], radars)
            if tracks:
                last = tracks[0]
                ids.append(last.id)
                self.assertLess(math.hypot(last.x - true_x, last.y - true_y), 0.2)
        self.assertIsNotNone(last)
        self.assertEqual(set(ids), {ids[0]})
        self.assertEqual(engine.last_debug["dropped_non_person"], 1)
        self.assertGreaterEqual(engine.last_debug["fused_pairs"], 1)
        # 若直接采用雷达 x，误差约 0.7 米，上面的 0.2 米门槛过不了。
        self.assertLess(abs(last.x - (0.4 * 1.9)), 0.15)

    def test_far_radar_does_not_pull_the_person(self):
        cfg = model()
        engine = FusionEngine(cfg)
        tracks = []
        for i in range(8):
            t = i * 0.1
            tracks = engine.update(
                t,
                [person_box(320, 240)],
                [RadarTarget(3.0, 4.0)],
            )
        self.assertTrue(tracks)
        self.assertLess(abs(tracks[0].x), 0.4)
        self.assertTrue(all(track.x < 1.0 for track in tracks))

    def test_non_person_names_never_confirm(self):
        cfg = model()
        engine = FusionEngine(cfg)
        junk = [
            Detection(100, 40, 180, 200, 0.9, "potted plant", 0),
            Detection(200, 40, 280, 200, 0.9, "car", 2),
            Detection(300, 40, 360, 200, 0.9, "", 58),
        ]
        for i in range(8):
            tracks = engine.update(i * 0.1, junk, [RadarTarget(0.0, 4.0)])
            self.assertEqual(tracks, [])
        self.assertEqual(engine.last_debug["vision_people"], 0)

    def test_id_survives_short_dropout_then_expires(self):
        cfg = model()
        engine = FusionEngine(cfg)
        seen = None
        for i in range(6):
            t = i * 0.1
            u, v, _ok = project_radar(0.2, 5.0, 0.0, cfg)
            tracks = engine.update(t, [person_box(u, v)], [RadarTarget(0.2, 5.0)])
            if tracks:
                seen = tracks[0].id
        self.assertIsNotNone(seen)
        # 丢 0.3 秒，还在滑行，id 不变。
        coast = engine.update(0.5 + 0.3, [], [])
        self.assertEqual([track.id for track in coast], [seen])
        gone = engine.update(0.5 + 0.3 + 0.5, [], [])
        self.assertEqual(gone, [])

    def test_second_person_gets_a_new_id(self):
        cfg = model()
        engine = FusionEngine(cfg)
        first_id = None
        for i in range(6):
            t = i * 0.1
            u, v, _ok = project_radar(0.0, 5.0, 0.0, cfg)
            tracks = engine.update(t, [person_box(u, v)], [RadarTarget(0.0, 5.0)])
            if tracks:
                first_id = tracks[0].id
        u2, v2, _ok = project_radar(2.0, 5.0, 0.0, cfg)
        tracks = []
        for i in range(6):
            t = 1.0 + i * 0.1
            u1, v1, _ok = project_radar(0.0, 5.0, 0.0, cfg)
            tracks = engine.update(
                t,
                [person_box(u1, v1), person_box(u2, v2)],
                [RadarTarget(0.0, 5.0, track_id=1), RadarTarget(2.0, 5.0, track_id=2)],
            )
        ids = {track.id for track in tracks}
        self.assertIn(first_id, ids)
        self.assertEqual(len(ids), 2)


if __name__ == "__main__":
    unittest.main()
