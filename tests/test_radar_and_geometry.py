"""外参投影、时间对齐、LD2450 解码。"""

import math
import unittest

from pathlib import Path

from vision.config import FusionConfig, load_config
from vision.geometry import project_radar, radar_to_camera, suggest_yaw_deg, xy_from_range_and_pixel
from vision.radar_io import LD2450Parser, RadarScan, RadarTarget, RadarTimeline, parse_ld2450_frame, parse_radar_json


def cfg(**kwargs):
    base = FusionConfig(calibrated=True, person_aim_height_m=0.0, radar_mount_height_m=0.0)
    for key, value in kwargs.items():
        setattr(base, key, value)
    return base


class ConfigTests(unittest.TestCase):
    def test_repo_config_loads(self):
        path = Path(__file__).resolve().parents[1] / "config" / "fusion.json"
        loaded = load_config(path)
        self.assertEqual(loaded.allowed_class_names, ("person",))
        self.assertEqual(loaded.allowed_class_ids, (0,))
        self.assertEqual(loaded.web_port, 8081)
        self.assertEqual(loaded.radar_baud, 256000)

    def test_unknown_key_is_rejected(self):
        from vision.config import config_from_dict

        with self.assertRaises(ValueError):
            config_from_dict({"not_a_real_key": 1})


class GeometryTests(unittest.TestCase):
    def test_forward_target_is_image_center(self):
        u, v, ok = project_radar(0.0, 5.0, 0.0, cfg())
        self.assertTrue(ok)
        self.assertAlmostEqual(u, 320.0, places=3)
        self.assertAlmostEqual(v, 240.0, places=3)

    def test_right_target_projects_right(self):
        u, _v, ok = project_radar(1.0, 5.0, 0.0, cfg())
        self.assertTrue(ok)
        self.assertAlmostEqual(u, 320.0 + 600.0 * 1.0 / 5.0, places=3)

    def test_axis_remap_height_goes_up_in_image(self):
        # 雷达 z 向上；相机 y 向下，所以更高的点 v 更小。
        cam = radar_to_camera(0.0, 4.0, 0.5, cfg())
        self.assertAlmostEqual(cam[0], 0.0, places=6)
        self.assertAlmostEqual(cam[1], -0.5, places=6)
        self.assertAlmostEqual(cam[2], 4.0, places=6)
        _u, v, ok = project_radar(0.0, 4.0, 0.5, cfg())
        self.assertTrue(ok)
        self.assertLess(v, 240.0)

    def test_range_and_pixel_roundtrip(self):
        model = cfg(yaw_deg=8.0, tx_m=0.04, tz_m=-0.02)
        x, y, z = 0.7, 4.5, 0.2
        u, _v, ok = project_radar(x, y, z, model)
        self.assertTrue(ok)
        radius = math.hypot(x, y)
        got_x, got_y = xy_from_range_and_pixel(radius, u, z, model)
        self.assertAlmostEqual(got_x, x, delta=0.02)
        self.assertAlmostEqual(got_y, y, delta=0.02)

    def test_suggest_yaw_centers_the_projection(self):
        model = cfg()
        yaw = suggest_yaw_deg(0.5, 5.0, 320.0, model)
        model.yaw_deg = yaw
        u, _v, ok = project_radar(0.5, 5.0, 0.0, model)
        self.assertTrue(ok)
        self.assertAlmostEqual(u, 320.0, delta=0.5)


class TimelineTests(unittest.TestCase):
    def test_interpolates_between_scans(self):
        line = RadarTimeline()
        line.add(RadarScan(1.0, [RadarTarget(0.0, 4.0, track_id=1)]))
        line.add(RadarScan(1.2, [RadarTarget(1.0, 4.0, track_id=1)]))
        got = line.at(1.1, 0.0, 0.2, 0.3, 1.5)
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(got[0].x, 0.5, places=5)
        self.assertAlmostEqual(got[0].y, 4.0, places=5)

    def test_offset_selects_the_scan_for_that_pose(self):
        # 雷达时间戳比场景晚 0.2 秒。对相机时刻 1.0 应该拿到戳在 1.2 上的那包。
        line = RadarTimeline()
        line.add(RadarScan(1.0, [RadarTarget(0.0, 4.0, track_id=1)]))
        line.add(RadarScan(1.2, [RadarTarget(5.0, 4.0, track_id=1)]))
        got = line.at(1.0, radar_time_offset_s=0.2, max_skew_s=0.05, max_gap_s=0.3, match_gate_m=1.0)
        self.assertAlmostEqual(got[0].x, 5.0, places=5)

    def test_stale_scan_is_dropped(self):
        line = RadarTimeline()
        line.add(RadarScan(0.0, [RadarTarget(0.0, 4.0)]))
        got = line.at(1.0, 0.0, max_skew_s=0.15, max_gap_s=0.25, match_gate_m=1.0)
        self.assertEqual(got, [])

    def test_json_mm_and_unix_stamp(self):
        scan = parse_radar_json(
            '{"t": 1700000000, "unit": "mm", "targets": [{"x": 200, "y": 3000, "id": 2}]}',
            fallback_t=3.5,
        )
        self.assertAlmostEqual(scan.t, 3.5)
        self.assertAlmostEqual(scan.targets[0].x, 0.2)
        self.assertAlmostEqual(scan.targets[0].y, 3.0)
        self.assertEqual(scan.targets[0].track_id, 2)


class LD2450Tests(unittest.TestCase):
    def test_datasheet_example(self):
        frame = bytes.fromhex("AAFF0300 0E03B186 10004001 0000000000000000 0000000000000000 55CC")
        scan = parse_ld2450_frame(frame, t=1.0)
        self.assertEqual(len(scan.targets), 1)
        target = scan.targets[0]
        self.assertAlmostEqual(target.x, -0.782, places=3)
        self.assertAlmostEqual(target.y, 1.713, places=3)
        self.assertAlmostEqual(target.speed_mps, -0.16, places=2)
        self.assertAlmostEqual(target.resolution_m, 0.32, places=3)
        # 速度为负表示远离，y 方向速度应为正。
        self.assertGreater(target.vy, 0.0)

    def test_parser_finds_frame_in_noise(self):
        frame = bytes.fromhex("AAFF03000E03B18610004001" + "00" * 16 + "55CC")
        parser = LD2450Parser()
        scans = parser.feed(b"\x00\x11" + frame[:10], 1.0)
        self.assertEqual(scans, [])
        scans = parser.feed(frame[10:], 2.0)
        self.assertEqual(len(scans), 1)
        self.assertAlmostEqual(scans[0].targets[0].y, 1.713, places=3)


if __name__ == "__main__":
    unittest.main()
