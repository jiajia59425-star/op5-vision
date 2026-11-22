"""检测后处理只留 person。"""

import unittest

import numpy as np

from vision.config import FusionConfig
from vision.detector import Detection, decode_yolov8_people, filter_people, is_person, letterbox_params


class FilterTests(unittest.TestCase):
    def test_allowlist(self):
        cfg = FusionConfig()
        self.assertTrue(is_person(0, "person", cfg))
        self.assertTrue(is_person(0, "", cfg))
        self.assertFalse(is_person(58, "potted plant", cfg))
        self.assertFalse(is_person(0, "potted plant", cfg))
        self.assertFalse(is_person(58, "", cfg))
        self.assertFalse(is_person(2, "car", cfg))
        kept, dropped = filter_people(
            [
                Detection(0, 0, 10, 10, 0.9, "person", 0),
                Detection(0, 0, 10, 10, 0.9, "potted plant", 58),
                Detection(0, 0, 10, 10, 0.9, "plant", 0),
            ],
            cfg,
        )
        self.assertEqual(len(kept), 1)
        self.assertEqual(dropped, 2)


class DecodeTests(unittest.TestCase):
    def test_drops_plant_channel_and_low_score(self):
        cfg = FusionConfig(det_conf=0.35, det_iou=0.45)
        pred = np.zeros((84, 3), dtype=np.float32)
        # cx, cy, w, h
        pred[0:4, 0] = (100, 80, 40, 90)
        pred[4, 0] = 0.92  # person
        pred[0:4, 1] = (300, 80, 50, 80)
        pred[4 + 58, 1] = 0.99  # potted plant
        pred[4, 1] = 0.02
        pred[0:4, 2] = (200, 80, 30, 70)
        pred[4, 2] = 0.10
        dets = decode_yolov8_people(pred, cfg, scale=1.0, pad_x=0.0, pad_y=0.0, width=640, height=480)
        self.assertEqual(len(dets), 1)
        self.assertEqual(dets[0].class_name, "person")
        self.assertAlmostEqual(dets[0].x1, 80.0, places=3)
        self.assertAlmostEqual(dets[0].y1, 35.0, places=3)
        # (候选框, 通道) 这种 RKNN 布局也要得到同一个框。
        flipped = decode_yolov8_people(pred.T, cfg, scale=1.0, pad_x=0.0, pad_y=0.0, width=640, height=480)
        self.assertEqual(len(flipped), 1)
        self.assertAlmostEqual(flipped[0].x1, dets[0].x1, places=3)

    def test_nms_keeps_higher_score(self):
        cfg = FusionConfig(det_conf=0.35, det_iou=0.4)
        pred = np.zeros((84, 2), dtype=np.float32)
        pred[0:4, 0] = (100, 100, 40, 80)
        pred[4, 0] = 0.95
        pred[0:4, 1] = (104, 102, 40, 80)
        pred[4, 1] = 0.55
        dets = decode_yolov8_people(pred, cfg, 1.0, 0.0, 0.0, 640, 480)
        self.assertEqual(len(dets), 1)
        self.assertGreater(dets[0].score, 0.9)

    def test_letterbox_maps_back(self):
        scale, pad_x, pad_y, _rw, _rh = letterbox_params(320, 240, 640)
        self.assertAlmostEqual(scale, 2.0)
        self.assertAlmostEqual(pad_x, 0.0)
        self.assertAlmostEqual(pad_y, 80.0)
        cfg = FusionConfig()
        pred = np.zeros((5, 1), dtype=np.float32)
        # 原图框 (100, 50, 140, 170) -> 输入坐标
        pred[0, 0] = (200 + 280) / 2
        pred[1, 0] = (180 + 420) / 2
        pred[2, 0] = 80
        pred[3, 0] = 240
        pred[4, 0] = 0.8
        dets = decode_yolov8_people(pred, cfg, scale, pad_x, pad_y, 320, 240)
        self.assertEqual(len(dets), 1)
        self.assertAlmostEqual(dets[0].x1, 100.0, delta=1.0)
        self.assertAlmostEqual(dets[0].y1, 50.0, delta=1.0)
        self.assertAlmostEqual(dets[0].x2, 140.0, delta=1.0)
        self.assertAlmostEqual(dets[0].y2, 170.0, delta=1.0)

    def test_end_to_end_rows_drop_other_classes(self):
        cfg = FusionConfig()
        pred = np.array(
            [
                [10, 10, 40, 80, 0.9, 0],
                [10, 10, 40, 80, 0.9, 58],
            ],
            dtype=np.float32,
        )
        dets = decode_yolov8_people(pred, cfg, 1.0, 0.0, 0.0, 640, 480)
        self.assertEqual(len(dets), 1)


if __name__ == "__main__":
    unittest.main()
