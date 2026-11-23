"""针对实机不稳的回归：非 person 不得融合，已有航迹不被单帧角差或深度竞争清掉。"""
import unittest
from fusion_core import FusionEngine, person_allowed

CFG = {'camera_hfov_deg': 70, 'radar_zero_deg': 0.0, 'radar_mount': 'chassis',
       'yaw_to_radar_sign': 1, 'camera_yaw_offset_deg': 0.0}


def visual(t, ident=418, cx=50, class_id=0, class_name=''):
    track = {'id': ident, 'class_id': class_id, 'box': [cx - 7, 25, cx + 7, 75]}
    if class_name:
        track['class_name'] = class_name
    return dict(live=True, found=True, selected=ident, selected_class=class_id,
                updated=t - .04, width=100, height=100, tracks=[track])


def group(angle, rng, n=8, strength=200):
    return [[(angle + (i - (n - 1) / 2) * .36) % 360, rng + 0.008 * ((i % 3) - 1), strength] for i in range(n)]


def radar(pkt, angle=0, rng=1.0, extra=None):
    return dict(online=True, packets=pkt, points=group(angle, rng) + list(extra or []), error='')


def run(engine, t=100, pkt=1, ident=418, cx=50, angle=0, rng=1, extra=None, yaw=0, vis=None, st=None):
    v = vis if vis is not None else visual(t, ident, cx)
    r = radar(pkt, angle, rng, extra)
    servo = {'yaw': {'angle': yaw}}
    return engine.fuse((v, t - .04), (r, t - .07), (servo, t - .08 if st is None else st), CFG, t)


class PersonFilterTests(unittest.TestCase):
    def test_plant_id_not_fused(self):
        vis = visual(100, class_id=58, class_name='potted plant')
        d = run(FusionEngine(), vis=vis)
        self.assertEqual(d['status'], 'CLASS_NOT_PERSON')
        self.assertIsNone(d['target'])
        self.assertIsNone(d['candidate'])
        self.assertIsNone(d['stable_candidate_m'])

    def test_plant_name_overrides_person_id(self):
        vis = visual(100, class_id=0, class_name='potted plant')
        d = run(FusionEngine(), vis=vis)
        self.assertEqual(d['status'], 'CLASS_NOT_PERSON')
        self.assertIsNone(d['candidate'])

    def test_other_classes_rejected(self):
        for name, cid in (('car', 2), ('plant', 1), ('', 16)):
            vis = visual(100, class_id=cid, class_name=name)
            d = run(FusionEngine(), vis=vis)
            self.assertEqual(d['status'], 'CLASS_NOT_PERSON', name or cid)
            self.assertIsNone(d['stable_candidate_m'])

    def test_person_and_chinese_alias_still_track(self):
        self.assertTrue(person_allowed(0, '', CFG))
        self.assertTrue(person_allowed(0, '行人', CFG))
        self.assertFalse(person_allowed(0, 'potted plant', CFG))
        e = FusionEngine()
        for i in range(3):
            d = run(e, 100 + i * .2, i + 1, vis=visual(100 + i * .2, class_name='person'))
        self.assertEqual(d['status'], 'CANDIDATE_STABLE_UNVERIFIED')
        self.assertEqual(d['target']['class_id'], 0)


class ContinuityTests(unittest.TestCase):
    def test_prior_keeps_person_when_background_competes(self):
        e = FusionEngine()
        for i in range(3):
            run(e, 100 + i * .2, i + 1, angle=1, rng=1)
        d = run(e, 100.6, 4, angle=1.2, rng=1.04, extra=group(0, 3))
        self.assertIsNotNone(d['candidate'])
        self.assertAlmostEqual(d['candidate']['range_m'], 1.04, places=2)
        self.assertNotEqual(d['status'], 'ASSOCIATION_COMPETING_DEPTHS')

    def test_competing_depths_still_refused_without_prior(self):
        e = FusionEngine()
        d = run(e, angle=-3, rng=1.0, extra=group(3, 2.0))
        self.assertEqual(d['status'], 'ASSOCIATION_COMPETING_DEPTHS')
        self.assertIsNone(d['candidate'])
        self.assertIsNone(d['stable_candidate_m'])
        self.assertEqual(e.history, [])

    def test_angular_glitch_does_not_erase_confirmations(self):
        e = FusionEngine()
        for i in range(3):
            run(e, 100 + i * .25, i + 1)
        # 框加宽后 11° 落在搜索窗内，但超过 7° 残差门，本帧必须拒绝。
        wide = visual(100.75)
        wide['tracks'][0]['box'] = [25, 25, 75, 75]
        missed = run(e, 100.75, 4, angle=11, vis=wide)
        self.assertEqual(missed['status'], 'ASSOCIATION_ANGULAR_UNCERTAIN')
        self.assertIsNone(missed['candidate'])
        self.assertIsNone(missed['stable_candidate_m'])
        self.assertEqual(len(e.history), 3)
        # 角度窗外的一帧同样不能把确认次数清掉。
        outside = run(e, 101.05, 5, angle=20)
        self.assertEqual(outside['status'], 'NO_RADAR_CLUSTER_IN_TARGET_DIRECTION')
        self.assertEqual(len(e.history), 3)
        recovered = run(e, 101.25, 6, angle=0, rng=1.0)
        self.assertEqual(recovered['status'], 'CANDIDATE_STABLE_UNVERIFIED')
        self.assertIsNotNone(recovered['stable_candidate_m'])

    def test_fast_yaw_pauses_without_forgetting_the_track(self):
        e = FusionEngine()
        for i in range(3):
            run(e, 100 + i * .2, i + 1, yaw=0)
        paused = run(e, 100.6, 4, yaw=20, st=100.52)
        self.assertEqual(paused['status'], 'YAW_MOVING_FAST_ASSOCIATION_PAUSED')
        self.assertIsNone(paused['candidate'])
        self.assertEqual(len(e.history), 3)
        resumed = run(e, 100.85, 5, yaw=20, st=100.77, angle=20)
        self.assertEqual(resumed['status'], 'CANDIDATE_STABLE_UNVERIFIED')
        self.assertAlmostEqual(resumed['stable_candidate_m'], 1.0, places=2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
