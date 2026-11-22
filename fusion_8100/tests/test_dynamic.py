"""Offline tests; every scene is synthetic, never drives physical motors."""
import copy
import math
import unittest
from fusion_core import FusionEngine,enumerate_clusters,angular_delta,camera_bearing_deg,radar_points

CFG={'camera_hfov_deg':70,'radar_zero_deg':0.0,'radar_mount':'chassis',
     'yaw_to_radar_sign':1,'camera_yaw_offset_deg':0.0}

def visual(t, ident=418, cx=50, found=True):
    return dict(live=True,found=found,selected=ident,selected_class=0,
                updated=t-.04,width=100,height=100,
                tracks=[{'id':ident,'class_id':0,'box':[cx-7,25,cx+7,75]}])

def servo(yaw=0):return {'yaw':{'angle':yaw}}

def group(angle,rng,n=8,strength=200):
    return [[(angle+(i-(n-1)/2)*.36)%360,rng+0.008*((i%3)-1),strength] for i in range(n)]

def radar(pkt,angle=0,rng=1.0,extra=None, online=True):
    return dict(online=online,packets=pkt,points=group(angle,rng)+list(extra or []),error='')

def run(engine,t=100,pkt=1,ident=418,cx=50,angle=0,rng=1,extra=None,vis=None,rad=None,yaw=0,rt=None,st=None,cfg=None):
    v=vis if vis is not None else visual(t,ident,cx)
    r=rad if rad is not None else radar(pkt,angle,rng,extra)
    return engine.fuse((v,t-.04),(r,t-.07 if rt is None else rt),
                       (servo(yaw),t-.08 if st is None else st),CFG if cfg is None else cfg,t)

class DynamicTests(unittest.TestCase):
    def test_version(self):
        self.assertEqual(run(FusionEngine())['version'],'V3_1_DYNAMIC_ASSOC_V1')
    def test_wrap_across_359_to_0(self):
        self.assertEqual(len(enumerate_clusters(radar_points(radar(1,359.8,1.1)),0,5)),1)
    def test_three_new_packets_confirms(self):
        e=FusionEngine()
        for i in range(3):d=run(e,100+i*.25,i+1)
        self.assertEqual(d['status'],'CANDIDATE_STABLE_UNVERIFIED')
        self.assertEqual(d['independent_confirmations'],3)
        self.assertAlmostEqual(d['stable_candidate_m'],1.0,2)
    def test_repeated_packet_not_counted(self):
        e=FusionEngine()
        for i in range(5):d=run(e,100+i*.1,1)
        self.assertIsNone(d['stable_candidate_m'])
        self.assertEqual(d['independent_confirmations'],1)
    def test_zero_crossing_series(self):
        e=FusionEngine()
        for i,a in enumerate((359.5,0.1,0.8)):
            d=run(e,100+i*.25,i+1,angle=a,rng=1+.02*i)
        self.assertEqual(d['status'],'CANDIDATE_STABLE_UNVERIFIED')
        self.assertLess(abs(d['angular_residual_deg']),2)
    def test_ambiguous_equal_angle_competing_ranges_hides(self):
        r=radar(1,-3,1.0,group(3,2.0))
        d=run(FusionEngine(),rad=r)
        self.assertEqual(d['status'],'ASSOCIATION_AMBIGUOUS')
        self.assertIsNone(d['candidate'])
        self.assertIsNone(d['stable_candidate_m'])
    def test_angle_residual_above_7_refused(self):
        v=visual(100);v['tracks'][0]['box']=[25,25,75,75]
        d=run(FusionEngine(),angle=11,rng=.56,vis=v)
        self.assertEqual(d['status'],'ASSOCIATION_ANGULAR_UNCERTAIN')
        self.assertIsNone(d['candidate'])
    def test_348_and_12_competition_at_zero_refused(self):
        v=visual(100);v['tracks'][0]['box']=[25,25,75,75]
        d=run(FusionEngine(),rad=radar(1,348,.7,group(12,.8)),vis=v)
        self.assertEqual(d['status'],'ASSOCIATION_ANGULAR_UNCERTAIN')
    def test_range_jump_1_to_3_rejected(self):
        e=FusionEngine()
        for i in range(3):run(e,100+i*.2,i+1,angle=0,rng=1)
        d=run(e,100.6,4,angle=0,rng=3)
        self.assertEqual(d['status'],'ASSOCIATION_RANGE_JUMP_REJECTED')
        self.assertIsNone(d['stable_candidate_m'])
    def test_prior_keeps_person_vs_background(self):
        e=FusionEngine()
        for i in range(3):run(e,100+i*.2,i+1,angle=1,rng=1)
        d=run(e,100.6,4,angle=1.2,rng=1.04,extra=group(0,3))
        self.assertIsNotNone(d['candidate'])
        self.assertAlmostEqual(d['candidate']['range_m'],1.04,2)
    def test_new_id_requires_three_again(self):
        e=FusionEngine()
        for i in range(3):run(e,100+i*.2,i+1)
        d=run(e,100.6,4,ident=999)
        self.assertIsNone(d['stable_candidate_m'])
        self.assertEqual(d['independent_confirmations'],1)
    def test_reacquire_after_target_lost(self):
        e=FusionEngine()
        for i in range(3):run(e,100+i*.2,i+1)
        d=run(e,100.6,4,vis=visual(100.6,found=False))
        self.assertIsNone(d['stable_candidate_m'])
        d=run(e,100.8,5)
        self.assertIsNone(d['stable_candidate_m'])
    def test_radar_offline_clears(self):
        e=FusionEngine()
        for i in range(3):run(e,100+i*.2,i+1)
        d=run(e,100.6,4,rad=radar(4,online=False))
        self.assertEqual(d['status'],'RADAR_NOT_ONLINE')
        self.assertIsNone(d['stable_candidate_m'])
        d=run(e,100.8,5)
        self.assertIsNone(d['stable_candidate_m'])
    def test_radar_old_clears(self):
        d=run(FusionEngine(),rt=99.0)
        self.assertEqual(d['status'],'SENSOR_DATA_STALE')
        self.assertIsNone(d['stable_candidate_m'])
    def test_servo_old_clears(self):
        self.assertEqual(run(FusionEngine(),st=99)['status'],'SENSOR_DATA_STALE')
    def test_missing_calibration_clears(self):
        c=dict(CFG,radar_zero_deg=None)
        self.assertEqual(run(FusionEngine(),cfg=c)['status'],'CALIBRATION_REQUIRED')
    def test_counter_reset_requires_reconfirm(self):
        e=FusionEngine()
        for i in range(3):run(e,100+i*.2,100+i)
        d=run(e,100.6,1)
        self.assertEqual(d['status'],'RADAR_PACKET_COUNTER_RESET')
        self.assertIsNone(d['stable_candidate_m'])
    def test_same_packet_new_direction_waits(self):
        e=FusionEngine()
        for i in range(3):run(e,100+i*.2,i+1)
        d=run(e,100.5,3,angle=5.5)
        self.assertEqual(d['status'],'WAIT_FOR_NEW_RADAR_PACKET')
        self.assertIsNone(d['stable_candidate_m'])
    def test_reuse_vision_box_big_teleport_resets(self):
        e=FusionEngine()
        for i in range(3):run(e,100+i*.1,i+1)
        # Move selected box 34% of the frame in a tenth of a second.
        cx=84
        bearing=camera_bearing_deg(cx,100,70)
        d=run(e,100.31,4,cx=cx,angle=bearing)
        self.assertIsNone(d['stable_candidate_m'])
        self.assertEqual(d['independent_confirmations'],1)
    def test_radial_velocity_signed(self):
        e=FusionEngine()
        for i in range(4):d=run(e,100+i*.25,i+1,rng=1.4-.1*i)
        self.assertIsNotNone(d['radial_speed_mps'])
        self.assertLess(d['radial_speed_mps'],0)
    def test_uncertain_does_not_keep_old_value(self):
        e=FusionEngine()
        for i in range(3):run(e,100+i*.25,i+1)
        d=run(e,100.75,4,angle=11)
        self.assertIsNone(d['candidate']);self.assertIsNone(d['stable_candidate_m'])
    def test_yaw_motion_pause(self):
        e=FusionEngine();run(e,100,1)
        d=run(e,100.2,2,yaw=15,st=100.15,angle=15)
        self.assertEqual(d['status'],'YAW_MOVING_FAST_ASSOCIATION_PAUSED')
    def test_no_motor_or_radar_writes_in_core(self):
        import inspect
        import fusion_core
        s=inspect.getsource(fusion_core)
        for forbidden in ('serial.Serial(','send(ser,','requests.post(','urllib.request.urlopen('):
            self.assertNotIn(forbidden,s)
    def test_single_read_identity_not_claimed(self):
        d=run(FusionEngine())
        self.assertIn('UNVERIFIED',d['association_quality'])
        self.assertIsNone(d['stable_candidate_m'])

if __name__=='__main__': unittest.main(verbosity=2)
