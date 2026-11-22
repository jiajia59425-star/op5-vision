import copy,json,math,unittest
from fusion_core import *

CFG={'camera_hfov_deg':70,'radar_zero_deg':0,'radar_mount':'chassis','yaw_to_radar_sign':1,'camera_yaw_offset_deg':0.0}
def target(now=100,ident=123,cx=50):
    return {'live':True,'found':True,'updated':now-.05,'selected':ident,
            'selected_class':0,'width':100,'height':100,
            'tracks':[{'id':ident,'class_id':0,'box':[cx-5,40,cx+5,60]}]}
def radar(packets=1,base=0,online=True):
    return {'online':online,'packets':packets,'points':[[base+a,d,200] for a,d in [(0,3.0),(.4,3.1),(.8,3.0),(1.2,3.1)]],
            'error':''}
def servo(yaw=0):return {'yaw':{'angle':yaw}}
def run(e,v=None,r=None,s=None,c=None,now=100,rt=None,st=None):
    v=target(now) if v is None else v
    r=radar() if r is None else r
    s=servo() if s is None else s
    return e.fuse((v,now-.05),(r,now-.08 if rt is None else rt),
                  (s,now-.09 if st is None else st),CFG if c is None else c,now)

class CoreTests(unittest.TestCase):
    def test_pinhole_center(self):self.assertAlmostEqual(camera_bearing_deg(50,100,70),0)
    def test_pinhole_edges(self):self.assertAlmostEqual(camera_bearing_deg(100,100,70),35)
    def test_angle_wrap(self):self.assertAlmostEqual(angular_delta(1,359),2)
    def test_angle_negative(self):self.assertAlmostEqual(angular_delta(359,1),-2)
    def test_fov_invalid(self):
        with self.assertRaises(ValueError):camera_bearing_deg(50,100,0)
    def test_target_valid(self):self.assertEqual(validated_selection(target(),100)[1],'OK')
    def test_target_stale(self):self.assertEqual(validated_selection(target(98),100)[1],'VISION_FRAME_STALE')
    def test_target_none(self):
        v=target();v['selected']=None
        self.assertEqual(validated_selection(v,100)[1],'NO_SELECTED_TARGET')
    def test_no_found(self):
        v=target();v['found']=False
        self.assertEqual(validated_selection(v,100)[1],'VISION_NOT_READY')
    def test_bad_box(self):
        v=target();v['tracks'][0]['box']=[-5,2,4,5]
        self.assertEqual(validated_selection(v,100)[1],'VISION_BOX_INVALID')
    def test_radar_offline_points(self):self.assertEqual(radar_points(radar(online=False)),[])
    def test_radar_cluster(self):self.assertGreaterEqual(radar_candidates(radar_points(radar()),0,5)['points'],3)
    def test_radar_no_cluster(self):self.assertIsNone(radar_candidates(radar_points(radar()),90,5))
    def test_radar_bad_values(self):
        r=radar();r['points'].extend([[2,-1,80],[3,float('nan'),88],[2,9,999]])
        self.assertEqual(len(radar_points(r)),4)
    def test_calibration_required(self):
        c=copy.deepcopy(CFG);c['radar_zero_deg']=None
        self.assertEqual(run(FusionEngine(),c=c)['status'],'CALIBRATION_REQUIRED')
    def test_calibration_unset_distance_none(self):
        c=copy.deepcopy(CFG);c['camera_hfov_deg']=None
        self.assertIsNone(run(FusionEngine(),c=c)['candidate'])
    def test_radar_offline(self):self.assertEqual(run(FusionEngine(),r=radar(online=False))['status'],'RADAR_NOT_ONLINE')
    def test_servo_stale(self):self.assertEqual(run(FusionEngine(),st=99.1)['status'],'SENSOR_DATA_STALE')
    def test_radar_stale(self):self.assertEqual(run(FusionEngine(),rt=99.1)['status'],'SENSOR_DATA_STALE')
    def test_fusion_cluster(self):
        d=run(FusionEngine());self.assertEqual(d['status'],'CANDIDATE_WAITING_STABILITY')
        self.assertEqual(d['candidate']['range_m'],3.05)
    def test_same_packet_no_fake_stability(self):
        e=FusionEngine()
        for i in range(5): d=run(e,now=100+i*.1,r=radar(packets=1))
        self.assertIsNone(d['stable_candidate_m'])
    def test_three_independent_updates(self):
        e=FusionEngine()
        for i in range(3):d=run(e,now=100+i*.1,r=radar(packets=1+i))
        self.assertEqual(d['status'],'CANDIDATE_STABLE_UNVERIFIED')
        self.assertAlmostEqual(d['stable_candidate_m'],3.05)
    def test_target_change_resets_stability(self):
        e=FusionEngine()
        for i in range(3):run(e,now=100+i*.1,r=radar(packets=i+1))
        d=run(e,now=100.4,v=target(100.4,ident=999),r=radar(packets=10))
        self.assertIsNone(d['stable_candidate_m'])
    def test_radar_jump_resets(self):
        e=FusionEngine()
        for i in range(3):run(e,now=100+i*.1,r=radar(packets=i+1))
        r=radar(packets=10);r['points']=[[a,6,150] for a in (0,.4,.8,1.2)]
        d=run(e,now=100.4,r=r)
        self.assertIsNone(d['stable_candidate_m'])
    def test_camera_offset(self):
        d=run(FusionEngine(),v=target(cx=75),r=radar())
        self.assertEqual(d['status'],'NO_RADAR_CLUSTER_IN_TARGET_DIRECTION')
    def test_yaw_angle_mapping(self):
        d=run(FusionEngine(),s=servo(10),r=radar(base=10))
        self.assertEqual(d['status'],'CANDIDATE_WAITING_STABILITY')
    def test_yaw_fast_pauses(self):
        e=FusionEngine();run(e,s=servo(0))
        d=run(e,now=100.4,st=100.31,s=servo(20),r=radar(packets=2))
        self.assertEqual(d['status'],'YAW_MOVING_FAST_ASSOCIATION_PAUSED')
        self.assertIsNone(d['candidate'])
    def test_mount_required(self):
        c=copy.deepcopy(CFG);c['radar_mount']=None
        self.assertEqual(run(FusionEngine(),c=c)['status'],'CALIBRATION_REQUIRED')
    def test_gimbal_mount_no_add_yaw(self):
        c=copy.deepcopy(CFG);c['radar_mount']='gimbal'
        d=run(FusionEngine(),c=c,s=servo(10),r=radar(base=0))
        self.assertIsNotNone(d['candidate'])
    def test_negative_sign(self):
        c=copy.deepcopy(CFG);c['yaw_to_radar_sign']=-1
        d=run(FusionEngine(),c=c,s=servo(-10),r=radar(base=10))
        self.assertIsNotNone(d['candidate'])
    def test_offline_resets_history(self):
        e=FusionEngine()
        for i in range(3):run(e,now=100+i*.1,r=radar(packets=i+1))
        run(e,now=100.35,r=radar(online=False))
        d=run(e,now=100.45,r=radar(packets=20))
        self.assertIsNone(d['stable_candidate_m'])
    def test_radar_packet_counter_reset(self):
        e=FusionEngine()
        for i in range(3):run(e,now=100+i*.1,r=radar(packets=30+i))
        d=run(e,now=100.4,r=radar(packets=1))
        self.assertIsNone(d['stable_candidate_m'])
    def test_non_selected_track(self):
        v=target();v['tracks'][0]['id']=999
        self.assertEqual(run(FusionEngine(),v=v)['status'],'SELECTED_BOX_NOT_FOUND')
    def test_invalid_strength(self):
        r=radar();r['points'][0][2]=-1
        self.assertEqual(len(radar_points(r)),3)
    def test_outdated_radar_frame(self):
        e=FusionEngine();d=run(e,rt=97)
        self.assertEqual(d['status'],'SENSOR_DATA_STALE')

if __name__=='__main__':unittest.main(verbosity=2)
