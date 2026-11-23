import json
import pathlib
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from gate import decide
import bridge


def sample_data():
    vision={'live':True,'found':True,'selected':418,'selected_class':0}
    fusion={'version':'V3_1_DYNAMIC_ASSOC_V1','status':'CANDIDATE_STABLE_UNVERIFIED',
        'association_quality':'TRACK_CONTINUOUS_UNVERIFIED','calibrated':True,
        'camera_online':True,'radar_online':True,'config':{'radar_mount':'chassis'},
        'sources':{'vision':{'ok':True},'radar':{'ok':True},'servo':{'ok':True}},
        'fusion_age_ms':55,'independent_confirmations':4,'angular_residual_deg':1.0,
        'stable_candidate_m':1.2,'target':{'id':418,'class_id':0}}
    auto={'active':True,'target_id':418,'class_id':0,'source':'rgb','range_mode':'safe',
          'fast_profile':'PREDICTIVE_FAST_V2','moves':20}
    return fusion,vision,auto

class GateTests(unittest.TestCase):
    def test_ready(self):
        f,v,a=sample_data()
        self.assertTrue(decide(f,v,a,(418,0))[0])
    def test_target_changed(self):
        f,v,a=sample_data();v['selected']=419
        self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_fusion_changed_id(self):
        f,v,a=sample_data();f['target']['id']=999
        self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_fail_closed_rad_range(self):
        for dist in [0.55,6.3,None,float('nan')]:
            f,v,a=sample_data();f['stable_candidate_m']=dist
            self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_angle(self):
        f,v,a=sample_data();f['angular_residual_deg']=10
        self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_stale(self):
        f,v,a=sample_data();f['fusion_age_ms']=500
        self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_unconfirmed(self):
        f,v,a=sample_data();f['independent_confirmations']=2
        self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_offline_source(self):
        for name in ('vision','radar','servo'):
            f,v,a=sample_data();f['sources'][name]['ok']=False
            self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_unsafe_mode(self):
        f,v,a=sample_data();a['range_mode']='full'
        self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_stop_external(self):
        f,v,a=sample_data();a['active']=False
        self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_no_unverified(self):
        f,v,a=sample_data();f['status']='CANDIDATE_WAITING_STABILITY'
        self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_version(self):
        f,v,a=sample_data();f['version']='V3_0'
        self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_mount(self):
        f,v,a=sample_data();f['config']['radar_mount']='gimbal'
        self.assertFalse(decide(f,v,a,(418,0))[0])
    def test_plant_class_refused(self):
        f,v,a=sample_data()
        f['target']['class_id']=58
        v['selected_class']=58
        f['target']['class_name']='potted plant'
        self.assertFalse(decide(f,v,a,(418,58))[0])
        f,v,a=sample_data()
        f['target']['class_name']='potted plant'
        self.assertFalse(decide(f,v,a,(418,0))[0])

class BridgeTests(unittest.TestCase):
    def setUp(self):
        with bridge.LOCK:
            bridge.STATUS.update(armed=False,session=None,target=None,reason='idle',
              last_gate_ok=False,last_gate_reason='',moves=None,start_at=None,
              last_pulse_at=None,operator_ping=None,stop_pending=False)
    def test_no_motion_when_not_armed(self):
        with patch.object(bridge,'request_json') as post:
            bridge.monitor_once()
            post.assert_not_called()
    def test_safe_stop_when_invalid(self):
        with bridge.LOCK:
            bridge.STATUS.update(armed=True,session='S',target=(418,0),operator_ping=time.monotonic())
        with patch.object(bridge,'sample',return_value=(False,'STALE',None,{},{})), patch.object(bridge,'request_json',return_value={'ok':True}) as api:
            bridge.monitor_once()
            self.assertFalse(bridge.STATUS['armed'])
            self.assertEqual(api.call_args.args[:2],(8093,'/api/auto/stop'))
    def test_stop_retry(self):
        with bridge.LOCK:bridge.STATUS.update(armed=True,session='S',target=(418,0))
        with patch.object(bridge,'request_json',side_effect=RuntimeError('offline')):
            bridge.safe_stop('test')
            self.assertTrue(bridge.STATUS['stop_pending'])
    def test_pulse_when_ready(self):
        _,_,a=sample_data()
        with bridge.LOCK:
            bridge.STATUS.update(armed=True,session='S',target=(418,0),last_pulse_at=time.monotonic()-3,operator_ping=time.monotonic())
        with patch.object(bridge,'sample',return_value=(True,'READY',(418,0),{},a)),patch.object(bridge,'request_json',return_value={'ok':True}) as post:
            bridge.monitor_once()
            self.assertTrue(bridge.STATUS['armed'])
            self.assertEqual(post.call_args.args[:2],(8093,'/api/auto/pulse'))
    def test_no_pulse_on_radar_invalidation(self):
        with bridge.LOCK:bridge.STATUS.update(armed=True,session='S',target=(418,0),last_pulse_at=0,operator_ping=time.monotonic())
        with patch.object(bridge,'sample',return_value=(False,'NO_RADAR',None,{},{})),patch.object(bridge,'request_json',return_value={'ok':True}) as post:
            bridge.monitor_once()
            self.assertNotIn('/api/auto/pulse',[x.args[1] for x in post.call_args_list])
    def test_operator_disconnected(self):
        with bridge.LOCK:bridge.STATUS.update(armed=True,session='S',target=(418,0),operator_ping=time.monotonic()-4)
        with patch.object(bridge,'request_json',return_value={'ok':True}) as post:
            bridge.monitor_once()
            self.assertFalse(bridge.STATUS['armed'])
            self.assertEqual(post.call_args.args[:2],(8093,'/api/auto/stop'))
    def test_no_auto_restart(self):
        with patch.object(bridge,'start_operator') as startup, patch.object(bridge,'request_json') as post:
            bridge.monitor_once()
            startup.assert_not_called()
            post.assert_not_called()
    def test_start_fails_on_other_operator(self):
        with patch.object(bridge,'sample',return_value=(True,'ok',(418,0),{},{})),patch.object(bridge,'request_json',return_value={'active':True}):
            with self.assertRaisesRegex(RuntimeError,'其他跟踪'):
                bridge.start_operator()
    def test_start_authorized_v2(self):
        seq=iter([{'active':False},
           {'unlocked':False},{'unlocked':False},{'unlocked':False},{'unlocked':True,'session':'tok'},
           {'ok':True,'result':{'active':True}}])
        def api(port,path,payload=None,token=None,timeout=0.9):return next(seq)
        with patch.object(bridge,'sample',return_value=(True,'ok',(418,0),{},{})),patch.object(bridge,'request_json',side_effect=api):
            bridge.start_operator()
        self.assertTrue(bridge.STATUS['armed'])
        self.assertEqual(bridge.STATUS['target'],(418,0))

if __name__=='__main__':unittest.main(verbosity=2)
