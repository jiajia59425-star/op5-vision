import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from fusion_core import FusionEngine,angular_delta,enumerate_clusters,radar_points

def group(angle,rng,spread=.008,n=16):
    return [[(angle+(i-(n-1)/2)*.36)%360,rng+(i%3-1)*spread,200] for i in range(n)]
CFG={'camera_hfov_deg':110,'radar_zero_deg':0.0,'radar_mount':'chassis',
     'yaw_to_radar_sign':1,'camera_yaw_offset_deg':0.0}
def run(engine,t,pkt,zero=0,pts=None):
    vis={'live':True,'found':True,'selected':612,'selected_class':0,'updated':t-.04,
         'width':1280,'height':720,'tracks':[{'id':612,'class_id':0,'box':[515,160,765,680]}]}
    rad={'online':True,'packets':pkt,'points':pts if pts is not None else group(0,1.8),'error':''}
    return engine.fuse((vis,t-.04),(rad,t-.07),({'yaw':{'angle':0}},t-.06),
                       dict(CFG,radar_zero_deg=zero),t)

class Tests(unittest.TestCase):
    def test_competing_depth_prevents_false_stable(self):
        engine=FusionEngine()
        pts=group(355.9,2.917,spread=.2)+group(4.9,1.815)
        for i in range(5):
            d=run(engine,100+i*.15,i+1,pts=pts)
            self.assertEqual(d['status'],'ASSOCIATION_COMPETING_DEPTHS')
            self.assertIsNone(d['candidate'])
            self.assertIsNone(d['stable_candidate_m'])
    def test_corrected_bearing_allows_only_near_candidate(self):
        engine=FusionEngine()
        pts=group(355.9,2.917,spread=.2)+group(4.9,1.815)
        for i in range(4):d=run(engine,100+i*.15,i+1,zero=4.9,pts=pts)
        self.assertEqual(d['status'],'CANDIDATE_STABLE_UNVERIFIED')
        self.assertAlmostEqual(d['stable_candidate_m'],1.815,places=2)
    def test_ordinary_singular_cluster_still_works(self):
        e=FusionEngine()
        for i in range(3):d=run(e,100+i*.2,i+1)
        self.assertEqual(d['status'],'CANDIDATE_STABLE_UNVERIFIED')
    def test_new_target_id_unsupported_no_identity_claim(self):
        d=run(FusionEngine(),100,1)
        self.assertIsNone(d['stable_candidate_m'])
        self.assertIn('UNVERIFIED',d['association_quality'])
    def test_zero_wrap(self):
        self.assertEqual(round(angular_delta(359.0,1.0),2),-2.0)
        self.assertEqual(len(enumerate_clusters(radar_points({'online':True,'points':group(359.9,1.5)}),0,5)),1)

if __name__=='__main__':unittest.main(verbosity=2)
