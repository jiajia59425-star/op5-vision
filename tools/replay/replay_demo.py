"""Synthetic point-cloud reconstruction from Oct 22 text log, NOT original LiDAR packet recording.

Compares previous V3 engine to new V3.1 with known worst case around 0 degrees.
This tool makes NO network or hardware requests.
"""
import importlib.util
import math
from pathlib import Path
from fusion_core import FusionEngine

PKG=Path(__file__).resolve().parent
oldpath=PKG/'old_fusion_core_reference.py'
spec=importlib.util.spec_from_file_location('legacy_fusion_core',oldpath)
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
CONF={'camera_hfov_deg':110,'radar_zero_deg':0,'radar_mount':'chassis','yaw_to_radar_sign':1,'camera_yaw_offset_deg':0}
# sample (expected visual radar bearing, observed candidate bearing, observed distance)
# Simplified from the user's 20-s TEXT summary; noise + point spread are generated,
# so this is NOT validation against real time-aligned raw radar scans.
ROWS=[(21.69,22.75,1.00),(15.70,12.00,.97),(5.98,11.86,.91),
      (2.81,11.97,.91),(1.53,10.79,.97),(354.22,348.97,.98),
      (343.60,344.78,.98),(336.71,340.57,.90),(324.76,326.85,.83),
      (318.52,319.37,.83),(329.76,329.92,.86),(333.50,333.59,.86),
      (359.21,348.09,.56),(1.05,12.00,.96),(358.96,348.95,1.04),
      (11.49,11.89,1.00),(15.07,15.21,.90),(3.39,11.78,.81)]

def cloud(bearing,rng):
    return [[(bearing+i*.2)%360,rng+.003*(i%3),180] for i in range(-13,14)]

def frame(t,n,pred,measured,rng):
    pred=((pred+180)%360)-180
    nx=math.tan(math.radians(pred))/math.tan(math.radians(CONF['camera_hfov_deg']/2))
    cx=50+nx*50
    x1,x2=max(0,cx-19),min(100,cx+19)
    v={'live':True,'found':True,'updated':t-.04,'selected':684,'selected_class':0,
       'width':100,'height':100,'tracks':[{'id':684,'class_id':0,'box':[x1,10,x2,90]}]}
    r={'online':True,'packets':n,'points':cloud(measured,rng),'error':''}
    return (v,t-.04),(r,t-.06),({'yaw':{'angle':0}},t-.07)

def main():
    engines=[('V3 original',old.FusionEngine()),('V3.1',FusionEngine())]
    stats={k:dict(stable=0,uncertain=0) for k,_ in engines}
    print('Synthetic reconstruction only; underlying real raw scans are NOT available.\n')
    for i,(pred,bearing,rng) in enumerate(ROWS):
        now=100+i*.3
        inputs=frame(now,10+i,pred,bearing,rng)
        results=[]
        for label,e in engines:
            out=e.fuse(*inputs,CONF,now)
            if out['stable_candidate_m'] is not None:stats[label]['stable']+=1
            if out['candidate'] is None:stats[label]['uncertain']+=1
            results.append(out['status'])
        print(f'{i+1:02d} predicted={pred:6.2f} radar={bearing:6.2f} range={rng:.2f} | old={results[0]} | v31={results[1]}')
    print('Simulated status counts:',stats)

if __name__=='__main__':main()
