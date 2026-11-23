#!/usr/bin/env python3
"""Offline identical-snapshot comparison of V3 and V3.1. No web or motor access."""
import argparse
import importlib.util
import json
from pathlib import Path
from fusion_core import FusionEngine

ROOT=Path(__file__).resolve().parent
old_file=ROOT/'old_fusion_core_reference.py'
if not old_file.is_file():
    old_file=ROOT.parents[1]/'fusion_8100'/'history'/'v3'/'fusion_core.py'
spec=importlib.util.spec_from_file_location('old_v3',old_file)
legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)

def main():
    p=argparse.ArgumentParser()
    p.add_argument('capture',type=Path)
    p.add_argument('--config',type=Path,default=Path('/home/orangepi/op5_v3_fusion/config.json'))
    a=p.parse_args()
    cfg=json.loads(a.config.read_text('utf-8'))
    engines=[('V3',legacy.FusionEngine()),('V3.1',FusionEngine())]
    totals={k:{'frames':0,'stable':0,'hidden':0,'angular_uncertain':0,'switches':0} for k,_ in engines}
    previous_key=None
    print('两版本用同一份原始雷达/YOLO/舵机JSON回放；稳定只表示候选连续，不证明身份\n')
    with a.capture.open('r',encoding='utf-8') as f:
        for line_no,line in enumerate(f,1):
            if not line.strip():continue
            obj=json.loads(line)
            now=obj['now'];s=obj['samples']
            if not all(k in s for k in ('vision','radar','servo')):continue
            key=(s['vision'][0].get('selected'),s['vision'][0].get('selected_class'))
            if previous_key is not None and key!=previous_key:
                for k,_ in engines:totals[k]['switches']+=1
            previous_key=key
            for name,engine in engines:
                out=engine.fuse(s['vision'],s['radar'],s['servo'],cfg,now)
                t=totals[name];t['frames']+=1
                if out.get('stable_candidate_m') is not None:t['stable']+=1
                if out.get('candidate') is None:t['hidden']+=1
                if 'ANGULAR_UNCERTAIN' in out.get('status',''):t['angular_uncertain']+=1
            if line_no%10==0:
                print(f'第{line_no}帧 | V3 {engines[0][1].last_id} | 观察累计: {totals}')
    print('\n回放统计 (不能代替目视/场景真值验证):')
    for name in totals:print(name,totals[name])

if __name__=='__main__':main()
