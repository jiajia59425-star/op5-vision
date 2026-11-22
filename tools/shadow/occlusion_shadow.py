#!/usr/bin/env python3
"""V3 occlusion probe. READ ONLY. Simulate zero=357.7 in RAM.

Two phases feed a *single* existing FusionEngine instance:
 A: same selected person, no decoy
 B: same person selected, foreground box physically blocks lidar
Never calls control APIs, writes config, or issues motion commands.
"""
import collections
import json
import pathlib
import select
import statistics
import sys
import time
import urllib.request

ROOT=pathlib.Path('/home/orangepi/op5_v3_fusion')
BRIDGE=pathlib.Path('/home/orangepi/op5_v3_fusion_bridge')
sys.path[:0]=[str(ROOT),str(BRIDGE)]
from fusion_core import FusionEngine
from gate import decide

PHASES=[('A','人物保持中央位置，纸箱移走'), ('B','同一人物保持不动且仍被RGB选中，把纸箱放到雷达扫描平面内挡住人物的雷达回波')]


def get(port,path):
    req=urllib.request.Request(f'http://127.0.0.1:{port}{path}',headers={'Cache-Control':'no-store','Accept':'application/json'})
    with urllib.request.urlopen(req, timeout=4) as r:
        return json.load(r)


def check_idle():
    bridge=get(8101,'/api/status')
    a=get(8093,'/api/auto/status')
    center=get(8093,'/api/center/status')
    hold=get(8093,'/api/hold/status')
    if bridge.get('armed') is not False or bridge.get('stop_pending') is not False:
        raise RuntimeError('8101仍在授权或停机待确认，请先停止')
    if a.get('active') is not False or center.get('active') is not False or hold.get('state')=='busy':
        raise RuntimeError('8093云台仍在跟踪、回中或保持恢复，请先停止')


def med(rows,key):
    values=[v[key] for v in rows if isinstance(v.get(key),(int,float))]
    return round(statistics.median(values),3) if values else None


def sample(engine,conf):
    v=get(8081,'/state')
    radar=get(8092,'/api/scan'); rt=time.monotonic()
    servo=get(8093,'/api/status'); st=time.monotonic()
    now=time.monotonic()
    f=engine.fuse((v,v.get('updated')),(radar,rt),(servo,st),conf,now=now)
    f['config']={'radar_mount':conf['radar_mount']}
    f['sources']={'vision':{'ok':True},'radar':{'ok':radar.get('online') is True},'servo':{'ok':True}}
    allowed,reason,_=decide(f,v)
    target=(f.get('target') or {})
    bbox=target.get('box')
    vw=target.get('width')
    bbox_width=(bbox[2]-bbox[0])/vw if bbox and vw else None
    bbox_height=(bbox[3]-bbox[1])/target['height'] if bbox and target.get('height') else None
    return {
        'id':target.get('id'), 'found':v.get('found'), 'status':f.get('status'),
        'allow':allowed,'reason':reason,
        'range_m':f.get('stable_candidate_m'),
        'candidate_m':(f.get('candidate') or {}).get('range_m'),
        'angle_error':f.get('angular_residual_deg'),
        'frame_age_ms':f.get('fusion_age_ms'),
        'width_ratio':bbox_width,'height_ratio':bbox_height,
        'packet':radar.get('packets'),
    }


def await_user(tty, text):
    print('\n'+text+'，按回车开始20次采样（最长等待75秒）。',flush=True)
    ready,_,_=select.select([tty],[],[],75)
    if not ready:raise TimeoutError('操作等待超时')
    tty.readline()
    time.sleep(1.2)


def run():
    check_idle()
    config=json.loads((ROOT/'config.json').read_text(encoding='utf-8'))
    if config.get('radar_mount')!='chassis' or config.get('yaw_to_radar_sign')!=1:
        raise RuntimeError('雷达安装参数不匹配')
    if abs(float(config.get('radar_zero_deg',999))-351.5)>0.1:
        raise RuntimeError('当前不是已知351.5°基线，拒绝影子测试')
    if get(8100,'/api/health').get('timing_patch')!='V3_1_2_SOURCE_TIMING':
        raise RuntimeError('8100版本不匹配')
    conf=dict(config)
    conf['radar_zero_deg']=357.7
    engine=FusionEngine()  # single instance shared across the two stages
    results=[]
    print('8100原配置保持351.5°；独立内存模拟357.7°；不会调用8101启停接口')
    print('特别注意：测试只证明某些场景的候选风险，不证明目标身份。')
    with open('/dev/tty','r') as tty:
        for phase,description in PHASES:
            await_user(tty,f'[{phase}] {description}')
            check_idle()
            rows=[]
            for i in range(20):
                if i%5==0:check_idle()
                entry=sample(engine,conf)
                rows.append(entry)
                print(f"{phase} {i+1:02d}/20 id={entry['id']} status={entry['status']} range={entry['range_m']} "+
                      f"candidate={entry['candidate_m']} permit={entry['allow']} bbox_w={entry['width_ratio']}",flush=True)
                time.sleep(.25)
            results.append(rows)
    A,B=results
    def summary(label,rows):
        return {'phase':label,'ids':sorted({r['id'] for r in rows if isinstance(r['id'],int)}),
                'found':sum(r['found'] is True for r in rows),
                'allowed':sum(r['allow'] for r in rows),'status_counts':dict(collections.Counter(r['status'] for r in rows)),
                'range_median':med(rows,'range_m'),'candidate_median':med(rows,'candidate_m'),
                'bbox_width_ratio_median':med(rows,'width_ratio'),'bbox_height_ratio_median':med(rows,'height_ratio'),
                'rejections':dict(collections.Counter(r['reason'] for r in rows if not r['allow']))}
    a,b=summary('A',A),summary('B',B)
    print('\n===== 同一FusionEngine连续两阶段 =====')
    print(json.dumps(a,ensure_ascii=False,indent=2))
    print(json.dumps(b,ensure_ascii=False,indent=2))
    if a['range_median'] is not None and b['range_median'] is not None:
        diff=b['range_median']-a['range_median']
        print(f'融合稳定距离变化：{diff:+.3f} m')
        if diff<=-0.25 and b['allowed']>0:
            print('!! OCCLUSION_FALSE_PERMIT_RISK: 距离明显缩短，然而融合门控仍放行；请勿实机启动8101。')
        elif diff<=-0.25:
            print('OCCLUSION_DETECTED_OR_REJECTED: 距离明显缩短，但该阶段许可为0；仍需检查长时间保持是否重新放行。')
        else:
            print('此轮未形成所需的距离突变；需要复核纸箱雷达扫描位置。')
    elif b['allowed']>0 and b['candidate_median'] is not None and a['candidate_median'] is not None and b['candidate_median'] < a['candidate_median']-0.25:
        print('!! OCCLUSION_FALSE_PERMIT_RISK: 纸箱候选距离缩短且曾获融合许可，请勿实机启动8101。')
    else:
        print('无法仅凭本轮数据判断距离替换是否被安全拒绝；请提供完整输出。')
    print('完成：只GET读取；未写入配置、未启用云台或底盘。')

if __name__=='__main__':
    try:run()
    except (EOFError,KeyboardInterrupt,TimeoutError) as exc:
        print('测试中止:',type(exc).__name__,str(exc),file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print('TEST ERROR:',type(exc).__name__,str(exc),file=sys.stderr)
        sys.exit(1)
