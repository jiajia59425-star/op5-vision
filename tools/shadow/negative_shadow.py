#!/usr/bin/env python3
"""OP5 V3 selected-human vs radar-depth decoy: strictly read-only shadow assessment.

No config writes, no service restarts, no POST/PUT/PATCH/DELETE, no actuator commands.
Simulate the calibrated 357.7 deg zero using in-memory FusionEngine and 8101 gate.
"""
import collections
import json
import math
import pathlib
import select
import statistics
import sys
import time
import urllib.request

FUSION = pathlib.Path('/home/orangepi/op5_v3_fusion')
BRIDGE = pathlib.Path('/home/orangepi/op5_v3_fusion_bridge')
sys.path.insert(0, str(FUSION))
sys.path.insert(0, str(BRIDGE))
from fusion_core import FusionEngine
from gate import decide

POSITIONS = (
    ('A', '人物站在中央标记点，8090选中该人物；附近不放其他障碍物'),
    ('B', '人物完全离开摄像头画面和雷达扫描方向，保留原先静止背景'),
    ('C', '人物仍不在画面里，在原位置放纸箱或椅子作替代物'),
    ('D', '移除替代物；人物返回A的标记点，在8090重新选中人物'),
    ('E', '人物仍在D的标记点且可被RGB看到，在相近雷达方向稍靠前位置放纸箱形成点簇竞争；不得完全遮挡人物'),
)
SAMPLES=20

def get(port,path):
    req=urllib.request.Request('http://127.0.0.1:%d%s'%(port,path),
        headers={'Accept':'application/json','Cache-Control':'no-store'})
    with urllib.request.urlopen(req,timeout=3) as r:
        return json.load(r)

def idle_guard():
    bridge=get(8101,'/api/status')
    auto=get(8093,'/api/auto/status')
    center=get(8093,'/api/center/status')
    hold=get(8093,'/api/hold/status')
    if bridge.get('armed') is not False or bridge.get('stop_pending') is not False:
        raise RuntimeError('8101仍授权或有停机待确认，拒绝测试')
    if auto.get('active') is not False or center.get('active') is not False or hold.get('state')=='busy':
        raise RuntimeError('8093云台尚未确认静止，拒绝测试')

def med(seq):
    return round(statistics.median(seq),3) if seq else None

def phase(label,description,conf,tty):
    print('\n[%s] %s\n准备好后按回车（限75秒）。保持云台、底盘静止。'%(label,description),flush=True)
    ok,_,_=select.select([tty],[],[],75)
    if not ok:raise TimeoutError('等待用户确认超时')
    tty.readline()
    time.sleep(1.5)
    idle_guard()
    e=FusionEngine()  # Reset phase history deliberately
    statuses=collections.Counter()
    reasons=collections.Counter()
    ids=set()
    found=0; allowed=0; stable=0; errors=0
    ranges=[]; residuals=[]; frameages=[]; radar_packets=0; lastpkt=None
    for i in range(SAMPLES):
        if i%5==0:idle_guard()
        try:
            v=get(8081,'/state')
            r=get(8092,'/api/scan'); rt=time.monotonic()
            s=get(8093,'/api/status'); st=time.monotonic()
            now=time.monotonic()
            pkt=r.get('packets')
            if lastpkt is not None and pkt!=lastpkt:radar_packets+=1
            lastpkt=pkt
            if v.get('found') is True:found+=1
            if type(v.get('selected')) is int:ids.add(v['selected'])
            fused=e.fuse((v,v.get('updated')),(r,rt),(s,st),conf,now=now)
            fused['config']={'radar_mount':conf.get('radar_mount')}
            fused['sources']={'vision':{'ok':True},'radar':{'ok':True},'servo':{'ok':True}}
            good,reason,_=decide(fused,v)
            allowed+=int(good)
            if not good:reasons[reason]+=1
            statuses[fused['status']]+=1
            stable+=int(fused['status']=='CANDIDATE_STABLE_UNVERIFIED')
            if isinstance(fused.get('stable_candidate_m'),(int,float)):
                ranges.append(fused['stable_candidate_m'])
            if isinstance(fused.get('angular_residual_deg'),(int,float)):
                residuals.append(fused['angular_residual_deg'])
            if isinstance(fused.get('fusion_age_ms'),(int,float)):
                frameages.append(fused['fusion_age_ms'])
        except Exception as exc:
            errors+=1
            print('样本 %02d 读取失败：%s'%(i+1,exc),flush=True)
        time.sleep(.25)
    row={'阶段':label,'视觉出现帧数':found,'YOLO选中IDs':sorted(ids),
         '模拟许可':'%d/%d'%(allowed,SAMPLES),'稳定候选':'%d/%d'%(stable,SAMPLES),
         '候选距离中位m':med(ranges),'方位偏差中位°':med(residuals),
         '融合数据年龄中位ms':med(frameages),'雷达包更新':'%d/%d'%(radar_packets,SAMPLES-1),
         '状态':dict(statuses),'拒绝原因':dict(reasons),'读取异常':errors}
    print('结果:',json.dumps(row,ensure_ascii=False),flush=True)
    return row

def countval(row,key):
    return int(row[key].split('/')[0])

def main():
    idle_guard()
    cfg=json.loads((FUSION/'config.json').read_text(encoding='utf-8'))
    actual=cfg.get('radar_zero_deg')
    if not isinstance(actual,(int,float)) or abs(actual-351.5)>.06:
        raise RuntimeError('现场配置不是恢复后的351.5°，为避免误判而终止')
    if cfg.get('radar_mount')!='chassis' or cfg.get('yaw_to_radar_sign')!=1:
        raise RuntimeError('现场安装参数不符合已验证配置')
    if 'ASSOCIATION_COMPETING_DEPTHS' not in (FUSION/'fusion_core.py').read_text(encoding='utf-8'):
        raise RuntimeError('缺少V3.1.3双深度歧义保护')
    if get(8100,'/api/health').get('timing_patch')!='V3_1_2_SOURCE_TIMING':
        raise RuntimeError('8100时序版本不符合已验证环境')
    cfg=dict(cfg);cfg['radar_zero_deg']=357.7
    print('V3误关联负样本影子测试｜现场351.5°保持不变｜内存使用357.7°')
    print('只读GET：不写配置、不重启、不启动8101、不向8093发送运动指令')
    print('模拟许可并不代表真实8101可启动或目标身份已确认。')
    results=[]
    with open('/dev/tty','r') as tty:
        for label,description in POSITIONS:
            results.append(phase(label,description,cfg,tty))
    print('\n===== 误关联诊断汇总 =====')
    for row in results:
        print('%s: visual=%s/20, would_allow=%s, stable=%s, range=%s, IDs=%s'%(
            row['阶段'],row['视觉出现帧数'],row['模拟许可'],row['稳定候选'],
            row['候选距离中位m'],row['YOLO选中IDs']))
    A,B,C,D,E=results
    warnings=[]
    if countval(B,'模拟许可')>0:
        warnings.append('B人物离开后仍存在模拟许可，需检查目标丢失判定')
    if countval(C,'模拟许可')>0:
        warnings.append('C只有替代物时仍存在模拟许可，需检查目标身份误关联')
    if countval(D,'模拟许可')<14:
        warnings.append('D人物重新选中后许可不足，需检查恢复能力')
    if D['候选距离中位m'] is not None and E['候选距离中位m'] is not None:
        drift=E['候选距离中位m']-D['候选距离中位m']
        print('D->E距离中位数变化：%+.3fm'%drift)
        if abs(drift)>=0.25 and countval(E,'模拟许可')>0:
            warnings.append('E引入竞争物后距离明显改变却仍被放行，存在误关联风险')
    if E['读取异常'] or any(row['读取异常'] for row in results):
        warnings.append('存在接口读取异常，数据不足以确认')
    if warnings:
        print('需处理：')
        for issue in warnings:print(' - '+issue)
    else:
        print('本轮未发现规则触发的明显异常；这不等于证明识别身份或零误关联。')
    print('全部测试只读完成。config.json未修改，云台/底盘未收到控制指令。')

if __name__=='__main__':
    try:
        main()
    except (KeyboardInterrupt,EOFError,TimeoutError) as exc:
        print('用户中止/超时：%s；没有配置修改。'%exc,flush=True)
        sys.exit(130)
    except Exception as exc:
        print('TEST ERROR: %s: %s'%(type(exc).__name__,exc),file=sys.stderr,flush=True)
        sys.exit(1)
