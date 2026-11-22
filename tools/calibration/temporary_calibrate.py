#!/usr/bin/env python3
"""Safely trial a 357.7-deg zero with live 8100 & 8101 read-only preflight.

All actuator interactions are GET-only. Checks bridge/gimbal are idle.
The actual config is restored in finally, with an independent systemd
rollback timer for terminal disconnects and unexpected process death.
"""
import argparse
import collections
import hashlib
import json
import math
import os
import pathlib
import select
import shutil
import statistics
import subprocess
import sys
import time
import urllib.request
from datetime import datetime
from restore import atomic_bytes, restore, sha

CFG=pathlib.Path('/home/orangepi/op5_v3_fusion/config.json')
CORE=pathlib.Path('/home/orangepi/op5_v3_fusion/fusion_core.py')
BACKUP_DIR=pathlib.Path('/var/lib/op5-v3cal')
EXPECTED_OLD=351.5
TARGET=357.7
POSES=('中央近距离（约1.5～2米）','画面左侧约20%','画面右侧约20%','中央较远（约2.5～3米）')

def get(port,path):
    req=urllib.request.Request(f'http://127.0.0.1:{port}{path}',
                               headers={'Cache-Control':'no-store','Accept':'application/json'})
    with urllib.request.urlopen(req,timeout=3) as r:
        return json.load(r)

def is_true_false(value):
    return value is True or value is False

def idle_guard():
    guard=get(8101,'/api/status')
    auto=get(8093,'/api/auto/status')
    if guard.get('armed') is not False or guard.get('stop_pending') is not False:
        raise RuntimeError('8101已授权/停机待确认，禁止标定')
    if auto.get('active') is not False:
        raise RuntimeError('8093自动跟踪未关闭，禁止标定')
    center=get(8093,'/api/center/status')
    hold=get(8093,'/api/hold/status')
    if center.get('active') is True or hold.get('state')=='busy':
        raise RuntimeError('8093回中或位置保持恢复仍在动作，禁止标定')
    if center.get('active') is not False:
        raise RuntimeError('无法确认回中未运行，禁止标定')

def initial_checks():
    if os.geteuid()!=0:raise RuntimeError('必须使用 sudo 执行')
    if not CFG.is_file():raise RuntimeError('找不到现场config.json')
    if not CORE.is_file() or 'ASSOCIATION_COMPETING_DEPTHS' not in CORE.read_text('utf-8'):
        raise RuntimeError('未检测到V3.1.3双深度歧义保护')
    conf=json.loads(CFG.read_text('utf-8'))
    if not isinstance(conf,dict):raise RuntimeError('config.json格式异常')
    if conf.get('radar_mount')!='chassis':raise RuntimeError('radar_mount不是chassis')
    if type(conf.get('radar_zero_deg')) not in (float,int) or abs(conf['radar_zero_deg']-EXPECTED_OLD)>0.05:
        raise RuntimeError('原零位并非351.5°，拒绝覆盖: '+str(conf.get('radar_zero_deg')))
    if type(conf.get('camera_hfov_deg')) not in (float,int) or abs(conf['camera_hfov_deg']-110)>0.1:
        raise RuntimeError('HFOV不是110°，拒绝覆盖')
    if conf.get('yaw_to_radar_sign')!=1:raise RuntimeError('Yaw符号不是+1，拒绝覆盖')
    idle_guard()
    h=get(8100,'/api/health')
    if h.get('ok') is not True or h.get('timing_patch')!='V3_1_2_SOURCE_TIMING':
        raise RuntimeError('8100采集时序版本不匹配')
    f=get(8100,'/api/fusion')
    if abs((f.get('config') or {}).get('radar_zero_deg',0)-EXPECTED_OLD)>0.05:
        raise RuntimeError('8100当前加载的雷达零位不是351.5°')
    v=get(8081,'/state')
    r=get(8092,'/api/scan')
    if not v.get('live') or r.get('online') is not True:
        raise RuntimeError('视觉或雷达当前不在线')
    if shutil.which('systemd-run') is None:
        raise RuntimeError('没有systemd-run，无法确保独立定时回退')
    return conf

def wait_for_cfg(value,deadline=5):
    end=time.monotonic()+deadline
    while time.monotonic()<end:
        f=get(8100,'/api/fusion')
        c=(f.get('config') or {}).get('radar_zero_deg')
        if type(c) in (int,float) and abs(c-value)<0.05:
            return True
        time.sleep(.18)
    return False

def prompt(tty,pose,timeout=75):
    print('\n请人物站到'+pose+'，保持云台和底盘静止。准备好后按回车（限75秒）',flush=True)
    ok,_,_=select.select([tty],[],[],timeout)
    if not ok: raise TimeoutError('无人确认，停止试验并恢复配置')
    tty.readline()

def read_samples(pose):
    statuses=collections.Counter()
    reasons=collections.Counter()
    ranges=[]; residues=[]; ages=[]; targets=set(); passes=0; stable=0; errors=0
    prev_packets=None; packet_updates=0
    for i in range(20):
        if i%5==0: idle_guard()
        f=get(8100,'/api/fusion')
        p=get(8101,'/api/preflight')
        r=get(8092,'/api/scan')
        value=(f.get('config') or {}).get('radar_zero_deg')
        if type(value) not in (float,int) or abs(value-TARGET)>.05:
            raise RuntimeError('配置在试验期间被修改或8100读取异常')
        statuses[f.get('status','UNKNOWN')]+=1
        if not p.get('ready'): reasons[p.get('reason','UNKNOWN')]+=1
        else: passes+=1
        if f.get('status')=='CANDIDATE_STABLE_UNVERIFIED': stable+=1
        rng=f.get('stable_candidate_m'); res=f.get('angular_residual_deg'); age=f.get('fusion_age_ms')
        if type(rng) in (int,float) and math.isfinite(rng): ranges.append(rng)
        if type(res) in (int,float) and math.isfinite(res): residues.append(res)
        if type(age) in (int,float) and math.isfinite(age): ages.append(age)
        t=f.get('target') or {}
        if type(t.get('id')) is int:targets.add(t['id'])
        packets=r.get('packets')
        if prev_packets is not None and packets!=prev_packets: packet_updates+=1
        prev_packets=packets
        time.sleep(.25)
    def med(a):return round(statistics.median(a),3) if a else None
    out={'position':pose,'preflight_ready':f'{passes}/20','stable':f'{stable}/20',
         'residual_deg_median':med(residues),'range_m_median':med(ranges),
         'fusion_age_ms_median':med(ages),'ids':sorted(targets),
         'radar_packet_changes':f'{packet_updates}/19',
         'status_counts':dict(statuses),'preflight_rejections':dict(reasons)}
    print('实时回归:',json.dumps(out,ensure_ascii=False),flush=True)
    return out

def run():
    initial_checks()
    print('CHECK PASS：8101未授权、8093未跟踪，8100为V3.1.2时序 + V3.1.3关联保护。')
    BACKUP_DIR.mkdir(mode=0o700,parents=True,exist_ok=True)
    os.chmod(BACKUP_DIR,0o700)
    saved=CFG.read_bytes()
    # Preserve byte-for-byte original in a root-only backup, not just rounded parameters.
    tag=datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid())
    backup=BACKUP_DIR/('config_original_'+tag+'.json')
    backup.write_bytes(saved);os.chmod(backup,0o600)
    if backup.read_bytes()!=saved:raise RuntimeError('备份校验失败')
    cfg=json.loads(saved)
    cfg['radar_zero_deg']=TARGET
    patched=(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n').encode('utf-8')
    patched_sha=sha(patched)
    unit='op5-v3cal-restore-'+str(os.getpid())
    cmd=['systemd-run','--quiet','--on-active=600s','--unit='+unit,
         '/usr/bin/python3',str(pathlib.Path(__file__).resolve().parent/'restore.py'),
         str(CFG),str(backup),patched_sha]
    subprocess.run(cmd,check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=8)
    print('独立回退保险已启用：10分钟后自动检查/恢复旧配置（若提前正常恢复则自动跳过）。')
    modified=False; restored=False
    try:
        idle_guard()
        atomic_bytes(CFG,patched); modified=True
        if not wait_for_cfg(TARGET,6):raise RuntimeError('8100没有读取357.7°配置，终止')
        print('临时零位 357.7° 已被8100实时读取。未重启服务、未向8093发送运动指令。')
        print('采集使用当前8101真实GET /api/preflight，不是离线模拟许可。')
        rows=[]
        with open('/dev/tty','r') as tty:
            for pose in POSES:
                idle_guard()
                prompt(tty,pose)
                time.sleep(2)   # Allow association history to settle after moving person.
                rows.append(read_samples(pose))
        print('\n===== 四位置真实8101许可汇总 =====',flush=True)
        for x in rows:
            print(x['position'], '| Ready:',x['preflight_ready'],
                  '| 稳定:',x['stable'], '| 残差:',x['residual_deg_median'],
                  '| 距离:',x['range_m_median'], '| Age:',x['fusion_age_ms_median'],flush=True)
        print('此次只读控制许可验证完成。下一步仍需独立验证误关联率。')
    finally:
        if modified:
            restored=restore(CFG,backup,patched_sha)
        else:
            restored=(CFG.read_bytes()==saved)
        if restored and CFG.read_bytes()==saved:
            if wait_for_cfg(EXPECTED_OLD,6):
                print('ROLLBACK VERIFIED：原始配置字节和8100运行时零位均已恢复351.5°。',flush=True)
            else:
                print('配置文件已恢复，但8100未确认加载旧值。手动检查8100服务。',file=sys.stderr,flush=True)
        else:
            print('!!! 自动恢复未确认！独立10分钟定时回退仍保留，请停止控制并检查。',file=sys.stderr,flush=True)
        # Leave fallback armed; it is idempotent and ensures restoration if any later issue.
        print('原始备份路径:',backup,flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--check',action='store_true',help='read-only prerequisites')
    parser.add_argument('--run',action='store_true',help='temporary on-device trial')
    args=parser.parse_args()
    try:
        if args.check:
            initial_checks()
            print('CHECK PASS: 原零位351.5°、雷达chassis、HFOV110、V3.1.2+V3.1.3校验通过；8093/8101未在跟踪')
        elif args.run:
            run()
        else: parser.error('specify --check or --run')
    except (KeyboardInterrupt,TimeoutError) as e:
        print('试验中断：',str(e),'配置回退流程已执行（如曾写入）。',flush=True)
        sys.exit(130)
    except Exception as exc:
        print('TEST FAILED:',type(exc).__name__,str(exc),file=sys.stderr,flush=True)
        sys.exit(1)
