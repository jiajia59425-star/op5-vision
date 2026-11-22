#!/usr/bin/env python3
"""Strict install/rollback of separate local-only V3 fusion supervision bridge."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import pwd
import secrets
import shutil
import socket
import subprocess
import sys
import urllib.request

PKG=Path(__file__).resolve().parent
DST=Path('/home/orangepi/op5_v3_fusion_bridge')
UNIT=Path('/etc/systemd/system/op5-v3-fusion-gimbal-8101.service')
FUSION=Path('/home/orangepi/op5_v3_fusion')
MOTOR=Path('/usr/local/lib/op5-gimbal-web-8093')
SOURCES=('gate.py','bridge.py','index.html')
EXPECTED_8093='69e8823ed889e565902ea097eeaaa6c4a3e5835c956ac8470d6daae6d51fb5ae'
EXPECTED_TEMP='4a882158f1209b2016fbe116970d4fadbcc81d1c795e7253a00b265a23df8666'
EXPECTED_FUSION='6d1cfcc68596e5e44e0b43da7cf8e2be28176092a4719b956ec8fc0b9a8cd1b5'
EXPECTED_SERVICE='aa13b140b19bf182badee8a3dbc5bd3bb1c0492749c17c00c1f6871ab4e02b64'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def fetch(port,path):
    with urllib.request.urlopen(f'http://127.0.0.1:{port}{path}',timeout=2) as r:
        return json.load(r)


def check():
    for name in SOURCES:
        if sha(PKG/name)!=json.loads((PKG/'manifest.json').read_text('utf-8'))[name]:
            raise RuntimeError('安装包SHA校验失败:'+name)
        if name.endswith('.py'):compile((PKG/name).read_bytes(),name,'exec')
    for path,expected in ((MOTOR/'server.py',EXPECTED_8093),(MOTOR/'op5_temp_diag.py',EXPECTED_TEMP),
                          (FUSION/'fusion_core.py',EXPECTED_FUSION),(FUSION/'service.py',EXPECTED_SERVICE)):
        if not path.is_file() or sha(path)!=expected:
            raise RuntimeError('现场版本不匹配，拒绝对控制系统安装: '+str(path))
    cfg=json.loads((FUSION/'config.json').read_text('utf-8'))
    if cfg.get('radar_mount')!='chassis' or not isinstance(cfg.get('camera_hfov_deg'),(int,float)) or not isinstance(cfg.get('radar_zero_deg'),(int,float)):
        raise RuntimeError('8100未完成底盘雷达标定')
    a=fetch(8093,'/api/auto/status')
    c=fetch(8093,'/api/center/status')
    h=fetch(8093,'/api/hold/status')
    v3=fetch(8100,'/api/health')
    if v3.get('version')!='V3_1_DYNAMIC_ASSOC_V1':
        raise RuntimeError('8100必须先升级为V3.1动态关联版本')
    if a.get('active') or c.get('active') or h.get('state')=='busy':
        raise RuntimeError('跟踪、回中或位置保持恢复未停止')
    fusion=fetch(8100,'/api/fusion')
    if fusion.get('version')!='V3_1_DYNAMIC_ASSOC_V1':
        raise RuntimeError('8100融合接口版本不匹配')
    if UNIT.exists() or UNIT.is_symlink() or DST.exists() or DST.is_symlink():
        if (UNIT.is_file() and DST.is_dir() and
            'OP5 V3 camera gimbal fusion supervision bridge' in UNIT.read_text('utf-8') and
            all((DST/n).is_file() and sha(DST/n)==sha(PKG/n) for n in SOURCES)):
            print('ALREADY_INSTALLED: 8101阶段版本已安装，未写入任何文件')
            return 'ALREADY_INSTALLED'
        raise RuntimeError('8101存在未知的文件或服务，拒绝覆盖')
    with socket.socket() as s:
        try:s.bind(('127.0.0.1',8101))
        except OSError:raise RuntimeError('8101端口被其他服务占用')
    print('CHECK PASS: 8093 V2、双轴温度、8100 V3.1源码与接口校验通过')
    print('当前跟踪和回中均已停止；本条命令没有写入文件')
    return 'READY_TO_APPLY'


def apply():
    if os.geteuid()!=0:raise RuntimeError('请使用sudo执行安装')
    state=check()
    if state!='READY_TO_APPLY':raise RuntimeError('当前不是可安装状态：'+str(state))
    user=pwd.getpwnam('orangepi')
    DST.mkdir(mode=0o750)
    try:
        for name in SOURCES:
            shutil.copy2(PKG/name,DST/name)
            os.chown(DST/name,user.pw_uid,user.pw_gid)
        os.chown(DST,user.pw_uid,user.pw_gid)
        pwdtext=secrets.token_urlsafe(14)
        salt=secrets.token_hex(16)
        data={'salt':salt,'digest':hashlib.sha256((salt+pwdtext).encode()).hexdigest()}
        (DST/'credentials.json').write_text(json.dumps(data),encoding='utf-8')
        os.chown(DST/'credentials.json',user.pw_uid,user.pw_gid)
        os.chmod(DST/'credentials.json',0o600)
        content='''[Unit]
Description=OP5 V3 camera gimbal fusion supervision bridge (human-armed)
After=network-online.target op5-gimbal-web-8093.service op5-v3-fusion-8100.service
Wants=network-online.target

[Service]
Type=simple
User=orangepi
ExecStart=/usr/bin/python3 -u /home/orangepi/op5_v3_fusion_bridge/bridge.py
Restart=on-failure
RestartSec=2
NoNewPrivileges=true
PrivateTmp=true

[Install]
WantedBy=multi-user.target
'''
        UNIT.write_text(content,encoding='utf-8')
        os.chmod(UNIT,0o644)
        print('INSTALL SUCCESS: 8101独立控制桥已安装，尚未启动')
        print('操作员PIN（请妥善保存，只显示本次）:',pwdtext)
        print('启动: sudo systemctl daemon-reload && sudo systemctl enable --now op5-v3-fusion-gimbal-8101')
    except Exception:
        if UNIT.exists():UNIT.unlink()
        if DST.exists():shutil.rmtree(DST)
        raise


def rollback():
    if os.geteuid()!=0:raise RuntimeError('请使用sudo执行卸载')
    if not UNIT.is_file() or 'OP5 V3 camera gimbal fusion supervision bridge' not in UNIT.read_text():
        raise RuntimeError('服务不是受控8101版本，拒绝回退')
    try:
        req=urllib.request.Request('http://127.0.0.1:8101/api/stop',data=b'{}',headers={'Content-Type':'application/json'})
        with urllib.request.urlopen(req,timeout=2) as r:json.load(r)
    except OSError:
        pass
    subprocess.run(['systemctl','disable','--now','op5-v3-fusion-gimbal-8101'],check=True)
    a=fetch(8093,'/api/auto/status')
    if a.get('active'):
        raise RuntimeError('8093仍在跟踪，未删除文件；请现场检查并人工停止')
    UNIT.unlink();subprocess.run(['systemctl','daemon-reload'],check=True)
    # Save the bridge including credential hash for audit, not active anymore.
    backup=DST.with_name(DST.name+'.removed_'+secrets.token_hex(4))
    DST.rename(backup)
    print('ROLLBACK SUCCESS: 已停止8101并移除服务，保留审计备份',backup)
    print('8093 / 8100 / 8092 / 8081 未被更改')

if __name__=='__main__':
    p=argparse.ArgumentParser()
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--check',action='store_true')
    group.add_argument('--apply',action='store_true')
    group.add_argument('--rollback',action='store_true')
    args=p.parse_args()
    try:
        if args.check:check()
        elif args.apply:apply()
        else:rollback()
    except Exception as exc:sys.exit('FAIL: '+str(exc))
