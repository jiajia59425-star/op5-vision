#!/usr/bin/env python3
# coding: utf-8
"""V3 final toggle HUD integration - only patches /home/orangepi/dual_overlay_switch.py.
--check dry-run; --apply atomic backup/patched restart and automatic rollback on failure;
--rollback restores exact bytes if source still matches installed hash.
"""
import argparse, hashlib, json, os, pathlib, stat, subprocess, tempfile, time, urllib.request

ROOT=pathlib.Path(__file__).resolve().parent
TARGET=pathlib.Path('/home/orangepi/dual_overlay_switch.py')
MANIFEST=pathlib.Path('/var/lib/op5-8090-v3-final-toggle-v1/manifest.json')
SERVICE='dual-vision.service'
BRIDGE='op5-v3-fusion-gimbal-8101.service'
MARK='OP5_8090_FUSION_TOGGLE_FINAL_V1'
ENTRY='if __name__ == "__main__":'

def sha(blob):return hashlib.sha256(blob).hexdigest()

def call(*args):
    r=subprocess.run(args,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=30)
    if r.returncode:raise RuntimeError(' '.join(args)+' -> '+r.stdout[-1500:])
    return r.stdout.strip()

def probe(port,path,timeout=3):
    with urllib.request.urlopen('http://127.0.0.1:%s%s'%(port,path),timeout=timeout) as resp:
        return json.load(resp)

def verify_idle():
    a=probe(8093,'/api/auto/status')
    if a.get('active') is not False:raise RuntimeError('8093正在跟踪，请先停止')
    c=probe(8093,'/api/center/status')
    if c.get('active') is not False:raise RuntimeError('8093自动回中状态未确认关闭')
    if active(BRIDGE):
        v=probe(8101,'/api/status')
        if v.get('armed') or v.get('stop_pending'):
            raise RuntimeError('8101正监护运行或待确认停机，拒绝替换8090')

def payload():
    source=TARGET.read_bytes();s=source.decode('utf-8')
    if MARK in s:raise RuntimeError('8090已安装该版本，拒绝重复安装')
    if 'OP5_V3_TOUCH_8090_V1' in s:raise RuntimeError('检测到旧版V3单独触摸面板，请先回退旧版，以免重复绑定')
    must=['class Handler(', 'def do_GET(', 'def do_POST(', 'HTML =', 'hud-follow-slot', 'OP5_HUD_V11_REAL_INTEGRATION_JS', 'OP5_AUTO_CENTER_V2_8090_ADDITION',ENTRY]
    missing=[x for x in must if x not in s]
    if missing:raise RuntimeError('8090现场源码结构不匹配，未修改：'+str(missing))
    if s.count(ENTRY)!=1:raise RuntimeError('8090入口数量异常')
    addon=(ROOT/'runtime_inject.py').read_text('utf-8')
    for asset in ('fusion_toggle.css','fusion_toggle.js'):
        if not (ROOT/asset).exists():raise RuntimeError('缺少'+asset)
    ext='\n# '+MARK+' : same-origin HUD touchscreen integration\n'
    ext+='\n'+addon+'\n'
    patched=s.replace(ENTRY,ext+'\n'+ENTRY,1).encode('utf-8')
    compile(patched.decode('utf-8'),str(TARGET),'exec')
    return source,patched

def atomic(blob):
    old=TARGET.stat();fd,name=tempfile.mkstemp(prefix='.v3-touch-',dir=str(TARGET.parent))
    try:
        with os.fdopen(fd,'wb') as f:
            f.write(blob);f.flush();os.fsync(f.fileno())
        os.chmod(name,stat.S_IMODE(old.st_mode));os.chown(name,old.st_uid,old.st_gid)
        os.replace(name,TARGET)
    finally:
        if os.path.exists(name):os.unlink(name)

def verify_running():
    last=None
    for _ in range(28):
        try:
            if call('systemctl','is-active',SERVICE)!='active':raise RuntimeError('8090 inactive')
            with urllib.request.urlopen('http://127.0.0.1:8090/',timeout=3) as r:
                page=r.read(1000000).decode('utf-8','replace')
            if MARK+'_JS' not in page or 'hud-follow-slot' not in page:
                raise RuntimeError('8090实际页面缺少V3或原HUD')
            data=probe(8090,'/op5-v3-toggle/health')
            if data.get('version')!='V3_STAGE1_FUSED_CAMERA_GUARD':
                raise RuntimeError('8101版本错误 '+str(data))
            data=probe(8090,'/op5-v3-toggle/preflight')
            if 'ready' not in data:raise RuntimeError('无法读取融合许可')
            return
        except Exception as e:
            last=str(e);time.sleep(.35)
    raise RuntimeError('真实HTTP检测失败：'+str(last))

def check():
    print('=== 8090 V3融合开关最终界面安装检查 ===')
    if not TARGET.exists():raise RuntimeError('未发现'+str(TARGET))
    print('8090原始SHA256',sha(TARGET.read_bytes()))
    if MANIFEST.exists():
        print('ALREADY_INSTALLED：发现安装记录',MANIFEST)
        if MARK not in TARGET.read_text('utf8'):raise RuntimeError('存在安装记录但源码标记丢失')
        return
    before,after=payload()
    print('8090结构匹配；补丁编译通过；升级后SHA256',sha(after))
    try:
        call('systemctl','cat',BRIDGE)
        print('8101 systemd 单元存在')
    except Exception:
        raise RuntimeError('8101控制桥未安装，先安装已有8101')
    if call('systemctl','is-active',SERVICE)!='active':raise RuntimeError('8090服务不是active')
    verify_idle()
    if active(BRIDGE):
        h=probe(8101,'/api/health')
        print('8101当前可连接：',h.get('version'))
        if h.get('version')!='V3_STAGE1_FUSED_CAMERA_GUARD':raise RuntimeError('8101版本不匹配')
    else:
        print('提示：8101目前未运行；正式安装时将 enable --now，且不自动开始跟踪')
    print('CHECK PASS: READY_TO_APPLY；检查模式未修改任何文件或服务')

def enabled(srv):
    r=subprocess.run(['systemctl','is-enabled',srv],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    return r.stdout.strip() in ('enabled','enabled-runtime')

def active(srv):
    r=subprocess.run(['systemctl','is-active',srv],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    return r.stdout.strip()=='active'

def apply():
    if MANIFEST.exists():raise RuntimeError('已有安装记录，拒绝重复应用')
    check()
    before,after=payload()
    MANIFEST.parent.mkdir(parents=True,exist_ok=True)
    backup=MANIFEST.parent/('backup_'+time.strftime('%Y%m%d_%H%M%S')+'.py')
    if backup.exists():raise RuntimeError('备份文件名冲突')
    with backup.open('xb') as f:f.write(before)
    os.chmod(backup,0o600)
    if sha(backup.read_bytes())!=sha(before):raise RuntimeError('备份校验失败')
    was_enabled=enabled(BRIDGE);was_active=active(BRIDGE);hud_was_enabled=enabled(SERVICE)
    try:
        atomic(after)
        call('systemctl','restart',SERVICE)
        call('systemctl','enable',SERVICE)
        call('systemctl','enable','--now',BRIDGE)
        verify_running()
        info={'original_sha256':sha(before),'patched_sha256':sha(after),'backup':str(backup),
              'installed':time.strftime('%Y-%m-%d %H:%M:%S'),
              'bridge_was_enabled':was_enabled,'bridge_was_active':was_active,'hud_was_enabled':hud_was_enabled}
        MANIFEST.write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8')
        os.chmod(MANIFEST,0o600)
        print('INSTALL SUCCESS: 已在原跟踪和回中滑块下方增加 V3雷达融合开关')
        print('8101服务已开机自启，但云台不会自动启动跟踪')
        print('浏览器地址: http://127.0.0.1:8090/ （香橙派触摸屏本机）')
        print('源文件备份:',backup)
    except Exception:
        print('安装失败：正在自动回滚8090原始字节和8101启动设置',flush=True)
        try:atomic(before);call('systemctl','restart',SERVICE)
        except Exception as exc:print('!!! 8090回滚异常：',exc,flush=True)
        try:
            if not hud_was_enabled:call('systemctl','disable',SERVICE)
            if not was_enabled:call('systemctl','disable',BRIDGE)
            if not was_active:call('systemctl','stop',BRIDGE)
        except Exception as exc:print('!!! 8101状态恢复异常：',exc,flush=True)
        raise

def rollback():
    if not MANIFEST.exists():raise RuntimeError('没有找到安装记录')
    verify_idle()
    data=json.loads(MANIFEST.read_text('utf8'))
    original=pathlib.Path(data['backup']).read_bytes()
    if sha(original)!=data['original_sha256']:raise RuntimeError('备份校验失败，拒绝覆盖')
    if sha(TARGET.read_bytes())!=data['patched_sha256']:
        raise RuntimeError('8090升级后又发生变化，拒绝直接回滚，防止覆盖后续修改')
    atomic(original)
    call('systemctl','restart',SERVICE)
    if not data.get('hud_was_enabled',True):call('systemctl','disable',SERVICE)
    if not data['bridge_was_enabled']:call('systemctl','disable',BRIDGE)
    if not data['bridge_was_active']:call('systemctl','stop',BRIDGE)
    MANIFEST.unlink()
    print('ROLLBACK VERIFIED: 8090原始源码已恢复；8101启用状态已按原记录恢复')

if __name__=='__main__':
    ap=argparse.ArgumentParser()
    g=ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--check',action='store_true')
    g.add_argument('--apply',action='store_true')
    g.add_argument('--rollback',action='store_true')
    a=ap.parse_args()
    if os.geteuid()!=0:raise SystemExit('请使用sudo运行')
    try:
        if a.check:check()
        elif a.apply:apply()
        else:rollback()
    except Exception as ex:
        print('ERROR:',ex,flush=True)
        raise SystemExit(2)
