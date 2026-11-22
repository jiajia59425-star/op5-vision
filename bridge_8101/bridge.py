#!/usr/bin/env python3
"""Human-initiated radar-gated camera gimbal AUTO START / PULSE / STOP bridge.

No direct ST3215 bus access. No automatic restart. Local SSH forwarding only.
"""
import argparse
import hashlib
import hmac
import json
import logging
import pathlib
import secrets
import signal
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from gate import decide

ROOT=pathlib.Path(__file__).resolve().parent
CREDENTIALS=ROOT/'credentials.json'
LOCK=threading.RLock()
STATUS={'armed':False,'session':None,'target':None,'reason':'等待人工授权',
        'last_gate_ok':False,'last_gate_reason':'尚未采样','moves':None,
        'start_at':None,'last_pulse_at':None,'operator_ping':None,'stop_pending':False}
AUTH={}  # one-use operator sessions with 20-min expiry; cleared after service restart
BASE='http://127.0.0.1:'
LOG=logging.getLogger('v3-bridge')


def request_json(port,path,payload=None,token=None,timeout=0.9):
    headers={'Accept':'application/json','Cache-Control':'no-store'}
    method='GET'
    data=None
    if payload is not None:
        method='POST';headers['Content-Type']='application/json'
        data=json.dumps(payload,separators=(',',':')).encode('utf-8')
    if token:headers['X-Gimbal-Session']=token
    req=urllib.request.Request(BASE+str(port)+path,data=data,headers=headers,method=method)
    with urllib.request.urlopen(req,timeout=timeout) as resp:
        raw=resp.read(250000)
        if resp.status!=200:raise RuntimeError('HTTP '+str(resp.status))
    out=json.loads(raw)
    if not isinstance(out,dict):raise RuntimeError('Invalid JSON response')
    return out


def sample(expected=None,check_auto=False):
    fusion=request_json(8100,'/api/fusion')
    vision=request_json(8081,'/state')
    auto=request_json(8093,'/api/auto/status') if check_auto else None
    return (*decide(fusion,vision,auto,expected),fusion,auto)


def safe_stop(reason):
    with LOCK:
        was_armed=STATUS['armed'] or STATUS['stop_pending']
        STATUS.update(armed=False,session=None,target=None,operator_ping=None,reason=reason,
                      stop_pending=was_armed)
    if was_armed:
        try:
            request_json(8093,'/api/auto/stop',{})
            with LOCK:STATUS['stop_pending']=False
            LOG.warning('V3 fused camera follow OFF: %s',reason)
        except Exception as exc:
            LOG.error('8093 STOP failed; retry needed, hardware disconnect if necessary: %s',exc)


def start_operator():
    # Called only by explicit PIN-authenticated user action; never from monitor thread.
    with LOCK:
        if STATUS['armed'] or STATUS['stop_pending']:
            raise RuntimeError('已有跟踪或停机未确认')
    ok,reason,target,fusion,ignored=sample()
    if not ok:raise RuntimeError(reason)
    current=request_json(8093,'/api/auto/status')
    if current.get('active') is not False:
        raise RuntimeError('8093已有其他跟踪会话，拒绝接管')
    # A user explicitly clicked ARM after PIN authentication. No background unlock.
    token=None
    for arrow in ('up','right','down','left'):
        x=request_json(8093,'/api/unlock-step',{'direction':arrow})
        if x.get('unlocked') and x.get('session'):
            token=x['session']
    if token is None:raise RuntimeError('8093人工会话解锁失败')
    # Last immediate recheck before issuing original V2 start command.
    ok,reason,target2,f2,_=sample(expected=target)
    if not ok or target2!=target:
        raise RuntimeError('启动前融合数据变化：'+reason)
    try:
        result=request_json(8093,'/api/auto/start',{'range_mode':'safe','source':'rgb'},token)
        if (result.get('ok') is not True or (result.get('result') or {}).get('active') is not True):
            raise RuntimeError('8093未确认启动')
        # Post-start keepalive is monitored separately; short initial lease is 6s.
        with LOCK:
            STATUS.update(armed=True,session=token,target=target,start_at=time.monotonic(),
                          last_pulse_at=time.monotonic(),operator_ping=time.monotonic(),
                          reason='人工启动；雷达监护中',stop_pending=False)
        LOG.warning('V3 fused camera follow ARM target=%s',target)
    except Exception:
        # On any ambiguous 8093 outcome, request STOP. Never continue unobserved.
        safe_stop('启动结果不明确，停止')
        try:request_json(8093,'/api/auto/stop',{})
        except Exception:pass
        raise


def monitor_once():
    with LOCK:
        armed=STATUS['armed'];target=STATUS['target'];last=STATUS['last_pulse_at']
        pending=STATUS['stop_pending'];operator_ping=STATUS.get('operator_ping')
    if pending:
        safe_stop('重试确认8093停机');return
    if not armed:
        return
    if operator_ping is None or time.monotonic()-operator_ping>3.0:
        safe_stop('8101操作页面断开或未持续确认，自动停止')
        return
    try:
        ok,reason,got,fusion,auto=sample(expected=target,check_auto=True)
        with LOCK:
            STATUS.update(last_gate_ok=ok,last_gate_reason=reason,
                          moves=auto.get('moves') if isinstance(auto,dict) else None)
        if not ok:
            safe_stop('融合安全门关闭：'+reason);return
        if time.monotonic()-(last or 0)>=0.8:
            result=request_json(8093,'/api/auto/pulse',{},STATUS['session'])
            if result.get('ok') is not True:
                safe_stop('8093续期被拒绝');return
            with LOCK:STATUS['last_pulse_at']=time.monotonic()
    except Exception as exc:
        safe_stop('融合检查或8093通信失败：'+str(exc)[:100])


def monitoring():
    while True:
        began=time.monotonic()
        monitor_once()
        time.sleep(max(0.06,0.22-(time.monotonic()-began)))


def credential_ok(pin):
    if not isinstance(pin,str) or len(pin)>128:return False
    x=json.loads(CREDENTIALS.read_text('utf-8'))
    return hmac.compare_digest(hashlib.sha256((x['salt']+pin).encode()).hexdigest(),x['digest'])


def new_session(pin):
    if not credential_ok(pin):return None
    token=secrets.token_urlsafe(32)
    with LOCK:AUTH[token]=time.monotonic()+1200
    return token


def authorized(token):
    now=time.monotonic()
    with LOCK:
        expired=[t for t,expiry in AUTH.items() if expiry<now]
        for t in expired:AUTH.pop(t,None)
        return bool(token and token in AUTH and AUTH[token]>now)


class Handler(BaseHTTPRequestHandler):
    def reply(self,status,obj):
        body=json.dumps(obj,ensure_ascii=False,allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Content-Length',str(len(body)))
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.end_headers()
        try:self.wfile.write(body)
        except (BrokenPipeError,ConnectionResetError):pass

    def do_GET(self):
        path=self.path.split('?',1)[0]
        if path=='/':
            payload=(ROOT/'index.html').read_bytes()
            self.send_response(200)
            self.send_header('Content-Type','text/html; charset=utf-8')
            self.send_header('Content-Length',str(len(payload)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers();self.wfile.write(payload)
        elif path=='/api/status':
            with LOCK:public={k:v for k,v in STATUS.items() if k!='session'}
            public['version']='V3_STAGE1_FUSED_CAMERA_GUARD';public['port']=8101
            self.reply(200,public)
        elif path=='/api/preflight':
            try:
                ok,reason,target,fusion,auto=sample()
                self.reply(200,{'ready':ok,'reason':reason,'target':target,
                                'distance_m':fusion.get('stable_candidate_m') if ok else None,
                                'association_quality':fusion.get('association_quality'),
                                'camera_only':True,'radar_authorizes_tracking':True})
            except Exception as exc:self.reply(200,{'ready':False,'reason':'读取失败：'+str(exc)[:100]})
        elif path=='/api/health':
            self.reply(200,{'ok':True,'version':'V3_STAGE1_FUSED_CAMERA_GUARD',
                            'control_paths':['/api/auto/start','/api/auto/pulse','/api/auto/stop'],
                            'direct_servo_writes':False,'auto_start_on_boot':False})
        else:self.reply(404,{'error':'Not found'})

    def do_POST(self):
        path=self.path.split('?',1)[0]
        # Never allow a browser on another Origin to send actuator commands.
        origin=self.headers.get('Origin')
        if origin and origin not in ('http://127.0.0.1:8101','http://localhost:8101'):
            return self.reply(403,{'error':'Origin rejected'})
        if path=='/api/stop':
            safe_stop('操作员按下停止');return self.reply(200,{'ok':True})
        try:
            n=int(self.headers.get('Content-Length','0'))
            if n<2 or n>512 or self.headers.get('Content-Type','').split(';')[0].strip()!='application/json':
                return self.reply(400,{'error':'Invalid body'})
            obj=json.loads(self.rfile.read(n))
            if not isinstance(obj,dict):raise ValueError('Body is not JSON object')
            if path=='/api/login':
                token=new_session(obj.get('pin'))
                if token is None:return self.reply(403,{'error':'PIN错误'})
                return self.reply(200,{'ok':True,'token':token})
            if not authorized(self.headers.get('X-Operator-Token')):
                return self.reply(403,{'error':'需要操作员PIN授权'})
            if path=='/api/heartbeat':
                with LOCK:
                    if STATUS['armed']:
                        STATUS['operator_ping']=time.monotonic()
                return self.reply(200,{'ok':True})
            if path=='/api/arm':
                start_operator();return self.reply(200,{'ok':True})
            self.reply(404,{'error':'Not found'})
        except Exception as exc:
            self.reply(400,{'ok':False,'error':str(exc)[:180]})

    def log_message(self,fmt,*args):
        if self.path not in ('/api/preflight','/api/status'):
            LOG.info('%s '+fmt,self.client_address[0],*args)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port',type=int,default=8101)
    args=p.parse_args()
    logging.basicConfig(level=logging.INFO,format='%(asctime)s %(levelname)s %(message)s')
    if not CREDENTIALS.is_file():raise SystemExit('Missing credentials; run installer first')
    threading.Thread(target=monitoring,daemon=True,name='fusion-v3-supervisor').start()
    # Local loopback only. Access from Windows via SSH -L 8101:127.0.0.1:8101.
    def on_exit(signum, frame):
        safe_stop('V3服务停止，撤销跟踪授权')
        raise SystemExit(0)
    signal.signal(signal.SIGTERM,on_exit)
    signal.signal(signal.SIGINT,on_exit)
    ThreadingHTTPServer(('127.0.0.1',args.port),Handler).serve_forever(poll_interval=0.2)

if __name__=='__main__':main()
