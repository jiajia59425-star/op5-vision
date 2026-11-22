#!/usr/bin/env python3
"""COIN-D6 + RKNN YOLO + ST3215 pose read-only dynamic association V3.1.
GET only; never imports pyserial, sends POST, or controls actuators.
"""
import argparse
import json
import logging
import pathlib
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from fusion_core import FusionEngine

ROOT=pathlib.Path(__file__).resolve().parent
CONFIG=ROOT/'config.json'
SRC={
    'vision': ('http://127.0.0.1:8081/state',0.18),
    'radar':  ('http://127.0.0.1:8092/api/scan',0.25),
    'servo':  ('http://127.0.0.1:8093/api/status',0.35),
}
LOCK=threading.RLock()
DATA={name:(None, None) for name in SRC}
ERRORS={name:'等待读取' for name in SRC}
ENGINE=FusionEngine()
ENGINE_LOCK=threading.RLock()


def fetch_json(url):
    req=urllib.request.Request(url, headers={'Accept':'application/json','Cache-Control':'no-store'})
    with urllib.request.urlopen(req,timeout=0.8) as r:
        if r.status != 200:raise ValueError('HTTP %s'%r.status)
        raw=r.read(750000)
    data=json.loads(raw)
    if not isinstance(data,dict):raise ValueError('invalid JSON root')
    return data


def worker(name):
    url,interval=SRC[name]
    last_packets=None
    last_radar_new_at=None
    while True:
        started=time.monotonic()
        try:
            obj=fetch_json(url)
            now=time.monotonic()
            with LOCK:
                if name=='radar':
                    pack=obj.get('packets')
                    if obj.get('online') is True and type(pack) is int:
                        if pack!=last_packets:
                            last_radar_new_at=now
                            last_packets=pack
                    else:
                        last_radar_new_at=None
                    timestamp=last_radar_new_at
                else:timestamp=now
                DATA[name]=(obj,timestamp)
                ERRORS[name]=''
        except Exception as exc:
            # Never re-use a last-good frame after any fetch failure.
            with LOCK:
                DATA[name]=(None,None)
                ERRORS[name]='%s: %s'%(type(exc).__name__,str(exc)[:130])
        time.sleep(max(0.02, interval-(time.monotonic()-started)))


def config_load():
    if not CONFIG.is_file(): return {'camera_hfov_deg':None,'radar_zero_deg':None,
                                   'radar_mount':None,'yaw_to_radar_sign':1,'camera_yaw_offset_deg':0.0}
    d=json.loads(CONFIG.read_text('utf-8'))
    if not isinstance(d,dict):raise ValueError('config must be dict')
    return d


def fuse_output():
    now=time.monotonic()
    with LOCK:
        visual,radar,servo=(DATA[k] for k in ('vision','radar','servo'))
        errs=dict(ERRORS)
    try:
        c=config_load()
        with ENGINE_LOCK:
            result=ENGINE.fuse(visual,radar,servo,c,now)
    except Exception as exc:
        logging.exception('fusion failure')
        result={'version':'V3_1_DYNAMIC_ASSOC_V1','status':'INTERNAL_ERROR','error':str(exc)[:160],
                'mode':'READ_ONLY_SHADOW','candidate':None,'stable_candidate_m':None}
    result['sources']={k:{'ok':(visual if k=='vision' else radar if k=='radar' else servo)[0] is not None,
                           'error':errs[k]} for k in SRC}
    result['config']={'camera_hfov_deg':c.get('camera_hfov_deg'),
                      'radar_zero_deg':c.get('radar_zero_deg'),
                      'radar_mount':c.get('radar_mount'),
                      'yaw_to_radar_sign':c.get('yaw_to_radar_sign')} if 'c' in locals() else {}
    return result


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path=self.path.split('?',1)[0]
        if path in ('/','/index.html'):
            content=(ROOT/'dashboard.html').read_bytes()
            typ='text/html; charset=utf-8'
        elif path=='/api/fusion':
            content=json.dumps(fuse_output(),ensure_ascii=False,allow_nan=False).encode('utf-8')
            typ='application/json; charset=utf-8'
        elif path=='/api/health':
            content=json.dumps({'ok':True,'version':'V3_1_DYNAMIC_ASSOC_V1',
                                'mode':'READ_ONLY_SHADOW', 'writes_to_hardware':False}).encode()
            typ='application/json'
        else:
            self.send_error(404);return
        self.send_response(200)
        self.send_header('Content-Type',typ)
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.send_header('Content-Length',str(len(content)))
        self.end_headers()
        try:self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError):pass

    def do_POST(self):
        self.send_error(405,'Read-only: POST disabled')

    def log_message(self, format,*args):
        if '/api/fusion' not in str(args):logging.info(format,*args)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',type=int,default=8100)
    parser.add_argument('--listen',default='0.0.0.0')
    args=parser.parse_args()
    logging.basicConfig(level=logging.INFO,format='%(asctime)s %(levelname)s %(message)s')
    for name in SRC:
        threading.Thread(target=worker,args=(name,),name='v3-readonly-'+name,daemon=True).start()
    srv=ThreadingHTTPServer((args.listen,args.port),Handler)
    logging.info('V3 READ-ONLY fusion: %s:%s',args.listen,args.port)
    srv.serve_forever(poll_interval=0.5)
