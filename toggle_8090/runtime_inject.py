# OP5_8090_FUSION_TOGGLE_FINAL_V1 - append before __main__, after all original HTML modifiers
import json as _v3_json
import urllib.request as _v3_req
import urllib.error as _v3_err
import urllib.parse as _v3_parse

_V3_ID='OP5_8090_FUSION_TOGGLE_FINAL_V1'
_V3_ROOT='/op5-v3-toggle/'
_V3_GET=frozenset(('health','preflight','status'))
_V3_POST=frozenset(('login','arm','heartbeat','stop'))

def _v3_modify_html():
    global HTML
    # Keep exact same physical slider listeners. Only route callback and keepalive by selected mode.
    mutations=[
        (
          "  for(const direction of ['up','right','down','left']){",
          "  if(window.OP5FusionMode && window.OP5FusionMode.enabled()){\n"
          "   await window.OP5FusionMode.arm();\n"
          "   owned=true;token='';status.textContent='● V3融合跟踪中 · 目标ID '+v.selected;\n"
          "   return '✓ V3融合跟踪中';\n"
          "  }\n"
          "  for(const direction of ['up','right','down','left']){"
        ),
        (
          "try{await call('auto/stop',{},false);owned=false;",
          "try{if(window.OP5FusionMode && window.OP5FusionMode.isFusedSession()){\n"
          "   try{await window.OP5FusionMode.stop();}\n"
          "   finally{await call('auto/stop',{},false);}\n"
          "  }else await call('auto/stop',{},false);owned=false;"
        ),
        (
          " try{await call('auto/pulse');}",
          " try{if(window.OP5FusionMode && window.OP5FusionMode.isFusedSession())"
          "await window.OP5FusionMode.heartbeat();else await call('auto/pulse');}"
        ),
        (
          "catch(e){owned=false;token='';slider.reset();status.textContent='跟踪续期失败：'+e.message;}",
          "catch(e){if(window.OP5FusionMode && window.OP5FusionMode.isFusedSession())"
          "{try{await window.OP5FusionMode.stop();}catch(_){}}"
          "owned=false;token='';slider.reset();status.textContent='跟踪续期失败：'+e.message;}"
        )
    ]
    for before,after in mutations:
        count=HTML.count(before)
        if count!=1:
            raise RuntimeError('8090 跟踪滑块源不匹配，拒绝安装：%r count=%d'%(before[:55],count))
        HTML=HTML.replace(before,after,1)
    css=(_v3_root/'fusion_toggle.css').read_text('utf-8')
    js=(_v3_root/'fusion_toggle.js').read_text('utf-8')
    if '</script' in js.lower():
        raise RuntimeError('V3 JS contains closing script tag')
    if '<div id="hud-follow-slot"' not in HTML or 'op5-center-v2-slide' not in HTML:
        raise RuntimeError('8090 云台跟踪和自动回中界面标记不存在')
    if HTML.count('</body>')<1:raise RuntimeError('8090 HTML missing </body>')
    additions='<style id="'+_V3_ID+'_CSS">'+css+'</style>'
    additions+='<script id="'+_V3_ID+'_JS">'+js+'</script>'
    HTML=HTML.replace('</body>',additions+'</body>',1)

_v3_root=__import__('pathlib').Path('/home/orangepi/OP5_8090_FUSION_TOGGLE_FINAL_V1')
_v3_modify_html()

def _v3_proxy(self,method):
    path=_v3_parse.urlsplit(self.path).path
    suffix=path[len(_V3_ROOT):] if path.startswith(_V3_ROOT) else ''
    if suffix not in (_V3_GET if method=='GET' else _V3_POST):
        self.send_error(404);return
    data=None
    if method=='POST':
        # Only real same-origin page can issue actuator requests.
        host=self.headers.get('Host','')
        origin=self.headers.get('Origin','')
        referer=self.headers.get('Referer','')
        expected='http://'+host
        if not host or ':' not in host or not (origin==expected or (not origin and referer.startswith(expected+'/'))):
            self.send_error(403,'Same-origin touchscreen required');return
        ctype=self.headers.get('Content-Type','').split(';',1)[0].strip().lower()
        try:size=int(self.headers.get('Content-Length','-1'))
        except ValueError:size=-1
        if ctype!='application/json' or size<2 or size>512:
            self.send_error(400,'JSON required');return
        try:
            obj=_v3_json.loads(self.rfile.read(size))
            if not isinstance(obj,dict):raise ValueError()
            data=_v3_json.dumps(obj,separators=(',',':')).encode('utf-8')
        except (ValueError,UnicodeDecodeError):
            self.send_error(400,'Invalid JSON');return
    headers={'Accept':'application/json','Cache-Control':'no-store'}
    if data is not None:headers['Content-Type']='application/json'
    tok=self.headers.get('X-Operator-Token','')
    if tok and len(tok)<=256:headers['X-Operator-Token']=tok
    req=_v3_req.Request('http://127.0.0.1:8101/api/'+suffix,data=data,headers=headers,method=method)
    try:
        with _v3_req.urlopen(req,timeout=2.5) as r:
            code=r.status;raw=r.read(16000)
    except _v3_err.HTTPError as e:
        code=e.code;raw=e.read(16000)
    except Exception:
        code=503;raw=b'{"ok":false,"error":"8101 unavailable"}'
    try:
        if not isinstance(_v3_json.loads(raw),dict):raise ValueError()
    except Exception:
        code=502;raw=b'{"ok":false,"error":"Invalid upstream JSON"}'
    self.send_response(code)
    self.send_header('Content-Type','application/json; charset=utf-8')
    self.send_header('Cache-Control','no-store, max-age=0')
    self.send_header('X-Content-Type-Options','nosniff')
    self.send_header('Content-Length',str(len(raw)))
    self.end_headers()
    try:self.wfile.write(raw)
    except (OSError,BrokenPipeError,ConnectionResetError):pass

_v3_get_original=Handler.do_GET
_v3_post_original=Handler.do_POST

def _v3_get(self):
    if _v3_parse.urlsplit(self.path).path.startswith(_V3_ROOT):return _v3_proxy(self,'GET')
    return _v3_get_original(self)

def _v3_post(self):
    if _v3_parse.urlsplit(self.path).path.startswith(_V3_ROOT):return _v3_proxy(self,'POST')
    return _v3_post_original(self)

Handler.do_GET=_v3_get
Handler.do_POST=_v3_post
