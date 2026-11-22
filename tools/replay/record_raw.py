#!/usr/bin/env python3
"""Read-only 20-s raw-scan snapshot logger for reproducible V3 vs V3.1 comparison.
No video frame, no hardware commands, no POST. Records detection metadata.
"""
import argparse
import datetime
import json
from pathlib import Path
import time
import urllib.request
URLS={'vision':'http://127.0.0.1:8081/state',
      'radar':'http://127.0.0.1:8092/api/scan',
      'servo':'http://127.0.0.1:8093/api/status'}

def request(url):
    req=urllib.request.Request(url,headers={'Accept':'application/json','Cache-Control':'no-cache'})
    with urllib.request.urlopen(req,timeout=2) as r:
        data=json.loads(r.read(850000))
    if not isinstance(data,dict):raise ValueError('Invalid JSON')
    return data

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--seconds',type=float,default=20.0)
    p.add_argument('--interval',type=float,default=.40)
    p.add_argument('--out',default=None)
    args=p.parse_args()
    if not 1<=args.seconds<=120 or not .2<=args.interval<=5:
        p.error('seconds must be 1..120 and interval .2..5 seconds')
    output=Path(args.out) if args.out else Path.cwd()/('V31_CAPTURE_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.jsonl')
    if output.exists():p.error('Output already exists: '+str(output))
    print('仅GET读取8081/8092/8093；含视觉目标位置与雷达点云，无RGB图像；输出:',output,flush=True)
    stopped=time.monotonic()+args.seconds
    good=bad=0
    with output.open('x',encoding='utf-8') as f:
        while time.monotonic()<stopped:
            started=time.monotonic()
            try:
                samples={}
                for name,url in URLS.items():
                    val=request(url)
                    ts=time.monotonic()
                    samples[name]=[val,ts]
                now=time.monotonic()
                row={'now':now,'wall_time':datetime.datetime.now().isoformat(), 'samples':samples}
                f.write(json.dumps(row,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')
                good+=1
            except Exception as exc:
                bad+=1
                print('取样失败:',str(exc)[:100],flush=True)
            time.sleep(max(.0,args.interval-(time.monotonic()-started)))
    print('采集完成 | 成功',good,'| 失败',bad,'| 文件',output)

if __name__=='__main__':main()
