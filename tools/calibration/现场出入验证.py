#!/usr/bin/env python3
"""Three-phase radar A/B occupancy audit. GET only; never changes settings."""
import sys,time,json,statistics,urllib.request,pathlib
sys.path.insert(0,'/home/orangepi/op5_v3_fusion')
from fusion_core import radar_points,enumerate_clusters,angular_delta

def get(port,path):
    with urllib.request.urlopen('http://127.0.0.1:%d%s'%(port,path),timeout=4) as r:
        return json.load(r)

def check_idle():
    bridge=get(8101,'/api/status')
    servo=get(8093,'/api/auto/status')
    if bridge.get('armed') is not False or bridge.get('stop_pending') is not False or servo.get('active') is not False:
        raise RuntimeError('仍有云台自动跟踪或8101待停机状态，停止本次诊断')

ANCHORS={
 'A_2.9m背景候选':{'angle':349.9,'range':2.917},
 'B_1.8m前景候选':{'angle':358.9,'range':1.815},
}

def median(values):
    return round(statistics.median(values),3) if values else None

def pick(clusters, anchor):
    eligible=[c for c in clusters if
              abs(angular_delta(c['radar_bearing_deg'],anchor['angle']))<=3.5 and
              abs(c['range_m']-anchor['range'])<=0.50]
    return min(eligible,key=lambda c:
        abs(angular_delta(c['radar_bearing_deg'],anchor['angle']))/3.5+
        abs(c['range_m']-anchor['range'])/.50) if eligible else None

def collect(label):
    check_idle()
    rows=[];predictions=[];ids=[];yaws=[];last_pkt=None
    started=time.monotonic()
    while len(rows)<12 and time.monotonic()-started<12:
        r=get(8092,'/api/scan')
        if r.get('online') is not True:raise RuntimeError('8092雷达离线')
        pkt=r.get('packets')
        if type(pkt) is int and pkt!=last_pkt:
            last_pkt=pkt
            clusters=enumerate_clusters(radar_points(r),354.0,12.5)
            rows.append({k:pick(clusters,a) for k,a in ANCHORS.items()})
            servo=get(8093,'/api/status')
            yaw=(servo.get('yaw') or {}).get('angle')
            if isinstance(yaw,(int,float)): yaws.append(yaw)
            if label!='人物离开':
                f=get(8100,'/api/fusion')
                b=f.get('radar_bearing_deg')
                t=f.get('target') or {}
                if isinstance(b,(int,float)) and t.get('class_id')==0 and type(t.get('id')) is int:
                    predictions.append(b);ids.append(t['id'])
        time.sleep(.20)
    summary={}
    for k in ANCHORS:
        matches=[row[k] for row in rows if row[k]]
        summary[k]={
          '出现帧数':len(matches),'总帧数':len(rows),
          '方位中位数':median([m['radar_bearing_deg'] for m in matches]),
          '距离中位数':median([m['range_m'] for m in matches]),
          '点数中位数':median([m['points'] for m in matches]),
        }
    summary['视觉预测方位中位数']=median(predictions)
    summary['云台Yaw中位数']=median(yaws)
    summary['视觉ID']=(ids[-1] if ids else None)
    print('\n===== '+label+' =====')
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    return summary

print('V3前景/背景验证：不读取8100 candidate，不需要跟踪启动；不会写入配置或发送任何控制命令。')
print('必须保持雷达、底盘与云台位置不变；离场时人物应走出345°至365°雷达扇区。')
with open('/dev/tty','r') as tty:
    print('\n① 人物站在刚才约1.8米雷达回波的位置，静止后按回车',flush=True)
    tty.readline();present=collect('人物在场')
    print('\n② 人物完全离开前方雷达扫描扇区，静止后按回车',flush=True)
    tty.readline();absent=collect('人物离开')
    print('\n③ 人物返回相同位置，静止后按回车',flush=True)
    tty.readline();returned=collect('人物返回')

A='A_2.9m背景候选';B='B_1.8m前景候选'
pb=present[B]['出现帧数'];ab=absent[B]['出现帧数'];rb=returned[B]['出现帧数']
pa=present[A]['出现帧数'];aa=absent[A]['出现帧数'];ra=returned[A]['出现帧数']
yaws=[x['云台Yaw中位数'] for x in (present,absent,returned)]
stationary=all(isinstance(v,(int,float)) for v in yaws) and max(yaws)-min(yaws)<=1.0
foreground=(pb>=9 and ab<=2 and rb>=9 and pa>=7 and aa>=7 and ra>=7 and stationary)
print('\n===== 独立身份验证结论 =====')
print('B存在/消失/返回:',pb,ab,rb,'A背景留存:',pa,aa,ra,'Yaw静止:',stationary)
if foreground:
    print('独立回波证据：B点簇高度符合人物出入变化；A更像静态背景。')
    angles=[x[B]['方位中位数'] for x in (present,returned)]
    predictions=[x['视觉预测方位中位数'] for x in (present,returned)]
    if all(isinstance(v,(int,float)) for v in angles+predictions):
        residual=[angular_delta(a,b) for a,b in zip(angles,predictions)]
        print('人物方位残差(在场/返回):',*[round(v,3) for v in residual])
        if abs(residual[0]-residual[1])<=1.5:
            correction=statistics.median(residual)
            cfg=json.loads(pathlib.Path('/home/orangepi/op5_v3_fusion/config.json').read_text('utf-8'))
            zero=cfg.get('radar_zero_deg')
            if isinstance(zero,(int,float)):
                print('仅供下一阶段人工确认的雷达零位建议:',round((zero+correction)%360,2),'度')
        else:
            print('在场与返回角度残差不一致，不提供零位调整建议')
else:
    print('证据不足：不能判定B属于人物，保留原标定并禁止修改融合安全门限。')
print('只读测试完成。')
