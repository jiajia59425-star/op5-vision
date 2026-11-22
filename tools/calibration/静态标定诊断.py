#!/usr/bin/env python3
"""V3 static target angle/staleness tests; GET-only, no actuator commands."""
import json,time,urllib.request,statistics,collections

def get(port,path):
    with urllib.request.urlopen(f'http://127.0.0.1:{port}{path}',timeout=3) as r:
        return json.load(r)

def observe(label):
    rows=[]; reasons=collections.Counter()
    for i in range(24):
        try:
            f=get(8100,'/api/fusion'); p=get(8101,'/api/preflight')
            c=f.get('candidate') or {}
            ages=f.get('sources') or {}
            residual=f.get('angular_residual_deg')
            signed=c.get('angle_error_deg')
            a={k:(ages.get(k) or {}).get('age_ms') for k in ('vision','radar','servo')}
            rows.append({'ready':p.get('ready'),'reason':p.get('reason'),
                'age':f.get('fusion_age_ms'),'residual':residual,'signed':signed,
                'distance':f.get('stable_candidate_m'),'yaw':f.get('yaw_deg'),
                'bearing':f.get('radar_bearing_deg'),'radar_bearing':c.get('radar_bearing_deg'),
                'target':(f.get('target') or {}).get('id'),'ages':a})
            reasons[str(p.get('reason') if not p.get('ready') else 'READY')]+=1
        except Exception as e:print('采样异常:',type(e).__name__,str(e)[:100])
        time.sleep(0.35)
    def stats(field):
        values=[r[field] for r in rows if type(r[field]) in (int,float)]
        return (round(statistics.median(values),2),round(max(values),2)) if values else ('--','--')
    print('\n===== %s (%d条) ====='%(label,len(rows)))
    for k,v in reasons.items():print('状态:',k,'次数:',v)
    for key in ('age','residual','signed','distance','yaw'):
        print(key,'中位/最大:',stats(key))
    for k in ('vision','radar','servo'):
        vals=[r['ages'][k] for r in rows if type(r['ages'][k]) in (int,float)]
        print(k,'年龄中位/最大:',(round(statistics.median(vals),1),round(max(vals),1)) if vals else '无数据')
    print('角度偏差样本:',[(r['bearing'],r['radar_bearing'],r['signed']) for r in rows if type(r['signed']) in (int,float)][:8])
    return rows

print('静态人物测试：关闭自动跟踪，云台底盘固定，雷达开启，目标已在8090选中。')
with open('/dev/tty','r') as tty:
    print('人物在约1.2米，画面中央，按回车采集约9秒：',flush=True)
    tty.readline()
    observe('约1.2米')
    print('人物退到约2.5米，保持画面中央，按回车采集约9秒：',flush=True)
    tty.readline()
    observe('约2.5米')
print('仅GET只读检测，未发送云台或底盘运动指令')
