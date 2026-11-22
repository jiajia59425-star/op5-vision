#!/usr/bin/env python3
"""Inspect read-only local endpoints, print compact sanitized format."""
import json,urllib.request
for name,url in [('视觉8081','http://127.0.0.1:8081/state'),
                 ('雷达8092','http://127.0.0.1:8092/api/scan'),
                 ('云台8093','http://127.0.0.1:8093/api/status'),
                 ('V3融合8100','http://127.0.0.1:8100/api/fusion')]:
    try:
        with urllib.request.urlopen(url,timeout=3) as r:d=json.loads(r.read(750000))
        if '8081' in name:
            d={k:d.get(k) for k in ('live','found','selected','selected_class','width','height','seq')}
        elif '8092' in name:
            points=d.get('points') or []
            d={'online':d.get('online'),'error':d.get('error'),'packets':d.get('packets'),
               'points_count':len(points),'first_point':points[0] if points else None}
        elif '8093' in name:
            d={k:{t:d.get(k,{}).get(t) for t in ('angle','voltage','temp','online')} for k in ('yaw','pitch')}
        else:
            d={k:d.get(k) for k in ('status','calibrated','radar_online','camera_online','yaw_deg',
                                   'radar_bearing_deg','candidate','stable_candidate_m','fusion_age_ms')}
        print(name, json.dumps(d,ensure_ascii=False))
    except Exception as e:print(name,'ERROR:',e)
