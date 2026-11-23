"""V3 read-only candidate fusion. No motor or radar commands."""
import math
import statistics
import time


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def wrap360(a):
    return a % 360.0


def angular_delta(a, b):
    return (a-b+180.0) % 360.0-180.0


def person_allowed(class_id, class_name, config):
    """只允许 person。名字非空时以名字为准，避免类别号是 0 但标签是植物。"""
    if not isinstance(config, dict):
        config = {}
    raw_ids = config.get('person_class_ids', [0])
    raw_names = config.get('person_class_names', ['person'])
    ids = set()
    if isinstance(raw_ids, (list, tuple)):
        for item in raw_ids:
            if type(item) is int:
                ids.add(item)
    if not ids:
        ids.add(0)
    names = {'person', '行人'}
    if isinstance(raw_names, (list, tuple)):
        for item in raw_names:
            if isinstance(item, str) and item.strip():
                names.add(item.strip().lower())
    if isinstance(class_name, str) and class_name.strip():
        return class_name.strip().lower() in names
    return type(class_id) is int and class_id in ids


def camera_bearing_deg(cx, width, hfov_deg):
    """Pinhole horizontal ray angle: avoids assuming equidistant projection."""
    if not (finite_number(cx) and finite_number(width) and 0 <= cx <= width and width >= 32):
        raise ValueError('camera coordinate invalid')
    if not (finite_number(hfov_deg) and 10 <= hfov_deg <= 150):
        raise ValueError('horizontal FOV not calibrated')
    nx=(cx-width/2)/(width/2)
    return math.degrees(math.atan(nx*math.tan(math.radians(hfov_deg/2))))


def validated_selection(v, now=None):
    if now is None: now=time.monotonic()
    if not isinstance(v, dict) or v.get('live') is not True or v.get('found') is not True:
        return None, 'VISION_NOT_READY'
    updated=v.get('updated')
    if not finite_number(updated) or not 0<=now-updated<=0.85:
        return None, 'VISION_FRAME_STALE'
    ident,cls=v.get('selected'),v.get('selected_class')
    if type(ident) is not int or type(cls) is not int:
        return None,'NO_SELECTED_TARGET'
    w,h=v.get('width'),v.get('height')
    if not all(finite_number(a) for a in (w,h)) or w<32 or h<32:
        return None,'VISION_SIZE_INVALID'
    tracks=v.get('tracks')
    if not isinstance(tracks,list):return None,'VISION_TRACKS_INVALID'
    for t in tracks:
        if not isinstance(t,dict) or t.get('id')!=ident or t.get('class_id')!=cls:continue
        b=t.get('box')
        if not isinstance(b,list) or len(b)!=4 or not all(finite_number(x) for x in b):
            return None,'VISION_BOX_INVALID'
        x1,y1,x2,y2=b
        if not (0<=x1<x2<=w and 0<=y1<y2<=h):return None,'VISION_BOX_INVALID'
        label = t.get('class_name') or t.get('label') or t.get('name') or v.get('selected_class_name') or v.get('class_name') or ''
        if not isinstance(label, str):
            label = ''
        return {'id':ident,'class_id':cls,'class_name':label,'box':list(b),'width':w,'height':h,
                'cx':(x1+x2)/2,'cy':(y1+y2)/2,'age_ms':round((now-updated)*1000,1)},'OK'
    return None,'SELECTED_BOX_NOT_FOUND'


def radar_points(obj):
    if not isinstance(obj,dict) or obj.get('online') is not True:return []
    pts=[]
    for p in (obj.get('points') or []):
        if (not isinstance(p,(tuple,list)) or len(p)<3 or
            not all(finite_number(x) for x in p[:3])):continue
        a,d,s=p[:3]
        if 0<=a<360 and 0.08<=d<=12 and 0<=s<=255:
            pts.append((float(a),float(d),float(s)))
    return pts[:3000]


def enumerate_clusters(points, bearing, half_angle_deg):
    """Make range-continuous clusters in a circular, bearing-centered axis.

    Sorting signed angular offsets (rather than raw 0..360 degrees) avoids
    splitting a target that crosses the 359.9/0.1-degree seam.
    """
    inside=[]
    for a,d,s in points:
        delta=angular_delta(a,bearing)
        if abs(delta)<=half_angle_deg:
            inside.append((delta,a,d,s))
    inside.sort(key=lambda x:x[0])
    groups=[]; current=[]
    for p in inside:
        if current and (p[0]-current[-1][0]>2.7 or abs(p[2]-current[-1][2])>0.40):
            if len(current)>=3:groups.append(current)
            current=[]
        current.append(p)
    if len(current)>=3:groups.append(current)
    result=[]
    for g in groups:
        distances=[p[2] for p in g]
        offsets=[p[0] for p in g]
        mid=statistics.median(offsets)
        candidate={
            'range_m':round(statistics.median(distances),3),
            'radar_bearing_deg':round(wrap360(bearing+mid),2),
            'angle_error_deg':round(mid,2),
            'points':len(g),
            'strength':round(statistics.median(p[3] for p in g),1),
            'range_spread_m':round(max(distances)-min(distances),3),
            'bearing_span_deg':round(max(offsets)-min(offsets),2),
        }
        result.append(candidate)
    return result


def radar_candidates(points,bearing,half_angle_deg):
    """Backward-compatible single-frame candidate for offline comparisons."""
    cs=enumerate_clusters(points,bearing,half_angle_deg)
    return min(cs,key=lambda c:(abs(c['angle_error_deg']),c['range_m'])) if cs else None


class FusionEngine:
    """Strict read-only data association. Stable != identity verified.

    A candidate can be exposed for inspection, but accepted distance requires:
    a small angular residual, independent advancing radar packet counters,
    continuous range trend, >=3 confirmations, and no competing equal-score
    cluster. No distance is retained in the output after lost/stale data.
    """
    VERSION='V3_1_DYNAMIC_ASSOC_V1'
    MAX_ANGLE_ERROR_DEG=7.0
    MAX_RADIAL_RATE_M_S=2.2  # only a continuity gate, not a physical speed claim
    MIN_CONFIRMATIONS=3

    def __init__(self):
        self.last_key=None
        self.last_packet=None
        self.history=[]
        self.last_yaw_sample=None
        self.yaw_rate_deg_s=None
        self.last_target_center=None
        self.last_selected=None

    def _reset_track(self,key):
        self.last_key=key
        self.last_packet=None
        self.history=[]
        self.last_target_center=None

    def _base(self,config):
        return dict(version=self.VERSION,mode='READ_ONLY_SHADOW',status='WAITING',
                    distance_label='候选测距，未验证目标身份，不允许用于控制',
                    target=None,candidate=None,stable_candidate_m=None,
                    laser_points=[],radar_online=False,camera_online=False,
                    radar_bearing_deg=None,angle_half_window_deg=None,
                    yaw_deg=None,fusion_age_ms=None,calibrated=False,
                    association_quality='NONE',association_score=None,
                    association_candidates=0,radial_speed_mps=None,
                    angular_residual_deg=None,independent_confirmations=0,
                    association_warning=None)

    def fuse(self,visual,radar,servo,config,now=None):
        if now is None:now=time.monotonic()
        out=self._base(config)
        vdata,vt=visual
        rdata,rt=radar
        sdata,st=servo
        target,reason=validated_selection(vdata,now)
        # 8081 源码不在本仓库。融合入口拒绝非 person，植物和其他类不会被跟踪或融合。
        if target is not None and not person_allowed(target.get('class_id'), target.get('class_name'), config):
            target = None
            reason = 'CLASS_NOT_PERSON'
        pts=radar_points(rdata)
        out['laser_points']=[list(p) for p in pts[:1200]]
        out['radar_online']=bool(isinstance(rdata,dict) and rdata.get('online') is True and pts)
        out['radar_packets']=rdata.get('packets') if isinstance(rdata,dict) else None
        out['radar_error']=rdata.get('error') if isinstance(rdata,dict) else 'NO_RADAR'
        if target:
            out['target']=target
            out['camera_online']=True
        if isinstance(sdata,dict) and isinstance(sdata.get('yaw'),dict):
            yy=sdata['yaw'].get('angle')
            if finite_number(yy):out['yaw_deg']=yy
        hfov=config.get('camera_hfov_deg')
        zero=config.get('radar_zero_deg')
        sign=config.get('yaw_to_radar_sign')
        mount=config.get('radar_mount')
        out['calibrated']=(finite_number(hfov) and 10<=hfov<=150 and
                          finite_number(zero) and 0<=zero<360 and sign in (-1,1) and
                          mount in ('chassis','gimbal'))
        key=(target['id'],target['class_id']) if target else None
        if key!=self.last_key:self._reset_track(key)
        if not target:
            out['status']=reason
            if reason=='CLASS_NOT_PERSON':
                out['association_warning']='视觉选中的不是 person，不跟踪、不融合'
            self._reset_track(None)
            return out
        if not out['radar_online']:
            out['status']='RADAR_NOT_ONLINE'
            self._reset_track(key)
            return out
        if not out['calibrated']:
            out['status']='CALIBRATION_REQUIRED'
            self._reset_track(key)
            return out
        if not finite_number(st) or not finite_number(rt):
            out['status']='TIME_UNAVAILABLE'
            self._reset_track(key)
            return out
        ages=[now-vdata['updated'],now-rt,now-st]
        if any(age<0 or age>limit for age,limit in zip(ages,(0.85,0.65,0.55))):
            out['status']='SENSOR_DATA_STALE'
            self._reset_track(key)
            return out
        if max(vdata['updated'],rt,st)-min(vdata['updated'],rt,st)>0.55:
            out['status']='SENSOR_TIME_MISMATCH'
            self._reset_track(key)
            return out
        yaw=out['yaw_deg']
        if not finite_number(yaw):
            out['status']='YAW_UNAVAILABLE'
            self._reset_track(key)
            return out
        if self.last_yaw_sample is None or st!=self.last_yaw_sample[0]:
            if self.last_yaw_sample is not None:
                delta=st-self.last_yaw_sample[0]
                if 0.025<=delta<=2.0:
                    self.yaw_rate_deg_s=angular_delta(yaw,self.last_yaw_sample[1])/delta
                else:self.yaw_rate_deg_s=None
            self.last_yaw_sample=(st,yaw)
        out['yaw_rate_deg_s']=round(self.yaw_rate_deg_s,1) if self.yaw_rate_deg_s is not None else None
        if self.yaw_rate_deg_s is not None and abs(self.yaw_rate_deg_s)>35:
            # 暂停本帧，但不要清掉已经确认的距离。云台跟着人转时经常超过 35°/s，
            # 每转一次就重计 3 包，稳定距离会一直起不来。
            out['status']='YAW_MOVING_FAST_ASSOCIATION_PAUSED'
            out['association_warning']='云台转动过快，本帧不更新关联，保留已有航迹'
            return out
        fov_angle=camera_bearing_deg(target['cx'],target['width'],hfov)
        body_yaw=yaw if mount=='chassis' else 0.0
        bearing=wrap360(zero+sign*(body_yaw+fov_angle+config.get('camera_yaw_offset_deg',0.0)))
        half_bbox=abs(camera_bearing_deg(target['box'][2],target['width'],hfov)-
                      camera_bearing_deg(target['box'][0],target['width'],hfov))/2
        window=min(12.0,max(2.0,half_bbox+1.5))
        out['radar_bearing_deg']=round(bearing,2)
        out['angle_half_window_deg']=round(window,2)
        out['fusion_age_ms']=round(max(ages)*1000,1)
        # Unexpected jump of the tracked box may be ID reuse/track switching.
        if self.last_target_center is not None:
            prev_t,prev_center=self.last_target_center
            dt=now-prev_t
            if 0<dt<0.35 and abs(target['cx']/target['width']-prev_center)>0.32:
                self.history=[]
                self.last_packet=None
                out['association_warning']='视觉框突跳，重新确认目标'
        self.last_target_center=(now,target['cx']/target['width'])
        candidates=enumerate_clusters(pts,bearing,window)
        out['association_candidates']=len(candidates)
        if not candidates:
            # 点落在角度窗外时同样不输出距离，但保留已有确认。
            # 否则一次 10° 左右的零位误差就会把航迹打回零。
            out['status']='NO_RADAR_CLUSTER_IN_TARGET_DIRECTION'
            return out
        good=[c for c in candidates if abs(c['angle_error_deg'])<=self.MAX_ANGLE_ERROR_DEG]
        if not good:
            # 现场记录里大量帧角差在 7° 门外（例如 359° 对 348°）。
            # 这些帧仍然不输出距离，但不再把已经积累的确认次数清零。
            out['status']='ASSOCIATION_ANGULAR_UNCERTAIN'
            out['association_warning']='雷达点簇方向与视觉目标偏差过大，本帧不更新距离'
            return out
        # Stable previous range gives inertia, particularly when point-cloud
        # clustering temporarily splits a person around radar 0 degrees.
        prev=self.history[-1] if self.history else None
        last_range=prev[1] if prev else None
        elapsed=max(0.02,now-prev[0]) if prev else 0.0
        predicted=last_range
        if prev and len(self.history)>=2:
            a,b=self.history[-2:]
            dv=(b[1]-a[1])/max(0.05,b[0]-a[0])
            predicted=b[1]+max(-self.MAX_RADIAL_RATE_M_S,min(self.MAX_RADIAL_RATE_M_S,dv))*elapsed
        def score(c):
            angle=abs(c['angle_error_deg'])/self.MAX_ANGLE_ERROR_DEG
            range_penalty=abs(c['range_m']-predicted) if predicted is not None else 0.0
            continuity=range_penalty*2.0
            prior_bearing_penalty=(abs(angular_delta(c['radar_bearing_deg'],prev[2]))/30.0
                                   if prev is not None else 0)
            return angle*1.8+continuity+prior_bearing_penalty+0.3*c['range_spread_m']
        ranked=sorted(good,key=score)
        # 深度歧义（V3.1.3）：没有航迹时，角度接近但距离差 ≥0.5m 的两簇一律拒绝。
        # 不能靠“更近”或分数去猜人和背景。
        # 已经有至少两包连续距离时，只留下和预测距离连续的那一簇；
        # 另一簇离预测 ≥0.5m 就当成背景，不再把整条航迹清掉。
        # 两簇都和预测连续、或都对不上时，仍然失败关闭。
        def _near_prediction(c):
            if predicted is None or len(self.history) < 2:
                return False
            max_step = 0.23 + self.MAX_RADIAL_RATE_M_S * elapsed
            return abs(c['range_m'] - predicted) <= max_step

        competing = []
        for i, c1 in enumerate(good):
            for c2 in good[i+1:]:
                if (abs(c1['range_m']-c2['range_m']) >= 0.50 and
                    abs(abs(c1['angle_error_deg'])-abs(c2['angle_error_deg'])) <= 2.0):
                    competing.append((c1, c2))
        if competing:
            resolved = None
            blocked = predicted is None or len(self.history) < 2
            if not blocked:
                for c1, c2 in competing:
                    keep1 = _near_prediction(c1) and abs(c2['range_m']-predicted) >= 0.50
                    keep2 = _near_prediction(c2) and abs(c1['range_m']-predicted) >= 0.50
                    pick = c1 if keep1 and not keep2 else c2 if keep2 and not keep1 else None
                    if pick is None:
                        blocked = True
                        break
                    if resolved is not None and abs(resolved['range_m']-pick['range_m']) > 0.22:
                        blocked = True
                        break
                    resolved = pick
            if blocked or resolved is None:
                out['status']='ASSOCIATION_COMPETING_DEPTHS'
                out['association_warning']='目标方向存在角度接近但距离不同的点簇，未证实身份，禁止融合控制'
                out['association_quality']='NONE'
                self.history=[]
                self.last_packet=None
                return out
            good = [c for c in good if _near_prediction(c)]
            ranked = sorted(good, key=score)
            out['association_warning']='同方位另有距离不一致的点簇，已按已确认距离连续性忽略'
        if not ranked:
            out['status']='ASSOCIATION_COMPETING_DEPTHS'
            out['association_quality']='NONE'
            self.history=[]
            self.last_packet=None
            return out
        chosen=ranked[0]
        out['angular_residual_deg']=round(abs(chosen['angle_error_deg']),2)
        if len(ranked)>=2 and abs(score(ranked[1])-score(chosen))<0.40 and abs(ranked[1]['range_m']-chosen['range_m'])>0.22:
            out['status']='ASSOCIATION_AMBIGUOUS'
            out['association_warning']='同方向存在竞争雷达点簇，暂不输出目标距离'
            self.history=[]
            return out
        if prev:
            max_step=0.23+self.MAX_RADIAL_RATE_M_S*elapsed
            range_gap=abs(chosen['range_m']-predicted)
            quiet_s=now-prev[0]
            # 角差把门控在门外的那几帧不会写入 history。
            # 只要新距离仍连续，不能仅因为中间空了 0.8 秒就当成距离跳变。
            # 空档超过 1.5 秒，或空档超过 0.8 秒且距离也变了，才重新确认。
            if (range_gap>max_step or quiet_s>1.5 or
                    (quiet_s>0.8 and range_gap>0.40)):
                out['status']='ASSOCIATION_RANGE_JUMP_REJECTED'
                out['association_warning']='候选距离变化超过连续性阈值，重新确认'
                self.history=[]
                return out
        pkt=rdata.get('packets') if isinstance(rdata,dict) else None
        if type(pkt) is not int or pkt<0:
            out['status']='RADAR_PACKET_COUNTER_INVALID'
            self.history=[]
            return out
        if type(self.last_packet) is int and pkt<self.last_packet:
            self.history=[]
            self.last_packet=None
            out['status']='RADAR_PACKET_COUNTER_RESET'
            return out
        if self.last_packet==pkt and self.history:
            previous=self.history[-1]
            if (abs(chosen['range_m']-previous[1])>0.12 or
                abs(angular_delta(chosen['radar_bearing_deg'],previous[2]))>4.0):
                out['status']='WAIT_FOR_NEW_RADAR_PACKET'
                out['association_warning']='视觉方位已变化，等待新雷达扫描包再确认'
                return out
        if self.last_packet!=pkt:
            self.history.append((now,chosen['range_m'],chosen['radar_bearing_deg'],pkt))
            self.history=self.history[-5:]
            self.last_packet=pkt
        out['candidate']=chosen
        out['independent_confirmations']=len(self.history)
        out['association_quality']='CANDIDATE_UNVERIFIED'
        score_heuristic=round(max(0,min(90,90-abs(chosen['angle_error_deg'])*7-
                                            chosen['range_spread_m']*25)),1)
        out['association_score']=score_heuristic
        if len(self.history)>=2:
            first,last=self.history[0],self.history[-1]
            dt=last[0]-first[0]
            if dt>=0.08:
                vr=(last[1]-first[1])/dt
                if abs(vr)<=self.MAX_RADIAL_RATE_M_S:
                    out['radial_speed_mps']=round(vr,2)
        if len(self.history)>=self.MIN_CONFIRMATIONS:
            recent=self.history[-3:]
            ok=all(abs(y[1]-x[1])<=0.23+self.MAX_RADIAL_RATE_M_S*(y[0]-x[0])
                   for x,y in zip(recent,recent[1:]))
            if ok and now-recent[-1][0]<=0.6:
                out['stable_candidate_m']=round(statistics.median(z[1] for z in recent),3)
                out['status']='CANDIDATE_STABLE_UNVERIFIED'
                out['association_quality']='TRACK_CONTINUOUS_UNVERIFIED'
                return out
        out['status']='CANDIDATE_WAITING_STABILITY'
        return out
