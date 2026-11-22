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
        return {'id':ident,'class_id':cls,'box':list(b),'width':w,'height':h,
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


def radar_candidates(points, bearing, half_angle_deg):
    """Angular+range clustering. Returns candidate, not identity-proven range."""
    inside=[]
    for a,d,s in points:
        delta=angular_delta(a,bearing)
        if abs(delta)<=half_angle_deg:inside.append((delta,a,d,s))
    inside.sort(key=lambda p:p[0])
    clusters=[]; cur=[]
    for p in inside:
        if cur and (abs(p[0]-cur[-1][0])>2.0 or abs(p[2]-cur[-1][2])>0.55):
            if len(cur)>=3:clusters.append(cur)
            cur=[]
        cur.append(p)
    if len(cur)>=3:clusters.append(cur)
    if not clusters:return None
    # Prefer a coherent cluster close to the target bearing. Do NOT simply pick the
    # closest range, which can pick foreground obstacles instead of the target.
    clusters.sort(key=lambda c:(abs(statistics.median(p[0] for p in c)),
                                  statistics.median(p[2] for p in c)))
    cluster=clusters[0]
    return {'range_m':round(statistics.median(p[2] for p in cluster),3),
            'radar_bearing_deg':round(wrap360(statistics.median(p[1] for p in cluster)),2),
            'angle_error_deg':round(statistics.median(p[0] for p in cluster),2),
            'points':len(cluster),'strength':round(statistics.median(p[3] for p in cluster),1)}


class FusionEngine:
    def __init__(self):
        self.last_id=None
        self.history=[]  # each independent advancing radar packet; not duplicate polls
        self.last_packet=None
        self.last_yaw_sample=None
        self.yaw_rate_deg_s=None

    def fuse(self, visual, radar, servo, config, now=None):
        if now is None:now=time.monotonic()
        output={'version':'V3_FUSION_V1','mode':'READ_ONLY_SHADOW',
                'status':'WAITING','distance_label':'候选距离，尚未验证目标身份',
                'target':None,'candidate':None,'stable_candidate_m':None,
                'laser_points':[],'radar_online':False,'camera_online':False,
                'radar_bearing_deg':None,'angle_half_window_deg':None,
                'yaw_deg':None,'fusion_age_ms':None,
                'calibrated':False}
        vdata, vt=visual
        rdata, rt=radar
        sdata, st=servo
        target, reason=validated_selection(vdata,now)
        if target:
            output['target']=target
            output['camera_online']=True
        # Always expose radar points for independent visualization (max 1200 pts).
        pts=radar_points(rdata)
        output['radar_online']=bool(isinstance(rdata,dict) and rdata.get('online') is True and pts)
        output['laser_points']=[list(p) for p in pts[:1200]]
        output['radar_packets']=rdata.get('packets') if isinstance(rdata,dict) else None
        output['radar_error']=rdata.get('error') if isinstance(rdata,dict) else 'NO_RADAR'
        if isinstance(sdata,dict) and isinstance(sdata.get('yaw'),dict):
            y=sdata['yaw'].get('angle')
            if finite_number(y):output['yaw_deg']=y
        hfov=config.get('camera_hfov_deg')
        zero=config.get('radar_zero_deg')
        sign=config.get('yaw_to_radar_sign')
        mount=config.get('radar_mount')
        output['calibrated']=(finite_number(hfov) and 10<=hfov<=150 and
                              finite_number(zero) and 0<=zero<360 and sign in (-1,1) and
                              mount in ('chassis','gimbal'))
        if not target:
            output['status']=reason
            self._reset_track(None)
            return output
        if not output['radar_online']:
            output['status']='RADAR_NOT_ONLINE'
            self._reset_track(target['id'])
            return output
        if not output['calibrated']:
            output['status']='CALIBRATION_REQUIRED'
            self._reset_track(target['id'])
            return output
        if not finite_number(st) or not finite_number(rt):
            output['status']='TIME_UNAVAILABLE';return output
        ages=[now-vdata['updated'], now-rt, now-st]
        # Radar point list includes bins as old as 1.2 sec, even on fresh scans.
        # Report a conservative candidate and keep all estimates explicitly unverified.
        if any(a<0 or a>lim for a,lim in zip(ages,(0.85,0.65,0.55))):
            output['status']='SENSOR_DATA_STALE';self._reset_track(target['id']);return output
        if max((vdata['updated'],rt,st))-min((vdata['updated'],rt,st))>0.55:
            output['status']='SENSOR_TIME_MISMATCH';self._reset_track(target['id']);return output
        yaw=output['yaw_deg']
        if not finite_number(yaw):
            output['status']='YAW_UNAVAILABLE';return output
        # Encoder samples arrive separately from camera/laser frames. Avoid
        # linking a laser return to the wrong person during rapid panning.
        if self.last_yaw_sample is None or st!=self.last_yaw_sample[0]:
            if self.last_yaw_sample is not None:
                dt=st-self.last_yaw_sample[0]
                if 0.025<=dt<=2.0:
                    rate=angular_delta(yaw,self.last_yaw_sample[1])/dt
                    self.yaw_rate_deg_s=rate
                else:self.yaw_rate_deg_s=None
            self.last_yaw_sample=(st,yaw)
        output['yaw_rate_deg_s']=round(self.yaw_rate_deg_s,1) if self.yaw_rate_deg_s is not None else None
        if self.yaw_rate_deg_s is not None and abs(self.yaw_rate_deg_s)>35.0:
            self._reset_track(target['id'])
            output['status']='YAW_MOVING_FAST_ASSOCIATION_PAUSED'
            return output
        fov_angle=camera_bearing_deg(target['cx'],target['width'],hfov)
        # Chassis radar stays fixed while gimbal yaws; a gimbal-mounted radar
        # rotates with the camera and must NOT add yaw again.
        body_yaw=yaw if mount=='chassis' else 0.0
        bearing=wrap360(zero + sign*(body_yaw+fov_angle+config.get('camera_yaw_offset_deg',0.0)))
        half_bbox=abs(camera_bearing_deg(target['box'][2],target['width'],hfov)-
                      camera_bearing_deg(target['box'][0],target['width'],hfov))/2
        # Avoid overly wide bounding boxes making association nearly meaningless.
        window=min(12.0,max(2.0,half_bbox+1.5))
        output['radar_bearing_deg']=round(bearing,2)
        output['angle_half_window_deg']=round(window,2)
        output['fusion_age_ms']=round(max(ages)*1000,1)
        cand=radar_candidates(pts,bearing,window)
        if cand is None:
            output['status']='NO_RADAR_CLUSTER_IN_TARGET_DIRECTION'
            self._reset_track(target['id'])
            return output
        output['candidate']=cand
        # One packet counter increment may encompass many packets; do not count the
        # same counter twice toward temporal stability.
        packets=rdata.get('packets') if isinstance(rdata,dict) else None
        if target['id']!=self.last_id:self._reset_track(target['id'])
        if type(packets) is int and type(self.last_packet) is int and packets<self.last_packet:
            self._reset_track(target['id'])
        if type(packets) is int and packets!=self.last_packet:
            if self.history and (abs(cand['range_m']-self.history[-1][1])>0.8 or
                                 now-self.history[-1][0]>1.0):
                self.history.clear()
            self.history.append((now,cand['range_m']))
            self.history=self.history[-5:]
            self.last_packet=packets
        if len(self.history)>=3:
            recent=self.history[-3:]
            if max(z for _,z in recent)-min(z for _,z in recent)<=0.6:
                output['stable_candidate_m']=round(statistics.median(z for _,z in recent),3)
                output['status']='CANDIDATE_STABLE_UNVERIFIED'
                return output
        output['status']='CANDIDATE_WAITING_STABILITY'
        return output

    def _reset_track(self, ident):
        self.last_id=ident
        self.history=[]
        self.last_packet=None
