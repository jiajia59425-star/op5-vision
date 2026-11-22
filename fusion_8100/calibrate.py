#!/usr/bin/env python3
"""Local SSH-only calibration editor; does not talk to motors or sensors."""
import argparse,json,math,os,pathlib,tempfile
ROOT=pathlib.Path(__file__).resolve().parent
DEST=ROOT/'config.json'
DEFAULT={'camera_hfov_deg':None,'radar_zero_deg':None,'radar_mount':None,'yaw_to_radar_sign':1,'camera_yaw_offset_deg':0.0}


def save_config(c):
    fd,p=tempfile.mkstemp(prefix='.v3calib_',suffix='.json',dir=str(ROOT))
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as f:
            json.dump(c,f,ensure_ascii=False,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
        os.chmod(p,0o644);os.replace(p,DEST)
    finally:
        if os.path.exists(p):os.unlink(p)


def main():
    p=argparse.ArgumentParser(description='V3 融合标定（只影响8100候选距离匹配）')
    p.add_argument('--hfov',type=float,help='RGB摄像头水平视场角，10~150度，需要实测/规格确认')
    p.add_argument('--zero',type=float,help='当Yaw为0且视觉目标位于画面正中时，雷达对应角度，0~360度')
    p.add_argument('--mount',choices=('chassis','gimbal'),help='雷达相对运动：固定车体chassis，或跟随云台gimbal')
    p.add_argument('--sign',type=int,choices=(-1,1),help='Yaw/视觉右偏映射到雷达角度的正负号')
    p.add_argument('--yaw-offset',type=float,help='额外摄像头安装角度偏移；默认0')
    p.add_argument('--show',action='store_true')
    a=p.parse_args()
    c=dict(DEFAULT)
    if DEST.exists():c.update(json.loads(DEST.read_text('utf-8')))
    changed=False
    for arg,key in [('hfov','camera_hfov_deg'),('zero','radar_zero_deg'),('mount','radar_mount'),('sign','yaw_to_radar_sign'),('yaw_offset','camera_yaw_offset_deg')]:
        v=getattr(a,arg)
        if v is not None:c[key]=v;changed=True
    if c['camera_hfov_deg'] is not None and not (10<=c['camera_hfov_deg']<=150 and math.isfinite(c['camera_hfov_deg'])):p.error('HFOV 超出10~150度')
    if c['radar_zero_deg'] is not None and not (0<=c['radar_zero_deg']<360 and math.isfinite(c['radar_zero_deg'])):p.error('雷达零位必须在0~360度之间')
    if not (-30<=c['camera_yaw_offset_deg']<=30 and math.isfinite(c['camera_yaw_offset_deg'])):p.error('安装角度偏移必须为±30度以内')
    if changed:
        save_config(c)
        print('已保存标定配置，仅影响V3读取展示；无需重启8093')
    print(json.dumps(c,ensure_ascii=False,indent=2))
    if c['camera_hfov_deg'] is None or c['radar_zero_deg'] is None or c['radar_mount'] is None:
        print('状态：CALIBRATION_REQUIRED（不显示关联候选距离）')
    else:
        print('状态：已填写参数，请先与实物静态距离和角度交叉验证；不代表身份匹配已确认')

if __name__=='__main__': main()
