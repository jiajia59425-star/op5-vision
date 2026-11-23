"""V3 camera-gimbal fusion interlock; never sends servo PWM or radar serial commands.

Association is unverified: conservative camera-only operator-supervised gating.
"""
import math

MIN_RANGE_M = 0.75
MAX_RANGE_M = 6.0
MAX_ANGULAR_RESIDUAL = 4.0
MAX_FUSION_AGE_MS = 350.0


def is_number(x):
    return type(x) in (int, float) and math.isfinite(x)


def decide(fusion, vision, auto=None, expected_target=None):
    """Return (ok, reason, (id,class)). No stale distance carried forward."""
    if not isinstance(fusion, dict) or not isinstance(vision, dict):
        return False, '融合/视觉无有效数据', None
    if fusion.get('version') != 'V3_1_DYNAMIC_ASSOC_V1':
        return False, '8100不是已验证的V3.1融合版本', None
    if fusion.get('status') != 'CANDIDATE_STABLE_UNVERIFIED':
        return False, '雷达关联未连续稳定：'+str(fusion.get('status')), None
    if fusion.get('association_quality') != 'TRACK_CONTINUOUS_UNVERIFIED':
        return False, '关联质量不足', None
    if (fusion.get('calibrated') is not True or fusion.get('radar_online') is not True
            or fusion.get('camera_online') is not True):
        return False, '传感器离线或未标定', None
    cfg=fusion.get('config') or {}
    if cfg.get('radar_mount') != 'chassis':
        return False, '雷达安装标定必须是chassis', None
    sources=fusion.get('sources') or {}
    if any((sources.get(n) or {}).get('ok') is not True for n in ('vision','radar','servo')):
        return False, '融合数据源不完整', None
    if not is_number(fusion.get('fusion_age_ms')) or fusion['fusion_age_ms'] > MAX_FUSION_AGE_MS or fusion['fusion_age_ms'] < 0:
        return False, '融合数据过期', None
    if type(fusion.get('independent_confirmations')) is not int or fusion['independent_confirmations'] < 3:
        return False, '雷达独立扫描确认次数不足', None
    if not is_number(fusion.get('angular_residual_deg')) or fusion['angular_residual_deg'] > MAX_ANGULAR_RESIDUAL or fusion['angular_residual_deg'] < 0:
        return False, '雷达与视觉角度不一致', None
    rng=fusion.get('stable_candidate_m')
    if not is_number(rng) or not MIN_RANGE_M <= rng <= MAX_RANGE_M:
        return False, '目标候选距离不在阶段测试范围内', None
    target=fusion.get('target') or {}
    fid,fcls=target.get('id'),target.get('class_id')
    vid,vcls=vision.get('selected'),vision.get('selected_class')
    if (type(fid) is not int or type(fcls) is not int or
        type(vid) is not int or type(vcls) is not int or
        (fid,fcls) != (vid,vcls)):
        return False, '视觉目标ID/类别与融合结果不一致', None
    label = target.get('class_name') or vision.get('selected_class_name') or ''
    if isinstance(label, str) and label.strip() and label.strip().lower() not in ('person', '行人'):
        return False, '视觉类别不是 person，禁止跟踪', None
    allowed = cfg.get('person_class_ids') or [0]
    if not isinstance(allowed, list) or fcls not in allowed or vcls not in allowed:
        return False, '视觉类别不是 person，禁止跟踪', None
    if vision.get('live') is not True or vision.get('found') is not True:
        return False, '视觉目标不可用', None
    if expected_target is not None and (fid,fcls) != expected_target:
        return False, '目标ID变化，禁止无授权重新捕捉', None
    if auto is not None:
        if not isinstance(auto,dict) or auto.get('active') is not True:
            return False, '8093跟踪不再运行', None
        if auto.get('target_id') != fid or auto.get('class_id') != fcls:
            return False, '8093目标与雷达融合目标不一致', None
        if auto.get('source') != 'rgb' or auto.get('range_mode') != 'safe':
            return False, '8093跟踪模式不符合试验配置', None
        if auto.get('fast_profile') != 'PREDICTIVE_FAST_V2':
            return False, '8093控制器版本发生变化', None
    return True, '融合候选稳定，可以人工授权开启摄像云台跟踪', (fid,fcls)
