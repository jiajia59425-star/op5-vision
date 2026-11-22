#!/usr/bin/env python3
"""Read-only V3 angular-offset A/B experiment.

GET from 8081/8092/8093/8101; instantiate two in-memory FusionEngines.
No service restart, no config writes, no HTTP POST and no servo motion.
"""
import argparse
import collections
import json
import pathlib
import statistics
import sys
import time
import urllib.request

FUSION = pathlib.Path('/home/orangepi/op5_v3_fusion')
BRIDGE = pathlib.Path('/home/orangepi/op5_v3_fusion_bridge')
sys.path.insert(0, str(FUSION))
sys.path.insert(0, str(BRIDGE))
from fusion_core import FusionEngine  # type: ignore
from gate import decide  # type: ignore


def get(port, path):
    req = urllib.request.Request(
        'http://127.0.0.1:%s%s' % (port, path),
        headers={'Accept': 'application/json', 'Cache-Control': 'no-store'})
    with urllib.request.urlopen(req, timeout=3) as resp:
        return json.load(resp)


def idle_guard():
    guard = get(8101, '/api/status')
    auto = get(8093, '/api/auto/status')
    if guard.get('armed') is not False or guard.get('stop_pending') is not False:
        raise RuntimeError('8101仍处于人工授权或停机待确认状态，拒绝进行标定模拟')
    if auto.get('active') is not False:
        raise RuntimeError('8093自动跟踪仍在运行，请先停止')
    try:
        center = get(8093, '/api/center/status')
        hold = get(8093, '/api/hold/status')
        if center.get('active') is True or hold.get('state') == 'busy':
            raise RuntimeError('云台回中或位置保持恢复正在运行，请先停止')
    except urllib.error.HTTPError as e:
        raise RuntimeError('不能确认云台回中/恢复状态，拒绝执行: %s' % e)


def show(label, data):
    statuses = collections.Counter(x['status'] for x in data)
    def median_of(name):
        values = [x[name] for x in data if isinstance(x.get(name), (int, float))]
        return round(statistics.median(values), 3) if values else None
    print('%-9s %2d/%2d   %2d/%2d   %-7s  %-7s  %s' % (
        label,
        sum(x['stable'] for x in data), len(data),
        sum(x['would_allow'] for x in data), len(data),
        str(median_of('residual')),
        str(median_of('range')),
        dict(statuses)), flush=True)


def capture(label, base_cfg, offset, samples, interval):
    eng0, eng1 = FusionEngine(), FusionEngine()
    tuned_cfg = dict(base_cfg)
    tuned_cfg['radar_zero_deg'] = (base_cfg['radar_zero_deg'] + offset) % 360
    rows = [[], []]
    selected_ids = set()
    prev_packets = None
    packet_changes = 0
    for i in range(samples):
        if i == 0 or i % 8 == 0:
            idle_guard()
        v = get(8081, '/state')
        r = get(8092, '/api/scan')
        rt = time.monotonic()
        s = get(8093, '/api/status')
        st = time.monotonic()
        now = time.monotonic()
        if v.get('selected') is not None:
            selected_ids.add(v.get('selected'))
        pkt = r.get('packets')
        if prev_packets is not None and pkt != prev_packets:
            packet_changes += 1
        prev_packets = pkt
        for j, (engine, cfg) in enumerate(((eng0, base_cfg), (eng1, tuned_cfg))):
            out = engine.fuse((v, v.get('updated')),
                              (r, rt), (s, st), cfg, now=now)
            # Simulate the exact 8101 preflight logic, but never call its POST routes.
            out['config'] = {'radar_mount': cfg.get('radar_mount')}
            out['sources'] = {name: {'ok': True} for name in ('vision', 'radar', 'servo')}
            allowed, reason, _ = decide(out, v)
            rows[j].append({
                'stable': out.get('status') == 'CANDIDATE_STABLE_UNVERIFIED',
                'would_allow': bool(allowed),
                'reason': reason,
                'status': out.get('status', 'UNKNOWN'),
                'residual': out.get('angular_residual_deg'),
                'range': out.get('stable_candidate_m'),
            })
        time.sleep(interval)
    print('\n位置: %s | YOLO IDs=%s | 雷达包更新=%d/%d' % (
        label, ','.join(map(str, sorted(selected_ids))) or '无', packet_changes, samples - 1))
    print('配置       稳定候选    模拟许可   误差中位°   测距中位m    状态计数')
    show('原标定', rows[0])
    show('补偿%+.1f°' % offset, rows[1])
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--offset', type=float, default=6.2)
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--interval', type=float, default=0.25)
    args = parser.parse_args()
    if not (0 < args.offset < 15 and 8 <= args.samples <= 100 and 0.15 <= args.interval <= 1):
        raise RuntimeError('采样或偏移参数不在诊断允许范围')
    idle_guard()
    if 'ASSOCIATION_COMPETING_DEPTHS' not in (FUSION / 'fusion_core.py').read_text(encoding='utf-8'):
        raise RuntimeError('现场融合核心未确认安装V3.1.3双深度歧义保护，请先检查版本')
    cfg = json.loads((FUSION / 'config.json').read_text(encoding='utf-8'))
    if cfg.get('radar_mount') != 'chassis' or not isinstance(cfg.get('radar_zero_deg'), (float,int)):
        raise RuntimeError('雷达安装参数不符合预期')
    print('V3雷达零位双版本影子测试（只读），原零位=%.2f° 模拟零位=%.2f°' % (
        cfg['radar_zero_deg'], (cfg['radar_zero_deg'] + args.offset) % 360))
    print('8101实际控制许可与8093均不更改；模拟许可不是实机启动许可。')
    poses = ('中央近距离（约1.5～2米）', '画面左侧约20%',
             '画面右侧约20%', '中央较远（约2.5～3米）')
    with open('/dev/tty', 'r') as tty:
        for pose in poses:
            print('\n人物移动到%s、保持不动，按回车采样' % pose, flush=True)
            tty.readline()
            capture(pose, cfg, args.offset, args.samples, args.interval)
    print('\n四位置模拟测试完成。没有修改config.json、没有重启服务、没有发送运动指令。')
    print('请将四组汇总发回，决定是否需要临时写入标定进行回归测试。')


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print('\n操作者已中止，只读测试退出。')
    except Exception as exc:
        print('ERROR:', type(exc).__name__, str(exc))
        raise SystemExit(1)
