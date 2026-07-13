#!/usr/bin/env python3
"""Video deseni gorevinin ROS'suz masaustu simulasyonu.

Kullanim (depo kokunden, ROS gerekmez, sadece PyYAML):
    python scripts/sim_video_pattern.py [gorev_adi]

Ne yapar: config/missions/<gorev>.yaml adimlarini basit arac dinamigi
(yaw hizi, derinlik hizi, hiz tablosu ile konum entegrasyonu) uzerinde
kosturur; adim surelerini, izlenen rotayi ve kapanma hatasini raporlar.

Havuz oncesi kullanim: hiz tablosu / yaw_rate_full_dps guncellendikce
calistirip bacak surelerini (17.3 sn telafiler) yeniden ayarlayin.
NOT: Ideal dinamik varsayar (akinti, atalet, asiri salinim yok) - gercek
sonuc havuzda dogrulanmali.
"""
import math
import sys
import types
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent

# ---- auv_msgs stub (ROS kurulu olmayan PC'de calisabilmek icin) ----
_msgs = types.ModuleType('auv_msgs')
_msg = types.ModuleType('auv_msgs.msg')


class _LaneStatus:
    SEARCHING, TRACKING, LOST, END_OF_LINE = 0, 1, 2, 3


_msg.LaneStatus = _LaneStatus
_msgs.msg = _msg
sys.modules.setdefault('auv_msgs', _msgs)
sys.modules.setdefault('auv_msgs.msg', _msg)

sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'auv_mission'))
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'auv_dead_reckoning'))
from auv_mission.primitives import build_steps          # noqa: E402
from auv_dead_reckoning.speed_table import SpeedTable   # noqa: E402


class SimCtx:
    YAW_FULL_DPS = 60.0   # r=1000'deki donus hizi (mission parametresiyle esle!)
    Z_FULL_MPS = 0.3      # z ofseti tam iken dikey hiz

    def __init__(self):
        self.cfg = dict(heading_kp=12.0, heading_kd=4.0, max_yaw_cmd=400,
                        depth_kp=500.0, max_z_offset=250, yaw_rate_full_dps=60.0)
        self.table = SpeedTable(str(ROOT / 'config' / 'speed_table.csv'))
        self.targets = {}
        self.target_heading_deg = 0.0
        self.yaw_deg = 0.0
        self.depth_m = 0.0
        self.altitude_m = 3.0
        self.altitude_valid = True
        self.armed = False
        self.mode = 'MANUAL'
        self.dr = None
        self.lane_cmd = None
        self.lane_status = None
        self.message = ''
        self.out = (0, 0, 500, 0)
        self.x = self.y = 0.0
        self.spin_trace = []

    def set_output(self, x, y, z, r):
        self.out = (int(x), int(y), int(z), int(r))

    def request_mode(self, name):
        self.mode = 'ALT_HOLD' if name == 'DEPTH_HOLD' else name

    def request_arm(self, arm):
        self.armed = bool(arm)

    def capture_origin(self):
        pass

    def step_dynamics(self, dt):
        x_cmd, _, z_cmd, r_cmd = self.out
        self.yaw_deg = (self.yaw_deg + (r_cmd / 1000.0) * self.YAW_FULL_DPS * dt) % 360.0
        self.depth_m = max(0.0, self.depth_m - ((z_cmd - 500) / 250.0) * self.Z_FULL_MPS * dt)
        v = self.table.speed(x_cmd)
        h = math.radians(self.yaw_deg)
        self.x += v * math.sin(h) * dt
        self.y += v * math.cos(h) * dt


def main():
    mission = sys.argv[1] if len(sys.argv) > 1 else 'video_pattern'
    with open(ROOT / 'config' / 'missions' / f'{mission}.yaml', encoding='utf-8') as f:
        steps = build_steps(yaml.safe_load(f)['steps'])

    ctx = SimCtx()
    dt, t = 0.1, 0.0
    print(f"Gorev '{mission}': {len(steps)} adim\n")
    for i, step in enumerate(steps):
        step.enter(ctx)
        t0 = t
        trace = []
        while True:
            done = step.update(ctx, dt)
            ctx.step_dynamics(dt)
            trace.append((ctx.x, ctx.y))
            t += dt
            if done:
                break
            if t - t0 > 400:
                print(f'!! adim takildi: {step.name}')
                return 1
        extra = ''
        if 'daire' in step.name and len(trace) > 4:
            xs = [p[0] for p in trace]
            ys = [p[1] for p in trace]
            dia = max(max(xs) - min(xs), max(ys) - min(ys))
            extra = f'  DAIRE CAPI ~{dia:.2f} m (kural: >= 1.00 m)'
        print(f'{i + 1:2d}. {step.name:<22s} sure={t - t0:6.1f}s  '
              f'poz=({ctx.x:+6.2f},{ctx.y:+6.2f})  pruva={ctx.yaw_deg:6.1f}  '
              f'derinlik={ctx.depth_m:4.2f}{extra}')
        ctx.message = ''

    err = math.hypot(ctx.x, ctx.y)
    print(f'\nToplam sure         : {t:.0f} s (video kurali: 1-5 dk)')
    print(f'Baslangica uzaklik  : {err:.2f} m (1x1 m kare icin < ~0.5 m hedefleyin)')
    print(f'Arm durumu          : {ctx.armed}')
    ok = err < 1.0 and not ctx.armed
    print('SONUC:', 'BASARILI' if ok else 'SORUNLU - parametre ayari gerekli')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
