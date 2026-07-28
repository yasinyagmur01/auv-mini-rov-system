#!/usr/bin/env python3
# =============================================================================
# FAZ 3 — Görev mantığı ARM'SIZ prova (dry-run) — SALT OKUNUR
# =============================================================================
# Görev primitiflerini (auv_mission.primitives) idealize bir "plant" ile simüle
# eder ve bir görev deseninin /mav/manual_control (x,y,z,r) dizisini üretir.
#
# ROS spin YOK · FC bağlantısı YOK · ARM YOK · motor YOK · MAVLink yazması YOK.
# request_arm/request_mode yalnızca yerelde taklit edilir; hiçbir yere komut gitmez.
# Amaç: durum makinesinin ürettiği komut vektörlerinin yön/işaretini gözle doğrulamak.
#
# Eksen sözleşmesi (primitives.py): x,y,r -1000..1000 · z 0..1000 (500 nötr)
#   x+ = ileri · z<500 = dal · z>500 = yüzey/yukarı · r+ = sağa dönüş (cw)
#
# Kullanım (Jetson'da, ROS ortamı source'lu):
#   source /opt/ros/humble/setup.bash
#   source ~/auv/ros2_ws/install/setup.bash
#   python3 scripts/faz3_mission_dryrun.py
# =============================================================================
import math

from auv_mission.primitives import build_steps
from auv_mission.mission_node import BUILTIN_VIDEO_PATTERN


class FakeTable:
    """SpeedTable taklidi: speed_mps -> gaz (0..1000) basit lineer."""
    def throttle_for(self, v): return min(1000.0, abs(v) * 1000.0)
    def speed(self, t): return abs(t) / 1000.0


class Ctx:
    """StepContext taklidi — ROS'a bağlanmaz, yalnız yerel durum tutar."""
    def __init__(self):
        self.cfg = {'heading_kp': 12.0, 'heading_kd': 4.0, 'max_yaw_cmd': 400,
                    'depth_kp': 500.0, 'max_z_offset': 250, 'yaw_rate_full_dps': 60.0}
        self.table = FakeTable()
        self.targets = {}
        self.target_heading_deg = 0.0
        self.yaw_deg = 0.0
        self.depth_m = 0.0
        self.altitude_m = 5.0
        self.altitude_valid = True
        self.armed = False
        self.mode = 'MANUAL'
        self.dr = None
        self.lane_cmd = None
        self.lane_status = None
        self.message = ''
        self.out = (0, 0, 500, 0)
        self._arm_pending = None

    def set_output(self, x, y, z, r): self.out = (int(x), int(y), int(z), int(r))

    def request_mode(self, name):
        # ARM etmez; yalnız yerel mod durumunu günceller (SetModeStep ilerlesin)
        self.mode = 'ALT_HOLD' if name.upper() == 'DEPTH_HOLD' else name.upper()

    def request_arm(self, arm):
        # GERÇEK ARM DEĞİL — 0.3 s sonra 'armed' olduğunu taklit eder (ArmStep ilerlesin)
        self._arm_pending = (bool(arm), 0.0)

    def capture_origin(self): pass


def plant(ctx, dt):
    """İdealize araç tepkisi — komut vektörünü telemetriye çevirir (sadece sim)."""
    if ctx._arm_pending is not None:
        want, t = ctx._arm_pending
        t += dt
        if t >= 0.3:
            ctx.armed = want
            ctx._arm_pending = None
        else:
            ctx._arm_pending = (want, t)
    x, y, z, r = ctx.out
    vz = (500 - z) / 250.0 * 0.6           # z<500 -> dal (derinlik artar)
    ctx.depth_m = max(0.0, ctx.depth_m + vz * dt)
    dyaw = (r / 1000.0) * ctx.cfg['yaw_rate_full_dps'] * dt   # r+ -> cw
    ctx.yaw_deg = (ctx.yaw_deg + dyaw) % 360.0


def fmt(o): return f"x={o[0]:+5d} y={o[1]:+5d} z={o[2]:4d} r={o[3]:+5d}"


def interp(o):
    x, y, z, r = o
    parts = []
    if x > 50: parts.append(f"ileri {x/10:.0f}%")
    elif x < -50: parts.append(f"geri {-x/10:.0f}%")
    if z < 450: parts.append(f"DAL (z-{500-z})")
    elif z > 550: parts.append(f"YÜKSEL (z+{z-500})")
    if r > 50: parts.append(f"sağa dön r+{r}")
    elif r < -50: parts.append(f"sola dön r{r}")
    if not parts: parts.append("nötr/tutma")
    return ", ".join(parts)


def run(step_list, dt=0.1, max_step=3000):
    steps = build_steps(step_list)
    ctx = Ctx()
    ctx.target_heading_deg = ctx.yaw_deg
    print(f"Adım sayısı: {len(steps)}\n")
    print(f"{'#':>2} {'ADIM':<16} {'süre(s)':>7}  {'komut (adım ortası)':<26}  yorum")
    print("-" * 92)
    gcnt = 0
    for i, step in enumerate(steps):
        step.enter(ctx)
        plant(ctx, dt)
        mid_out = ctx.out
        t = 0.0
        done = False
        guard = 0
        while not done and guard < max_step:
            done = step.update(ctx, dt)
            plant(ctx, dt)
            t += dt
            guard += 1
            gcnt += 1
            if guard == int(0.5 / dt):
                mid_out = ctx.out
        print(f"{i+1:>2} {step.name:<16} {t:>7.1f}  {fmt(mid_out):<26}  {interp(mid_out)}")
    print("-" * 92)
    print(f"toplam sim adımı: {gcnt}  (~{gcnt*dt:.0f} s sanal görev süresi)")
    print(f"son durum: armed={ctx.armed} mode={ctx.mode} "
          f"depth={ctx.depth_m:.2f}m yaw={ctx.yaw_deg:.0f}°")
    print("\nNOT: Bu prova ROS/FC'ye BAĞLANMADI. Hiçbir ARM/motor/MAVLink komutu gönderilmedi.")


if __name__ == '__main__':
    print("=== FAZ 3 — Gömülü video deseni ARM'sız prova ===\n")
    run(BUILTIN_VIDEO_PATTERN)
