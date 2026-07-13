"""Kontrol primitifleri (gorev adimlari).

Her adim Step arayuzunu uygular:
    enter(ctx)              adim baslarken bir kez
    update(ctx, dt) -> bool True donunce adim biter
    exit(ctx)               adim biterken bir kez

ctx (StepContext) mission_node tarafindan saglanir:
    okuma : yaw_deg, depth_m, altitude_m, dr (DeadReckonState|None),
            lane_cmd (Twist|None), lane_status (int|None),
            targets {'turn':(lat,lon), 'finish':(lat,lon)}, table (SpeedTable)
    yazma : set_output(x, y, z, r)  [MANUAL_CONTROL eksenleri]
            request_mode(name), request_arm(bool), capture_origin()
            target_heading_deg  (pruva tutma referansi; adimlararasi tasinir)
    ayar  : cfg sozlugu (kazanclar, sinirlar)

Tum eksenler MANUAL_CONTROL olceginde: x,y,r -1000..1000; z 0..1000 (500 notr).
Ucus kontrol DEPTH_HOLD modundayken derinligi/pruvayi kendi tutar; adimlar
yalnizca manevra bileseni ekler.
"""
import math
import time

from auv_msgs.msg import LaneStatus


def wrap180(a):
    """Aciyi -180..180 araligina sar."""
    return (a + 180.0) % 360.0 - 180.0


class HeadingPID:
    def __init__(self, kp, kd, max_out):
        self.kp, self.kd, self.max_out = kp, kd, max_out
        self.prev_err = None

    def reset(self):
        self.prev_err = None

    def update(self, target_deg, current_deg, dt):
        err = wrap180(target_deg - current_deg)
        derr = 0.0 if self.prev_err is None or dt <= 0 else (err - self.prev_err) / dt
        self.prev_err = err
        out = self.kp * err + self.kd * derr
        return max(-self.max_out, min(self.max_out, out)), err


class Step:
    name = 'step'

    def enter(self, ctx):
        pass

    def update(self, ctx, dt) -> bool:
        raise NotImplementedError

    def exit(self, ctx):
        pass


class WaitStep(Step):
    def __init__(self, seconds, name=None, announce=False):
        self.seconds = float(seconds)
        self.announce = announce
        self.name = name or f'bekle {seconds:.0f}s'
        self.t = 0.0

    def enter(self, ctx):
        self.t = 0.0
        ctx.set_output(0, 0, 500, 0)

    def update(self, ctx, dt):
        self.t += dt
        ctx.set_output(0, 0, 500, 0)
        if self.announce:
            remaining = max(0, int(self.seconds - self.t) + 1)
            ctx.message = f'Geri sayim: {remaining}'
        return self.t >= self.seconds


class ArmStep(Step):
    def __init__(self, arm=True):
        self.arm = bool(arm)
        self.name = 'arm' if arm else 'disarm'
        self.t = 0.0

    def enter(self, ctx):
        self.t = 0.0
        ctx.request_arm(self.arm)

    def update(self, ctx, dt):
        self.t += dt
        ctx.set_output(0, 0, 500, 0)
        if self.t > 1.0 and ctx.armed != self.arm:
            ctx.request_arm(self.arm)  # tekrar dene
            self.t = 0.0
        return ctx.armed == self.arm


class SetModeStep(Step):
    def __init__(self, mode):
        self.mode = mode.upper()
        self.name = f'mod {self.mode}'
        self.t = 0.0

    def enter(self, ctx):
        self.t = 0.0
        ctx.request_mode(self.mode)

    def update(self, ctx, dt):
        self.t += dt
        ctx.set_output(0, 0, 500, 0)
        want = 'ALT_HOLD' if self.mode == 'DEPTH_HOLD' else self.mode
        if ctx.mode == want:
            return True
        if self.t > 2.0:
            ctx.request_mode(self.mode)
            self.t = 0.0
        return False


class SetDepthStep(Step):
    def __init__(self, depth_m, tol_m=0.15, hold_s=2.0, timeout_s=30.0):
        self.target = float(depth_m)
        self.tol = float(tol_m)
        self.hold_s = float(hold_s)
        self.timeout = float(timeout_s)
        self.name = f'derinlik {self.target:.1f}m'
        self.ok_since = None
        self.t = 0.0

    def enter(self, ctx):
        self.ok_since = None
        self.t = 0.0

    def update(self, ctx, dt):
        self.t += dt
        err = self.target - ctx.depth_m  # + : daha derine inilmeli
        kp = ctx.cfg['depth_kp']
        z = 500 - max(-ctx.cfg['max_z_offset'],
                      min(ctx.cfg['max_z_offset'], kp * err))
        ctx.set_output(0, 0, int(z), 0)

        if abs(err) < self.tol:
            if self.ok_since is None:
                self.ok_since = self.t
            if self.t - self.ok_since >= self.hold_s:
                ctx.set_output(0, 0, 500, 0)  # DEPTH_HOLD devralir
                return True
        else:
            self.ok_since = None

        if self.t > self.timeout:
            ctx.message = f'derinlik zaman asimi (hata {err:+.2f} m), devam'
            ctx.set_output(0, 0, 500, 0)
            return True
        return False


class TurnStep(Step):
    """Sabit yerde donus: delta_deg (goreli, + = saga) veya heading_deg (mutlak)."""

    def __init__(self, delta_deg=None, heading_deg=None, tol_deg=3.0,
                 settle_s=1.0, timeout_s=30.0):
        self.delta = delta_deg
        self.absolute = heading_deg
        self.tol = float(tol_deg)
        self.settle = float(settle_s)
        self.timeout = float(timeout_s)
        self.name = (f'don {delta_deg:+.0f}deg' if delta_deg is not None
                     else f'pruva {heading_deg:.0f}deg')
        self.pid = None
        self.ok_since = None
        self.t = 0.0

    def enter(self, ctx):
        if self.absolute is not None:
            ctx.target_heading_deg = float(self.absolute) % 360.0
        else:
            ctx.target_heading_deg = (ctx.target_heading_deg + float(self.delta)) % 360.0
        self.pid = HeadingPID(ctx.cfg['heading_kp'], ctx.cfg['heading_kd'],
                              ctx.cfg['max_yaw_cmd'])
        self.ok_since = None
        self.t = 0.0

    def update(self, ctx, dt):
        self.t += dt
        r, err = self.pid.update(ctx.target_heading_deg, ctx.yaw_deg, dt)
        ctx.set_output(0, 0, 500, int(r))
        if abs(err) < self.tol:
            if self.ok_since is None:
                self.ok_since = self.t
            if self.t - self.ok_since >= self.settle:
                ctx.set_output(0, 0, 500, 0)
                return True
        else:
            self.ok_since = None
        if self.t > self.timeout:
            ctx.message = f'donus zaman asimi (hata {err:+.0f}deg), devam'
            return True
        return False


class DriveStep(Step):
    """Pruva tutarak duz gidis: seconds veya distance_m (DR gerektirir)."""

    def __init__(self, seconds=None, distance_m=None, speed=0.4, speed_mps=None):
        self.seconds = float(seconds) if seconds is not None else None
        self.distance = float(distance_m) if distance_m is not None else None
        self.speed_norm = float(speed)      # 0..1 -> x = 1000*speed
        self.speed_mps = speed_mps          # verilirse hiz tablosundan gaz
        if self.seconds is not None:
            self.name = f'duz {self.seconds:.0f}s'
        else:
            self.name = f'duz {self.distance:.0f}m'
        self.pid = None
        self.t = 0.0
        self.dist0 = 0.0

    def enter(self, ctx):
        self.pid = HeadingPID(ctx.cfg['heading_kp'], ctx.cfg['heading_kd'],
                              ctx.cfg['max_yaw_cmd'])
        self.t = 0.0
        self.dist0 = ctx.dr.distance_total_m if ctx.dr else 0.0

    def update(self, ctx, dt):
        self.t += dt
        if self.speed_mps is not None:
            x = int(ctx.table.throttle_for(self.speed_mps))
        else:
            x = int(1000 * self.speed_norm)
        r, _ = self.pid.update(ctx.target_heading_deg, ctx.yaw_deg, dt)
        ctx.set_output(x, 0, 500, int(r))

        if self.seconds is not None and self.t >= self.seconds:
            ctx.set_output(0, 0, 500, 0)
            return True
        if self.distance is not None and ctx.dr:
            if (ctx.dr.distance_total_m - self.dist0) >= self.distance:
                ctx.set_output(0, 0, 500, 0)
                return True
        return False


class SpinStep(Step):
    """Kendi etrafinda donus. turns=1.25 -> 450 derece (video deseninde kare
    rotayi kapatmak icin daire + 90 derece sag donus tek harekette yapilir;
    sartname 'en az 1 tur' istedigi icin 1.25 tur kurala uygundur).
    forward > 0 verilirse donus daire cizer (>=1 m cap garantisi)."""

    def __init__(self, turns=1.0, yaw_rate=0.25, forward=0.0, direction='cw',
                 timeout_s=90.0):
        self.turns = float(turns)
        self.yaw_cmd = abs(float(yaw_rate)) * 1000.0
        self.forward = float(forward)
        self.sign = 1.0 if direction == 'cw' else -1.0
        self.timeout = float(timeout_s)
        self.name = f'daire {self.turns:.2f} tur'
        self.acc = 0.0
        self.prev_yaw = None
        self.t = 0.0

    def enter(self, ctx):
        self.acc = 0.0
        self.prev_yaw = ctx.yaw_deg
        self.t = 0.0

    def update(self, ctx, dt):
        self.t += dt
        d = wrap180(ctx.yaw_deg - self.prev_yaw)
        self.prev_yaw = ctx.yaw_deg
        self.acc += self.sign * d  # cw'de pozitif birikir

        ctx.set_output(int(1000 * self.forward), 0, 500, int(self.sign * self.yaw_cmd))

        target = self.turns * 360.0
        if self.acc >= target or self.t > self.timeout:
            if self.t > self.timeout:
                ctx.message = 'daire zaman asimi, devam'
            # yeni pruva referansi: baslangic + donulen aci
            ctx.target_heading_deg = (ctx.target_heading_deg
                                      + self.sign * target) % 360.0
            ctx.set_output(0, 0, 500, 0)
            return True
        return False


class OrbitStep(Step):
    """Tahmini nokta etrafinda yaricapli tur (Gorev 2 samandira donusu).

    Araci noktaya goturmek GotoTargetStep'in isi; bu adim mevcut konumdan
    R yaricapli cember cizer: x = sabit hiz, r = v/R acisal hizi.
    """

    def __init__(self, radius_m=12.0, speed_mps=0.5, direction='cw', timeout_s=240.0):
        self.radius = float(radius_m)
        self.speed = float(speed_mps)
        self.sign = 1.0 if direction == 'cw' else -1.0
        self.timeout = float(timeout_s)
        self.name = f'orbit R={self.radius:.0f}m'
        self.acc = 0.0
        self.prev_yaw = None
        self.t = 0.0

    def enter(self, ctx):
        self.acc = 0.0
        self.prev_yaw = ctx.yaw_deg
        self.t = 0.0

    def update(self, ctx, dt):
        self.t += dt
        d = wrap180(ctx.yaw_deg - self.prev_yaw)
        self.prev_yaw = ctx.yaw_deg
        self.acc += self.sign * d

        x = int(ctx.table.throttle_for(self.speed))
        omega_dps = math.degrees(self.speed / self.radius)
        r = int(self.sign * (omega_dps / ctx.cfg['yaw_rate_full_dps']) * 1000.0)
        r = max(-ctx.cfg['max_yaw_cmd'], min(ctx.cfg['max_yaw_cmd'], r))
        ctx.set_output(x, 0, 500, r)

        if self.acc >= 360.0 or self.t > self.timeout:
            if self.t > self.timeout:
                ctx.message = 'orbit zaman asimi, devam'
            ctx.target_heading_deg = ctx.yaw_deg
            ctx.set_output(0, 0, 500, 0)
            return True
        return False


class CaptureOriginStep(Step):
    """Yuzeyde GPS origin fix'i al (DR node'una servis cagrisi)."""

    def __init__(self, timeout_s=120.0):
        self.timeout = float(timeout_s)
        self.name = 'GPS fix al'
        self.t = 0.0
        self.requested = False

    def enter(self, ctx):
        self.t = 0.0
        self.requested = False

    def update(self, ctx, dt):
        self.t += dt
        ctx.set_output(0, 0, 500, 0)
        if not self.requested:
            ctx.capture_origin()
            self.requested = True
            return False
        if ctx.dr and ctx.dr.origin_valid:
            ctx.message = f'Origin: {ctx.dr.origin_lat:.6f}, {ctx.dr.origin_lon:.6f}'
            return True
        if self.t > self.timeout:
            ctx.message = 'GPS fix zaman asimi! Gorev DR origin OLMADAN devam ediyor'
            return True
        return False


class GotoTargetStep(Step):
    """DR konumundan hedefe surekli kerteriz duzelterek gidis."""

    def __init__(self, target='turn', arrive_radius_m=5.0, speed_mps=0.6,
                 timeout_s=240.0):
        self.target_key = target
        self.arrive = float(arrive_radius_m)
        self.speed = float(speed_mps)
        self.timeout = float(timeout_s)
        self.name = f'hedefe git: {target}'
        self.pid = None
        self.t = 0.0

    def enter(self, ctx):
        self.pid = HeadingPID(ctx.cfg['heading_kp'], ctx.cfg['heading_kd'],
                              ctx.cfg['max_yaw_cmd'])
        self.t = 0.0

    def update(self, ctx, dt):
        self.t += dt
        tgt = ctx.targets.get(self.target_key)
        if tgt is None or ctx.dr is None or not ctx.dr.origin_valid:
            ctx.message = f'HEDEF/DR yok ({self.target_key}) - adim atlaniyor'
            ctx.set_output(0, 0, 500, 0)
            return True

        from .geo_local import bearing_distance  # dairesel import onlemek icin yerel
        brg, dist = bearing_distance(ctx.dr.est_lat, ctx.dr.est_lon, tgt[0], tgt[1])

        if dist <= self.arrive:
            ctx.message = f'{self.target_key} hedefine varildi (kalan {dist:.1f} m)'
            ctx.target_heading_deg = ctx.yaw_deg
            ctx.set_output(0, 0, 500, 0)
            return True

        ctx.target_heading_deg = brg
        x = int(ctx.table.throttle_for(self.speed))
        r, err = self.pid.update(brg, ctx.yaw_deg, dt)
        # buyuk pruva hatasinda once don, sonra ilerle
        if abs(err) > 45.0:
            x = 0
        ctx.set_output(x, 0, 500, int(r))

        if self.t > self.timeout:
            ctx.message = f'{self.target_key} zaman asimi (kalan {dist:.0f} m), devam'
            ctx.set_output(0, 0, 500, 0)
            return True
        return False


class SurfaceStep(Step):
    def __init__(self, surface_depth_m=0.3, timeout_s=60.0):
        self.limit = float(surface_depth_m)
        self.timeout = float(timeout_s)
        self.name = 'yuzeye cik'
        self.t = 0.0

    def enter(self, ctx):
        self.t = 0.0

    def update(self, ctx, dt):
        self.t += dt
        if ctx.depth_m <= self.limit or self.t > self.timeout:
            ctx.set_output(0, 0, 500, 0)
            return True
        ctx.set_output(0, 0, 500 + ctx.cfg['max_z_offset'], 0)
        return False


class LaneFollowStep(Step):
    """/lane/cmd_vel onerisini MANUAL_CONTROL'e cevirir.

    END_OF_LINE veya LOST'ta biter; mission node ardindan MANUAL'e gecer
    (tahta sonunda pilot Mini ROV gorevine baslar).
    Minimum dip yuksekligi korumasi: altitude < min ise yukari kacinma.
    """

    def __init__(self, timeout_s=300.0, min_altitude_m=0.5):
        self.timeout = float(timeout_s)
        self.min_alt = float(min_altitude_m)
        self.name = 'serit takibi'
        self.t = 0.0

    def enter(self, ctx):
        self.t = 0.0

    def update(self, ctx, dt):
        self.t += dt

        if ctx.lane_status == LaneStatus.END_OF_LINE:
            ctx.message = 'Tahta sonu - manuele geciliyor'
            ctx.set_output(0, 0, 500, 0)
            return True
        if ctx.lane_status == LaneStatus.LOST:
            ctx.message = 'Serit KAYIP - manuele geciliyor'
            ctx.set_output(0, 0, 500, 0)
            return True
        if self.t > self.timeout:
            ctx.message = 'Serit takibi zaman asimi'
            ctx.set_output(0, 0, 500, 0)
            return True

        x, r = 0, 0
        if ctx.lane_cmd is not None:
            x = int(1000 * max(0.0, min(1.0, ctx.lane_cmd.linear.x)))
            r = int(ctx.cfg['max_yaw_cmd'] * max(-1.0, min(1.0, ctx.lane_cmd.angular.z)))

        z = 500
        if ctx.altitude_valid and ctx.altitude_m < self.min_alt:
            z = 500 + ctx.cfg['max_z_offset'] // 2  # tabana yaklasildi: yukari
        ctx.set_output(x, 0, z, r)
        return False


# YAML adim tipi -> sinif eslesmesi
STEP_TYPES = {
    'wait': lambda p: WaitStep(p.get('seconds', 1.0)),
    'countdown': lambda p: WaitStep(p.get('seconds', 10.0), name='geri sayim',
                                    announce=True),
    'arm': lambda p: ArmStep(True),
    'disarm': lambda p: ArmStep(False),
    'set_mode': lambda p: SetModeStep(p.get('mode', 'ALT_HOLD')),
    'set_depth': lambda p: SetDepthStep(p.get('depth_m', 0.5),
                                        p.get('tol_m', 0.15),
                                        p.get('hold_s', 2.0),
                                        p.get('timeout_s', 30.0)),
    'turn': lambda p: TurnStep(p.get('delta_deg'), p.get('heading_deg'),
                               p.get('tol_deg', 3.0), p.get('settle_s', 1.0),
                               p.get('timeout_s', 30.0)),
    'drive': lambda p: DriveStep(p.get('seconds'), p.get('distance_m'),
                                 p.get('speed', 0.4), p.get('speed_mps')),
    'spin': lambda p: SpinStep(p.get('turns', 1.0), p.get('yaw_rate', 0.25),
                               p.get('forward', 0.0), p.get('direction', 'cw'),
                               p.get('timeout_s', 90.0)),
    'orbit': lambda p: OrbitStep(p.get('radius_m', 12.0), p.get('speed_mps', 0.5),
                                 p.get('direction', 'cw'), p.get('timeout_s', 240.0)),
    'capture_origin': lambda p: CaptureOriginStep(p.get('timeout_s', 120.0)),
    'goto_target': lambda p: GotoTargetStep(p.get('target', 'turn'),
                                            p.get('arrive_radius_m', 5.0),
                                            p.get('speed_mps', 0.6),
                                            p.get('timeout_s', 240.0)),
    'surface': lambda p: SurfaceStep(p.get('surface_depth_m', 0.3),
                                     p.get('timeout_s', 60.0)),
    'lane_follow': lambda p: LaneFollowStep(p.get('timeout_s', 300.0),
                                            p.get('min_altitude_m', 0.5)),
}


def build_steps(step_list):
    """YAML'dan gelen [{type: ..., ...}, ...] listesini Step nesnelerine cevir."""
    steps = []
    for s in step_list:
        t = s.get('type')
        if t not in STEP_TYPES:
            raise ValueError(f'Bilinmeyen adim tipi: {t}')
        steps.append(STEP_TYPES[t](s))
    return steps
