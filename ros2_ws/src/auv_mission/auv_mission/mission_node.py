#!/usr/bin/env python3
"""Gorev durum makinesi.

- /mav/manual_control'un TEK yazari bu node'dur (yetki cakismasi olmaz).
- Durumlar: IDLE / MANUAL / VIDEO_PATTERN / LANE_FOLLOW / AUTONOMOUS_NAV / ABORTED
- Gorevler config/missions/*.yaml dosyalarindan yuklenir; video_pattern
  dosya bulunamazsa gomulu varsayilanla calisir.
- Panel komutlari WebSocket (ws://<jetson>:8765) uzerinden gelir.

Onemli: IDLE/MANUAL durumlarinda bu node MANUAL_CONTROL YAYINLAMAZ;
QGC joystick dogrudan ucus kontrole gider. Gorev basladiginda pilot eli
cekilir; STOP ile her an geri alinabilir.
"""
import os
import time

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32, Bool, String, Float32MultiArray, UInt16MultiArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

from auv_msgs.msg import (ManualControl, MissionState, VerticalState,
                          DeadReckonState, LaneStatus)

from .primitives import build_steps, GotoTargetStep
from .ws_server import WsServer

try:
    from auv_dead_reckoning.speed_table import SpeedTable
except ImportError:  # bagimsiz test icin yedek
    class SpeedTable:
        def __init__(self, *_): ...
        def speed(self, t): return abs(t) / 1000.0
        def throttle_for(self, v): return min(1000.0, abs(v) * 1000.0)


STATE_NAMES = {
    MissionState.STATE_IDLE: 'IDLE',
    MissionState.STATE_MANUAL: 'MANUAL',
    MissionState.STATE_VIDEO_PATTERN: 'VIDEO_PATTERN',
    MissionState.STATE_LANE_FOLLOW: 'LANE_FOLLOW',
    MissionState.STATE_AUTONOMOUS_NAV: 'AUTONOMOUS_NAV',
    MissionState.STATE_ABORTED: 'ABORTED',
}
MISSION_STATE_BY_NAME = {
    'video_pattern': MissionState.STATE_VIDEO_PATTERN,
    'lane_follow': MissionState.STATE_LANE_FOLLOW,
    'task2_nav': MissionState.STATE_AUTONOMOUS_NAV,
}

# Dosya yoksa kullanilacak gomulu video deseni (sartname 2.4.3.3, Ileri Kategori)
BUILTIN_VIDEO_PATTERN = [
    {'type': 'countdown', 'seconds': 10},
    {'type': 'arm'},
    {'type': 'set_mode', 'mode': 'ALT_HOLD'},
    {'type': 'set_depth', 'depth_m': 0.5},
    # 1. ve 4. bacaklar daire kiris kaymasini telafi icin uzun (bkz. YAML)
    {'type': 'drive', 'seconds': 17.5, 'speed': 0.4},
    {'type': 'turn', 'delta_deg': 90},
    {'type': 'drive', 'seconds': 16, 'speed': 0.4},
    # 1.25 tur = "en az 1 tur" + rotayi kapatan 90 derece sag donus.
    # forward/yaw_rate orani daire capini belirler (>= 1 m sart) - havuzda olc!
    {'type': 'spin', 'turns': 1.25, 'yaw_rate': 0.25, 'forward': 0.20},
    {'type': 'drive', 'seconds': 16, 'speed': 0.4},
    {'type': 'turn', 'delta_deg': 90},
    {'type': 'drive', 'seconds': 17.5, 'speed': 0.4},
    {'type': 'wait', 'seconds': 2},
    {'type': 'surface'},
    {'type': 'disarm'},
]


class StepContext:
    """Adimlarin gordugu arayuz (primitives.py dokumantasyonuna bakin)."""

    def __init__(self, node):
        self._node = node
        self.cfg = {}
        self.table = None
        self.targets = {}
        self.target_heading_deg = 0.0
        # telemetri (node gunceller)
        self.yaw_deg = 0.0
        self.depth_m = 0.0
        self.altitude_m = 0.0
        self.altitude_valid = False
        self.armed = False
        self.mode = ''
        self.dr = None
        self.lane_cmd = None
        self.lane_status = None
        self.message = ''
        # cikti
        self.out = (0, 0, 500, 0)

    def set_output(self, x, y, z, r):
        self.out = (int(x), int(y), int(z), int(r))

    def request_mode(self, name):
        self._node.pub_mode.publish(String(data=name))

    def request_arm(self, arm):
        self._node.pub_arm.publish(Bool(data=bool(arm)))

    def capture_origin(self):
        self._node.call_capture_origin()


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('missions_dir', '')
        self.declare_parameter('speed_table_csv', '')
        self.declare_parameter('ws_port', 8765)
        # kontrol kazanclari (havuzda ayarlanacak)
        self.declare_parameter('heading_kp', 12.0)     # deg basina MANUAL r
        self.declare_parameter('heading_kd', 4.0)
        self.declare_parameter('max_yaw_cmd', 400)
        self.declare_parameter('depth_kp', 500.0)      # m basina z ofseti
        self.declare_parameter('max_z_offset', 250)
        self.declare_parameter('yaw_rate_full_dps', 60.0)  # r=1000'de deg/s (kalibre!)

        self.ctx = StepContext(self)
        self.ctx.cfg = {
            'heading_kp': float(self.get_parameter('heading_kp').value),
            'heading_kd': float(self.get_parameter('heading_kd').value),
            'max_yaw_cmd': int(self.get_parameter('max_yaw_cmd').value),
            'depth_kp': float(self.get_parameter('depth_kp').value),
            'max_z_offset': int(self.get_parameter('max_z_offset').value),
            'yaw_rate_full_dps': float(self.get_parameter('yaw_rate_full_dps').value),
        }
        csv_path = self.get_parameter('speed_table_csv').value
        self.ctx.table = SpeedTable(csv_path if csv_path else None)
        self.missions_dir = self.get_parameter('missions_dir').value

        # --- ROS arayuzu ---
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_manual = self.create_publisher(ManualControl, '/mav/manual_control', 10)
        self.pub_arm = self.create_publisher(Bool, '/mav/cmd/arm', 10)
        self.pub_mode = self.create_publisher(String, '/mav/cmd/mode', 10)
        self.pub_state = self.create_publisher(MissionState, '/mission/state', qos)
        self.pub_bias = self.create_publisher(Float32MultiArray, '/dr/set_current_bias', 10)
        self.pub_motor_test = self.create_publisher(UInt16MultiArray, '/mav/cmd/motor_test', 10)
        self.pub_direct = self.create_publisher(UInt16MultiArray, '/mav/cmd/direct_output', 10)

        self.create_subscription(Float32, '/mav/heading_deg', self._s_hdg, qos)
        self.create_subscription(VerticalState, '/vertical_state', self._s_vert, qos)
        self.create_subscription(DeadReckonState, '/dr/state', self._s_dr, qos)
        self.create_subscription(Twist, '/lane/cmd_vel', self._s_lane_cmd, qos)
        self.create_subscription(LaneStatus, '/lane/status', self._s_lane_st, qos)
        self.create_subscription(Bool, '/mav/armed', self._s_armed, 10)
        self.create_subscription(String, '/mav/mode', self._s_mode, 10)
        self.create_subscription(UInt16MultiArray, '/mav/servo_out', self._s_servo, qos)
        self._servo = []   # son 8 motor PWM (motor gorsellestirmesi icin)

        self.cli_capture = self.create_client(Trigger, '/dr/capture_origin')

        # --- durum makinesi ---
        self.state = MissionState.STATE_IDLE
        self.steps = []
        self.step_i = 0
        self.step_t = 0.0
        self.mission_t = 0.0
        self._last_tick = time.monotonic()

        # --- WebSocket ---
        self.ws = WsServer(port=int(self.get_parameter('ws_port').value),
                           logger=self.get_logger())
        self.ws.start()

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(0.2, self.broadcast_state)  # 5 Hz panel akisi
        self.get_logger().info('Mission node hazir')

    # ---------- telemetri abonelikleri ----------
    def _s_hdg(self, m):  self.ctx.yaw_deg = m.data
    def _s_armed(self, m): self.ctx.armed = m.data
    def _s_mode(self, m): self.ctx.mode = m.data
    def _s_dr(self, m):   self.ctx.dr = m
    def _s_lane_cmd(self, m): self.ctx.lane_cmd = m
    def _s_lane_st(self, m):  self.ctx.lane_status = m.status
    def _s_servo(self, m):    self._servo = [int(v) for v in m.data]

    def _s_vert(self, m):
        self.ctx.depth_m = m.depth_m
        self.ctx.altitude_m = m.altitude_m
        self.ctx.altitude_valid = m.altitude_valid

    def call_capture_origin(self):
        if self.cli_capture.service_is_ready():
            self.cli_capture.call_async(Trigger.Request())
        else:
            self.get_logger().warn('/dr/capture_origin servisi hazir degil')

    # ---------- gorev yukleme ----------
    def load_mission(self, name: str):
        path = os.path.join(self.missions_dir, f'{name}.yaml') if self.missions_dir else ''
        if path and os.path.isfile(path):
            with open(path) as f:
                data = yaml.safe_load(f)
            step_list = data['steps'] if isinstance(data, dict) else data
            self.get_logger().info(f'Gorev dosyadan yuklendi: {path}')
        elif name == 'video_pattern':
            step_list = BUILTIN_VIDEO_PATTERN
            self.get_logger().warn('video_pattern.yaml yok - gomulu desen kullaniliyor')
        else:
            raise FileNotFoundError(f'Gorev bulunamadi: {name}')
        return build_steps(step_list)

    def list_missions(self):
        names = set()
        if self.missions_dir and os.path.isdir(self.missions_dir):
            for f in os.listdir(self.missions_dir):
                if f.endswith('.yaml'):
                    names.add(f[:-5])
        names.add('video_pattern')
        return sorted(names)

    # ---------- komut isleme ----------
    def handle_command(self, cmd: dict):
        c = cmd.get('cmd', '')
        try:
            if c == 'start_mission':
                self.start_mission(cmd.get('mission', 'video_pattern'))
            elif c == 'stop':
                self.abort('Panel STOP')
            elif c == 'arm':
                self.pub_arm.publish(Bool(data=True))
            elif c == 'disarm':
                self.pub_arm.publish(Bool(data=False))
            elif c == 'set_mode':
                self.pub_mode.publish(String(data=cmd.get('mode', 'MANUAL')))
            elif c == 'set_targets':
                self.ctx.targets['turn'] = (float(cmd['turn_lat']), float(cmd['turn_lon']))
                self.ctx.targets['finish'] = (float(cmd['finish_lat']), float(cmd['finish_lon']))
                self.ctx.message = 'Hedef koordinatlar alindi'
                self.get_logger().info(f'Hedefler: {self.ctx.targets}')
            elif c == 'goto':
                self.goto_waypoint(float(cmd['lat']), float(cmd['lon']),
                                   float(cmd.get('speed_mps', 0.5)),
                                   float(cmd.get('arrive_radius_m', 3.0)))
            elif c == 'set_current_bias':
                arr = Float32MultiArray()
                arr.data = [float(cmd.get('east_mps', 0.0)),
                            float(cmd.get('north_mps', 0.0))]
                self.pub_bias.publish(arr)
            elif c == 'capture_origin':
                self.call_capture_origin()
            elif c == 'motor_test':
                if self.state not in (MissionState.STATE_IDLE, MissionState.STATE_MANUAL):
                    self.ws.broadcast({'type': 'error',
                                       'message': 'Motor testi yalniz IDLE/MANUAL durumda'})
                else:
                    arr = UInt16MultiArray()
                    arr.data = [int(cmd.get('motor', 0)), int(cmd.get('throttle', 10)),
                                int(cmd.get('duration', 3))]
                    self.pub_motor_test.publish(arr)
            elif c == 'motor_test_stop':
                arr = UInt16MultiArray()
                arr.data = [0, 0, 0]     # motor 0 = hepsini durdur
                self.pub_motor_test.publish(arr)
            elif c == 'direct_output':
                # Dogrudan cikis modu: 8 motoru ayni anda sur (cooldown yok, arm gerekmez).
                if self.state not in (MissionState.STATE_IDLE, MissionState.STATE_MANUAL):
                    self.ws.broadcast({'type': 'error',
                                       'message': 'Dogrudan cikis yalniz IDLE/MANUAL durumda'})
                else:
                    pwms = cmd.get('pwms', [])
                    arr = UInt16MultiArray()
                    if pwms and len(pwms) >= 8:
                        arr.data = [1] + [max(1100, min(1900, int(p))) for p in pwms[:8]]
                    else:
                        arr.data = [0]   # gecersiz -> durdur
                    self.pub_direct.publish(arr)
            elif c == 'direct_output_stop':
                arr = UInt16MultiArray()
                arr.data = [0]
                self.pub_direct.publish(arr)
            elif c == 'list_missions':
                self.ws.broadcast({'type': 'missions', 'missions': self.list_missions()})
            else:
                self.ws.broadcast({'type': 'error', 'message': f'bilinmeyen komut: {c}'})
        except Exception as e:
            self.get_logger().error(f'Komut hatasi ({c}): {e}')
            self.ws.broadcast({'type': 'error', 'message': f'{c}: {e}'})

    def start_mission(self, name: str):
        if self.state not in (MissionState.STATE_IDLE, MissionState.STATE_MANUAL,
                              MissionState.STATE_ABORTED):
            self.ctx.message = 'Once STOP ile mevcut gorevi durdurun'
            return
        self.steps = self.load_mission(name)
        self.step_i = 0
        self.step_t = 0.0
        self.mission_t = 0.0
        self.ctx.message = f'Gorev basladi: {name}'
        self.ctx.target_heading_deg = self.ctx.yaw_deg  # mevcut pruva referans
        self.state = MISSION_STATE_BY_NAME.get(name, MissionState.STATE_VIDEO_PATTERN)
        self.steps[0].enter(self.ctx)
        self.get_logger().info(f'GOREV BASLADI: {name} ({len(self.steps)} adim)')

    def goto_waypoint(self, lat, lon, speed_mps=0.5, arrive_radius_m=3.0):
        """Panelden tek nokta hedefi: dinamik 1 adimli GotoTargetStep gorevi.

        On kosul: DR origin (GPS fix) yakalanmis olmali; degilse GotoTargetStep
        'HEDEF/DR yok' deyip hemen biter. Once ARM + ALT_HOLD + capture_origin
        (panel dugmeleri) yapilmali. STOP ile her an iptal.
        """
        if self.state not in (MissionState.STATE_IDLE, MissionState.STATE_MANUAL,
                               MissionState.STATE_ABORTED):
            self.ctx.message = 'Once STOP ile mevcut gorevi durdurun'
            return
        self.ctx.targets['wp'] = (lat, lon)
        self.steps = [GotoTargetStep(target='wp', arrive_radius_m=arrive_radius_m,
                                     speed_mps=speed_mps)]
        self.step_i = 0
        self.step_t = 0.0
        self.mission_t = 0.0
        self.ctx.target_heading_deg = self.ctx.yaw_deg
        self.ctx.message = f'Waypoint: {lat:.6f}, {lon:.6f}'
        self.state = MissionState.STATE_AUTONOMOUS_NAV
        self.steps[0].enter(self.ctx)
        self.get_logger().info(f'WAYPOINT git: {lat:.6f}, {lon:.6f}')

    def abort(self, reason: str):
        if self.state in (MissionState.STATE_IDLE, MissionState.STATE_MANUAL):
            return
        self.get_logger().warn(f'GOREV IPTAL: {reason}')
        self.ctx.message = f'IPTAL: {reason}'
        # notr cikis + bir sure yayinlamaya devam (guvenli teslim)
        self._publish_manual((0, 0, 500, 0))
        self.state = MissionState.STATE_ABORTED
        self.steps = []

    # ---------- ana dongu ----------
    def tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        # panel komutlari
        while not self.ws.inbox.empty():
            self.handle_command(self.ws.inbox.get())

        if self.state in (MissionState.STATE_IDLE, MissionState.STATE_MANUAL):
            return  # pilot QGC'den surer; biz sessiziz

        if self.state == MissionState.STATE_ABORTED:
            # kisa sure notr yayinla, sonra IDLE'a dus
            self._publish_manual((0, 0, 500, 0))
            self.mission_t += dt
            if self.mission_t > 2.0:
                self.state = MissionState.STATE_IDLE
                self.mission_t = 0.0
            return

        if not self.steps:
            self.state = MissionState.STATE_IDLE
            return

        self.step_t += dt
        self.mission_t += dt
        step = self.steps[self.step_i]
        done = False
        try:
            done = step.update(self.ctx, dt)
        except Exception as e:
            self.get_logger().error(f'Adim hatasi ({step.name}): {e}')
            self.abort(f'adim hatasi: {e}')
            return

        self._publish_manual(self.ctx.out)

        if done:
            step.exit(self.ctx)
            self.step_i += 1
            self.step_t = 0.0
            if self.step_i >= len(self.steps):
                self.get_logger().info('GOREV TAMAMLANDI')
                self.ctx.message = 'Gorev tamamlandi'
                self.state = MissionState.STATE_IDLE
                self.steps = []
            else:
                nxt = self.steps[self.step_i]
                self.get_logger().info(f'Adim {self.step_i + 1}/{len(self.steps)}: {nxt.name}')
                nxt.enter(self.ctx)

    def _publish_manual(self, out):
        m = ManualControl()
        m.header.stamp = self.get_clock().now().to_msg()
        m.x, m.y, m.z, m.r = out
        self.pub_manual.publish(m)

    # ---------- durum yayini ----------
    def broadcast_state(self):
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.state_name = STATE_NAMES.get(self.state, '?')
        msg.step_index = self.step_i
        msg.step_count = len(self.steps)
        msg.step_name = (self.steps[self.step_i].name
                         if self.steps and self.step_i < len(self.steps) else '')
        msg.step_elapsed_s = self.step_t
        msg.mission_elapsed_s = self.mission_t
        msg.message = self.ctx.message
        self.pub_state.publish(msg)

        dr = self.ctx.dr
        self.ws.broadcast({
            'type': 'state',
            'mission': {
                'state': int(self.state),
                'state_name': msg.state_name,
                'step_name': msg.step_name,
                'step_index': self.step_i,
                'step_count': len(self.steps),
                'step_elapsed_s': round(self.step_t, 1),
                'mission_elapsed_s': round(self.mission_t, 1),
                'message': self.ctx.message,
            },
            'telemetry': {
                'depth_m': round(self.ctx.depth_m, 2),
                'altitude_m': round(self.ctx.altitude_m, 2),
                'altitude_valid': self.ctx.altitude_valid,
                'heading_deg': round(self.ctx.yaw_deg, 1),
                'armed': self.ctx.armed,
                'mode': self.ctx.mode,
                'dr': None if dr is None else {
                    'origin_valid': dr.origin_valid,
                    'x_east_m': round(dr.x_east_m, 1),
                    'y_north_m': round(dr.y_north_m, 1),
                    'est_lat': dr.est_lat,
                    'est_lon': dr.est_lon,
                    'speed_mps': round(dr.speed_mps, 2),
                },
                'lane_status': self.ctx.lane_status,
                'motors': self._servo,
            },
        })


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
