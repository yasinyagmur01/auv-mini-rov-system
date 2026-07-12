#!/usr/bin/env python3
"""pymavlink <-> ROS 2 koprusu.

mavlink-router'in 127.0.0.1:14551 sunucu ucuna baglanir (udpout).
MAVROS bilerek kullanilmiyor: ArduSub'da GUIDED/SET_POSITION_TARGET su alti
konum kestirimi olmadan calismadigi icin ihtiyacimiz olan mesaj seti kucuk
ve tamamen bizim kontrolumuzde olmali.

Yayinlar:
  /mav/attitude      geometry_msgs/Vector3Stamped  (x=roll, y=pitch, z=yaw [rad])
  /mav/heading_deg   std_msgs/Float32              (0-360, 0=Kuzey)
  /mav/depth         std_msgs/Float32              (m, asagi pozitif)
  /mav/rangefinder   sensor_msgs/Range             (Ping sonar, m)
  /mav/gps           sensor_msgs/NavSatFix
  /mav/gps_info      auv_msgs/GpsInfo
  /mav/battery       sensor_msgs/BatteryState
  /mav/armed         std_msgs/Bool
  /mav/mode          std_msgs/String
  /mav/statustext    std_msgs/String
  /mav/servo_out     std_msgs/UInt16MultiArray     (8 motor PWM us)

Abonelikler:
  /mav/manual_control  auv_msgs/ManualControl  (10 Hz ile FC'ye MANUAL_CONTROL)
  /mav/cmd/arm         std_msgs/Bool
  /mav/cmd/mode        std_msgs/String         (MANUAL, ALT_HOLD, STABILIZE...)
  /vertical_state      auv_msgs/VerticalState  (QGC'ye NAMED_VALUE_FLOAT)
  /mission/state       auv_msgs/MissionState   (QGC'ye NAMED_VALUE_FLOAT + STATUSTEXT)
"""
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32, Bool, String, UInt16MultiArray
from sensor_msgs.msg import Range, NavSatFix, NavSatStatus, BatteryState
from geometry_msgs.msg import Vector3Stamped

from auv_msgs.msg import ManualControl, VerticalState, MissionState, GpsInfo

from pymavlink import mavutil

# --- pymavlink "instance-cache" bug gecici cozumu ---
# Instance alanli bazi mesajlarda ( or. bir DISTANCE_SENSOR/ESC vb.) pymavlink'in modul
# seviyesindeki add_message()'i iceride msg._instances None olunca
# "'NoneType' object does not support item assignment" firlatiyor. Bu, recv dongusunu
# her ~0.5s'de bir bogup telemetriyi kesiyor. Orijinali sarmalayip hatada mesaji
# instance-cache olmadan sakliyoruz (recv_match yine tipe gore eslesip donduruyor).
if hasattr(mavutil, 'add_message'):
    _orig_add_message = mavutil.add_message
    def _safe_add_message(messages, mtype, msg):
        try:
            _orig_add_message(messages, mtype, msg)
        except (TypeError, AttributeError):
            messages[mtype] = msg
    mavutil.add_message = _safe_add_message

# ArduSub ucus modlari (custom_mode)
ARDUSUB_MODES = {
    'STABILIZE': 0,
    'ACRO': 1,
    'ALT_HOLD': 2,      # DEPTH_HOLD
    'AUTO': 3,
    'GUIDED': 4,
    'CIRCLE': 7,
    'SURFACE': 9,
    'POSHOLD': 16,
    'MANUAL': 19,
    'MOTOR_DETECT': 20,
}
MODE_NAMES = {v: k for k, v in ARDUSUB_MODES.items()}


class MavBridge(Node):
    def __init__(self):
        super().__init__('mav_bridge')

        self.declare_parameter('connection_url', 'udpout:127.0.0.1:14551')
        self.declare_parameter('target_system', 1)
        self.declare_parameter('manual_rate_hz', 10.0)
        self.declare_parameter('manual_timeout_s', 1.0)
        # 'vfr_hud': VFR_HUD.alt (EKF, negatif=su alti) | 'pressure2': SCALED_PRESSURE2'den hesap
        self.declare_parameter('depth_source', 'vfr_hud')
        self.declare_parameter('water_density_kgm3', 997.0)   # deniz: 1024.0
        self.declare_parameter('surface_pressure_hpa', 1013.25)
        # 'pressure2' modunda ilk okumayi yuzey referansi al (rakim/hava basincindan
        # bagimsiz; derinlik acilista 0, basinc artinca hemen yukselir).
        self.declare_parameter('auto_zero_surface', True)

        self.url = self.get_parameter('connection_url').value
        self.target = int(self.get_parameter('target_system').value)
        self.manual_timeout = float(self.get_parameter('manual_timeout_s').value)
        self.depth_source = self.get_parameter('depth_source').value
        self.rho = float(self.get_parameter('water_density_kgm3').value)
        self.p0_hpa = float(self.get_parameter('surface_pressure_hpa').value)
        self.auto_zero = bool(self.get_parameter('auto_zero_surface').value)
        self._p0_captured = None   # auto-zero: ilk Bar30 okumasi

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.pub_att = self.create_publisher(Vector3Stamped, '/mav/attitude', qos)
        self.pub_hdg = self.create_publisher(Float32, '/mav/heading_deg', qos)
        self.pub_depth = self.create_publisher(Float32, '/mav/depth', qos)
        self.pub_range = self.create_publisher(Range, '/mav/rangefinder', qos)
        self.pub_gps = self.create_publisher(NavSatFix, '/mav/gps', qos)
        self.pub_gpsinfo = self.create_publisher(GpsInfo, '/mav/gps_info', qos)
        self.pub_batt = self.create_publisher(BatteryState, '/mav/battery', qos)
        self.pub_armed = self.create_publisher(Bool, '/mav/armed', 10)
        self.pub_mode = self.create_publisher(String, '/mav/mode', 10)
        self.pub_stext = self.create_publisher(String, '/mav/statustext', 10)
        # 8 motor PWM (us) - motor yerlesim/itki gorsellestirmesi icin
        self.pub_servo = self.create_publisher(UInt16MultiArray, '/mav/servo_out', qos)

        self.create_subscription(ManualControl, '/mav/manual_control', self.on_manual, 10)
        self.create_subscription(Bool, '/mav/cmd/arm', self.on_arm, 10)
        self.create_subscription(String, '/mav/cmd/mode', self.on_mode, 10)
        self.create_subscription(UInt16MultiArray, '/mav/cmd/motor_test', self.on_motor_test, 10)
        self.create_subscription(UInt16MultiArray, '/mav/cmd/direct_output', self.on_direct_output, 10)
        self.create_subscription(VerticalState, '/vertical_state', self.on_vertical, qos)
        self.create_subscription(MissionState, '/mission/state', self.on_mission_state, qos)

        self._lock = threading.Lock()
        self._manual = (0, 0, 500, 0)
        self._manual_stamp = 0.0
        self._last_mission_state = -1
        # Motor testi durumu (20Hz tekrarli, order=BOARD). Bkz. _mt_tick.
        # ArduSub motor test sira no 0-TABANLI: seq k -> SERVO cikis (k+1).
        # UI "Motor N" (1..8) -> seq (N-1) -> fiziksel SERVO cikis N. (canli dogrulandi)
        self._armed = False
        self._mt_active = False
        self._mt_started = False       # FC arm onayi beklendi mi (arm yarisi bug'i icin)
        self._mt_seq = 0
        self._mt_pwm = 1500
        self._mt_duration = 0.0
        self._mt_arm_time = 0.0
        self._mt_until = 0.0
        # Dogrudan cikis modu: DO_SET_SERVO ile 8 motoru ayni anda sur (motor fonksiyonu
        # gecici 0). arm gerekmez, cooldown yok. Cikista SERVO1-8_FUNCTION=33-40 geri yuklenir.
        self._do_active = False
        self._do_pwms = [1500] * 8
        self._do_last_cmd = 0.0        # watchdog: istemci koparsa motorlar takili kalmasin
        self._do_marker = os.path.expanduser('~/auv/.direct_mode_active')
        self._do_restore_timer = None

        self.get_logger().info(f'MAVLink baglantisi aciliyor: {self.url}')
        self.mav = mavutil.mavlink_connection(
            self.url, source_system=self.target, source_component=191)  # 191 = onboard computer

        self._rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self._rx_thread.start()

        rate = float(self.get_parameter('manual_rate_hz').value)
        self.create_timer(1.0 / rate, self.tx_manual)
        self.create_timer(1.0, self.tx_heartbeat)
        self.create_timer(0.05, self._mt_tick)   # 20Hz motor test tekrari (watchdog beslemesi)
        self.create_timer(0.2, self._do_tick)    # 5Hz dogrudan cikis DO_SET_SERVO tekrari
        # Cokme-guvenligi: onceki oturum dogrudan modda cakildiysa fonksiyonlari geri yukle
        if os.path.exists(self._do_marker):
            self.get_logger().warn('Onceki oturum dogrudan cikis modunda kalmis; 3s sonra SERVO fonksiyonlari geri yuklenecek')
            self._do_restore_timer = self.create_timer(3.0, self._do_startup_restore)

    # ---------------- TX ----------------

    def _fc_engaged(self):
        """Bridge FC seri portuna YALNIZ aktif kullanimda yazsin. Bosta (gorev IDLE +
        yakin zamanda gercek manuel girdi yok) sessiz kalir; boylece QGC seri portu tek
        basina kullanir ve motor test/arm calismasi disaridan bozulmaz.

        Kok neden: naif mav_router iki yazari (QGC 14550 + bridge 14551) ayni FC seri
        portuna koyunca, bridge'in surekli 10Hz MANUAL_CONTROL + 1Hz HEARTBEAT trafigi
        QGC'nin 20Hz DO_MOTOR_TEST akisini bozup ~5s icinde OTO-DISARM'a yol aciyordu.
        Bosta yazmayi kesince QGC Motors Setup sayfasi ROS2 yigini ile ayni anda calisir."""
        if self._mt_active or self._do_active:
            return True
        if time.monotonic() - self._manual_stamp <= self.manual_timeout:
            return True  # web manuel surus: yakin zamanda gercek girdi var
        # otonom surus: gorev kendisi surer, MANUAL_CONTROL akisi pilot failsafe'ini besler
        return self._last_mission_state in (
            MissionState.STATE_VIDEO_PATTERN,
            MissionState.STATE_LANE_FOLLOW,
            MissionState.STATE_AUTONOMOUS_NAV)

    def tx_heartbeat(self):
        if not self._fc_engaged():
            return
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def tx_manual(self):
        if self._do_active:
            return   # dogrudan cikis modunda MANUAL_CONTROL gonderme (mikser DO_SET_SERVO'yu ezmesin)
        if not self._fc_engaged():
            return
        with self._lock:
            x, y, z, r = self._manual
            age = time.monotonic() - self._manual_stamp
        if age > self.manual_timeout:
            x, y, z, r = 0, 0, 500, 0  # notr: pilot girdisi failsafe'ini besler
        self.mav.mav.manual_control_send(self.target, x, y, z, r, 0)

    def on_manual(self, msg: ManualControl):
        with self._lock:
            self._manual = (
                max(-1000, min(1000, msg.x)),
                max(-1000, min(1000, msg.y)),
                max(0, min(1000, msg.z)),
                max(-1000, min(1000, msg.r)),
            )
            self._manual_stamp = time.monotonic()

    def on_arm(self, msg: Bool):
        self.mav.mav.command_long_send(
            self.target, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1.0 if msg.data else 0.0, 0, 0, 0, 0, 0, 0)
        self.get_logger().info('ARM komutu' if msg.data else 'DISARM komutu')

    def on_motor_test(self, msg: UInt16MultiArray):
        # [motor_no, throttle_pct, duration_s]. motor_no=0 -> DURDUR + DISARM.
        if self._do_active:
            self.get_logger().warn('Motor testi yok sayildi: once Dogrudan Cikis Modundan cik')
            return
        d = list(msg.data)
        motor = d[0] if len(d) > 0 else 0
        throttle = d[1] if len(d) > 1 else 10
        duration = d[2] if len(d) > 2 else 3
        if motor == 0:
            self._mt_active = False
            self._mt_until = 0.0
            self.mav.mav.command_long_send(
                self.target, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0.0, 0, 0, 0, 0, 0, 0)
            self.get_logger().info('Motor test STOP + DISARM')
            return
        # ArduSub her motor testi bitiste araci OTO-DISARM eder -> her testte yeniden ARM.
        self.mav.mav.command_long_send(
            self.target, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1.0, 0, 0, 0, 0, 0, 0)
        # cift yonlu ESC: 1500 notr; gaz% -> PWM = 1500 + %*4 (10% ~1540 ileri).
        self._mt_pwm = max(1000, min(2000, 1500 + int(throttle) * 4))
        self._mt_seq = max(0, int(motor) - 1)   # UI Motor N -> 0-tabanli seq (N-1) -> SERVO N
        self._mt_duration = float(duration)
        self._mt_active = True
        self._mt_started = False                # once FC gercekten ARM olsun (arm yarisi bug'i)
        self._mt_arm_time = time.monotonic()
        self.get_logger().info(
            f'Motor test BASLA: Motor {motor} (seq {self._mt_seq}) %{throttle} {duration}s '
            f'pwm={self._mt_pwm} (order=BOARD, ARM onayi bekleniyor)')

    def _mt_tick(self):
        # DO_MOTOR_TEST'i test suresince ~20Hz tekrarla. Tek atis gonderirsen ArduSub'in
        # 500ms watchdog'u "Motor test timed out!" deyip motoru durdurur ve DISARM eder.
        # KRITIK 1: y (param6 = MOTOR_TEST_ORDER) = 2 (BOARD) olmali; degilse FC test-tipine
        #   HIC bakmadan "bad test type" verip reddeder.
        # KRITIK 2: Once FC GERCEKTEN arm olmali. ARM'dan hemen sonra test gonderirsek FC daha
        #   arm olmadan reddeder ("Arm motors before testing") ve bu init-fail ArduSub'in 10s
        #   cooldown'unu tetikler -> motor hic donmez. Bu yuzden heartbeat'ten arm onayini bekle.
        if not self._mt_active:
            return
        now = time.monotonic()
        if not self._mt_started:
            if self._armed:
                self._mt_started = True
                self._mt_until = now + self._mt_duration   # arm onaylandi -> sureyi simdi baslat
            elif now - self._mt_arm_time > 4.0:
                self.get_logger().warn('Motor test iptal: FC ARM onayi 4s icinde gelmedi')
                self._mt_active = False
            return
        if now < self._mt_until:
            self.mav.mav.command_int_send(
                self.target, 1,
                mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                0, 0,                    # current, autocontinue
                float(self._mt_seq),     # p1: motor test sira no (0-tabanli, board sirasi)
                1.0,                     # p2: MOTOR_TEST_THROTTLE_PWM
                float(self._mt_pwm),     # p3: PWM (us)
                0.0,                     # p4: timeout (kullanilmiyor; watchdog'u biz besliyoruz)
                0,                       # x = p5 (motor sayisi)
                2,                       # y = p6 = MOTOR_TEST_ORDER_BOARD  <-- KRITIK DUZELTME
                0.0)                     # z = p7
        else:
            self._mt_active = False      # sure doldu: gonderimi durdur (FC motoru durdurur)

    # ---------------- Dogrudan cikis modu (8 motor ayni anda, cooldown yok) ----------------
    def _param_set(self, name, value):
        self.mav.mav.param_set_send(
            self.target, 1, name.encode(), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def _set_motor_funcs(self, disabled):
        # disabled=True -> SERVO1..8_FUNCTION=0 (motor degil, DO_SET_SERVO calisir)
        # disabled=False -> 33..40 (Motor1..Motor8) normal
        for ch in range(1, 9):
            self._param_set(f'SERVO{ch}_FUNCTION', 0 if disabled else (32 + ch))

    def _do_enter(self):
        self._mt_active = False           # motor testiyle cakismasin
        try:
            open(self._do_marker, 'w').close()
        except Exception:
            pass
        self._set_motor_funcs(True)
        self._do_active = True
        self.get_logger().warn('DOGRUDAN CIKIS MODU ACIK: SERVO1-8_FUNCTION=0 (motorlar mikserden cikti)')

    def _do_exit(self):
        self._do_pwms = [1500] * 8
        for _ in range(4):                # notr'a al
            for ch in range(1, 9):
                self.mav.mav.command_long_send(
                    self.target, 1, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                    float(ch), 1500.0, 0, 0, 0, 0, 0)
            time.sleep(0.05)
        self._set_motor_funcs(False)      # 33-40 geri yukle
        self._do_active = False
        try:
            if os.path.exists(self._do_marker):
                os.remove(self._do_marker)
        except Exception:
            pass
        self.get_logger().info('Dogrudan cikis modu KAPANDI; SERVO1-8_FUNCTION=33-40 geri yuklendi')

    def on_direct_output(self, msg: UInt16MultiArray):
        # data=[0] -> DURDUR/CIK ; data=[1, pwm1..pwm8] -> moda gir + o PWM'leri sur
        d = list(msg.data)
        mode = d[0] if len(d) > 0 else 0
        if mode == 0:
            if self._do_active:
                self._do_exit()
            return
        pwms = [max(1000, min(2000, int(v))) for v in d[1:9]]   # tam servo araligi (1900 ustu/1100 alti destekli)
        if len(pwms) < 8:
            return
        self._do_last_cmd = time.monotonic()   # watchdog besle
        if not self._do_active:
            self._do_enter()
        self._do_pwms = pwms

    def _do_tick(self):
        if not self._do_active:
            return
        # WATCHDOG: istemci (tarayici/script) komut gondermeyi kesince ~2s'de guvenli dur.
        # Motorlar son komutta TAKILI KALMASIN (WS kopmasi/cokme durumunda).
        if time.monotonic() - self._do_last_cmd > 2.0:
            self.get_logger().warn('Dogrudan cikis: komut zaman asimi (>2s) -> guvenli durus + geri yukleme')
            self._do_exit()
            return
        for ch in range(1, 9):
            self.mav.mav.command_long_send(
                self.target, 1, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                float(ch), float(self._do_pwms[ch - 1]), 0, 0, 0, 0, 0)

    def _do_startup_restore(self):
        # tek atislik: cokme sonrasi fonksiyonlari geri yukle
        if self._do_restore_timer is not None:
            self._do_restore_timer.cancel()
            self._do_restore_timer = None
        self._set_motor_funcs(False)
        try:
            if os.path.exists(self._do_marker):
                os.remove(self._do_marker)
        except Exception:
            pass
        self.get_logger().warn('Cokme-guvenligi: SERVO1-8_FUNCTION=33-40 geri yuklendi')

    def on_mode(self, msg: String):
        name = msg.data.strip().upper()
        if name == 'DEPTH_HOLD':
            name = 'ALT_HOLD'
        if name not in ARDUSUB_MODES:
            self.get_logger().error(f'Bilinmeyen mod: {name}')
            return
        self.mav.mav.set_mode_send(
            self.target,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            ARDUSUB_MODES[name])
        self.get_logger().info(f'Mod istegi: {name}')

    def _named_float(self, name: str, value: float):
        self.mav.mav.named_value_float_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            name.encode()[:10], float(value))

    def on_vertical(self, msg: VerticalState):
        # QGC MAVLink Inspector / grafiklerinde gorunur
        self._named_float('DEPTH', msg.depth_m)
        self._named_float('ALT_SB', msg.altitude_m)
        self._named_float('WCOL', msg.water_column_m)

    def on_mission_state(self, msg: MissionState):
        self._named_float('MSTATE', float(msg.state))
        self._named_float('MSTEP', float(msg.step_index))
        if msg.state != self._last_mission_state:
            self._last_mission_state = msg.state
            text = f'GOREV: {msg.state_name}'[:49]
            self.mav.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO, text.encode())

    # ---------------- RX ----------------

    def rx_loop(self):
        handlers = {
            'HEARTBEAT': self.h_heartbeat,
            'ATTITUDE': self.h_attitude,
            'VFR_HUD': self.h_vfr_hud,
            'SCALED_PRESSURE2': self.h_pressure2,
            'DISTANCE_SENSOR': self.h_distance,
            'GPS_RAW_INT': self.h_gps,
            'SYS_STATUS': self.h_sys_status,
            'STATUSTEXT': self.h_statustext,
            'SERVO_OUTPUT_RAW': self.h_servo,
            'COMMAND_ACK': self.h_cmd_ack,
        }
        while rclpy.ok():
            try:
                m = self.mav.recv_match(blocking=True, timeout=1.0)
            except Exception as e:  # baglanti kopmalarinda dongu devam etsin
                self.get_logger().warn(f'MAVLink rx hatasi: {e}')
                time.sleep(0.5)
                continue
            if m is None:
                continue
            # yalnizca hedef aracin (sysid) mesajlari
            if m.get_srcSystem() != self.target:
                continue
            h = handlers.get(m.get_type())
            if h:
                try:
                    h(m)
                except Exception as e:
                    self.get_logger().warn(f'{m.get_type()} isleme hatasi: {e}')

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def h_heartbeat(self, m):
        if m.get_srcComponent() != 1:  # yalniz otopilot
            return
        armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self._armed = armed
        self.pub_armed.publish(Bool(data=armed))
        self.pub_mode.publish(String(data=MODE_NAMES.get(m.custom_mode, f'#{m.custom_mode}')))

    def h_cmd_ack(self, m):
        # Motor test reddedilirse (result!=0) uyar. 20Hz tekrar oldugu icin
        # basarili (result=0) ACK'leri loglamayiz (spam olur).
        if m.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST and m.result != 0:
            self.get_logger().warn(f'Motor test ACK result={m.result} (0=ACCEPTED disi)')

    def h_attitude(self, m):
        v = Vector3Stamped()
        v.header.stamp = self._stamp()
        v.header.frame_id = 'base_link'
        v.vector.x, v.vector.y, v.vector.z = m.roll, m.pitch, m.yaw
        self.pub_att.publish(v)
        hdg = math.degrees(m.yaw) % 360.0
        self.pub_hdg.publish(Float32(data=hdg))

    def h_vfr_hud(self, m):
        if self.depth_source == 'vfr_hud':
            # ArduSub'da alt su alti icin negatiftir -> derinlik = -alt
            self.pub_depth.publish(Float32(data=max(0.0, -m.alt)))

    def h_pressure2(self, m):
        if self.depth_source != 'pressure2':
            return
        # press_abs hPa cinsinden; derinlik = dP / (rho*g)
        if self.auto_zero:
            if self._p0_captured is None:
                self._p0_captured = m.press_abs
                self.get_logger().info(
                    f'Bar30 yuzey referansi (auto-zero): {self._p0_captured:.2f} hPa')
            p0 = self._p0_captured
        else:
            p0 = self.p0_hpa
        dp_pa = (m.press_abs - p0) * 100.0
        depth = max(0.0, dp_pa / (self.rho * 9.80665))
        self.pub_depth.publish(Float32(data=depth))

    def h_distance(self, m):
        r = Range()
        r.header.stamp = self._stamp()
        r.header.frame_id = 'ping_sonar'
        r.radiation_type = Range.ULTRASOUND
        r.min_range = m.min_distance / 100.0
        r.max_range = m.max_distance / 100.0
        r.range = m.current_distance / 100.0
        self.pub_range.publish(r)

    def h_gps(self, m):
        fix = NavSatFix()
        fix.header.stamp = self._stamp()
        fix.header.frame_id = 'gps'
        fix.status.status = (NavSatStatus.STATUS_FIX if m.fix_type >= 3
                             else NavSatStatus.STATUS_NO_FIX)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = m.lat / 1e7
        fix.longitude = m.lon / 1e7
        fix.altitude = m.alt / 1000.0
        self.pub_gps.publish(fix)

        info = GpsInfo()
        info.header = fix.header
        info.fix_type = m.fix_type
        info.satellites = m.satellites_visible
        info.hdop = m.eph / 100.0 if m.eph != 65535 else float('nan')
        info.lat = fix.latitude
        info.lon = fix.longitude
        self.pub_gpsinfo.publish(info)

    def h_sys_status(self, m):
        b = BatteryState()
        b.header.stamp = self._stamp()
        b.voltage = m.voltage_battery / 1000.0
        b.current = m.current_battery / 100.0 if m.current_battery != -1 else float('nan')
        b.percentage = m.battery_remaining / 100.0 if m.battery_remaining != -1 else float('nan')
        b.present = True
        self.pub_batt.publish(b)

    def h_statustext(self, m):
        text = m.text if isinstance(m.text, str) else m.text.decode(errors='ignore')
        self.pub_stext.publish(String(data=text))
        self.get_logger().info(f'FC: {text}')

    def h_servo(self, m):
        # ArduSub MAIN OUT 1-8 PWM (us). Vectored-6DOF: 1-4 yatay, 5-8 dikey.
        arr = UInt16MultiArray()
        arr.data = [int(getattr(m, f'servo{i}_raw', 0)) for i in range(1, 9)]
        self.pub_servo.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = MavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
