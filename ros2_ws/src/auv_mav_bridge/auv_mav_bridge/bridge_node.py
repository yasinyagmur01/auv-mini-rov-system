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

Abonelikler:
  /mav/manual_control  auv_msgs/ManualControl  (10 Hz ile FC'ye MANUAL_CONTROL)
  /mav/cmd/arm         std_msgs/Bool
  /mav/cmd/mode        std_msgs/String         (MANUAL, ALT_HOLD, STABILIZE...)
  /vertical_state      auv_msgs/VerticalState  (QGC'ye NAMED_VALUE_FLOAT)
  /mission/state       auv_msgs/MissionState   (QGC'ye NAMED_VALUE_FLOAT + STATUSTEXT)
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import Range, NavSatFix, NavSatStatus, BatteryState
from geometry_msgs.msg import Vector3Stamped

from auv_msgs.msg import ManualControl, VerticalState, MissionState, GpsInfo

from pymavlink import mavutil

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

        self.url = self.get_parameter('connection_url').value
        self.target = int(self.get_parameter('target_system').value)
        self.manual_timeout = float(self.get_parameter('manual_timeout_s').value)
        self.depth_source = self.get_parameter('depth_source').value
        self.rho = float(self.get_parameter('water_density_kgm3').value)
        self.p0_hpa = float(self.get_parameter('surface_pressure_hpa').value)

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

        self.create_subscription(ManualControl, '/mav/manual_control', self.on_manual, 10)
        self.create_subscription(Bool, '/mav/cmd/arm', self.on_arm, 10)
        self.create_subscription(String, '/mav/cmd/mode', self.on_mode, 10)
        self.create_subscription(VerticalState, '/vertical_state', self.on_vertical, qos)
        self.create_subscription(MissionState, '/mission/state', self.on_mission_state, qos)

        self._lock = threading.Lock()
        self._manual = (0, 0, 500, 0)
        self._manual_stamp = 0.0
        self._last_mission_state = -1

        self.get_logger().info(f'MAVLink baglantisi aciliyor: {self.url}')
        self.mav = mavutil.mavlink_connection(
            self.url, source_system=self.target, source_component=191)  # 191 = onboard computer

        self._rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self._rx_thread.start()

        rate = float(self.get_parameter('manual_rate_hz').value)
        self.create_timer(1.0 / rate, self.tx_manual)
        self.create_timer(1.0, self.tx_heartbeat)

    # ---------------- TX ----------------

    def tx_heartbeat(self):
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def tx_manual(self):
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
        self.pub_armed.publish(Bool(data=armed))
        self.pub_mode.publish(String(data=MODE_NAMES.get(m.custom_mode, f'#{m.custom_mode}')))

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
        if self.depth_source == 'pressure2':
            # press_abs hPa cinsinden; derinlik = dP / (rho*g)
            dp_pa = (m.press_abs - self.p0_hpa) * 100.0
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
