#!/usr/bin/env python3
"""ROS2 -> Web koprusu (Jetson, internetsiz - Flask+rclpy+PIL).

rosbridge/web_video_server apt gerektiriyor ama Jetson'in interneti yok.
Bu kopru mevcut paketlerle ayni isi yapar: ROS2 topic'lerine abone olur,
goruntuleri MJPEG, sayisal verileri JSON olarak servis eder.

Veri akisi: ROS2 node'lari (d435, zed, mav_bridge) -> topic'ler ->
bu kopru (abone) -> web. Yani her sey ROS2'den gecer.

Portlar/uc noktalar (varsayilan :8000):
  /                       panel sayfasi
  /video/<ad>            MJPEG (ad: d435, zed, ...)
  /stream/color          d435 rengi (rsweb UYUMLU - PC paneli bozulmaz)
  /sensors               JSON (derinlik/sonar/pruva/roll/pitch/batarya + kamera durumu)

/sensors sozlesmesi, web/app.py MavReader.snapshot() ve web/templates/index.html
ile birebir ayni (drop-in): connected, depth_m, altitude_m, water_column_m,
roll, pitch, yaw, voltage, depth_valid, sonar_valid, cameras.

Calistirma (Jetson, ROS2 sourcelu):
  source /opt/ros/humble/setup.bash
  python3 ros2_web_bridge.py --port 8000

Donanimsiz onizleme (ROS2/Jetson gerekmez - Windows'ta da calisir):
  python3 ros2_web_bridge.py --demo
"""
import argparse
import io
import math
import os
import threading
import time

import numpy as np
from PIL import Image as PILImage
from PIL import ImageDraw
from flask import Flask, Response, jsonify, render_template_string, request

# Ham sensor_msgs/Image aboneleri (ad -> topic). D435 kaldirildi; su an bos.
IMAGE_TOPICS = {}
# Sikistirilmis (sensor_msgs/CompressedImage, JPEG) aboneleri.
# ZED 2i Docker konteynerinde: ham 720p kare (~2.7MB) FastDDS UDP ile
# konteyner->host'a gecmiyor (SHM sinirı); compressed (~30KB JPEG) gecer.
# Ayrica JPEG oldugu icin yeniden kodlama yok (dogrudan servis).
COMPRESSED_TOPICS = {
    'zed': '/zed/zed_node/rgb/color/rect/image/compressed',
}
CAM_NAMES = list(IMAGE_TOPICS) + list(COMPRESSED_TOPICS)
# Sensor topic'leri (V6X Jetson'a tasininca mav_bridge yayinlar - Faz 2)
DEPTH_TOPIC = '/mav/depth'            # std_msgs/Float32 (m)
RANGE_TOPIC = '/mav/rangefinder'      # sensor_msgs/Range (m)
HEADING_TOPIC = '/mav/heading_deg'    # std_msgs/Float32 (deg)
ATTITUDE_TOPIC = '/mav/attitude'      # geometry_msgs/Vector3Stamped (rad: x=roll,y=pitch,z=yaw)
BATTERY_TOPIC = '/mav/battery'        # sensor_msgs/BatteryState (V, A, %)
# --- P0 guvenlik telemetrisi topic'leri (auv_mav_bridge yayinlar) ---
ARMED_TOPIC = '/mav/armed'            # std_msgs/Bool
MODE_TOPIC = '/mav/mode'              # std_msgs/String (ucus modu)
STATUSTEXT_TOPIC = '/mav/statustext'  # std_msgs/String (FC prearm/failsafe/leak metni)
LEAK_TOPIC = '/mav/leak'              # std_msgs/Bool (sizinti; True=ALARM, yok=bilinmiyor)
LINK_OK_TOPIC = '/mav/link_ok'        # std_msgs/Bool (AUV FC heartbeat tazeligi)
GPS_TOPIC = '/mav/gps'                # sensor_msgs/NavSatFix (lat/lon)
GPSINFO_TOPIC = '/mav/gps_info'       # auv_msgs/GpsInfo (fix_type, satellites, hdop)
WATERTEMP_TOPIC = '/mav/water_temp'   # std_msgs/Float32 (Bar30 su sicakligi, C)
BOARDTEMP_TOPIC = '/mav/board_temp'   # std_msgs/Float32 (CUAV ADC analog ic/kart sicakligi, C)
# ZED VO/odometri topic'i. VARSAYILAN BOŞ = kapalı (demo sentetik zed_pose kalir).
# ARAÇTA: `ros2 topic list | grep zed` ile gercek odom topic'ini bul (nav_msgs/Odometry;
# tipik: /zed/zed_node/odom) ve buraya yaz -> gercek VO izi /sensors.zed_pose'a akar.
ZED_ODOM_TOPIC = '/zed/zed_node/odom'
# ZED 3D SLAM nokta bulutu (PointCloud2) -> kopru seyreltip /pointcloud JSON'a servis eder.
ZED_PCL_TOPIC = '/zed/zed_node/point_cloud/cloud_registered'
PCL_MAX_POINTS = 3500     # tarayiciya gonderilecek azami nokta (bant genisligi kontrolu)
PCL_MIN_PERIOD = 0.5      # sn; bu periyottan sik islemez (~2 Hz)
# Mini ROV (BlueOS/Navigator) mavlink2rest sysid. ARAÇTA DOĞRULA: docs SYSID_THISMAV=2
# der ama motor-test POST target_system=1 kullaniyor. GET yolunda /mavlink agacina
# bakip aracin ilan ettigi sysid'yi teyit et.
NAV_SYSID = 1

JPEG_QUALITY = 70
SENSOR_FRESH_S = 3.0                  # bu sureden eski veri "connected=false"


def img_to_jpeg(msg):
    """sensor_msgs/Image -> JPEG bytes (ROS import'u gerekmez, duck-typed)."""
    h, w, enc = msg.height, msg.width, msg.encoding
    raw = bytes(msg.data)
    try:
        if enc in ('rgb8', 'bgr8'):
            arr = np.frombuffer(raw, np.uint8).reshape(h, w, 3)
            if enc == 'bgr8':
                arr = arr[:, :, ::-1]
            im = PILImage.fromarray(arr, 'RGB')
        elif enc in ('rgba8', 'bgra8'):
            arr = np.frombuffer(raw, np.uint8).reshape(h, w, 4)
            arr = arr[:, :, [2, 1, 0]] if enc == 'bgra8' else arr[:, :, :3]
            im = PILImage.fromarray(arr, 'RGB')
        elif enc == 'mono8':
            im = PILImage.fromarray(np.frombuffer(raw, np.uint8).reshape(h, w), 'L')
        elif enc in ('16UC1', 'mono16'):
            a = np.frombuffer(raw, np.uint16).reshape(h, w)
            a8 = (a / a.max() * 255).astype(np.uint8) if a.max() else a.astype(np.uint8)
            im = PILImage.fromarray(a8, 'L')
        else:
            return None
    except Exception:
        return None
    out = io.BytesIO()
    im.save(out, 'JPEG', quality=JPEG_QUALITY)
    return out.getvalue()


def _blank_sensors():
    return {'depth_m': None, 'altitude_m': None, 'yaw': None,
            'roll': None, 'pitch': None, 'voltage': None,
            'depth_valid': False, 'sonar_valid': False,
            # --- P0 guvenlik telemetrisi (additive) ---
            # Veri gelene kadar None = "bilinmiyor". DIKKAT: leak varsayilani
            # False (sizinti yok) DEGIL None olmali; yanlis "guvenli" gostergesi
            # hic gostermemekten beterdir. UI None'i "bilinmiyor" gosterir.
            'current_a': None, 'battery_pct': None,
            'armed': None, 'mode': None,
            'statustext': None, 'leak': None,
            # --- kokpit header ek gostergeleri (additive; None = bilinmiyor) ---
            # AUV (V6X) tarafi
            'gps_lat': None, 'gps_lon': None, 'gps_fix': None,
            'gps_sats': None, 'gps_hdop': None, 'water_temp_c': None,
            # CUAV ADC analog ic/kart sicakligi (Bar30 su sicakligindan AYRI).
            # 90C asim uyarisi bu deger uzerinden calisir. None = sensor/kaynak yok.
            'board_temp_c': None,
            'link_ok': None,   # AUV FC HEARTBEAT tazeligi (net kopma sinyali)
            # Mini ROV (BlueOS/Navigator, mavlink2rest) — per-arac ayri gostergeler
            'mrov_link': None, 'mrov_leak': None, 'mrov_temp_c': None,
            'mrov_voltage': None, 'mrov_current_a': None, 'mrov_battery_pct': None,
            'mrov_armed': None,
            # ZED VO/SLAM izi (demo'da sentetik+DEMO etiketli; gercek topic arac-uzeri baglanir)
            'zed_pose': None}


def _sensor_snapshot(sensors, sensor_stamp, cam_status_fn):
    """Ortak /sensors sekli (gercek node ve demo ayni fonksiyonu kullanir)."""
    s = dict(sensors)
    if s['depth_valid'] and s['sonar_valid'] \
            and s['depth_m'] is not None and s['altitude_m'] is not None:
        s['water_column_m'] = round(s['depth_m'] + s['altitude_m'], 2)
    else:
        s['water_column_m'] = None
    s['connected'] = bool(sensor_stamp and (time.time() - sensor_stamp < SENSOR_FRESH_S))
    s['cameras'] = {name: cam_status_fn(name) for name in CAM_NAMES}
    return s


# ---------------- Gercek ROS2 node (rclpy TEMBEL import - demo icin gerekmez) ---
def build_ros_node():
    """rclpy'yi burada import eder; --demo yolunda hic cagrilmaz."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, CompressedImage, Range, BatteryState, NavSatFix, PointCloud2
    from std_msgs.msg import Float32, Bool, String
    from geometry_msgs.msg import Vector3Stamped
    from auv_msgs.msg import GpsInfo

    class BridgeNode(Node):
        def __init__(self):
            super().__init__('ros2_web_bridge')
            self.jpegs = {name: None for name in CAM_NAMES}
            self.stamps = {name: 0.0 for name in CAM_NAMES}
            self.locks = {name: threading.Lock() for name in CAM_NAMES}
            self.sensors = _blank_sensors()
            self.sensor_stamp = 0.0

            # goruntu QoS: best-effort (hem reliable hem best-effort publisher'la uyumlu)
            img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
            for name, topic in IMAGE_TOPICS.items():
                self.create_subscription(Image, topic, self._img_cb(name), img_qos)
            for name, topic in COMPRESSED_TOPICS.items():
                self.create_subscription(CompressedImage, topic, self._cimg_cb(name), img_qos)

            sqos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Float32, DEPTH_TOPIC, self._depth_cb, sqos)
            self.create_subscription(Range, RANGE_TOPIC, self._range_cb, sqos)
            self.create_subscription(Float32, HEADING_TOPIC, self._heading_cb, sqos)
            self.create_subscription(Vector3Stamped, ATTITUDE_TOPIC, self._att_cb, sqos)
            self.create_subscription(BatteryState, BATTERY_TOPIC, self._batt_cb, sqos)
            # --- P0 guvenlik telemetrisi abonelikleri (additive) ---
            # Bu topic'leri auv_mav_bridge zaten yayinliyor (arm/mode/statustext);
            # /mav/leak bridge'e P0 kapsaminda eklenir (donanim-kapili).
            self.create_subscription(Bool, ARMED_TOPIC, self._armed_cb, sqos)
            self.create_subscription(String, MODE_TOPIC, self._mode_cb, sqos)
            self.create_subscription(String, STATUSTEXT_TOPIC, self._stext_cb, sqos)
            self.create_subscription(Bool, LEAK_TOPIC, self._leak_cb, sqos)
            # kokpit header ek gostergeleri
            self.create_subscription(NavSatFix, GPS_TOPIC, self._gps_cb, sqos)
            self.create_subscription(GpsInfo, GPSINFO_TOPIC, self._gpsinfo_cb, sqos)
            self.create_subscription(Float32, WATERTEMP_TOPIC, self._wtemp_cb, sqos)
            self.create_subscription(Float32, BOARDTEMP_TOPIC, self._btemp_cb, sqos)
            self.create_subscription(Bool, LINK_OK_TOPIC, self._link_cb, sqos)
            # ZED VO/odometri (opsiyonel; ZED_ODOM_TOPIC set edilirse gercek pose izi)
            if ZED_ODOM_TOPIC:
                from nav_msgs.msg import Odometry
                self.create_subscription(Odometry, ZED_ODOM_TOPIC, self._zed_cb, sqos)
                self.get_logger().info(f'ZED odometri aboneligi: {ZED_ODOM_TOPIC}')
            # ZED 3D nokta bulutu (SLAM) -> seyreltilip /pointcloud'a servis edilir
            self._pcl = {'n': 0, 'pts': []}
            self._pcl_lock = threading.Lock()
            self._pcl_stamp = 0.0
            if ZED_PCL_TOPIC:
                self.create_subscription(PointCloud2, ZED_PCL_TOPIC, self._pcl_cb, img_qos)
                self.get_logger().info(f'ZED nokta bulutu aboneligi: {ZED_PCL_TOPIC}')
            self.get_logger().info('ROS2 web koprusu hazir. Topic aboneleri kuruldu.')
            # Mini ROV telemetrisi ROS2 disi (BlueOS 192.168.2.2) -> mavlink2rest GET poll
            self._mrov_tok = None      # son tazelik damgasi (mesaj sayaci/zamani)
            self._mrov_stall = 0       # ayni damganin ust uste tekrar sayisi
            threading.Thread(target=self._mrov_loop, daemon=True).start()

        def _img_cb(self, name):
            def cb(msg):
                jpg = img_to_jpeg(msg)
                if jpg:
                    with self.locks[name]:
                        self.jpegs[name] = jpg
                        self.stamps[name] = time.time()
            return cb

        def _cimg_cb(self, name):
            # CompressedImage: format genelde 'jpeg' -> veri zaten JPEG, dogrudan servis.
            # (PNG gelirse PIL ile JPEG'e cevir.)
            def cb(msg):
                data = bytes(msg.data)
                fmt = (msg.format or '').lower()
                jpg = None
                if 'jpeg' in fmt or 'jpg' in fmt:
                    jpg = data
                else:
                    try:
                        im = PILImage.open(io.BytesIO(data)).convert('RGB')
                        out = io.BytesIO(); im.save(out, 'JPEG', quality=JPEG_QUALITY)
                        jpg = out.getvalue()
                    except Exception:
                        jpg = None
                if jpg:
                    with self.locks[name]:
                        self.jpegs[name] = jpg
                        self.stamps[name] = time.time()
            return cb

        def _depth_cb(self, m):
            self.sensors['depth_m'] = round(float(m.data), 2)
            self.sensors['depth_valid'] = True
            self.sensor_stamp = time.time()

        def _range_cb(self, m):
            self.sensors['altitude_m'] = round(float(m.range), 2)
            self.sensors['sonar_valid'] = 0.3 <= m.range <= 30.0
            self.sensor_stamp = time.time()

        def _heading_cb(self, m):
            self.sensors['yaw'] = round(float(m.data), 1)
            self.sensor_stamp = time.time()

        def _att_cb(self, m):
            # Vector3Stamped: x=roll, y=pitch, z=yaw (radyan). yaw'i heading otoriter tutar.
            self.sensors['roll'] = round(math.degrees(m.vector.x), 1)
            self.sensors['pitch'] = round(math.degrees(m.vector.y), 1)
            self.sensor_stamp = time.time()

        def _batt_cb(self, m):
            v = float(m.voltage)
            self.sensors['voltage'] = round(v, 2) if v > 0 else None
            # A ve % FC yoksa NaN gelir (bridge -1 -> NaN). NaN JSON'u kirar;
            # kontrol ">0" degil math.isnan olmali (0.0 gecerli bir okumadir).
            c = float(m.current)
            self.sensors['current_a'] = round(c, 1) if not math.isnan(c) else None
            p = float(m.percentage)   # BatteryState.percentage 0..1
            self.sensors['battery_pct'] = round(p * 100.0) if not math.isnan(p) else None
            self.sensor_stamp = time.time()

        def _armed_cb(self, m):
            self.sensors['armed'] = bool(m.data)
            self.sensor_stamp = time.time()

        def _mode_cb(self, m):
            self.sensors['mode'] = str(m.data)
            self.sensor_stamp = time.time()

        def _stext_cb(self, m):
            self.sensors['statustext'] = str(m.data)
            self.sensor_stamp = time.time()

        def _leak_cb(self, m):
            self.sensors['leak'] = bool(m.data)
            self.sensor_stamp = time.time()

        def _gps_cb(self, m):
            self.sensors['gps_lat'] = round(float(m.latitude), 7)
            self.sensors['gps_lon'] = round(float(m.longitude), 7)
            self.sensor_stamp = time.time()

        def _gpsinfo_cb(self, m):
            self.sensors['gps_fix'] = int(m.fix_type)
            self.sensors['gps_sats'] = int(m.satellites)
            h = float(m.hdop)
            self.sensors['gps_hdop'] = round(h, 2) if not math.isnan(h) else None
            self.sensor_stamp = time.time()

        def _wtemp_cb(self, m):
            self.sensors['water_temp_c'] = round(float(m.data), 1)
            self.sensor_stamp = time.time()

        def _btemp_cb(self, m):
            # CUAV ADC analog ic/kart sicakligi (90C asim uyarisi bu degerden)
            self.sensors['board_temp_c'] = round(float(m.data), 1)
            self.sensor_stamp = time.time()

        def _link_cb(self, m):
            self.sensors['link_ok'] = bool(m.data)
            self.sensor_stamp = time.time()

        def _zed_cb(self, m):
            # ZED odometri -> ustten-gorunum VO izi (x=east, y=north, yaw derece)
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z))) % 360.0
            self.sensors['zed_pose'] = {'x': round(p.x, 2), 'y': round(p.y, 2),
                                        'yaw': round(yaw, 1), 'demo': False}
            self.sensor_stamp = time.time()

        def _pcl_cb(self, msg):
            # ZED PointCloud2 -> seyreltilmis xyz listesi (tarayici 3D render). ~2 Hz, NaN filtreli.
            now = time.time()
            if now - self._pcl_stamp < PCL_MIN_PERIOD:
                return
            self._pcl_stamp = now
            try:
                step = int(msg.point_step)
                raw = bytes(msg.data)
                n = len(raw) // step
                if n <= 0:
                    return
                a = np.frombuffer(raw, dtype=np.uint8).reshape(n, step)
                xyz = a[:, 0:12].copy().view(np.float32).reshape(-1, 3)   # x@0,y@4,z@8 (float32)
                xyz = xyz[np.isfinite(xyz).all(axis=1)]                   # ONCE NaN/inf ele
                if len(xyz) > PCL_MAX_POINTS:                             # SONRA seyrelt (yogunlugu korur)
                    xyz = xyz[::max(1, len(xyz) // PCL_MAX_POINTS)][:PCL_MAX_POINTS]
                pts = np.round(xyz, 2).flatten().tolist()
                with self._pcl_lock:
                    self._pcl = {'n': len(pts) // 3, 'pts': pts}
            except Exception:
                pass

        def get_pointcloud(self):
            with self._pcl_lock:
                return dict(self._pcl)

        # ---- Mini ROV (BlueOS/Navigator) telemetrisi: mavlink2rest GET ----
        # DIKKAT: mavlink2rest JSON alan sekli surume bagli degisir. Alan cikarimlari
        # bridge_node.py formulleriyle ayni ama ARAC-UZERI DOGRULANMALI (base_mode
        # int/dict, text list/str, sysid). En guvenilir sinyal mrov_link (erisilebilirlik).
        def _mrov_read(self, mtype):
            # Tam yaniti dondurur ({message, status}); tazelik icin status gerekir.
            import json as _json
            import urllib.request as _u
            url = f'{NAV_M2R_URL}/vehicles/{NAV_SYSID}/components/1/messages/{mtype}'
            try:
                with _u.urlopen(url, timeout=2) as r:
                    return _json.loads(r.read())
            except Exception:
                return None

        @staticmethod
        def _mrov_fresh_token(data):
            # mavlink2rest 'status' sarmali: mesaj her alindiginda ilerleyen sayac/zaman.
            # Sekil surume bagli -> counter, yoksa time.last_update dene.
            st = (data or {}).get('status') or {}
            if not isinstance(st, dict):
                return None
            if st.get('counter') is not None:
                return st.get('counter')
            t = st.get('time') or {}
            return t.get('last_update') if isinstance(t, dict) else None

        def _mrov_reset(self):
            # Link kopuk/bayat -> telemetri "bilinmiyor" (None). Leak MANDALI korunur
            # (bir kez sizinti gorulduyse fail-safe True kalir).
            for k in ('mrov_voltage', 'mrov_current_a', 'mrov_battery_pct',
                      'mrov_temp_c', 'mrov_armed'):
                self.sensors[k] = None

        def _mrov_loop(self):
            while True:
                ssd = self._mrov_read('SYS_STATUS')
                htd = self._mrov_read('HEARTBEAT')
                spd = self._mrov_read('SCALED_PRESSURE2')
                std = self._mrov_read('STATUSTEXT')
                reachable = any(x is not None for x in (ssd, htd, spd, std))
                # tazelik: HEARTBEAT/SYS_STATUS token'i ilerliyor mu? (FC dondu ama Pi
                # HTTP'de -> eski onbellekli mesaj -> token sabit -> bayat=kopuk say)
                tok = self._mrov_fresh_token(htd)
                if tok is None:
                    tok = self._mrov_fresh_token(ssd)
                if tok is not None:
                    if tok == self._mrov_tok:
                        self._mrov_stall += 1
                    else:
                        self._mrov_stall = 0
                    self._mrov_tok = tok
                    fresh = reachable and self._mrov_stall < 3   # ~3 sn ilerlemezse bayat
                else:
                    # status damgasi yok -> yalniz erisilebilirlik (ARAÇTA DOGRULA)
                    fresh = reachable
                self.sensors['mrov_link'] = bool(fresh)
                if not fresh:
                    self._mrov_reset()
                    time.sleep(1.0)
                    continue
                ss = (ssd or {}).get('message', ssd)
                ht = (htd or {}).get('message', htd)
                sp = (spd or {}).get('message', spd)
                st = (std or {}).get('message', std)
                s = self.sensors
                try:
                    if ss:
                        v = ss.get('voltage_battery')
                        s['mrov_voltage'] = round(v / 1000.0, 2) if v not in (None, 0, 65535) else None
                        c = ss.get('current_battery')
                        s['mrov_current_a'] = round(c / 100.0, 1) if c not in (None, -1) else None
                        p = ss.get('battery_remaining')
                        s['mrov_battery_pct'] = int(p) if p not in (None, -1) else None
                except Exception:
                    pass
                try:
                    if ht is not None:
                        bm = ht.get('base_mode', 0)
                        bits = bm.get('bits', 0) if isinstance(bm, dict) else int(bm or 0)
                        s['mrov_armed'] = bool(bits & 128)   # MAV_MODE_FLAG_SAFETY_ARMED
                except Exception:
                    pass
                try:
                    if sp:
                        t = sp.get('temperature')
                        s['mrov_temp_c'] = round(t / 100.0, 1) if t not in (None, 0) else None
                except Exception:
                    pass
                try:
                    if st:
                        txt = st.get('text', '')
                        if isinstance(txt, list):
                            # mavlink2rest surumune gore text ya int listesi ya da tek-karakter
                            # STRING listesi ("L","e","a","k",...) olabilir. ESKI KOD yalniz int
                            # bekliyordu; string gelince hepsini eliyor -> metin bos -> "Leak
                            # Detected" olsa bile mrov_leak hep null kaliyordu (ARAÇTA GORULEN BUG).
                            # Ikisini de destekle:
                            chars = []
                            for x in txt:
                                if isinstance(x, int):
                                    if x: chars.append(chr(x))
                                elif isinstance(x, str):
                                    chars.append(x)
                            txt = ''.join(chars)
                        txt = str(txt).replace('\x00', '').strip()
                        low = txt.lower()
                        if 'leak' in low or 'flood' in low:
                            s['mrov_leak'] = True   # mandalli: bir kez True -> True kalir
                except Exception:
                    pass
                time.sleep(1.0)

        def get_jpeg(self, name):
            with self.locks.get(name, threading.Lock()):
                return self.jpegs.get(name)

        def cam_status(self, name):
            fresh = (time.time() - self.stamps.get(name, 0)) < 2.0
            return 'canli (ROS2)' if (self.jpegs.get(name) and fresh) else 'topic yok'

        def snapshot(self):
            return _sensor_snapshot(self.sensors, self.sensor_stamp, self.cam_status)

    rclpy.init()
    node = BridgeNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    return node


# ---------------- Demo kaynagi (rclpy YOK - yerel dogrulama) --------------------
class DemoSource:
    """Sentetik sensor + kamera; HTTP/panel katmanini donanimsiz dogrulamak icin.
    web/app.py._run_demo ve panel/demo.py ile ayni desen."""

    def __init__(self):
        self.jpegs = {name: None for name in CAM_NAMES}
        self._lock = threading.Lock()
        self.sensors = _blank_sensors()
        self.sensor_stamp = 0.0
        self._t0 = time.time()
        threading.Thread(target=self._run, daemon=True).start()

    def _frame(self, name, t):
        im = PILImage.new('RGB', (640, 480), (30, 45, 60))
        d = ImageDraw.Draw(im)
        x = int(320 + 220 * math.sin(t * 0.7))
        d.line([(x, 40), (640 - x, 440)], fill=(210, 40, 40), width=22)
        d.text((18, 18), f'DEMO {name}', fill=(255, 255, 255))
        d.text((18, 452), time.strftime('%H:%M:%S'), fill=(200, 200, 200))
        out = io.BytesIO()
        im.save(out, 'JPEG', quality=JPEG_QUALITY)
        return out.getvalue()

    def _run(self):
        while True:
            t = time.time() - self._t0
            with self._lock:
                for name in CAM_NAMES:
                    self.jpegs[name] = self._frame(name, t)
            depth = round(0.6 + 0.25 * math.sin(t / 3), 2)
            alt = round(2.2 + 0.3 * math.cos(t / 4), 2)
            # Batarya sentetik desarj: 16.0V -> ~14.4V dususu, akim dalgalanir
            volt = round(16.0 - 0.02 * (t % 80), 2)
            self.sensors.update(
                depth_m=depth, depth_valid=True,
                altitude_m=alt, sonar_valid=True,
                roll=round(4 * math.sin(t / 5), 1),
                pitch=round(3 * math.cos(t / 6), 1),
                yaw=round((t * 6) % 360, 1),
                voltage=volt, link_ok=True,
                current_a=round(7.5 + 3 * abs(math.sin(t / 4)), 1),
                battery_pct=max(0, round(100 - (t % 80) * 1.2)),
                armed=False, mode='MANUAL',
                # sentetik GPS (yuzey origin etrafinda kucuk drift) + su sicakligi
                gps_lat=round(40.7891 + 0.00008 * math.sin(t / 20), 7),
                gps_lon=round(29.4501 + 0.00008 * math.cos(t / 20), 7),
                gps_fix=3, gps_sats=int(10 + 2 * abs(math.sin(t / 9))),
                gps_hdop=round(0.8 + 0.3 * abs(math.sin(t / 7)), 2),
                water_temp_c=round(18.0 + 0.6 * math.sin(t / 30), 1),
                # CUAV ADC ic/kart sicakligi (DEMO): 90C esik uyarisini GORSEL
                # dogrulamak icin kasitli ucgen dalga 35<->100 (~80 sn periyot);
                # ~90C'yi periyodik asar -> kokpit uyarisi/bannerı/olay gunlugu test edilir.
                # GERCEKTE: /mav/board_temp topic'inden gelir (SCALED_PRESSURE3).
                board_temp_c=round(35.0 + 65.0 * (1 - abs((t % 80) - 40) / 40.0), 1),
                # Mini ROV (sentetik; gercekte mavlink2rest'ten gelir)
                mrov_link=True, mrov_leak=None,
                mrov_temp_c=round(19.0 + 0.4 * math.sin(t / 25), 1),
                mrov_voltage=round(15.6 - 0.01 * (t % 60), 2),
                mrov_current_a=round(3.0 + 1.5 * abs(math.sin(t / 5)), 1),
                mrov_battery_pct=max(0, round(100 - (t % 60) * 1.0)),
                mrov_armed=False,
                # ZED VO izi (SENTETIK — DEMO): ustten-gorunum daire cizer
                zed_pose={'x': round(2.2 * math.sin(t / 12), 2),
                          'y': round(2.2 * math.cos(t / 15), 2),
                          'yaw': round((t * 8) % 360, 1), 'demo': True})
            # leak ve statustext: demo'da GERCEK sensor yok -> None (bilinmiyor).
            # Kasitli olarak "sizinti yok" gibi sahte guvenli deger URETMIYORUZ;
            # panel bunlari "bilinmiyor" gosterir (bkz. reviewer karari).
            self.sensor_stamp = time.time()
            time.sleep(1 / 15)

    def get_jpeg(self, name):
        with self._lock:
            return self.jpegs.get(name)

    def cam_status(self, name):
        return 'demo'

    def get_pointcloud(self):
        # DEMO: sentetik nokta bulutu (spiral disk) — gercekte ZED PointCloud2'den gelir
        pts = []
        for i in range(1400):
            ang = i * 2.399963
            rad = (i / 1400.0) ** 0.5 * 2.2
            pts += [round(rad * math.cos(ang), 2),
                    round(0.5 * math.sin(i * 0.04) - 0.4, 2),
                    round(rad * math.sin(ang) + 2.2, 2)]
        return {'n': len(pts) // 3, 'pts': pts}

    def snapshot(self):
        return _sensor_snapshot(self.sensors, self.sensor_stamp, self.cam_status)


# ---------------- Flask ----------------
app = Flask(__name__)
node = None
PLACEHOLDER = None


def placeholder():
    global PLACEHOLDER
    if PLACEHOLDER is None:
        im = PILImage.new('RGB', (640, 480), (25, 25, 25))
        out = io.BytesIO(); im.save(out, 'JPEG'); PLACEHOLDER = out.getvalue()
    return PLACEHOLDER


def mjpeg(name):
    while True:
        jpg = node.get_jpeg(name) or placeholder()
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n'
        time.sleep(0.04)


PAGE = """<!doctype html><html lang=tr><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>AUV ROS2 Panel</title><style>
:root{color-scheme:dark}body{margin:0;font-family:system-ui,sans-serif;background:#0d1117;color:#e6edf3}
header{padding:10px 18px;background:#161b22;border-bottom:1px solid #30363d;display:flex;gap:14px;align-items:center;flex-wrap:wrap}
h1{font-size:16px;margin:0}.pill{font-size:12px;padding:3px 10px;border-radius:12px;background:#21262d}
.pill.ok{background:#12401f;color:#7ee787}.pill.bad{background:#4a1215;color:#ff9d9d}
main{display:grid;grid-template-columns:2fr 1fr;gap:14px;padding:14px}@media(max-width:900px){main{grid-template-columns:1fr}}
.cams{display:grid;grid-template-columns:1fr 1fr;gap:14px}@media(max-width:640px){.cams{grid-template-columns:1fr}}
.card{background:#161b22;border:1px solid #30363d;border-radius:10px;overflow:hidden}
.card h2{font-size:13px;margin:0;padding:8px 12px;background:#1c2230;display:flex;justify-content:space-between}
.card h2 .st{font-size:11px;color:#8b949e;font-weight:400}.card img{width:100%;display:block;background:#000;aspect-ratio:4/3;object-fit:contain}
.big{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px}
.metric{background:#0d1117;border:1px solid #30363d;border-radius:8px;padding:10px 12px}
.metric .label{font-size:11px;color:#8b949e;text-transform:uppercase}.metric .val{font-size:26px;font-weight:700}
.metric .val small{font-size:13px;font-weight:400;color:#8b949e}.metric.stale .val{color:#6e7681}
.att{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;padding:12px;text-align:center}
.att .k{font-size:11px;color:#8b949e;display:block}.att .v{font-size:20px;font-weight:700}
</style></head><body>
<header><h1>AUV Panel — ROS2</h1><span id=link class=pill>?</span>
<span id=volt class=pill>— V</span>
<span style=flex:1></span>
<a href="/test" class=pill style="text-decoration:none;color:#ffd23f;font-weight:700">⚙ Motor Test</a>
<a href="/control" class=pill style="text-decoration:none;color:#7fd4ff">Kontrol Paneli →</a></header>
<main>
<div class=cams>
<div class=card><h2>AUV — ZED 2i (ROS2) <span class=st id=st-zed>—</span></h2><img src="/video/zed"></div>
<div class=card><h2><a href="/minirov" target=_blank style="color:#7fd4ff;text-decoration:none">Mini ROV — WebRTC 30fps ⤢ tam ekran</a></h2><iframe src="/minirov" style="width:100%;aspect-ratio:16/9;border:0;display:block;background:#000"></iframe></div>
</div>
<div>
<div class=card><h2>Dikey Eksen (Bar30+Sonar, ROS2)</h2>
<div class=big>
<div class=metric id=m-d><div class=label>Derinlik</div><div class=val><span id=depth>—</span> <small>m</small></div></div>
<div class=metric id=m-a><div class=label>Dipten Yükseklik</div><div class=val><span id=alt>—</span> <small>m</small></div></div>
<div class=metric id=m-c style="grid-column:1/3"><div class=label>Su Sütunu</div><div class=val><span id=col>—</span> <small>m</small></div></div>
</div>
<div style="padding:0 12px 12px"><canvas id=prof width=360 height=250 style="width:100%;height:auto;background:#0b2740;border-radius:6px"></canvas></div></div>
<div class=card style=margin-top:14px><h2>IMU (Attitude)</h2><div class=att>
<div><span class=k>Roll</span><span class=v id=roll>—</span></div>
<div><span class=k>Pitch</span><span class=v id=pitch>—</span></div>
<div><span class=k>Yaw</span><span class=v id=yaw>—</span></div>
</div></div>
</div></main>
<script>
function setm(id,el,v,valid){const b=document.getElementById(id);if(v==null){el.textContent='—';b.classList.add('stale');return}el.textContent=(+v).toFixed(2);b.classList.toggle('stale',valid===false)}
async function poll(){try{const d=await(await fetch('/sensors')).json();
const anycam=Object.values(d.cameras||{}).some(s=>s.includes('canli')||s==='demo');
const lk=document.getElementById('link');
const on=d.connected||anycam||d.depth_m!=null;lk.textContent=on?'ROS2 bağlı':'veri bekleniyor';lk.className='pill '+(on?'ok':'bad');
document.getElementById('volt').textContent=d.voltage!=null?(+d.voltage).toFixed(1)+' V':'— V';
setm('m-d',document.getElementById('depth'),d.depth_m,d.depth_valid);
setm('m-a',document.getElementById('alt'),d.altitude_m,d.sonar_valid);
setm('m-c',document.getElementById('col'),d.water_column_m,d.depth_valid&&d.sonar_valid);
document.getElementById('roll').textContent=d.roll!=null?d.roll+'°':'—';
document.getElementById('pitch').textContent=d.pitch!=null?d.pitch+'°':'—';
document.getElementById('yaw').textContent=d.yaw!=null?d.yaw+'°':'—';
if(d.cameras){var z=document.getElementById('st-zed');if(z)z.textContent=d.cameras.zed||'—'}
drawProfile(d.depth_m,d.altitude_m,d.depth_valid,d.sonar_valid);
}catch(e){document.getElementById('link').textContent='sunucu yok';document.getElementById('link').className='pill bad'}}

// Dikey profil: AUV HER ZAMAN ORTADA sabit. Yuzey yukarida (derinlik kadar),
// taban asagida (dipten yukseklik kadar). Arac yukari/asagi kaymaz; su yuzeyi
// ve taban ona gore konumlanir.
function drawProfile(depth,alt,dv,av){
  const c=document.getElementById('prof'),x=c.getContext('2d'),W=c.width,H=c.height,m=30;
  x.clearRect(0,0,W,H);x.fillStyle='#0b2740';x.fillRect(0,0,W,H);
  const yV=H/2;                                  // AUV ekranin tam ortasi
  const d=(dv?(depth||0):0), a=(av?(alt||0):0);
  const span=Math.max(d,a,1.0);                  // buyuk mesafe kenara yakin olur
  const scale=(H/2-m)/span;                      // metre -> piksel
  const yS=yV-d*scale, yF=yV+a*scale;
  x.font='11px sans-serif';x.textAlign='left';
  // hava/su ayrimi: yuzey ustu hafif farkli ton
  if(dv){x.fillStyle='rgba(10,25,45,0.6)';x.fillRect(0,0,W,Math.max(0,yS));}
  // yuzey cizgisi
  if(dv){x.strokeStyle='#7fd4ff';x.lineWidth=2;x.beginPath();x.moveTo(0,yS);x.lineTo(W,yS);x.stroke();
    x.fillStyle='#7fd4ff';x.fillText('YÜZEY',8,yS-6);
    x.fillText('↑ yüzeye '+d.toFixed(2)+' m',W-118,yS-6);}
  else{x.fillStyle='#ff9d9d';x.fillText('Bar30 / derinlik verisi yok',8,18);}
  // taban cizgisi (taramali)
  if(av){x.strokeStyle='#c9a24b';x.lineWidth=2;x.beginPath();x.moveTo(0,yF);x.lineTo(W,yF);x.stroke();
    for(let px=0;px<W;px+=14){x.beginPath();x.moveTo(px,yF);x.lineTo(px+7,yF+7);x.stroke();}
    x.fillStyle='#c9a24b';x.fillText('TABAN',8,yF+18);
    x.fillText('↓ tabana '+a.toFixed(2)+' m',W-118,yF+18);}
  else{x.fillStyle='#8b949e';x.fillText('sonar yok',8,H-8);}
  // AUV govdesi (ortada, sabit)
  x.fillStyle='#ffd23f';x.beginPath();
  x.moveTo(W/2-20,yV-4);x.lineTo(W/2+20,yV-4);x.lineTo(W/2+26,yV+6);x.lineTo(W/2-26,yV+6);
  x.closePath();x.fill();
  x.fillStyle='#0d1117';x.font='bold 12px sans-serif';x.textAlign='center';
  x.fillText('AUV',W/2,yV+4);x.textAlign='left';
}

// Mini ROV artik WebRTC iframe (/minirov) ile 30fps -> thumbnail polling yok.
setInterval(poll,300);poll();
</script></body></html>"""


def _serve_cockpit():
    # Tek-sayfa kokpit kabugu; /, /ops, /control, /test ayni dosyayi doner
    # (client-side mod; endpoint yollari korunur).
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cockpit.html')
    try:
        with open(p, encoding='utf-8') as f:
            return f.read()
    except FileNotFoundError:
        return render_template_string(PAGE)   # yedek (eski gomulu panel)


@app.route('/')
def index():
    return _serve_cockpit()


@app.route('/video/<name>')
def video(name):
    if name not in CAM_NAMES:
        return 'yok', 404
    return Response(mjpeg(name), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stream/color')      # geriye donuk uyumluluk: artik birincil kamera ZED
def stream_color():
    return Response(mjpeg('zed'), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/panel.css')
def panel_css():
    # Ortak tasarim sistemi (tum sayfalar paylasir). Sadece sunum; sozlesme degil.
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'panel.css')
    try:
        with open(p, encoding='utf-8') as f:
            r = Response(f.read(), mimetype='text/css')
            # Operator eski/onbellekli CSS almasin (deploy sonrasi tazelik + dev iterasyonu)
            r.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
            return r
    except FileNotFoundError:
        return 'panel.css yok', 404


@app.route('/sensors')
def sensors():
    return jsonify(node.snapshot())


@app.route('/pointcloud')
def pointcloud():
    # ZED 3D SLAM nokta bulutu (seyreltilmis). Kaynak yoksa {n:0,pts:[]}.
    return jsonify(node.get_pointcloud())


@app.route('/control')
def control_page():
    # Kokpit "kontrol" modu (client-side); ayni kabugu doner.
    return _serve_cockpit()


# Mini ROV (Navigator/BlueOS) motor testi: tarayici -> bu kopru -> mavlink2rest (.2:6040)
# -> Navigator ArduSub (sysid 1, comp 1). Sunucu-tarafi relay (CORS yok).
NAV_M2R_URL = 'http://192.168.2.2:6040/mavlink'

# Mini ROV dogrudan cikis (DO_SET_SERVO) GUVENLIK: DO_SET_SERVO PWM'i degistirilene
# kadar TUTAR. Istemci koparsa/tarayici kapanirsa motorlar takili kalir -> su disinda
# YANMA riski. Dead-man watchdog: son /minirov/direct'ten >2 sn gecerse notrle (1500).
_mrov_direct_last = 0.0
_mrov_direct_active = False


def _mrov_direct_watchdog():
    global _mrov_direct_active
    while True:
        time.sleep(0.5)
        if _mrov_direct_active and (time.time() - _mrov_direct_last) > 2.0:
            _mrov_direct_active = False
            try:
                for ch in range(1, 5):
                    _nav_set_servo(ch, 1500)   # notr — motorlar dursun
            except Exception:
                pass


def _nav_motor_test(motor, throttle_pct, duration_s):
    import json as _json
    import urllib.request as _u
    body = {"header": {"system_id": 255, "component_id": 240, "sequence": 0},
            "message": {"type": "COMMAND_LONG",
                        "param1": float(motor), "param2": 0.0,      # 0 = THROTTLE_PERCENT
                        "param3": float(throttle_pct), "param4": float(duration_s),
                        "param5": 0.0, "param6": 0.0, "param7": 0.0,
                        "command": {"type": "MAV_CMD_DO_MOTOR_TEST"},
                        "target_system": 1, "target_component": 1, "confirmation": 0}}
    r = _u.Request(NAV_M2R_URL, data=_json.dumps(body).encode(),
                   headers={'Content-Type': 'application/json'}, method='POST')
    _u.urlopen(r, timeout=4).read()


@app.route('/minirov/motor', methods=['POST'])
def minirov_motor():
    motor = int(request.args.get('motor', 0))
    throttle = int(request.args.get('throttle', 10))
    duration = int(request.args.get('duration', 3))
    try:
        if motor == 0:                       # 0 = HEPSINI DURDUR (cikis 1..6)
            for m in range(1, 7):
                _nav_motor_test(m, 0, 0)
            return jsonify({'ok': True, 'msg': 'Mini ROV motorlari durduruldu'})
        _nav_motor_test(motor, throttle, duration)
        return jsonify({'ok': True, 'motor': motor, 'throttle': throttle, 'duration': duration})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 502


def _nav_set_servo(channel, pwm):
    # Mini ROV cift-yon dogrudan cikis: DO_SET_SERVO ile bir SERVO kanalina PWM.
    # AUV'deki direct_output'un mavlink2rest karsiligi.
    import json as _json
    import urllib.request as _u
    body = {"header": {"system_id": 255, "component_id": 240, "sequence": 0},
            "message": {"type": "COMMAND_LONG",
                        "param1": float(channel), "param2": float(pwm),
                        "param3": 0.0, "param4": 0.0, "param5": 0.0, "param6": 0.0, "param7": 0.0,
                        "command": {"type": "MAV_CMD_DO_SET_SERVO"},
                        "target_system": 1, "target_component": 1, "confirmation": 0}}
    r = _u.Request(NAV_M2R_URL, data=_json.dumps(body).encode(),
                   headers={'Content-Type': 'application/json'}, method='POST')
    _u.urlopen(r, timeout=4).read()


@app.route('/minirov/direct', methods=['POST'])
def minirov_direct():
    # 4 motor icin cift-yon dogrudan cikis (PWM 1100-1900). pwms=1500,1500,1500,1500
    # ARAÇTA DOĞRULA: DO_SET_SERVO'nun mikseri ezmesi icin SERVO1-4_FUNCTION gecici
    # 'Disabled/RCPassThru' yapilmasi gerekebilir (AUV bridge'de oldugu gibi). Kanal
    # eslemesi (1-4) araca gore ayarlanmali. Bu relay sadece DO_SET_SERVO gonderir.
    try:
        raw = request.args.get('pwms', '')
        pwms = [max(1100, min(1900, int(float(v)))) for v in raw.split(',') if v.strip()][:4]
        if len(pwms) < 4:
            return jsonify({'ok': False, 'error': '4 PWM gerekir (1100-1900)'}), 400
        for ch, p in enumerate(pwms, start=1):
            _nav_set_servo(ch, p)
        global _mrov_direct_last, _mrov_direct_active
        _mrov_direct_last = time.time()
        _mrov_direct_active = True            # dead-man watchdog'u besle
        return jsonify({'ok': True, 'pwms': pwms})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 502


@app.route('/minirov/direct/stop', methods=['POST'])
def minirov_direct_stop():
    try:
        global _mrov_direct_active
        _mrov_direct_active = False
        for ch in range(1, 5):
            _nav_set_servo(ch, 1500)   # notr
        return jsonify({'ok': True, 'msg': 'Mini ROV dogrudan cikis notr (1500)'})
    except Exception as e:
        return jsonify({'ok': False, 'error': str(e)}), 502


@app.route('/ops')
def ops_page():
    return _serve_cockpit()


@app.route('/test')
def test_page():
    # Kokpit "test" modu (client-side); ayni kabugu doner.
    return _serve_cockpit()


@app.route('/minirov')
def minirov_page():
    # Mini ROV 30fps WebRTC izleyici (BlueOS camera-manager :6021 signalling).
    # Tarayici dogrudan .2'ye baglanir; Jetson yalnizca sayfayi sunar.
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'minirov.html')
    try:
        with open(p, encoding='utf-8') as f:
            return f.read()
    except FileNotFoundError:
        return 'minirov.html yok (~/webpanel/minirov.html olmali)', 404


def main():
    global node
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8000)
    ap.add_argument('--demo', action='store_true',
                    help='rclpy/donanim olmadan sentetik veri (yerel dogrulama)')
    args = ap.parse_args()

    if args.demo:
        node = DemoSource()
        print(f'ROS2 web koprusu [DEMO]: http://0.0.0.0:{args.port}')
    else:
        node = build_ros_node()
        print(f'ROS2 web koprusu: http://0.0.0.0:{args.port}')
    # Mini ROV dogrudan cikis dead-man watchdog (istemci koparsa motorlar takili kalmasin)
    threading.Thread(target=_mrov_direct_watchdog, daemon=True).start()
    app.run(host='0.0.0.0', port=args.port, threaded=True)


if __name__ == '__main__':
    main()
