"""Web arayuzu yapilandirmasi - kurulumuna gore duzenle."""
import os

# ---------------- Sensor kaynagi ----------------
# 'mavlink': V6X'ten dogrudan MAVLink oku (bench/gecis - asagidaki MAVLINK_URL).
# 'bridge' : Jetson ROS2 koprusunun /sensors JSON'ini oku (tam ROS2 gecisi).
#            V6X Jetson'a tasinip mav_bridge + ros2_web_bridge kosunca bunu sec.
SENSOR_SOURCE = os.environ.get('SENSOR_SOURCE', 'mavlink')   # 'mavlink' | 'bridge'
# Kopru /sensors uc noktasi (ros2_web_bridge.py). Jetson saha DHCP ile .135;
# statik .3 uygulaninca guncelle. Kopru portu degisirse (fallback 8080) buna yaz.
BRIDGE_SENSORS_URL = os.environ.get('BRIDGE_SENSORS_URL',
                                    'http://192.168.2.135:8000/sensors')

# ---------------- Kontrol (otonom + waypoint) ----------------
# Jetson auv_mission node WebSocket sunucusu. /control sayfasi tarayicidan
# DOGRUDAN buraya baglanir (arm/mod/gorev/goto komutlari + telemetri).
MISSION_WS_URL = os.environ.get('MISSION_WS_URL', 'ws://192.168.2.135:8765')

# ---------------- MAVLink (Bar30 + Ping sonar + IMU) ----------------
# Bench: V6X dogrudan USB -> COM portu (QGC ile AYNI ANDA kullanilamaz, seri tek
#        programa acilir). Windows'ta 'COM7' gibi; Linux'ta '/dev/ttyACM0'.
# Aglı sistem: mavlink-router varsa 'udpin:0.0.0.0:14552' kullan.
# (SENSOR_SOURCE='bridge' iken kullanilmaz.)
MAVLINK_URL = os.environ.get('MAVLINK_URL', 'COM7')
MAVLINK_BAUD = 115200

# Bar30 derinlik hesabi
WATER_DENSITY = 997.0      # tatli su; deniz 1024.0
SURFACE_HPA = 1013.25      # yuzey basinci (kalibrasyon: havada olcup guncelle)

# Ping sonar gecerli menzil (m)
SONAR_MIN_M = 0.3
SONAR_MAX_M = 30.0

# ---------------- Kameralar ----------------
# OpenCV VideoCapture ile acilir. Her kamera icin sirayla denenecek kaynaklar.
# Tur secenekleri:
#   ('gst', pipeline)  - OpenCV GStreamer ile derlenmisse (Jetson/Linux)
#   ('ffmpeg', yol)    - SDP dosyasi veya RTSP/URL (Windows'ta bu calisir)
#   ('device', index)  - PC'ye DOGRUDAN takili kamera (indeks 0,1,...)
_SDP = os.path.join(os.path.dirname(__file__), 'sdp')

# Intel RealSense D435: Jetson'daki mevcut 'rsweb' uygulamasi D435'i tutup
# renk+derinlik MJPEG olarak :8000'de yayinliyor. Onu tuketiyoruz (kamera
# cakismasi olmadan). Jetson IP = 192.168.2.135 (DHCP; statik .3 onerilir).
D435_SOURCES = [
    ('ffmpeg', 'http://192.168.2.135:8000/stream/color'),
    # ROS2 yoluna gecince: Jetson'da realsense/auv_video node'u RTP 5601'e akitir
    ('gst', 'udpsrc port=5601 caps="application/x-rtp,media=video,'
            'encoding-name=H264,payload=96" ! rtph264depay ! h264parse ! '
            'avdec_h264 ! videoconvert ! appsink drop=true sync=false'),
    ('ffmpeg', os.path.join(_SDP, 'd435.sdp')),
]

# Mini ROV low-light kamera (BlueOS RTSP - Windows OpenCV ile en saglam yol).
# BlueOS'ta 'MiniROV-Web' adli RTSP stream olusturuldu (mavlink-camera-manager).
MINIROV_SOURCES = [
    ('ffmpeg', 'rtsp://192.168.2.2:8554/minirov'),
    # Yedekler:
    ('gst', 'udpsrc port=5600 caps="application/x-rtp,media=video,'
            'encoding-name=H264,payload=96" ! rtph264depay ! h264parse ! '
            'avdec_h264 ! videoconvert ! appsink drop=true sync=false'),
    ('ffmpeg', os.path.join(_SDP, 'minirov.sdp')),
]

# Web sunucusu
HOST = '0.0.0.0'
PORT = 5000
JPEG_QUALITY = 70
