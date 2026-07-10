#!/usr/bin/env python3
"""Bench sensor + kamera web arayuzu (Flask).

Tek tarayuci sayfasinda:
  - Intel RealSense D435 (Jetson) ve Mini ROV low-light kamera (BlueOS) goruntuleri
  - Bar30 derinlik, Ping sonar (dipten yukseklik), dikey profil
  - IMU (roll/pitch/yaw), baglanti/batarya durumu

    python app.py            gercek (config.py'deki kaynaklar)
    python app.py --demo     donanimsiz onizleme (sahte veri)

Kameralar OpenCV ile yakalanip MJPEG olarak servis edilir (her tarayucida acilir).
Sensorler MAVLink'ten okunur. Kaynaklar config.py'de.
"""
import argparse
import math
import os
import threading
import time

# RTSP'yi TCP ile cek (Windows OpenCV/FFMPEG icin daha guvenilir) - cv2'den ONCE
os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'rtsp_transport;tcp')

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template

import config

DEMO = False

app = Flask(__name__)


# ============================================================ Kamera
class CameraStream:
    """Arka planda kaynaktan kare yakalar, en son JPEG'i tutar."""

    def __init__(self, name, sources):
        self.name = name
        self.sources = sources
        self.status = 'baglaniyor'
        self._jpeg = None
        self._lock = threading.Lock()
        self._t0 = time.time()
        threading.Thread(target=self._run, daemon=True).start()

    def _open(self):
        for kind, src in self.sources:
            try:
                if kind == 'gst':
                    cap = cv2.VideoCapture(src, cv2.CAP_GSTREAMER)
                elif kind == 'ffmpeg':
                    cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
                else:  # device
                    cap = cv2.VideoCapture(int(src))
                if cap.isOpened():
                    self.status = f'canli ({kind})'
                    return cap
                cap.release()
            except Exception:
                pass
        return None

    def _run(self):
        if DEMO:
            self._run_demo()
            return
        while True:
            cap = self._open()
            if cap is None:
                self.status = 'kaynak yok'
                self._set_placeholder(f'{self.name}\n(sinyal yok)')
                time.sleep(3.0)
                continue
            while True:
                ok, frame = cap.read()
                if not ok:
                    self.status = 'akis kesildi'
                    cap.release()
                    self._set_placeholder(f'{self.name}\n(akis kesildi)')
                    time.sleep(1.0)
                    break
                self._encode(frame)

    def _run_demo(self):
        self.status = 'demo'
        while True:
            t = time.time() - self._t0
            img = np.full((480, 640, 3), (60, 45, 30), np.uint8)
            x = int(320 + 220 * math.sin(t * 0.7))
            cv2.line(img, (x, 40), (640 - x, 440), (40, 40, 210), 22)
            cv2.putText(img, f'DEMO {self.name}', (18, 42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            cv2.putText(img, time.strftime('%H:%M:%S'), (18, 466),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            self._encode(img)
            time.sleep(1 / 25)

    def _set_placeholder(self, text):
        img = np.full((480, 640, 3), (25, 25, 25), np.uint8)
        y = 220
        for line in text.split('\n'):
            cv2.putText(img, line, (140, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.9, (120, 120, 120), 2)
            y += 40
        self._encode(img)

    def _encode(self, frame):
        ok, buf = cv2.imencode('.jpg', frame,
                               [cv2.IMWRITE_JPEG_QUALITY, config.JPEG_QUALITY])
        if ok:
            with self._lock:
                self._jpeg = buf.tobytes()

    def get_jpeg(self):
        with self._lock:
            return self._jpeg


# ============================================================ MAVLink
class MavReader:
    """Bar30, Ping sonar, IMU verisini MAVLink'ten okur."""

    def __init__(self):
        self.data = {
            'connected': False, 'depth_m': None, 'altitude_m': None,
            'water_column_m': None, 'roll': None, 'pitch': None, 'yaw': None,
            'voltage': None, 'sonar_valid': False, 'depth_valid': False,
            'last_seen': 0,
        }
        self._lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def snapshot(self):
        with self._lock:
            d = dict(self.data)
        d['link_age'] = round(time.time() - d['last_seen'], 1) if d['last_seen'] else None
        d['connected'] = bool(d['last_seen'] and (time.time() - d['last_seen'] < 3))
        return d

    def _set(self, **kw):
        with self._lock:
            self.data.update(kw)
            self.data['last_seen'] = time.time()

    def _run(self):
        if DEMO:
            self._run_demo()
            return
        from pymavlink import mavutil
        while True:
            try:
                m = mavutil.mavlink_connection(config.MAVLINK_URL,
                                               baud=config.MAVLINK_BAUD)
            except Exception:
                time.sleep(2.0)
                continue
            # veri akisini hizlandir
            try:
                m.wait_heartbeat(timeout=5)
                m.mav.request_data_stream_send(
                    m.target_system, m.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
            except Exception:
                pass
            self._loop(m)

    def _loop(self, m):
        while True:
            try:
                msg = m.recv_match(blocking=True, timeout=2.0)
            except Exception:
                time.sleep(0.5)
                break
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'SCALED_PRESSURE2':   # Bar30
                dp = (msg.press_abs - config.SURFACE_HPA) * 100.0
                depth = dp / (config.WATER_DENSITY * 9.80665)
                self._set(depth_m=round(max(0.0, depth), 2), depth_valid=True)
            elif t == 'DISTANCE_SENSOR':  # Ping sonar
                d = msg.current_distance / 100.0
                valid = config.SONAR_MIN_M <= d <= config.SONAR_MAX_M
                self._set(altitude_m=round(d, 2), sonar_valid=valid)
            elif t == 'ATTITUDE':
                self._set(roll=round(math.degrees(msg.roll), 1),
                          pitch=round(math.degrees(msg.pitch), 1),
                          yaw=round(math.degrees(msg.yaw) % 360, 1))
            elif t == 'SYS_STATUS':
                v = msg.voltage_battery / 1000.0
                self._set(voltage=round(v, 2) if v > 0 else None)
            # su sutunu
            with self._lock:
                if self.data['depth_valid'] and self.data['sonar_valid'] \
                        and self.data['depth_m'] is not None \
                        and self.data['altitude_m'] is not None:
                    self.data['water_column_m'] = round(
                        self.data['depth_m'] + self.data['altitude_m'], 2)

    def _run_demo(self):
        t0 = time.time()
        while True:
            t = time.time() - t0
            depth = round(0.6 + 0.25 * math.sin(t / 3), 2)
            alt = round(2.2 + 0.3 * math.cos(t / 4), 2)
            self._set(depth_m=depth, depth_valid=True, altitude_m=alt,
                      sonar_valid=True, water_column_m=round(depth + alt, 2),
                      roll=round(4 * math.sin(t / 5), 1),
                      pitch=round(3 * math.cos(t / 6), 1),
                      yaw=round((t * 6) % 360, 1), voltage=15.8)
            time.sleep(0.2)


# ============================================================ Flask
cameras = {}
mav = None


def mjpeg(cam):
    boundary = b'--frame'
    while True:
        jpg = cam.get_jpeg()
        if jpg is not None:
            yield (boundary + b'\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
        time.sleep(0.04)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video/<cam>')
def video(cam):
    if cam not in cameras:
        return 'yok', 404
    return Response(mjpeg(cameras[cam]),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/sensors')
def sensors():
    d = mav.snapshot()
    d['cameras'] = {k: v.status for k, v in cameras.items()}
    return jsonify(d)


def main():
    global DEMO, mav
    ap = argparse.ArgumentParser()
    ap.add_argument('--demo', action='store_true')
    args = ap.parse_args()
    DEMO = args.demo

    cameras['d435'] = CameraStream('D435', config.D435_SOURCES)
    cameras['minirov'] = CameraStream('Mini ROV', config.MINIROV_SOURCES)
    mav = MavReader()

    print(f'Web arayuzu: http://localhost:{config.PORT}  '
          f'({"DEMO" if DEMO else config.MAVLINK_URL})')
    app.run(host=config.HOST, port=config.PORT, threaded=True, debug=False)


if __name__ == '__main__':
    main()
