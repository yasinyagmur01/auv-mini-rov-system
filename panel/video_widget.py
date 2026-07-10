"""Video akis widget'i: cv2.VideoCapture arka plan thread'i + QLabel cizimi.

Kaynak listesi sirayla denenir: ('gst', pipeline) OpenCV GStreamer ile
derlenmisse; ('ffmpeg', sdp_yolu) FFMPEG arka ucu ile. Demo modda sentetik
goruntu uretilir.
"""
import threading
import time

import cv2
import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel


class _CaptureThread(threading.Thread):
    def __init__(self, sources, demo=False, demo_label=''):
        super().__init__(daemon=True)
        self.sources = sources
        self.demo = demo
        self.demo_label = demo_label
        self.frame = None
        self.lock = threading.Lock()
        self.status = 'baglaniyor...'

    def run(self):
        if self.demo:
            self._run_demo()
            return
        while True:
            cap = self._open()
            if cap is None:
                self.status = 'kaynak acilamadi, tekrar deneniyor'
                time.sleep(3.0)
                continue
            self.status = 'canli'
            while True:
                ok, frame = cap.read()
                if not ok:
                    self.status = 'akis kesildi'
                    cap.release()
                    time.sleep(1.0)
                    break
                with self.lock:
                    self.frame = frame

    def _open(self):
        for kind, src in self.sources:
            try:
                if kind == 'gst':
                    cap = cv2.VideoCapture(src, cv2.CAP_GSTREAMER)
                else:
                    cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
                if cap.isOpened():
                    return cap
                cap.release()
            except Exception:
                pass
        return None

    def _run_demo(self):
        t0 = time.monotonic()
        self.status = 'demo'
        while True:
            t = time.monotonic() - t0
            img = np.full((480, 640, 3), (90, 60, 20), np.uint8)  # koyu mavi ton
            x = int(320 + 200 * np.sin(t * 0.8))
            cv2.line(img, (x, 0), (640 - x, 480), (0, 0, 200), 24)  # "serit"
            cv2.putText(img, f'DEMO {self.demo_label} t={t:5.1f}s', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            with self.lock:
                self.frame = img
            time.sleep(1 / 30)


class VideoWidget(QLabel):
    def __init__(self, title, sources, demo=False):
        super().__init__()
        self.title = title
        self.setMinimumSize(480, 360)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet('background:#111; color:#888;')
        self.setText(f'{title}\n(baglaniyor...)')

        self.cap = _CaptureThread(sources, demo=demo, demo_label=title)
        self.cap.start()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(50)  # 20 fps cizim

    def _refresh(self):
        with self.cap.lock:
            frame = None if self.cap.frame is None else self.cap.frame.copy()
        if frame is None:
            self.setText(f'{self.title}\n({self.cap.status})')
            return
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        img = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        self.setPixmap(QPixmap.fromImage(img).scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
