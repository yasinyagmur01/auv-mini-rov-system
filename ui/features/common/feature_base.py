"""AUV UI ozellik (feature) process'lerinin temel sinifi.

Her ozellik AYRI bir dosya ve AYRI bir process'tir (discrete mimari):
  - Ilgili ROS 2 topic'lerine abone olur,
  - Veriyi hem KAYDEDER (session klasoru: stream.ndjson + frames/ + cloud/)
  - hem de CANLI yayinlar (stdout'a satir basina bir JSON -> backend -> WebSocket).
  - SIGINT ile durdurulunca pandas/numpy ANALITIGINI hesaplar (analytics.json)
    ve meta.json'i tamamlar.

Stdout protokolu (her satir bir JSON):
  {"feature": id, "t": epoch, "kind": "frame|scalar|pose|cloud|status",
   "channel": str, ...}
  frame : {"b64": jpeg-base64, "file": "frames/000001.jpg", "w":..,"h":..}
  cloud : {"count": N, "file": "cloud/000001.bin"} (+canli: "xyz","rgb" b64)
  scalar/pose: {"data": {...}}
"""

import base64
import io
import json
import os
import signal
import sys
import time
from collections import defaultdict, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from PIL import Image as PILImage


def sensor_qos(depth=2):
    """Her yayinci ile uyumlu abonelik QoS'u (BEST_EFFORT + VOLATILE)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class Throttle:
    """Basit hiz sinirlayici: hedef Hz'i asan cagrilarda False doner."""

    def __init__(self, hz):
        self.period = 1.0 / hz if hz > 0 else 0.0
        self._last = 0.0

    def ok(self):
        now = time.monotonic()
        if now - self._last >= self.period:
            self._last = now
            return True
        return False


class FeatureBase(Node):
    FEATURE_ID = 'base'          # alt sinif doldurur
    TITLE = 'Base'

    def __init__(self, session_dir):
        super().__init__(f'auv_ui_{self.FEATURE_ID}')
        self.session_dir = session_dir
        self.frames_dir = os.path.join(session_dir, 'frames')
        self.cloud_dir = os.path.join(session_dir, 'cloud')
        os.makedirs(self.frames_dir, exist_ok=True)
        os.makedirs(self.cloud_dir, exist_ok=True)

        self._stream = open(os.path.join(session_dir, 'stream.ndjson'), 'a',
                            buffering=1)
        self._t0 = time.time()
        self._counts = defaultdict(int)      # channel -> toplam mesaj
        self._rate_win = defaultdict(lambda: deque(maxlen=200))  # hz olcumu
        self._frame_idx = defaultdict(int)
        self._cloud_idx = 0
        self.series = defaultdict(list)      # analitik icin: name -> [(t, v)]
        self._status_thr = Throttle(1.0)

        self._write_meta(running=True)
        self._shutting_down = False
        self._cb_errors = 0
        signal.signal(signal.SIGINT, self._on_sigint)
        signal.signal(signal.SIGTERM, self._on_sigint)

    def create_subscription(self, msg_type, topic, callback, qos, **kw):
        """Tum abonelik callback'lerini hata zirhina alir: tek bir bozuk mesaj
        (beklenmedik encoding vb.) ozellik process'ini OLDURMEZ; hata stderr'e
        (feature.log) yazilir ve akis devam eder."""

        def guarded(msg, _cb=callback):
            try:
                _cb(msg)
            except Exception:  # noqa: BLE001
                self._cb_errors += 1
                if self._cb_errors <= 50:   # log tasmasin
                    import traceback
                    print(f'[callback hatasi #{self._cb_errors}] {topic}:',
                          file=sys.stderr)
                    traceback.print_exc()
        return super().create_subscription(msg_type, topic, guarded, qos, **kw)

    # ---------------- kayit + canli yayin ----------------

    def _stdout(self, obj):
        sys.stdout.write(json.dumps(obj, separators=(',', ':')) + '\n')
        sys.stdout.flush()

    def _record(self, obj):
        self._stream.write(json.dumps(obj, separators=(',', ':')) + '\n')

    def _base_msg(self, kind, channel):
        t = time.time()
        self._counts[channel] += 1
        self._rate_win[channel].append(t)
        return {'feature': self.FEATURE_ID, 't': round(t - self._t0, 3),
                'kind': kind, 'channel': channel}

    def emit_scalar(self, channel, data, kind='scalar'):
        m = self._base_msg(kind, channel)
        m['data'] = data
        self._record(m)
        self._stdout(m)
        self._maybe_status()

    def emit_pose(self, channel, data):
        self.emit_scalar(channel, data, kind='pose')

    def emit_frame(self, channel, rgb_or_gray, quality=70, live=True):
        """numpy goruntuyu JPEG olarak kaydet + canli gonder."""
        m = self._base_msg('frame', channel)
        img = PILImage.fromarray(rgb_or_gray)
        idx = self._frame_idx[channel]
        self._frame_idx[channel] += 1
        fname = f'frames/{channel}_{idx:06d}.jpg'
        path = os.path.join(self.session_dir, fname)
        buf = io.BytesIO()
        img.save(buf, format='JPEG', quality=quality)
        data = buf.getvalue()
        with open(path, 'wb') as fh:
            fh.write(data)
        m['file'] = fname
        m['w'], m['h'] = img.width, img.height
        self._record(m)
        if live:
            m2 = dict(m)
            m2['b64'] = base64.b64encode(data).decode('ascii')
            self._stdout(m2)
        self._maybe_status()

    def emit_cloud(self, channel, xyz, rgb, live_max=40000):
        """Nokta bulutu: kayda .bin (xyz f32 + rgb u8), canliya alt-orneklem b64."""
        m = self._base_msg('cloud', channel)
        n = xyz.shape[0]
        fname = f'cloud/{channel}_{self._cloud_idx:06d}.bin'
        self._cloud_idx += 1
        with open(os.path.join(self.session_dir, fname), 'wb') as fh:
            fh.write(np.ascontiguousarray(xyz, dtype='<f4').tobytes())
            fh.write(np.ascontiguousarray(rgb, dtype=np.uint8).tobytes())
        m['count'] = int(n)
        m['file'] = fname
        self._record(m)

        if n > live_max:
            sel = np.random.default_rng(0).choice(n, live_max, replace=False)
            xyz_l, rgb_l = xyz[sel], rgb[sel]
        else:
            xyz_l, rgb_l = xyz, rgb
        m2 = dict(m)
        m2['live_count'] = int(xyz_l.shape[0])
        m2['xyz'] = base64.b64encode(
            np.ascontiguousarray(xyz_l, dtype='<f4').tobytes()).decode('ascii')
        m2['rgb'] = base64.b64encode(
            np.ascontiguousarray(rgb_l, dtype=np.uint8).tobytes()).decode('ascii')
        self._stdout(m2)
        self._maybe_status()

    def record_series(self, name, value, t=None):
        self.series[name].append((round((t or time.time()) - self._t0, 3),
                                  float(value)))

    def _maybe_status(self):
        if not self._status_thr.ok():
            return
        now = time.time()
        rates = {}
        for ch, win in self._rate_win.items():
            recent = [x for x in win if now - x <= 5.0]
            rates[ch] = round(len(recent) / 5.0, 1)
        self._stdout({'feature': self.FEATURE_ID, 't': round(now - self._t0, 3),
                      'kind': 'status', 'channel': '_status',
                      'data': {'rates': rates,
                               'counts': dict(self._counts),
                               'uptime_s': round(now - self._t0, 1)}})

    # ---------------- yasam dongusu ----------------

    def _write_meta(self, running):
        meta = {
            'feature': self.FEATURE_ID,
            'title': self.TITLE,
            'session_id': os.path.basename(self.session_dir),
            'started_at': time.strftime('%Y-%m-%d %H:%M:%S',
                                        time.localtime(self._t0)),
            'running': running,
        }
        if not running:
            meta['ended_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
            meta['duration_s'] = round(time.time() - self._t0, 1)
            meta['message_counts'] = dict(self._counts)
        with open(os.path.join(self.session_dir, 'meta.json'), 'w') as fh:
            json.dump(meta, fh, indent=1)

    def series_df(self):
        """Analitik icin: series sozlugunu pandas DataFrame'lerine cevir."""
        import pandas as pd
        return {name: pd.DataFrame(vals, columns=['t', name])
                for name, vals in self.series.items() if vals}

    @staticmethod
    def downsample(pairs, max_pts=2000):
        """[(t,v)] listesini grafik icin en fazla max_pts noktaya indir."""
        if len(pairs) <= max_pts:
            return [[round(t, 3), round(v, 5)] for t, v in pairs]
        idx = np.linspace(0, len(pairs) - 1, max_pts).astype(int)
        return [[round(pairs[i][0], 3), round(pairs[i][1], 5)] for i in idx]

    def compute_analytics(self):
        """Alt sinif doldurur: {'summary': {...}, 'charts': [...]} dondurur.

        charts elemani: {'id','title','unit','series':[{'name','points':[[t,v],..]}]}
        """
        return {'summary': {}, 'charts': []}

    def _on_sigint(self, *_):
        if self._shutting_down:
            return
        self._shutting_down = True
        raise KeyboardInterrupt

    def finalize(self):
        try:
            analytics = self.compute_analytics()
            analytics['generated_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
            with open(os.path.join(self.session_dir, 'analytics.json'), 'w') as fh:
                json.dump(analytics, fh)
        except Exception as exc:  # noqa: BLE001 — analitik hatasi kaydi bozmasin
            with open(os.path.join(self.session_dir, 'analytics_error.txt'),
                      'w') as fh:
                import traceback
                fh.write(f'{exc}\n{traceback.format_exc()}')
        self._write_meta(running=False)
        self._stream.close()


def run_feature(cls):
    """Ozellik dosyasinin main'i: --session-dir argumaniyla calistirir."""
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--session-dir', required=True)
    args = ap.parse_args()
    os.makedirs(args.session_dir, exist_ok=True)

    rclpy.init()
    node = cls(args.session_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        rclpy.try_shutdown()
