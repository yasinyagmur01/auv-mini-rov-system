#!/usr/bin/env python3
"""ROS2 -> Web koprusu (Jetson, internetsiz - Flask+rclpy+PIL).

rosbridge/web_video_server apt gerektiriyor ama Jetson'in interneti yok.
Bu kopru mevcut paketlerle ayni isi yapar: ROS2 topic'lerine abone olur,
goruntuleri MJPEG, sayisal verileri JSON olarak servis eder.

Veri akisi: ROS2 node'lari (d435, zed, ileride mav_bridge) -> topic'ler ->
bu kopru (abone) -> web. Yani her sey ROS2'den gecer.

Portlar/uc noktalar (varsayilan :8000):
  /                       panel sayfasi
  /video/<ad>            MJPEG (ad: d435, zed, ...)
  /stream/color          d435 rengi (rsweb UYUMLU - PC paneli bozulmaz)
  /sensors               JSON (derinlik/sonar/pruva + kamera durumu)

Calistirma (Jetson, ROS2 sourcelu):
  source /opt/ros/humble/setup.bash
  python3 ros2_web_bridge.py --port 8000
"""
import argparse
import io
import threading
import time

import numpy as np
from PIL import Image as PILImage
from flask import Flask, Response, jsonify, render_template_string

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Float32

# Abone olunacak goruntu topic'leri: ekranda gosterilecek ad -> ROS2 topic
IMAGE_TOPICS = {
    'd435': '/camera/color/image_raw',
    'zed':  '/zed/zed_node/rgb/image_rect_color',
}
# Sensor topic'leri (V6X Jetson'a tasininca mav_bridge yayinlar - Faz 2)
DEPTH_TOPIC = '/mav/depth'          # std_msgs/Float32 (m)
RANGE_TOPIC = '/mav/rangefinder'    # sensor_msgs/Range (m)
HEADING_TOPIC = '/mav/heading_deg'  # std_msgs/Float32 (deg)

JPEG_QUALITY = 70


def img_to_jpeg(msg):
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


class BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_web_bridge')
        self.jpegs = {name: None for name in IMAGE_TOPICS}
        self.stamps = {name: 0.0 for name in IMAGE_TOPICS}
        self.locks = {name: threading.Lock() for name in IMAGE_TOPICS}
        self.sensors = {'depth_m': None, 'altitude_m': None, 'yaw': None,
                        'depth_valid': False, 'sonar_valid': False}
        self.sensor_stamp = 0.0

        # goruntu QoS: best-effort (hem reliable hem best-effort publisher'la uyumlu)
        img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        for name, topic in IMAGE_TOPICS.items():
            self.create_subscription(Image, topic, self._img_cb(name), img_qos)

        sqos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float32, DEPTH_TOPIC, self._depth_cb, sqos)
        self.create_subscription(Range, RANGE_TOPIC, self._range_cb, sqos)
        self.create_subscription(Float32, HEADING_TOPIC, self._heading_cb, sqos)
        self.get_logger().info('ROS2 web koprusu hazir. Topic aboneleri kuruldu.')

    def _img_cb(self, name):
        def cb(msg):
            jpg = img_to_jpeg(msg)
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

    def get_jpeg(self, name):
        with self.locks.get(name, threading.Lock()):
            return self.jpegs.get(name)

    def cam_status(self, name):
        fresh = (time.time() - self.stamps.get(name, 0)) < 2.0
        return 'canli (ROS2)' if (self.jpegs.get(name) and fresh) else 'topic yok'

    def snapshot(self):
        s = dict(self.sensors)
        if self.sensors['depth_valid'] and self.sensors['sonar_valid'] \
                and self.sensors['depth_m'] is not None \
                and self.sensors['altitude_m'] is not None:
            s['water_column_m'] = round(self.sensors['depth_m'] + self.sensors['altitude_m'], 2)
        else:
            s['water_column_m'] = None
        s['cameras'] = {name: self.cam_status(name) for name in IMAGE_TOPICS}
        return s


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
<span style=flex:1></span><span class=pill>veriler ROS2 topic'lerinden</span></header>
<main>
<div class=cams>
<div class=card><h2>D435 (ROS2) <span class=st id=st-d435>—</span></h2><img src="/video/d435"></div>
<div class=card><h2>ZED 2i (ROS2) <span class=st id=st-zed>—</span></h2><img src="/video/zed"></div>
</div>
<div>
<div class=card><h2>Dikey Eksen (Bar30+Sonar, ROS2)</h2>
<div class=big>
<div class=metric id=m-d><div class=label>Derinlik</div><div class=val><span id=depth>—</span> <small>m</small></div></div>
<div class=metric id=m-a><div class=label>Dipten Yükseklik</div><div class=val><span id=alt>—</span> <small>m</small></div></div>
<div class=metric id=m-c style="grid-column:1/3"><div class=label>Su Sütunu</div><div class=val><span id=col>—</span> <small>m</small></div></div>
</div></div>
<div class=card style=margin-top:14px><h2>IMU</h2><div class=att>
<div><span class=k>Yaw</span><span class=v id=yaw>—</span></div>
<div><span class=k>Derinlik geçerli</span><span class=v id=dv>—</span></div>
<div><span class=k>Sonar geçerli</span><span class=v id=sv>—</span></div>
</div></div>
</div></main>
<script>
function setm(id,el,v,valid){const b=document.getElementById(id);if(v==null){el.textContent='—';b.classList.add('stale');return}el.textContent=(+v).toFixed(2);b.classList.toggle('stale',valid===false)}
async function poll(){try{const d=await(await fetch('/sensors')).json();
const anycam=Object.values(d.cameras||{}).some(s=>s.includes('canli'));
const lk=document.getElementById('link');
const on=anycam||d.depth_m!=null;lk.textContent=on?'ROS2 bağlı':'veri bekleniyor';lk.className='pill '+(on?'ok':'bad');
setm('m-d',document.getElementById('depth'),d.depth_m,d.depth_valid);
setm('m-a',document.getElementById('alt'),d.altitude_m,d.sonar_valid);
setm('m-c',document.getElementById('col'),d.water_column_m,d.depth_valid&&d.sonar_valid);
document.getElementById('yaw').textContent=d.yaw!=null?d.yaw+'°':'—';
document.getElementById('dv').textContent=d.depth_valid?'✓':'—';
document.getElementById('sv').textContent=d.sonar_valid?'✓':'—';
if(d.cameras){document.getElementById('st-d435').textContent=d.cameras.d435||'—';document.getElementById('st-zed').textContent=d.cameras.zed||'—'}
}catch(e){document.getElementById('link').textContent='sunucu yok';document.getElementById('link').className='pill bad'}}
setInterval(poll,300);poll();
</script></body></html>"""


@app.route('/')
def index():
    return render_template_string(PAGE)


@app.route('/video/<name>')
def video(name):
    if name not in IMAGE_TOPICS:
        return 'yok', 404
    return Response(mjpeg(name), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stream/color')      # rsweb UYUMLU - mevcut PC paneli bunu kullaniyor
def stream_color():
    return Response(mjpeg('d435'), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/sensors')
def sensors():
    return jsonify(node.snapshot())


def main():
    global node
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8000)
    args = ap.parse_args()

    rclpy.init()
    node = BridgeNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    print(f'ROS2 web koprusu: http://0.0.0.0:{args.port}')
    app.run(host='0.0.0.0', port=args.port, threaded=True)


if __name__ == '__main__':
    main()
