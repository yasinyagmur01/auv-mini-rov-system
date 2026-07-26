#!/usr/bin/env python3
"""MOCK kokpit host'u — GERCEK arac/kopruye DOKUNMAZ (ayri makinede/portta calisir).

Amac: veri gelsin gelmesin TUM panelleri (3D VO, Sonar haritasi, Hat Takibi sim,
Operasyon, 3D SLAM nokta bulutu, telemetri) SAHTE veriyle besleyip "ne ne degilmis"
gormek. cockpit.html'i oldugu gibi servis eder; sadece veri uclarini mock'lar.

Calistir (kontrol PC'sinde, .196):
    pip install pillow numpy    # (zaten var)
    python3 mock_host.py         # http://127.0.0.1:8001/  (WS :8765, sim :8091 alias)

Sonra tarayicida:  http://127.0.0.1:8001/
  - /sensors, /pointcloud, kameralar, WS :8765 hepsi sahte.
  - Hat Takibi (Kontrol modu) icin SIM_URL varsayilani 127.0.0.1:8091 -> bu host onu da sunar.
  - Sonar haritasi: WS'ten gelen sahte DR (daire ciziyor) + gecerli dip -> harita dolar.
  - Nokta bulutu: bilinen "oda kosesi" sekli (zemin + on duvar) -> EKSEN dogru mu gorunur.

GERCEK panel ayri: http://192.168.2.135:8000/  (bu mock onu etkilemez).
"""
import base64, hashlib, io, json, math, os, socket, struct, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
# Gercek arac paneliyle karistirilmasin diye MOCK'a gorunur bant enjekte edilir (yalniz mock).
MOCK_BANNER = ('<div style="position:fixed;bottom:8px;left:50%;transform:translateX(-50%);'
               'z-index:99999;background:#b91c1c;color:#fff;font:700 12px system-ui;'
               'padding:4px 14px;border-radius:8px;box-shadow:0 2px 8px rgba(0,0,0,.5);'
               'pointer-events:none;opacity:.93">⚠ MOCK VERİ — GERÇEK ARAÇ DEĞİL (yalnız arayüz testi)</div>')
T0 = time.time()
def _t():
    return time.time() - T0


# ---------------- sahte nokta bulutu: "oda kosesi" (eksen dogrulama icin) ----------------
# ZED optik cerceve: x=sag, y=asagi, z=ileri. Zemin y=+ (asagi), on duvar z=+ (ileri).
# Dogru render'da: zemin ALTTA, duvar ILERIDE gorunmeli.
def _build_room():
    pts = []
    # zemin (y = +1.2 asagi), x in [-2,2], z in [0.4,4.4]
    for i in range(1600):
        x = -2 + 4 * (i % 40) / 39.0
        z = 0.4 + 4.0 * (i // 40) / 39.0
        pts += [round(x, 2), 1.2, round(z, 2)]
    # on duvar (z = 4.4 ileri), x in [-2,2], y in [-1.2,1.2]
    for i in range(1000):
        x = -2 + 4 * (i % 40) / 39.0
        y = -1.2 + 2.4 * (i // 40) / 24.0
        pts += [round(x, 2), round(y, 2), 4.4]
    # sol duvar (x = -2 sol), z in [0.4,4.4], y in [-1.2,1.2]
    for i in range(600):
        z = 0.4 + 4.0 * (i % 30) / 29.0
        y = -1.2 + 2.4 * (i // 30) / 19.0
        pts += [-2.0, round(y, 2), round(z, 2)]
    return {'n': len(pts) // 3, 'pts': pts}
ROOM = _build_room()


# ---------------- sahte /sensors ----------------
def mock_sensors():
    t = _t()
    depth = round(1.5 + 0.6 * math.sin(t / 5), 2)
    alt = round(2.6 + 1.6 * math.sin(t / 7 + 1), 2)          # 0.3-30 icinde -> sonar_valid
    return {
        'depth_m': depth, 'depth_valid': True,
        'altitude_m': alt, 'sonar_valid': (0.3 <= alt <= 30.0),
        'water_column_m': round(depth + alt, 2),
        'yaw': round((t * 12) % 360, 1),
        'roll': round(5 * math.sin(t / 4), 1), 'pitch': round(4 * math.cos(t / 6), 1),
        'voltage': round(15.8 - 0.01 * (t % 120), 2),
        'current_a': round(9 + 3 * abs(math.sin(t / 4)), 1),
        'battery_pct': max(0, round(100 - (t % 120) * 0.6)),
        'armed': (int(t / 8) % 2 == 1), 'mode': 'ALT_HOLD',
        'statustext': 'MOCK: sistem nominal',
        'leak': (True if (int(t) % 47 == 0) else None),        # arada bir alarm (mandal test)
        'gps_lat': round(40.7891 + 0.0001 * math.sin(t / 20), 7),
        'gps_lon': round(29.4501 + 0.0001 * math.cos(t / 20), 7),
        'gps_fix': 3, 'gps_sats': 11, 'gps_hdop': 0.8,
        'water_temp_c': round(19 + 0.5 * math.sin(t / 30), 1),
        # CUAV ADC ic/kart sicakligi (MOCK): ucgen dalga 35<->100 -> 90C uyarisi test
        'board_temp_c': round(35.0 + 65.0 * (1 - abs((t % 80) - 40) / 40.0), 1),
        'link_ok': True,
        'mrov_link': True, 'mrov_leak': None, 'mrov_temp_c': 20.1,
        'mrov_voltage': 15.5, 'mrov_current_a': 3.2, 'mrov_battery_pct': 78, 'mrov_armed': False,
        'zed_pose': {'x': round(2.2 * math.sin(t / 12), 2), 'y': round(2.2 * math.cos(t / 15), 2),
                     'yaw': round((t * 8) % 360, 1), 'demo': True},
        'connected': True,
        'cameras': {'zed': 'MOCK'},
    }


# DR daire: sonar haritasi + kontrol harita izi dolsun diye
def mock_dr():
    t = _t()
    r = 3 + 0.4 * t % 6
    return {'origin_valid': True, 'est_lat': 40.7891, 'est_lon': 29.4501,
            'x_east_m': round(4 * math.sin(t / 6), 2), 'y_north_m': round(4 * math.cos(t / 6), 2),
            'speed_mps': round(0.4 + 0.2 * abs(math.sin(t / 3)), 2)}


def mock_state():
    t = _t()
    phase = int(t / 10) % 3
    st = ['IDLE', 'RUNNING', 'RUNNING'][phase]
    step = ['—', 'hat takip ediliyor', 'hedefe yaklasiliyor'][phase]
    return {'type': 'state',
            'mission': {'state_name': st, 'step_name': (None if phase == 0 else step),
                        'step_index': phase, 'step_count': 3,
                        'message': ('MOCK gorev bekliyor' if phase == 0 else 'MOCK: ' + step)},
            'telemetry': {'depth_m': round(1.5 + 0.6 * math.sin(t / 5), 2),
                          'altitude_m': round(2.6 + 1.6 * math.sin(t / 7 + 1), 2),
                          'altitude_valid': True, 'heading_deg': round((t * 12) % 360, 1),
                          'armed': (int(t / 8) % 2 == 1), 'mode': 'ALT_HOLD',
                          'motors': [1500] * 8, 'dr': mock_dr(),
                          'lane_status': ['—', 'cizgi izleniyor', 'cizgi izleniyor'][phase],
                          'battery': {'voltage': 15.6, 'current': 9.1, 'percentage': 0.78},
                          'statustext': 'MOCK nominal', 'leak': None}}


# ---------------- sahte goruntuler ----------------
def _jpeg(im):
    b = io.BytesIO(); im.save(b, 'JPEG', quality=70); return b.getvalue()

def cam_frame():
    t = _t()
    im = Image.new('RGB', (640, 400), (24, 40, 52)); d = ImageDraw.Draw(im)
    x = int(320 + 220 * math.sin(t * 0.8))
    d.ellipse([x - 26, 180, x + 26, 232], outline=(120, 220, 255), width=4)
    d.text((14, 12), 'MOCK ZED 2i', fill=(255, 255, 255))
    d.text((14, 372), time.strftime('%H:%M:%S'), fill=(180, 200, 210))
    return _jpeg(im)

def lane_frame():
    t = _t()
    im = Image.new('RGB', (480, 360), (20, 30, 40)); d = ImageDraw.Draw(im)
    for yy in range(0, 360, 8):
        cx = 240 + 90 * math.sin((yy / 360.0 * 3) + t * 0.6)   # kivrimli hat
        d.line([cx - 10, yy, cx - 10, yy + 6], fill=(240, 210, 80), width=6)
        d.line([cx + 10, yy, cx + 10, yy + 6], fill=(240, 210, 80), width=6)
    d.line([240, 0, 240, 360], fill=(90, 90, 90), width=1)      # merkez referans
    d.text((10, 10), 'MOCK hat-takibi sim', fill=(255, 255, 255))
    return _jpeg(im)

def lane_data():
    t = _t()
    lost = (int(t) % 23 == 0)
    off = None if lost else round(0.35 * math.sin(t * 0.6), 3)
    yaw = None if lost else round(0.5 * math.sin(t * 0.6 + 0.3), 3)
    base = 1500
    return {'offset': off, 'yaw': yaw,
            'pwm_left': base + (0 if lost else int(120 * math.sin(t * 0.6))),
            'pwm_right': base - (0 if lost else int(120 * math.sin(t * 0.6))),
            'line_lost': lost}


# ---------------- HTTP ----------------
def _read_file(name):
    with open(os.path.join(HERE, name), encoding='utf-8') as f:
        return f.read()

def _mjpeg(gen):
    while True:
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + gen() + b'\r\n'
        time.sleep(0.06)


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype='application/json', code=200):
        if isinstance(body, str):
            body = body.encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except Exception:
            pass

    def _stream(self, gen):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        try:
            for chunk in _mjpeg(gen):
                self.wfile.write(chunk)
        except Exception:
            pass

    def do_POST(self):
        self._send(json.dumps({'ok': True, 'mock': True}))

    def do_GET(self):
        p = self.path.split('?')[0]
        if p in ('/', '/control', '/test', '/ops'):
            html = _read_file('cockpit.html').replace('<body>', '<body>' + MOCK_BANNER, 1)
            self._send(html, 'text/html; charset=utf-8')
        elif p == '/panel.css':
            self._send(_read_file('panel.css'), 'text/css')
        elif p == '/minirov':
            try:
                self._send(_read_file('minirov.html'), 'text/html; charset=utf-8')
            except Exception:
                self._send('<h3 style="color:#ccc;font-family:sans-serif">MOCK Mini ROV video</h3>', 'text/html')
        elif p == '/sensors':
            self._send(json.dumps(mock_sensors()))
        elif p == '/pointcloud':
            self._send(json.dumps(ROOM))
        elif p == '/data':
            self._send(json.dumps(lane_data()))
        elif p == '/stream':
            self._stream(lane_frame)
        elif p.startswith('/video/') or p == '/stream/color':
            self._stream(cam_frame)
        else:
            self._send(json.dumps({'ok': False, 'mock': True, 'path': p}), code=404)


# ---------------- WebSocket (:8765) — sahte mission state ----------------
WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

def ws_handshake(conn):
    data = b''
    conn.settimeout(3.0)
    try:
        while b'\r\n\r\n' not in data:
            chunk = conn.recv(1024)
            if not chunk:
                return False
            data += chunk
    except Exception:
        return False
    key = None
    for line in data.decode('utf-8', 'ignore').split('\r\n'):
        if line.lower().startswith('sec-websocket-key:'):
            key = line.split(':', 1)[1].strip()
    if not key:
        return False
    acc = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
    conn.sendall(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n'
                  'Connection: Upgrade\r\nSec-WebSocket-Accept: ' + acc + '\r\n\r\n').encode())
    return True

def ws_send(conn, text):
    b = text.encode('utf-8'); n = len(b); hdr = bytearray([0x81])
    if n < 126:
        hdr.append(n)
    elif n < 65536:
        hdr.append(126); hdr += struct.pack('>H', n)
    else:
        hdr.append(127); hdr += struct.pack('>Q', n)
    conn.sendall(bytes(hdr) + b)

def ws_client(conn):
    if not ws_handshake(conn):
        try: conn.close()
        except Exception: pass
        return
    try:
        ws_send(conn, json.dumps({'type': 'missions', 'missions': ['video_pattern', 'kare_tur']}))
    except Exception:
        return
    conn.settimeout(0.05)
    while True:
        try:
            try:
                h = conn.recv(2)
                if h == b'':
                    break
                if h and len(h) == 2:
                    ln = h[1] & 0x7f
                    if ln == 126:
                        ln = struct.unpack('>H', conn.recv(2))[0]
                    elif ln == 127:
                        ln = struct.unpack('>Q', conn.recv(8))[0]
                    if h[1] & 0x80:
                        conn.recv(4)
                    if ln:
                        conn.recv(ln)
            except socket.timeout:
                pass
            ws_send(conn, json.dumps(mock_state()))
            time.sleep(0.2)
        except Exception:
            break
    try:
        conn.close()
    except Exception:
        pass

def ws_serve(port=8765):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', port)); s.listen(8)
    while True:
        conn, _ = s.accept()
        threading.Thread(target=ws_client, args=(conn,), daemon=True).start()


def main():
    ThreadingHTTPServer.daemon_threads = True
    for port in (8001, 8091):     # 8091 = sim alias (SIM_URL varsayilani)
        srv = ThreadingHTTPServer(('0.0.0.0', port), H)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=ws_serve, args=(8765,), daemon=True).start()
    print('MOCK kokpit: http://127.0.0.1:8001/   (WS :8765, sim :8091)  — GERCEGE dokunmaz')
    print('Gercek panel ayri: http://192.168.2.135:8000/')
    while True:
        time.sleep(3600)


if __name__ == '__main__':
    main()
