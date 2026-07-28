# -*- coding: utf-8 -*-
"""
GPS Saha Arayuzu — u-blox ZED-F9P canli izleme (konum + uydu).

Dogrudan F9P'nin USB seri portundan okur (varsayilan COM10 = rover).
NMEA (GGA/RMC/VTG) + UBX-NAV-SAT (izlenen/sinyalli/kullanilan uydu, C/N0, gok konumu)
cozer ve bir web panelde canli gosterir:
  - Konum (harita + buyuk enlem/boylam), yukseklik, HDOP, fix tipi, hiz
  - 3 uydu metrigi: IZLENEN / SINYALLI (C/N0>0) / KULLANILAN + "25 uydu" rozeti
  - Takim kirilimi (GPS/GLONASS/Galileo/BeiDou/SBAS/QZSS)
  - Sinyal (C/N0) cubuklari ve gok haritasi (skyplot)

Kullanim:
  python scripts/gps_web.py            # COM10, http://localhost:8008
  python scripts/gps_web.py COM6 8009  # port + http portu sec

Salt-okunur: alicinin ayarlarini DEGISTIRMEZ (yalniz NAV-SAT poll gonderir).
"""
import sys, time, json, threading, math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial
except ImportError:
    print("HATA: pyserial gerekli ->  pip install pyserial")
    sys.exit(1)

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
HTTP_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8008
BAUD = 115200

GNSS = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou", 5: "QZSS", 6: "GLONASS"}
FIX_Q = {"0": "FIX YOK", "1": "3D (GPS)", "2": "DGPS", "4": "RTK FIXED",
         "5": "RTK FLOAT", "6": "Dead Reckoning"}

state = {
    "ok": False, "port": PORT, "err": None, "ts": 0,
    "fix": None, "lat": None, "lon": None, "alt": None,
    "hdop": None, "used_gga": None, "speed_kmh": None, "course": None, "utc": None,
    "tracked": 0, "signal": 0, "used": 0,
    "by_const": {}, "sats": [],
}
lock = threading.Lock()


def nmea_deg(v, hemi):
    if not v:
        return None
    try:
        f = float(v)
    except ValueError:
        return None
    d = int(f / 100)
    m = f - d * 100
    dec = d + m / 60.0
    if hemi in ("S", "W"):
        dec = -dec
    return dec


def ubx(cls, mid, payload=b""):
    body = bytes([cls, mid, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def parse_nmea(line):
    body = line[1:].split("*")[0]
    p = body.split(",")
    if len(p) < 2:
        return
    code = p[0][2:] if len(p[0]) >= 5 else p[0]
    with lock:
        if code == "GGA" and len(p) >= 10:
            state["fix"] = FIX_Q.get(p[6], p[6])
            lat = nmea_deg(p[2], p[3])
            lon = nmea_deg(p[4], p[5])
            if lat is not None:
                state["lat"] = round(lat, 7)
            if lon is not None:
                state["lon"] = round(lon, 7)
            state["used_gga"] = int(p[7]) if p[7] else None
            state["hdop"] = float(p[8]) if p[8] else None
            state["alt"] = float(p[9]) if p[9] else None
        elif code == "RMC" and len(p) >= 9:
            if p[7]:
                state["speed_kmh"] = round(float(p[7]) * 1.852, 2)
            if p[8]:
                state["course"] = round(float(p[8]), 1)
            if p[1]:
                state["utc"] = f"{p[1][0:2]}:{p[1][2:4]}:{p[1][4:6]} UTC"
        elif code == "VTG" and len(p) >= 8 and p[6]:
            try:
                state["speed_kmh"] = round(float(p[6]), 2)
            except ValueError:
                pass


def parse_navsat(pl):
    if len(pl) < 8:
        return
    numSvs = pl[5]
    tracked = signal = used = 0
    by_const = {}
    sats = []
    off = 8
    for _ in range(numSvs):
        if off + 12 > len(pl):
            break
        gnssId = pl[off]
        svId = pl[off + 1]
        cno = pl[off + 2]
        elev = int.from_bytes(pl[off + 3:off + 4], "little", signed=True)
        azim = int.from_bytes(pl[off + 4:off + 6], "little", signed=True)
        flags = int.from_bytes(pl[off + 8:off + 12], "little")
        svUsed = bool(flags & 0x08)
        name = GNSS.get(gnssId, f"id{gnssId}")
        c = by_const.setdefault(name, {"tracked": 0, "signal": 0, "used": 0})
        c["tracked"] += 1
        tracked += 1
        if cno > 0:
            c["signal"] += 1
            signal += 1
        if svUsed:
            c["used"] += 1
            used += 1
        sats.append({"g": name, "sv": svId, "cno": cno,
                     "el": elev, "az": azim, "u": svUsed})
        off += 12
    sats.sort(key=lambda s: (-s["cno"], s["g"]))
    with lock:
        state["tracked"] = tracked
        state["signal"] = signal
        state["used"] = used
        state["by_const"] = by_const
        state["sats"] = sats
        state["ts"] = time.time()
        state["ok"] = True


def reader():
    while True:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=1)
        except Exception as e:
            with lock:
                state["ok"] = False
                state["err"] = str(e)
            time.sleep(2)
            continue
        with lock:
            state["err"] = None
        buf = bytearray()
        last_poll = 0
        try:
            while True:
                if time.time() - last_poll > 1.0:
                    ser.write(ubx(0x01, 0x35))  # NAV-SAT poll (read-only)
                    last_poll = time.time()
                chunk = ser.read(1024)
                if chunk:
                    buf += chunk
                i = 0
                while i < len(buf):
                    b = buf[i]
                    if b == 0xB5 and i + 1 < len(buf) and buf[i + 1] == 0x62:
                        if i + 6 > len(buf):
                            break
                        ln = buf[i + 4] | (buf[i + 5] << 8)
                        if i + 6 + ln + 2 > len(buf):
                            break
                        cls, mid = buf[i + 2], buf[i + 3]
                        pl = buf[i + 6:i + 6 + ln]
                        if cls == 0x01 and mid == 0x35:
                            parse_navsat(bytes(pl))
                        i += 6 + ln + 2
                        continue
                    if b == 0x24:  # '$'
                        nl = buf.find(b"\n", i)
                        if nl == -1:
                            break
                        try:
                            parse_nmea(buf[i:nl].decode("ascii", "ignore").strip())
                        except Exception:
                            pass
                        i = nl + 1
                        continue
                    i += 1
                buf = buf[i:]
                if len(buf) > 8192:
                    buf = buf[-2048:]
        except Exception as e:
            with lock:
                state["ok"] = False
                state["err"] = str(e)
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(2)


PAGE = """<!doctype html><html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>GPS Saha Arayuzu</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<style>
:root{--bg:#0d1117;--card:#161b22;--line:#232a33;--fg:#e6edf3;--mut:#8b949e;
--ok:#3fb950;--warn:#d29922;--bad:#f85149;--acc:#4f8cff}
*{box-sizing:border-box}body{margin:0;font:14px/1.4 system-ui,Segoe UI,Roboto,sans-serif;
background:var(--bg);color:var(--fg)}
header{display:flex;align-items:center;gap:12px;padding:12px 16px;border-bottom:1px solid var(--line);
background:var(--card);position:sticky;top:0;z-index:500;flex-wrap:wrap}
header h1{font-size:16px;margin:0;font-weight:600}
.badge{padding:4px 10px;border-radius:20px;font-weight:700;font-size:12px}
.b-ok{background:rgba(63,185,80,.15);color:var(--ok);border:1px solid var(--ok)}
.b-warn{background:rgba(210,153,34,.15);color:var(--warn);border:1px solid var(--warn)}
.b-bad{background:rgba(248,81,73,.15);color:var(--bad);border:1px solid var(--bad)}
.mut{color:var(--mut)}
main{display:grid;grid-template-columns:1fr 1fr;gap:14px;padding:14px;max-width:1200px;margin:0 auto}
@media(max-width:820px){main{grid-template-columns:1fr}}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px}
.card h2{font-size:12px;letter-spacing:.06em;text-transform:uppercase;color:var(--mut);margin:0 0 10px}
.tiles{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
.tile{background:var(--bg);border:1px solid var(--line);border-radius:10px;padding:12px;text-align:center}
.tile .n{font-size:34px;font-weight:800;line-height:1}
.tile .l{font-size:11px;color:var(--mut);margin-top:6px;text-transform:uppercase;letter-spacing:.04em}
.t-track .n{color:var(--acc)}.t-sig .n{color:var(--ok)}.t-use .n{color:var(--warn)}
.coord{font-size:26px;font-weight:800;font-variant-numeric:tabular-nums;word-break:break-all}
.kv{display:grid;grid-template-columns:auto 1fr;gap:6px 14px;margin-top:10px}
.kv .k{color:var(--mut)}.kv .v{text-align:right;font-variant-numeric:tabular-nums;font-weight:600}
#map{height:300px;border-radius:10px;border:1px solid var(--line)}
.maplinks{display:flex;gap:8px;margin-top:8px;flex-wrap:wrap}
a.btn,button.btn{background:var(--bg);border:1px solid var(--line);color:var(--fg);
text-decoration:none;padding:7px 12px;border-radius:8px;font-size:13px;cursor:pointer}
a.btn:hover,button.btn:hover{border-color:var(--acc)}
.const{display:grid;grid-template-columns:auto 1fr auto;gap:6px 10px;align-items:center}
.dot{width:10px;height:10px;border-radius:50%}
.bar{height:8px;background:var(--bg);border-radius:5px;overflow:hidden}
.bar>i{display:block;height:100%}
.sig{display:flex;align-items:flex-end;gap:3px;height:130px;overflow-x:auto;padding-top:6px}
.sig .s{min-width:14px;display:flex;flex-direction:column;align-items:center;gap:3px}
.sig .s .b{width:12px;border-radius:3px 3px 0 0}
.sig .s .lab{font-size:9px;color:var(--mut);writing-mode:vertical-rl}
.full{grid-column:1/-1}
.legend{display:flex;gap:14px;flex-wrap:wrap;font-size:12px;margin-top:8px}
.legend span{display:flex;align-items:center;gap:5px}
.stale{opacity:.5}
</style></head><body>
<header>
  <h1>🛰️ GPS Saha Arayuzu</h1>
  <span id="fixb" class="badge b-warn">baglaniyor…</span>
  <span id="critb" class="badge b-warn">25 uydu: —</span>
  <span class="mut" id="portlbl"></span>
  <span class="mut" id="age" style="margin-left:auto"></span>
</header>
<main>
  <section class="card">
    <h2>Uydu</h2>
    <div class="tiles">
      <div class="tile t-track"><div class="n" id="n-track">0</div><div class="l">İzlenen</div></div>
      <div class="tile t-sig"><div class="n" id="n-sig">0</div><div class="l">Sinyalli C/N0&gt;0</div></div>
      <div class="tile t-use"><div class="n" id="n-use">0</div><div class="l">Kullanılan</div></div>
    </div>
    <div style="margin-top:14px" class="const" id="const"></div>
  </section>

  <section class="card">
    <h2>Konum</h2>
    <div class="coord" id="coord">—</div>
    <div class="kv">
      <div class="k">Fix</div><div class="v" id="v-fix">—</div>
      <div class="k">Yükseklik</div><div class="v" id="v-alt">—</div>
      <div class="k">HDOP</div><div class="v" id="v-hdop">—</div>
      <div class="k">Hız</div><div class="v" id="v-spd">—</div>
      <div class="k">Yön</div><div class="v" id="v-crs">—</div>
      <div class="k">UTC</div><div class="v" id="v-utc">—</div>
    </div>
    <div class="maplinks">
      <a class="btn" id="gmap" target="_blank" rel="noopener">Google Maps'te aç</a>
      <button class="btn" id="copy">Koordinatı kopyala</button>
    </div>
  </section>

  <section class="card full">
    <h2>Harita</h2>
    <div id="map"></div>
    <div class="mut" style="margin-top:6px;font-size:12px">İnternet yoksa harita yüklenmez; koordinatlar yine yukarıda görünür.</div>
  </section>

  <section class="card full">
    <h2>Sinyal gücü (C/N0, dBHz) — dolu = çözümde kullanılan</h2>
    <div class="sig" id="sig"></div>
    <div class="legend" id="legend"></div>
  </section>

  <section class="card full">
    <h2>Gök haritası (skyplot) — merkez = tepe, kenar = ufuk</h2>
    <svg id="sky" viewBox="0 0 320 320" style="width:320px;max-width:100%;display:block;margin:auto"></svg>
  </section>
</main>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const COL={GPS:'#4f8cff',GLONASS:'#f85149',Galileo:'#3fb950',BeiDou:'#d29922',SBAS:'#a371f7',QZSS:'#ff922b'};
let map,marker,mapReady=false;
function initMap(){
  try{
    map=L.map('map',{zoomControl:true}).setView([39.9,32.7],3);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'© OpenStreetMap'}).addTo(map);
    mapReady=true;
  }catch(e){}
}
if(window.L) initMap();

function fmt(v,d){return v==null?'—':(typeof v==='number'?v.toFixed(d):v)}
async function tick(){
  let s; try{ s=await (await fetch('/gps')).json(); }catch(e){ return; }
  document.getElementById('portlbl').textContent=s.port+(s.err?(' — HATA: '+s.err):'');
  const age=s.ts?((Date.now()/1000)-s.ts):999;
  document.getElementById('age').textContent=s.ts?('güncelleme '+age.toFixed(1)+' sn önce'):'veri yok';
  document.body.classList.toggle('stale',age>4);

  // fix rozeti
  const fb=document.getElementById('fixb');
  const hasFix=s.fix && !/YOK/.test(s.fix||'');
  fb.textContent=s.fix||'—';
  fb.className='badge '+(hasFix?'b-ok':'b-bad');

  // 25 kriteri: sinyalli sayi >=25 (izlenen daha yuksek olabilir)
  const cb=document.getElementById('critb');
  const pass=s.signal>=25;
  cb.textContent='25 uydu: '+(pass?('✓ ('+s.signal+' sinyalli)'):(s.signal+' sinyalli'));
  cb.className='badge '+(pass?'b-ok':(s.signal>=15?'b-warn':'b-bad'));

  document.getElementById('n-track').textContent=s.tracked;
  document.getElementById('n-sig').textContent=s.signal;
  document.getElementById('n-use').textContent=s.used;

  // takim kirilimi
  const order=['GPS','GLONASS','Galileo','BeiDou','SBAS','QZSS'];
  let ch='';
  order.forEach(k=>{ if(!s.by_const[k])return; const c=s.by_const[k];
    const pct=c.tracked?Math.round(100*c.signal/c.tracked):0;
    ch+=`<span class="dot" style="background:${COL[k]||'#888'}"></span>`+
        `<div><b>${k}</b> <span class="mut">${c.signal}/${c.tracked} sinyalli · ${c.used} kull.</span>`+
        `<div class="bar" style="margin-top:3px"><i style="width:${pct}%;background:${COL[k]||'#888'}"></i></div></div>`+
        `<div class="v">${c.tracked}</div>`;
  });
  document.getElementById('const').innerHTML=ch||'<span class="mut">uydu verisi yok</span>';

  // konum
  document.getElementById('coord').textContent=(s.lat==null?'—':s.lat.toFixed(7)+', '+s.lon.toFixed(7));
  document.getElementById('v-fix').textContent=s.fix||'—';
  document.getElementById('v-alt').textContent=s.alt==null?'—':s.alt.toFixed(1)+' m';
  document.getElementById('v-hdop').textContent=fmt(s.hdop,2);
  document.getElementById('v-spd').textContent=s.speed_kmh==null?'—':s.speed_kmh.toFixed(2)+' km/s';
  document.getElementById('v-crs').textContent=s.course==null?'—':s.course.toFixed(0)+'°';
  document.getElementById('v-utc').textContent=s.utc||'—';
  if(s.lat!=null){
    const g=document.getElementById('gmap');
    g.href='https://www.google.com/maps?q='+s.lat+','+s.lon;
    if(mapReady){ const ll=[s.lat,s.lon];
      if(!marker){ marker=L.marker(ll).addTo(map); map.setView(ll,18); }
      else marker.setLatLng(ll);
    }
  }

  // sinyal cubuklari
  const MAX=55;
  document.getElementById('sig').innerHTML=s.sats.map(t=>{
    const h=Math.max(2,Math.round((t.cno/MAX)*110));
    const col=COL[t.g]||'#888';
    return `<div class="s" title="${t.g} #${t.sv} — ${t.cno} dBHz${t.u?' (kullanılıyor)':''}">`+
      `<div class="b" style="height:${h}px;background:${col};opacity:${t.u?1:.4}"></div>`+
      `<div class="lab">${t.sv}</div></div>`;
  }).join('')||'<span class="mut">—</span>';
  document.getElementById('legend').innerHTML=order.filter(k=>s.by_const[k])
    .map(k=>`<span><span class="dot" style="background:${COL[k]}"></span>${k}</span>`).join('');

  // skyplot
  const R=140,cx=160,cy=160; let sk='';
  [90,60,30,0].forEach(el=>{const r=R*(90-el)/90; sk+=`<circle cx="${cx}" cy="${cy}" r="${r}" fill="none" stroke="#232a33"/>`;});
  sk+=`<line x1="${cx}" y1="${cy-R}" x2="${cx}" y2="${cy+R}" stroke="#232a33"/>`;
  sk+=`<line x1="${cx-R}" y1="${cy}" x2="${cx+R}" y2="${cy}" stroke="#232a33"/>`;
  sk+=`<text x="${cx}" y="14" fill="#8b949e" font-size="10" text-anchor="middle">K</text>`;
  s.sats.forEach(t=>{ if(t.el<0||t.az==null)return;
    const r=R*(90-t.el)/90, a=t.az*Math.PI/180;
    const x=cx+r*Math.sin(a), y=cy-r*Math.cos(a);
    const col=COL[t.g]||'#888';
    sk+=`<circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="${t.u?5:3.5}" fill="${col}" opacity="${t.cno>0?1:.35}" stroke="${t.u?'#fff':'none'}" stroke-width="1"/>`;
  });
  document.getElementById('sky').innerHTML=sk;
}
document.getElementById('copy').onclick=async()=>{
  const t=document.getElementById('coord').textContent;
  try{await navigator.clipboard.writeText(t);document.getElementById('copy').textContent='kopyalandı ✓';
    setTimeout(()=>document.getElementById('copy').textContent='Koordinatı kopyala',1500);}catch(e){}
};
tick(); setInterval(tick,1000);
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/gps"):
            with lock:
                data = json.dumps(state).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        else:
            data = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)


if __name__ == "__main__":
    threading.Thread(target=reader, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), H)
    print(f"GPS arayuzu:  http://localhost:{HTTP_PORT}   (port={PORT})")
    print("Durdurmak icin Ctrl+C")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nkapatiliyor…")
