/* AUV Gorev Kontrol — SPA
 * Ayni renderer hattini hem CANLI (WebSocket) hem PLAYBACK (stream.ndjson)
 * kullanir. Renk/isaret kurallari: dataviz reference palette (dark). */
'use strict';

const COLORS = { s1: '#3987e5', s2: '#199e70', s3: '#c98500', s4: '#9085e9', s5: '#e66767' };
const INK = { muted: '#898781', grid: '#2c2c2a', base: '#383835', ink2: '#c3c2b7' };

/* ------------------------------------------------------------------ *
 * Panel konfigurasyonlari — kanal/alan adlari feature dosyalariyla birebir
 * ------------------------------------------------------------------ */
const PANELS = {
  rgb: {
    mediaSwitch: { title: 'Canlı Görüntü', channels: [
      { ch: 'rgb', label: 'Ana (RGB)' }, { ch: 'left', label: 'Sol Göz' }, { ch: 'right', label: 'Sağ Göz' }] },
    tiles: [
      { ch: 'img_stats', f: 'fps', k: 'Kamera FPS', d: 1 },
      { ch: 'img_stats', f: 'brightness', k: 'Parlaklık (0-255)', d: 1 },
      { ch: 'img_stats', f: 'contrast', k: 'Kontrast', d: 1 },
      { ch: 'img_stats', f: 'sharpness', k: 'Netlik', d: 1 },
      { ch: 'img_stats', f: 'clip_ratio', k: 'Pozlama Taşması', d: 3 },
    ],
    charts: [
      { title: 'Kare Hızı', unit: 'FPS', ch: 'img_stats', series: [{ f: 'fps', name: 'fps', c: COLORS.s3 }] },
      { title: 'Parlaklık', unit: '0-255', ch: 'img_stats', series: [{ f: 'brightness', name: 'parlaklık', c: COLORS.s1 }] },
      { title: 'Netlik (gradyan varyansı)', ch: 'img_stats', series: [{ f: 'sharpness', name: 'netlik', c: COLORS.s2 }] },
      { title: 'Kontrast (std)', ch: 'img_stats', series: [{ f: 'contrast', name: 'kontrast', c: COLORS.s4 }] },
    ],
  },
  depth: {
    media: [{ ch: 'depth', title: 'Derinlik Haritası (açık=yakın, koyu=uzak)' },
            { ch: 'confidence', title: 'Güvenilirlik Haritası (açık=güvenilir)' }],
    tiles: [
      { ch: 'depth_stats', f: 'mean', k: 'Ort. Derinlik (m)', d: 2 },
      { ch: 'depth_stats', f: 'min', k: 'Min (m)', d: 2 },
      { ch: 'depth_stats', f: 'max', k: 'Maks (m)', d: 2 },
      { ch: 'depth_stats', f: 'valid_ratio', k: 'Geçerli Piksel', d: 2 },
    ],
    charts: [
      { title: 'Ortalama Derinlik', unit: 'm', ch: 'depth_stats', series: [{ f: 'mean', name: 'ort', c: COLORS.s1 }] },
      { title: 'Geçerli Piksel Oranı', ch: 'depth_stats', series: [{ f: 'valid_ratio', name: 'oran', c: COLORS.s2 }] },
      { title: 'Min / Maks Derinlik', unit: 'm', ch: 'depth_stats', series: [{ f: 'min', name: 'min', c: COLORS.s4 }, { f: 'max', name: 'maks', c: COLORS.s3 }] },
    ],
    hist: { ch: 'depth_hist', title: 'Derinlik Dağılımı (canlı histogram)', unit: 'm' },
  },
  tracking: {
    traj: { title: 'XY Yörünge (üstten görünüm)' },
    tiles: [
      { ch: 'pose', f: 'x', k: 'X (m)', d: 3 }, { ch: 'pose', f: 'y', k: 'Y (m)', d: 3 },
      { ch: 'pose', f: 'z', k: 'Z (m)', d: 3 }, { ch: 'cov', f: 'trace_pos', k: 'Kovaryans İzi', d: 5 },
    ],
    charts: [
      { title: 'Z (derinlik/yükseklik)', unit: 'm', ch: 'pose', series: [{ f: 'z', name: 'z', c: COLORS.s1 }] },
      { title: 'Yönelim — Yaw', unit: 'rad', ch: 'pose', series: [{ f: 'yaw', name: 'yaw', c: COLORS.s4 }] },
      { title: 'Konum Kovaryans İzi', unit: 'm²', ch: 'cov', series: [{ f: 'trace_pos', name: 'iz', c: COLORS.s5 }] },
    ],
  },
  mapping: {
    cloud: { title: '3D Harita — Fused Point Cloud (fare ile döndür/yakınlaş)' },
    tiles: [
      { ch: 'map_stats', f: 'count', k: 'Nokta Sayısı', d: 0 },
      { ch: 'map_stats', f: 'volume_m3', k: 'Hacim (m³)', d: 1 },
      { ch: 'map_stats', f: 'growth_pts_s', k: 'Büyüme (nokta/s)', d: 0 },
    ],
    charts: [
      { title: 'Nokta Sayısı Büyümesi', ch: 'map_stats', series: [{ f: 'count', name: 'nokta', c: COLORS.s1 }] },
      { title: 'Haritalanan Hacim', unit: 'm³', ch: 'map_stats', series: [{ f: 'volume_m3', name: 'hacim', c: COLORS.s2 }] },
      { title: 'Büyüme Hızı', unit: 'nokta/s', ch: 'map_stats', series: [{ f: 'growth_pts_s', name: 'büyüme', c: COLORS.s3 }] },
    ],
  },
  sensors: {
    tiles: [
      { ch: 'imu', expr: d => Math.hypot(d.ax, d.ay, d.az), k: 'İvme |a| (m/s²)', d: 2 },
      { ch: 'imu', expr: d => Math.hypot(d.wx, d.wy, d.wz), k: 'Açısal Hız (rad/s)', d: 3 },
      { ch: 'mag', f: 'norm_ut', k: 'Manyetik Alan (µT)', d: 1 },
      { ch: 'baro', f: 'hpa', k: 'Basınç (hPa)', d: 1 },
    ],
    charts: [
      { title: 'İvme Büyüklüğü', unit: 'm/s²', ch: 'imu', series: [{ expr: d => Math.hypot(d.ax, d.ay, d.az), name: '|a|', c: COLORS.s1 }] },
      { title: 'Yönelim (RPY)', unit: 'rad', ch: 'imu', series: [
        { f: 'roll', name: 'roll', c: COLORS.s1 }, { f: 'pitch', name: 'pitch', c: COLORS.s2 }, { f: 'yaw', name: 'yaw', c: COLORS.s3 }] },
      { title: 'Açısal Hız Büyüklüğü', unit: 'rad/s', ch: 'imu', series: [{ expr: d => Math.hypot(d.wx, d.wy, d.wz), name: '|ω|', c: COLORS.s4 }] },
      { title: 'Manyetik Alan Normu', unit: 'µT', ch: 'mag', series: [{ f: 'norm_ut', name: '|B|', c: COLORS.s2 }] },
      { title: 'Atmosferik Basınç', unit: 'hPa', ch: 'baro', series: [{ f: 'hpa', name: 'basınç', c: COLORS.s1 }] },
    ],
  },
};

/* ------------------------------------------------------------------ *
 * Yardimcilar
 * ------------------------------------------------------------------ */
const $ = (sel, el = document) => el.querySelector(sel);
const h = (tag, attrs = {}, ...kids) => {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') el.className = v;
    else if (k.startsWith('on')) el.addEventListener(k.slice(2), v);
    else el.setAttribute(k, v);
  }
  for (const kid of kids) el.append(kid);
  return el;
};
const api = async (path, method = 'GET') => {
  const res = await fetch(path, { method });
  const body = await res.json().catch(() => ({}));
  if (!res.ok || body.ok === false) {
    showErrorBanner(body.error || `${method} ${path} → HTTP ${res.status}`);
  }
  return body;
};
const fmtT = s => `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

/* ------------------------------------------------------------------ *
 * Grafik bilesenleri
 * ------------------------------------------------------------------ */
class LiveChart {
  constructor(host, cfg, windowSec = 150) {
    this.cfg = cfg; this.win = windowSec;
    this.data = [[]]; cfg.series.forEach(() => this.data.push([]));
    const card = h('div', { class: 'chart-card' });
    card.append(h('div', { class: 'title' }, cfg.title + (cfg.unit ? ` (${cfg.unit})` : '')));
    if (cfg.series.length > 1) {
      const lg = h('div', { class: 'legend' });
      cfg.series.forEach(s => {
        const item = h('span', {});
        item.append(h('span', { class: 'chip', style: `background:${s.c}` }), s.name);
        lg.append(item);
      });
      card.append(lg);
    }
    this.plotEl = h('div', {});
    card.append(this.plotEl);
    host.append(card);
    this._make();
  }
  _make() {
    const w = this.plotEl.clientWidth || 380;
    const opts = {
      width: w, height: 170,
      cursor: { points: { size: 7 } },
      legend: { show: false },
      scales: { x: { time: false } },
      axes: [
        { stroke: INK.muted, grid: { stroke: INK.grid, width: 1 }, ticks: { stroke: INK.base }, size: 34,
          values: (u, vs) => vs.map(v => fmtT(v)) },
        { stroke: INK.muted, grid: { stroke: INK.grid, width: 1 }, ticks: { stroke: INK.base }, size: 52 },
      ],
      series: [
        {},
        ...this.cfg.series.map(s => ({ label: s.name, stroke: s.c, width: 2, points: { show: false } })),
      ],
    };
    this.u = new uPlot(opts, this.data, this.plotEl);
  }
  push(t, values) {
    // Ozellik yeniden baslatildiginda t sifirlanir; uPlot artan x ister ->
    // gerileme gorursek tampondan yeni oturuma temiz basla.
    const xs = this.data[0];
    if (xs.length && t < xs[xs.length - 1]) this.reset();
    this.data[0].push(t);
    values.forEach((v, i) => this.data[i + 1].push(v));
    const cut = t - this.win;
    while (this.data[0].length > 2 && this.data[0][0] < cut) {
      this.data.forEach(a => a.shift());
    }
    // setData'yi rAF ile birlestir: seek/replay sirasinda binlerce ardisik
    // push tek cizime iner (O(N^2) donmasini engeller).
    if (!this._raf) {
      this._raf = requestAnimationFrame(() => {
        this._raf = null;
        this.u.setData(this.data);
      });
    }
  }
  handle(msg) {
    if (msg.channel !== this.cfg.ch || !msg.data) return;
    const vals = this.cfg.series.map(s =>
      s.expr ? s.expr(msg.data) : msg.data[s.f]);
    if (vals.some(v => v === undefined || v === null)) return;
    this.push(msg.t, vals);
  }
  reset() { this.data = [[]]; this.cfg.series.forEach(() => this.data.push([])); this.u.setData(this.data); }
}

class HistChart {
  constructor(host, cfg) {
    this.cfg = cfg;
    const card = h('div', { class: 'chart-card' });
    card.append(h('div', { class: 'title' }, cfg.title));
    this.cv = h('canvas', { height: 170 });
    card.append(this.cv); host.append(card);
  }
  handle(msg) {
    if (msg.channel !== this.cfg.ch || !msg.data) return;
    this.draw(msg.data.bins, msg.data.counts);
  }
  draw(bins, counts) {
    const cv = this.cv, dpr = devicePixelRatio || 1;
    const W = cv.clientWidth || 380, H = 170;
    cv.width = W * dpr; cv.height = H * dpr;
    const g = cv.getContext('2d'); g.scale(dpr, dpr);
    g.clearRect(0, 0, W, H);
    const max = Math.max(...counts, 1);
    const n = counts.length, pad = 24, bw = (W - pad * 2) / n;
    g.fillStyle = COLORS.s1;
    counts.forEach((c, i) => {
      const bh = (H - 30) * c / max;
      const x = pad + i * bw + 1, y = H - 20 - bh;
      g.beginPath();
      g.roundRect(x, y, bw - 2, bh, [4, 4, 0, 0]);
      g.fill();
    });
    g.fillStyle = INK.muted; g.font = '10px system-ui';
    g.fillText(`${bins[0]}m`, pad, H - 6);
    g.textAlign = 'right';
    g.fillText(`${bins[bins.length - 1]}m`, W - pad, H - 6);
    g.textAlign = 'left';
  }
  reset() { const g = this.cv.getContext('2d'); g && g.clearRect(0, 0, this.cv.width, this.cv.height); }
}

class TrajCanvas {
  constructor(host, cfg) {
    const card = h('div', { class: 'media-card', style: 'max-width:660px' });
    const head = h('div', { class: 'title' });
    head.append(cfg.title);
    const lg = h('span', {});
    [['pose', COLORS.s1], ['odom', COLORS.s2]].forEach(([n, c]) => {
      lg.append(h('span', { class: 'chip', style: `background:${c};margin-left:10px` }), n);
    });
    head.append(lg);
    card.append(head);
    this.cv = h('canvas', { height: 340 });
    card.append(this.cv); host.append(card);
    this.pose = []; this.odom = [];
  }
  handle(msg) {
    if (msg.kind !== 'pose' || !msg.data) return;
    if (msg.channel === 'pose') this.pose.push([msg.data.x, msg.data.y]);
    else if (msg.channel === 'odom') this.odom.push([msg.data.x, msg.data.y]);
    else return;
    if (this.pose.length + this.odom.length > 60000) { this.pose.splice(0, 1000); this.odom.splice(0, 1000); }
    this.draw();
  }
  draw() {
    const cv = this.cv, dpr = devicePixelRatio || 1;
    const W = cv.clientWidth || 620, H = 340;
    cv.width = W * dpr; cv.height = H * dpr;
    const g = cv.getContext('2d'); g.scale(dpr, dpr);
    g.clearRect(0, 0, W, H);
    const all = this.pose.concat(this.odom);
    if (all.length < 2) { g.fillStyle = INK.muted; g.fillText('veri bekleniyor…', 20, 30); return; }
    let xs = all.map(p => p[0]), ys = all.map(p => p[1]);
    let x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
    const span = Math.max(x1 - x0, y1 - y0, 0.5), pad = 30;
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    const k = (Math.min(W, H) - pad * 2) / span;
    const px = p => [W / 2 + (p[0] - cx) * k, H / 2 - (p[1] - cy) * k];
    // izgara
    g.strokeStyle = INK.grid; g.lineWidth = 1;
    const step = span > 8 ? 2 : span > 3 ? 1 : 0.5;
    for (let gx = Math.floor((cx - span) / step) * step; gx < cx + span; gx += step) {
      const [sx] = px([gx, 0]); g.beginPath(); g.moveTo(sx, 0); g.lineTo(sx, H); g.stroke();
    }
    for (let gy = Math.floor((cy - span) / step) * step; gy < cy + span; gy += step) {
      const [, sy] = px([0, gy]); g.beginPath(); g.moveTo(0, sy); g.lineTo(W, sy); g.stroke();
    }
    const drawPath = (pts, color) => {
      if (pts.length < 2) return;
      g.strokeStyle = color; g.lineWidth = 2; g.beginPath();
      pts.forEach((p, i) => { const [sx, sy] = px(p); i ? g.lineTo(sx, sy) : g.moveTo(sx, sy); });
      g.stroke();
      const [ex, ey] = px(pts[pts.length - 1]);
      g.fillStyle = color; g.beginPath(); g.arc(ex, ey, 5, 0, 7); g.fill();
      g.strokeStyle = '#1a1a19'; g.lineWidth = 2; g.stroke();
    };
    drawPath(this.odom, COLORS.s2);
    drawPath(this.pose, COLORS.s1);
    // olcek cubugu
    g.strokeStyle = INK.ink2; g.lineWidth = 2;
    g.beginPath(); g.moveTo(20, H - 16); g.lineTo(20 + step * k, H - 16); g.stroke();
    g.fillStyle = INK.muted; g.font = '11px system-ui';
    g.fillText(`${step} m`, 24 + step * k, H - 12);
    const last = this.pose[this.pose.length - 1];
    if (last) g.fillText(`x=${last[0].toFixed(2)}  y=${last[1].toFixed(2)}`, 20, 18);
  }
  reset() { this.pose = []; this.odom = []; this.draw(); }
}

class CloudView {
  constructor(host, cfg) {
    const card = h('div', { class: 'media-card', style: 'max-width:900px;flex:2' });
    this.head = h('div', { class: 'title' }); this.head.textContent = cfg.title;
    card.append(this.head);
    this.holder = h('div', { style: 'height:380px;border-radius:6px;overflow:hidden' });
    card.append(this.holder); host.append(card);
    this.mode = '3d';
    try {
      this._init3d();
    } catch (e) {
      // WebGL yok / three.js kurulamadi -> 2B ustten gorunum fallback'i
      console.warn('3D gorunum kurulamadi, 2B moda geciliyor:', e);
      this.mode = '2d';
      this.holder.innerHTML = '';
      this.cv = h('canvas', { height: 380, style: 'width:100%;background:#222220' });
      this.holder.append(this.cv);
      this.head.textContent = cfg.title + ' — ⚠ WebGL yok: 2B üstten görünüm';
    }
  }
  _init3d() {
    if (typeof THREE === 'undefined') throw new Error('three.js yuklenemedi');
    const W = this.holder.clientWidth || 800, H = 380;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x222220);
    this.camera = new THREE.PerspectiveCamera(60, W / H, 0.05, 200);
    this.camera.position.set(4, 4, 3); this.camera.up.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({ antialias: false });  // WebGL yoksa THROW
    this.renderer.setSize(W, H);
    this.holder.append(this.renderer.domElement);
    this.controls = THREE.OrbitControls
      ? new THREE.OrbitControls(this.camera, this.renderer.domElement) : null;
    this.scene.add(new THREE.AxesHelper(1));
    const grid = new THREE.GridHelper(20, 20, 0x383835, 0x2c2c2a);
    grid.rotation.x = Math.PI / 2; this.scene.add(grid);
    this.points = null;
    const loop = () => { if (this.controls) this.controls.update();
      this.renderer.render(this.scene, this.camera); requestAnimationFrame(loop); };
    loop();
  }
  setCloud(xyz, rgb, count) {
    if (this.mode === '2d') { this._draw2d(xyz, rgb, count); return; }
    if (this.points) { this.scene.remove(this.points); this.points.geometry.dispose(); }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(xyz, 3));
    const colors = new Float32Array(rgb.length);
    for (let i = 0; i < rgb.length; i++) colors[i] = rgb[i] / 255;
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    const mat = new THREE.PointsMaterial({ size: 0.035, vertexColors: true });
    this.points = new THREE.Points(geo, mat);
    this.scene.add(this.points);
    this.head.textContent = `3D Harita — ${count.toLocaleString('tr')} nokta (döndür: sol tık · yakınlaş: tekerlek)`;
  }
  _draw2d(xyz, rgb, count) {
    const cv = this.cv, dpr = devicePixelRatio || 1;
    const W = cv.clientWidth || 800, H = 380;
    cv.width = W * dpr; cv.height = H * dpr;
    const g = cv.getContext('2d'); g.scale(dpr, dpr);
    g.fillStyle = '#222220'; g.fillRect(0, 0, W, H);
    const n = xyz.length / 3;
    this.head.textContent = `3D Harita (2B üstten) — ${count.toLocaleString('tr')} nokta`;
    if (n < 2) { g.fillStyle = INK.muted; g.fillText('harita boş — kamerayı gezdirin', 20, 30); return; }
    let x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
    for (let i = 0; i < n; i++) { const x = xyz[i * 3], y = xyz[i * 3 + 1];
      if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; }
    const span = Math.max(x1 - x0, y1 - y0, 0.5);
    const k = (Math.min(W, H) - 40) / span, cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    // ImageData ile piksel-dogrudan cizim: fillStyle string kurmaktan ~20x hizli
    const id = g.getImageData(0, 0, W * dpr, H * dpr), px = id.data, Wd = W * dpr;
    for (let i = 0; i < n; i++) {
      const sx = ((W / 2 + (xyz[i * 3] - cx) * k) * dpr) | 0;
      const sy = ((H / 2 - (xyz[i * 3 + 1] - cy) * k) * dpr) | 0;
      if (sx < 0 || sy < 0 || sx >= Wd || sy >= id.height) continue;
      const o = (sy * Wd + sx) * 4;
      px[o] = rgb[i * 3]; px[o + 1] = rgb[i * 3 + 1]; px[o + 2] = rgb[i * 3 + 2]; px[o + 3] = 255;
    }
    g.putImageData(id, 0, 0);
    g.fillStyle = INK.ink2; g.font = '11px system-ui';
    g.fillText(`görünüm: üstten (X-Y) · alan ~${(x1 - x0).toFixed(1)}×${(y1 - y0).toFixed(1)} m`, 14, 20);
  }
  handle(msg) {
    if (msg.kind !== 'cloud') return;
    if (msg.xyz && msg.rgb) {           // canli: b64
      const xb = Uint8Array.from(atob(msg.xyz), c => c.charCodeAt(0));
      const cb = Uint8Array.from(atob(msg.rgb), c => c.charCodeAt(0));
      this.setCloud(new Float32Array(xb.buffer), cb, msg.count);
    } else if (msg.file && msg._base) {  // playback: .bin dosyasi
      // Nesil sayaci: hizli seek'te sirasiz donen eski cevaplar uygulanmaz.
      const gen = (this._gen = (this._gen || 0) + 1);
      fetch(msg._base + msg.file).then(r => {
        if (!r.ok) throw new Error(`bulut dosyasi ${r.status}`);
        return r.arrayBuffer();
      }).then(buf => {
        if (gen !== this._gen) return;   // daha yeni bir istek var
        const n = msg.count;
        if (buf.byteLength < n * 15) throw new Error('eksik bulut verisi');
        const xyz = new Float32Array(buf, 0, n * 3);
        const rgb = new Uint8Array(buf, n * 12, n * 3);
        this.setCloud(xyz, rgb, n);
      }).catch(e => console.warn('bulut yukleme:', e.message));
    }
  }
  reset() { if (this.points) { this.scene.remove(this.points); this.points = null; } }
}

class SwitchMediaCard {
  /* Kanal secicili goruntu karti (or. RGB / Sol / Sag goz) */
  constructor(host, cfg) {
    this.cfg = cfg; this.active = cfg.channels[0].ch; this.latest = {};
    const card = h('div', { class: 'media-card' });
    const head = h('div', { class: 'title' });
    head.append(h('span', {}, cfg.title));
    const btns = h('span', {});
    this.btnEls = {};
    cfg.channels.forEach(c => {
      const b = h('button', { class: 'btn', style: 'font-size:11px;padding:2px 10px;margin-left:6px',
        onclick: () => this.switchTo(c.ch) }, c.label);
      this.btnEls[c.ch] = b; btns.append(b);
    });
    head.append(btns); card.append(head);
    this.img = h('img', { alt: cfg.title });
    card.append(this.img); host.append(card);
    this._paintBtns();
  }
  _paintBtns() {
    for (const [ch, b] of Object.entries(this.btnEls))
      b.classList.toggle('primary', ch === this.active);
  }
  switchTo(ch) {
    this.active = ch; this._paintBtns();
    const m = this.latest[ch];
    if (m) this._show(m); else this.img.removeAttribute('src');
  }
  _show(m) {
    if (m.b64) this.img.src = 'data:image/jpeg;base64,' + m.b64;
    else if (m.file && m._base) this.img.src = m._base + m.file;
  }
  handle(msg) {
    if (msg.kind !== 'frame') return;
    if (!this.cfg.channels.some(c => c.ch === msg.channel)) return;
    this.latest[msg.channel] = msg;
    if (msg.channel === this.active) this._show(msg);
  }
  reset() { this.latest = {}; this.img.removeAttribute('src'); }
}

class MediaCard {
  constructor(host, cfg) {
    this.cfg = cfg;
    const card = h('div', { class: 'media-card' });
    this.title = h('div', { class: 'title' });
    this.title.append(h('span', {}, cfg.title), this.hz = h('span', { class: 'muted' }, ''));
    card.append(this.title);
    this.img = h('img', { alt: cfg.title });
    card.append(this.img); host.append(card);
  }
  handle(msg) {
    if (msg.kind !== 'frame' || msg.channel !== this.cfg.ch) return;
    if (msg.b64) this.img.src = 'data:image/jpeg;base64,' + msg.b64;
    else if (msg.file && msg._base) this.img.src = msg._base + msg.file;
  }
  setHz(hz) { this.hz.textContent = hz != null ? `${hz} Hz` : ''; }
  reset() { this.img.removeAttribute('src'); }
}

class Tiles {
  constructor(host, defs) {
    this.defs = defs; this.els = [];
    const grid = h('div', { class: 'tiles' });
    defs.forEach(d => {
      const v = h('div', { class: 'v' }, '—');
      grid.append(h('div', { class: 'tile' }, h('div', { class: 'k' }, d.k), v));
      this.els.push(v);
    });
    host.append(grid);
  }
  handle(msg) {
    if (!msg.data) return;
    this.defs.forEach((d, i) => {
      if (msg.channel !== d.ch) return;
      const v = d.expr ? d.expr(msg.data) : msg.data[d.f];
      if (v !== undefined) this.els[i].textContent = Number(v).toLocaleString('tr', { maximumFractionDigits: d.d });
    });
  }
  reset() { this.els.forEach(e => e.textContent = '—'); }
}

/* ------------------------------------------------------------------ *
 * Ozellik paneli (canli + playback ayni bileseni kullanir)
 * ------------------------------------------------------------------ */
class FeaturePanel {
  constructor(host, fid, opts = {}) {
    this.fid = fid; this.cfg = PANELS[fid]; this.parts = [];
    const root = h('div', {});
    host.append(root);
    const media = h('div', { class: 'media-row' });
    root.append(media);
    // Her bilesen izole kurulur: biri cokerse digerleri + gecmis YASAR.
    const safe = (label, make) => {
      try { const p = make(); if (p) this.parts.push(p); }
      catch (e) {
        console.error(`${fid}/${label} kurulamadi:`, e);
        media.append(h('div', { class: 'media-card' },
          h('div', { class: 'title', style: 'color:var(--critical)' },
            `⚠ ${label} bileşeni kurulamadı: ${e.message}`)));
      }
    };
    if (this.cfg.media) this.cfg.media.forEach(m => safe(m.ch, () => new MediaCard(media, m)));
    if (this.cfg.mediaSwitch) safe('görüntü', () => new SwitchMediaCard(media, this.cfg.mediaSwitch));
    if (this.cfg.traj) safe('yörünge', () => new TrajCanvas(media, this.cfg.traj));
    if (this.cfg.cloud) safe('3D harita', () => new CloudView(media, this.cfg.cloud));
    if (this.cfg.tiles) safe('kutucuklar', () => new Tiles(root, this.cfg.tiles));
    const charts = h('div', { class: 'charts-grid' });
    root.append(charts);
    (this.cfg.charts || []).forEach(c => safe(c.title, () => new LiveChart(charts, c, opts.windowSec || 150)));
    if (this.cfg.hist) safe('histogram', () => new HistChart(charts, this.cfg.hist));
    this.statusCb = opts.onStatus || null;
  }
  handle(msg) {
    if (msg.kind === 'status') {
      if (this.statusCb) this.statusCb(msg.data);
      const rates = msg.data && msg.data.rates;
      if (rates) this.parts.forEach(p => { if (p instanceof MediaCard) p.setHz(rates[p.cfg.ch]); });
      return;
    }
    this.parts.forEach(p => p.handle(msg));
  }
  reset() { this.parts.forEach(p => p.reset()); }
}

/* ------------------------------------------------------------------ *
 * Uygulama durumu + WS
 * ------------------------------------------------------------------ */
const App = {
  view: null,            // {type:'feature'|'all'|'playback', fid, panels:{}}
  viewSeq: 0,            // gorunum nesli: eski async isler kendini iptal eder
  features: {},
  ws: null,
};

function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = ev => {
    let msg; try { msg = JSON.parse(ev.data); } catch { return; }
    if (msg.kind === 'feature_state') {
      refreshState();
      // Yeni oturum basladiysa canli panelleri sifirla (eski yorunge/grafik
      // yeni oturumla karismasin).
      if (msg.running) {
        const vv = App.view;
        const pp = vv && vv.panels && vv.panels[msg.feature];
        if (pp) pp.reset();
      }
      return;
    }
    const v = App.view;
    if (!v || v.type === 'playback') return;
    const p = v.panels && v.panels[msg.feature];
    if (p) p.handle(msg);
  };
  ws.onclose = () => setTimeout(connectWS, 2000);
  App.ws = ws;
}

async function refreshState() {
  App.features = await api('/api/features');
  renderNavState();
  const v = App.view;
  if (v && (v.type === 'feature' || v.type === 'all')) updateHeadButtons();
  const z = await api('/api/zed/status');
  const dot = $('#zed-dot');
  dot.className = 'dot ' + z.status;
  $('#zed-status').textContent = z.status === 'running' ? 'çalışıyor' : z.status;
  $('#zed-toggle').textContent = z.status === 'running' ? 'Durdur' : 'Başlat';
}

/* ------------------------------------------------------------------ *
 * Gezinme / gorunumler
 * ------------------------------------------------------------------ */
function buildNav() {
  const nav = $('#nav');
  nav.innerHTML = '';
  const items = [...Object.entries(PANELS).map(([fid]) => [fid, null]), ['__all', null]];
  for (const [fid] of items) {
    const title = fid === '__all' ? '⚡ Tüm Özellikler' : navTitle(fid);
    const btn = h('button', { class: 'nav-item', id: `nav-${fid}`,
      onclick: () => fid === '__all' ? showAll() : showFeature(fid) });
    btn.append(h('span', {}, title), h('span', { class: 'dot live-dot', id: `live-${fid}` }));
    if (fid === '__all') nav.append(h('div', { class: 'nav-sep' }));
    nav.append(btn);
  }
}
const navTitle = fid => ({ rgb: '📷 Kamera (RGB)', depth: '🌊 Derinlik', tracking: '🧭 Konum Takibi',
  mapping: '🗺️ 3D Haritalama', sensors: '📈 Sensörler' }[fid] || fid);

function renderNavState() {
  for (const fid of Object.keys(PANELS)) {
    const el = $(`#live-${fid}`);
    if (el) el.className = 'dot live-dot' + (App.features[fid] && App.features[fid].running ? ' running' : '');
    const nb = $(`#nav-${fid}`);
    if (nb) nb.classList.toggle('active', App.view && App.view.type === 'feature' && App.view.fid === fid);
  }
  const ab = $('#nav-__all');
  if (ab) ab.classList.toggle('active', App.view && App.view.type === 'all');
}

function panelHead(fid, container) {
  const st = App.features[fid] || {};
  const head = h('div', { class: 'panel-head' });
  head.append(h('h2', {}, navTitle(fid)));
  const badges = h('div', { class: 'badges', id: `badges-${fid}` });
  head.append(badges);
  head.append(h('div', { class: 'spacer' }));
  const rec = h('span', { class: 'rec-indicator', id: `rec-${fid}`, style: st.running ? '' : 'display:none' });
  rec.append(h('span', { class: 'dot' }), h('span', {}, `KAYIT: ${st.session_id || ''}`));
  head.append(rec);
  const btn = h('button', { class: 'btn ' + (st.running ? 'danger' : 'primary'), id: `btn-${fid}`,
    onclick: () => toggleFeature(fid) }, st.running ? '■ Durdur' : '▶ Başlat');
  head.append(btn);
  head.append(h('div', { class: 'desc' }, (PANELS[fid] && App.features[fid] ? App.features[fid].desc : '') || ''));
  container.append(head);
}

function updateHeadButtons() {
  for (const fid of Object.keys(PANELS)) {
    const st = App.features[fid] || {};
    const btn = $(`#btn-${fid}`);
    if (btn) { btn.textContent = st.running ? '■ Durdur' : '▶ Başlat';
      btn.className = 'btn ' + (st.running ? 'danger' : 'primary'); }
    const rec = $(`#rec-${fid}`);
    if (rec) { rec.style.display = st.running ? '' : 'none';
      if (st.running) rec.lastChild.textContent = `KAYIT: ${st.session_id}`; }
  }
}

async function toggleFeature(fid) {
  const st = App.features[fid] || {};
  await api(`/api/features/${fid}/${st.running ? 'stop' : 'start'}`, 'POST');
  await refreshState();
  setTimeout(() => loadHistory(fid), 1500);   // durdurunca gecmis tazelensin
}

function showFeature(fid) {
  const main = $('#main'); main.innerHTML = '';
  ++App.viewSeq;
  App.view = { type: 'feature', fid, panels: {} };
  panelHead(fid, main);
  const panel = new FeaturePanel(main, fid, {
    onStatus: data => {
      const b = $(`#badges-${fid}`); if (!b || !data) return;
      b.innerHTML = '';
      for (const [ch, hz] of Object.entries(data.rates || {}))
        b.append(h('span', { class: 'badge' }, `${ch}: `, h('b', {}, `${hz} Hz`)));
    },
  });
  App.view.panels[fid] = panel;
  const hist = h('div', { class: 'history' });
  hist.append(h('h3', {}, '📼 Geçmiş Kayıtlar'), h('div', { id: `hist-${fid}` }, 'yükleniyor…'));
  main.append(hist);
  renderNavState(); updateHeadButtons(); loadHistory(fid);
}

function showAll() {
  const main = $('#main'); main.innerHTML = '';
  ++App.viewSeq;
  App.view = { type: 'all', panels: {} };
  const head = h('div', { class: 'panel-head' });
  head.append(h('h2', {}, '⚡ Tüm Özellikler'), h('div', { class: 'spacer' }),
    h('button', { class: 'btn primary', onclick: async () => { await api('/api/features/all/start', 'POST'); refreshState(); } }, '▶ Tümünü Başlat'),
    h('button', { class: 'btn danger', onclick: async () => { await api('/api/features/all/stop', 'POST'); refreshState(); } }, '■ Tümünü Durdur'));
  main.append(head);
  for (const fid of Object.keys(PANELS)) {
    const block = h('div', { class: 'all-block' });
    main.append(block);
    panelHead(fid, block);
    App.view.panels[fid] = new FeaturePanel(block, fid, { windowSec: 90 });
  }
  renderNavState(); updateHeadButtons();
}

/* ------------------------------------------------------------------ *
 * Gecmis + playback
 * ------------------------------------------------------------------ */
async function loadHistory(fid) {
  const holder = $(`#hist-${fid}`);
  if (!holder) return;
  const sessions = await api(`/api/sessions/${fid}`);
  if (!sessions.length) { holder.innerHTML = '<span class="muted">Henüz kayıt yok — ▶ Başlat ile ilk kaydı oluştur.</span>'; return; }
  const tbl = h('table', { class: 'sessions' });
  tbl.append(h('tr', {}, ...['Oturum', 'Başlangıç', 'Süre', 'Boyut', 'Durum'].map(x => h('th', {}, x))));
  sessions.forEach(s => {
    const row = h('tr', { class: 'clickable', onclick: () => showPlayback(fid, s.session_id) },
      h('td', {}, s.session_id),
      h('td', {}, s.started_at || '—'),
      h('td', {}, s.duration_s != null ? fmtT(s.duration_s) : '⏺ sürüyor'),
      h('td', {}, `${s.size_mb} MB`),
      h('td', {}, h('span', { class: 'pill' }, s.running ? 'kayıtta'
        : s.interrupted ? 'kesildi' : (s.has_analytics ? 'analitik hazır' : 'tamamlandı'))));
    tbl.append(row);
  });
  holder.innerHTML = ''; holder.append(tbl);
}

async function showPlayback(fid, sid) {
  const main = $('#main'); main.innerHTML = '';
  const seq = ++App.viewSeq;                       // gorunum nesli
  App.view = { type: 'playback', fid, sid };
  const base = `/recordings/${fid}/${sid}/`;
  main.append(h('button', { class: 'back-link', onclick: () => showFeature(fid) }, '← ' + navTitle(fid) + ' paneline dön'));
  main.append(h('h2', {}, `📼 ${navTitle(fid)} — ${sid}`));

  // stream + analitik yukle
  const [streamTxt, analytics] = await Promise.all([
    fetch(base + 'stream.ndjson').then(r => r.ok ? r.text() : ''),
    fetch(base + 'analytics.json').then(r => r.ok ? r.json() : null).catch(() => null),
  ]);
  if (App.viewSeq !== seq) return;   // kullanici beklerken baska goruname gecti
  const msgs = streamTxt.split('\n').filter(Boolean).map(l => { try { return JSON.parse(l); } catch { return null; } })
    .filter(Boolean).map(m => (m._base = base, m));
  const T = msgs.length ? msgs[msgs.length - 1].t : 0;

  // playback cubugu
  const bar = h('div', { class: 'pb-bar' });
  const playBtn = h('button', { class: 'btn primary' }, '▶');
  const slider = h('input', { type: 'range', min: 0, max: T.toFixed(1), step: 0.1, value: 0 });
  const timeEl = h('span', { class: 'pb-time' }, `00:00 / ${fmtT(T)}`);
  const speed = h('select', {}, ...[0.5, 1, 2, 4, 8].map(s =>
    h('option', s === 1 ? { value: s, selected: '' } : { value: s }, s + '×')));
  bar.append(playBtn, slider, timeEl, speed);
  main.append(bar);

  const panel = new FeaturePanel(main, fid, { windowSec: 1e9 });

  // ---- playback motoru: ayni renderer'a zaman sirali mesaj dagitimi ----
  let clock = 0, playing = false, ptr = 0, lastTs = null;
  const frameLatest = {};   // seek icin kanal basina son kare
  function dispatchTo(t, fastSeek = false) {
    if (t < clock) { panel.reset(); ptr = 0; Object.keys(frameLatest).forEach(k => delete frameLatest[k]); }
    clock = t;
    while (ptr < msgs.length && msgs[ptr].t <= clock) {
      const m = msgs[ptr++];
      if (fastSeek && m.kind === 'frame') { frameLatest[m.channel] = m; continue; }
      if (fastSeek && m.kind === 'cloud') { frameLatest['__cloud'] = m; continue; }
      panel.handle(m);
    }
    if (fastSeek) Object.values(frameLatest).forEach(m => panel.handle(m));
    slider.value = clock.toFixed(1);
    timeEl.textContent = `${fmtT(clock)} / ${fmtT(T)}`;
  }
  function loop(ts) {
    if (App.viewSeq !== seq) { playing = false; return; }  // gorunum degisti: dur
    if (!playing) return;
    if (lastTs != null) {
      const next = clock + (ts - lastTs) / 1000 * Number(speed.value);
      dispatchTo(Math.min(next, T));
      if (clock >= T) { playing = false; playBtn.textContent = '▶'; }
    }
    lastTs = ts;
    requestAnimationFrame(loop);
  }
  playBtn.onclick = () => {
    playing = !playing; playBtn.textContent = playing ? '⏸' : '▶'; lastTs = null;
    if (playing) { if (clock >= T) dispatchTo(0); requestAnimationFrame(loop); }
  };
  slider.oninput = () => { playing = false; playBtn.textContent = '▶';
    dispatchTo(Number(slider.value), true); };  // geri sarma temizligi dispatchTo'da

  // ---- analitik bolumu ----
  const an = h('div', { class: 'history' });
  an.append(h('h3', {}, '🔬 Oturum Analitiği (pandas/numpy — kayıt sonunda hesaplandı)'));
  if (!analytics) {
    an.append(h('div', { class: 'muted' }, 'analytics.json yok — kayıt hâlâ sürüyor ya da erken kapandı.'));
  } else {
    const tiles = h('div', { class: 'tiles' });
    for (const [k, v] of Object.entries(analytics.summary || {}))
      tiles.append(h('div', { class: 'tile' }, h('div', { class: 'k' }, k.replaceAll('_', ' ')),
        h('div', { class: 'v' }, Array.isArray(v) ? v.join(' … ') : (typeof v === 'object' ? JSON.stringify(v) : String(v)))));
    an.append(tiles);
    const grid = h('div', { class: 'charts-grid' });
    an.append(grid);
    (analytics.charts || []).forEach(c => renderAnalyticsChart(grid, c));
  }
  main.append(an);
  dispatchTo(0, true);
}

function renderAnalyticsChart(host, c) {
  const card = h('div', { class: 'chart-card' });
  card.append(h('div', { class: 'title' }, c.title + (c.unit ? ` (${c.unit})` : '')));
  if ((c.series || []).length > 1) {
    const lg = h('div', { class: 'legend' });
    c.series.forEach((s, i) => lg.append(h('span', {},
      h('span', { class: 'chip', style: `background:${Object.values(COLORS)[i % 5]}` }), s.name)));
    card.append(lg);
  }
  const holder = h('div', {}); card.append(holder); host.append(card);
  if (c.type === 'trajectory') {
    const cv = h('canvas', { height: 280, style: 'width:100%' }); holder.append(cv);
    requestAnimationFrame(() => drawTrajStatic(cv, c.series));
    return;
  }
  if (c.type === 'histogram') {
    const cv = h('canvas', { height: 180, style: 'width:100%' }); holder.append(cv);
    requestAnimationFrame(() => {
      const pts = (c.series[0] || {}).points || [];
      const hc = Object.create(HistChart.prototype); hc.cv = cv; hc.cfg = {};
      hc.draw(pts.map(p => Number(p[0]).toFixed(1)), pts.map(p => p[1]));
    });
    return;
  }
  requestAnimationFrame(() => {
    const w = holder.clientWidth || 380;
    // seriler farkli t eksenlerine sahip olabilir -> ilk serinin t'sine hizala
    const xs = (c.series[0].points || []).map(p => p[0]);
    const data = [xs, ...c.series.map(s => {
      const map = new Map(s.points.map(p => [p[0], p[1]]));
      return xs.map(x => map.has(x) ? map.get(x) : null);
    })];
    new uPlot({
      width: w, height: 170, legend: { show: false },
      cursor: { points: { size: 7 } }, scales: { x: { time: false } },
      axes: [
        { stroke: INK.muted, grid: { stroke: INK.grid }, ticks: { stroke: INK.base }, size: 34, values: (u, vs) => vs.map(v => fmtT(v)) },
        { stroke: INK.muted, grid: { stroke: INK.grid }, ticks: { stroke: INK.base }, size: 52 },
      ],
      series: [{}, ...c.series.map((s, i) => ({ stroke: Object.values(COLORS)[i % 5], width: 2, points: { show: false } }))],
    }, data, holder);
  });
}

function drawTrajStatic(cv, series) {
  const dpr = devicePixelRatio || 1, W = cv.clientWidth || 380, H = 280;
  cv.width = W * dpr; cv.height = H * dpr;
  const g = cv.getContext('2d'); g.scale(dpr, dpr);
  const all = series.flatMap(s => s.points);
  if (all.length < 2) return;
  const xs = all.map(p => p[0]), ys = all.map(p => p[1]);
  const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
  const span = Math.max(x1 - x0, y1 - y0, 0.5), cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  const k = (Math.min(W, H) - 50) / span;
  const px = p => [W / 2 + (p[0] - cx) * k, H / 2 - (p[1] - cy) * k];
  series.forEach((s, i) => {
    g.strokeStyle = Object.values(COLORS)[i % 5]; g.lineWidth = 2; g.beginPath();
    s.points.forEach((p, j) => { const [sx, sy] = px(p); j ? g.lineTo(sx, sy) : g.moveTo(sx, sy); });
    g.stroke();
    const [lx, ly] = px(s.points[s.points.length - 1]);
    g.fillStyle = g.strokeStyle; g.font = '11px system-ui';
    g.fillText(s.name, lx + 8, ly);
  });
}

/* ------------------------------------------------------------------ *
 * Gorunur hata bandi: JS hatalari asla sessizce kaybolmasin
 * ------------------------------------------------------------------ */
function showErrorBanner(text) {
  let bar = $('#err-banner');
  if (!bar) {
    bar = h('div', { id: 'err-banner', style:
      'position:fixed;top:0;left:0;right:0;z-index:99;background:var(--critical);' +
      'color:#fff;padding:6px 14px;font-size:12px;display:flex;gap:10px;align-items:center' });
    bar.append(h('span', { style: 'flex:1' }, ''),
      h('button', { class: 'btn', style: 'padding:1px 8px;font-size:11px',
        onclick: () => bar.remove() }, '×'));
    document.body.append(bar);
  }
  bar.firstChild.textContent = '⚠ ' + text;
}
window.addEventListener('error', e => showErrorBanner(`JS hatası: ${e.message} (${(e.filename || '').split('/').pop()}:${e.lineno})`));
window.addEventListener('unhandledrejection', e =>
  showErrorBanner('İstek hatası: ' + (e.reason && e.reason.message || e.reason)));

/* ------------------------------------------------------------------ */
$('#zed-toggle').addEventListener('click', async () => {
  const z = await api('/api/zed/status');
  await api(z.status === 'running' ? '/api/zed/stop' : '/api/zed/start', 'POST');
  refreshState();
});

buildNav();
connectWS();
refreshState().then(() => showFeature('tracking'));
setInterval(refreshState, 10000);
