"""AUV Gorev Kontrol Paneli — backend.

FastAPI + WebSocket:
  - Ozellik process'lerini baslatir/durdurur (her ozellik AYRI dosya/process).
  - Her baslatma YENI bir kayit oturumu klasoru acar (asla ustune yazilmaz).
  - Ozellik stdout'undan gelen NDJSON mesajlarini WebSocket'e yayinlar.
  - Kayit oturumlarini listeler; playback icin dosyalari statik sunar.
  - ZED konteynerini (docker compose) yonetir.

Calistirma:  bash ui/run_ui.sh   ->  http://<jetson-ip>:8080
"""

import asyncio
import json
import os
import signal
import subprocess
import time

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

UI_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AUV_DIR = os.path.dirname(UI_DIR)
FEATURES_DIR = os.path.join(UI_DIR, 'features')
RECORDINGS_DIR = os.path.join(UI_DIR, 'recordings')
STATIC_DIR = os.path.join(UI_DIR, 'static')
COMPOSE = os.path.join(AUV_DIR, 'docker', 'compose.yaml')

os.makedirs(RECORDINGS_DIR, exist_ok=True)

FEATURES = {
    'rgb': {'title': 'Kamera (RGB)', 'file': 'rgb_feature.py',
            'desc': 'Canli goruntu + parlaklik/kontrast/netlik analizi'},
    'depth': {'title': 'Derinlik Algilama', 'file': 'depth_feature.py',
              'desc': 'Derinlik haritasi + confidence (hata payi) + istatistik'},
    'tracking': {'title': 'Konum Takibi (VIO)', 'file': 'tracking_feature.py',
                 'desc': 'Pose/odom yorungesi, hiz, drift, kovaryans'},
    'mapping': {'title': '3D Haritalama', 'file': 'mapping_feature.py',
                'desc': 'Fused point cloud (3D piksel) buyume analizi'},
    'sensors': {'title': 'Sensorler (IMU/Mag/Baro)', 'file': 'sensors_feature.py',
                'desc': 'IMU titresim, yonelim, manyetik alan, basinc, sicaklik'},
}

app = FastAPI(title='AUV Gorev Kontrol')


# --------------------------------------------------------------------------
# WebSocket yayin merkezi
# --------------------------------------------------------------------------
class Hub:
    def __init__(self):
        self.clients: set[WebSocket] = set()

    async def add(self, ws: WebSocket):
        await ws.accept()
        self.clients.add(ws)

    def remove(self, ws: WebSocket):
        self.clients.discard(ws)

    async def broadcast_text(self, text: str):
        """Yavas/donuk bir istemci ASLA yayini kilitleyemez: gonderim istemci
        basina 2 sn ile sinirlidir; asan istemci dusurulur. (Aksi halde donan
        bir tarayici, stdout borusunu geri-basincla doldurup ozellik
        process'inin ROS callback'lerini ve DISK KAYDINI durduruyordu.)"""
        dead = []
        for ws in list(self.clients):
            try:
                await asyncio.wait_for(ws.send_text(text), timeout=2.0)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.remove(ws)
            try:
                await ws.close()
            except Exception:
                pass


hub = Hub()


# --------------------------------------------------------------------------
# Ozellik process yoneticisi
# --------------------------------------------------------------------------
class Runner:
    """Tek bir ozellik process'inin yasam dongusu."""

    def __init__(self, fid: str):
        self.fid = fid
        self.proc: asyncio.subprocess.Process | None = None
        self.session_id: str | None = None
        self.session_dir: str | None = None
        self.started_at: float | None = None
        self._lock = asyncio.Lock()   # cift tiklama = cift process yarisina karsi

    @property
    def running(self):
        return self.proc is not None and self.proc.returncode is None

    async def start(self):
        async with self._lock:
            return await self._start_locked()

    async def _start_locked(self):
        if self.running:
            return self.session_id
        self.session_id = time.strftime('%Y%m%d_%H%M%S')
        self.session_dir = os.path.join(RECORDINGS_DIR, self.fid, self.session_id)
        os.makedirs(self.session_dir, exist_ok=True)
        self.started_at = time.time()

        feature_file = os.path.join(FEATURES_DIR, FEATURES[self.fid]['file'])
        env = dict(os.environ)
        env['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
        env['PYTHONUNBUFFERED'] = '1'
        cmd = (f'source /opt/ros/humble/setup.bash && '
               f'exec python3 {feature_file} --session-dir {self.session_dir}')
        stderr_log = open(os.path.join(self.session_dir, 'feature.log'), 'wb')
        # limit: nokta bulutu mesajlari tek satirda ~1 MB olabilir; asyncio'nun
        # 64 KB varsayilani LimitOverrunError ile pompayi olduruyordu.
        self.proc = await asyncio.create_subprocess_exec(
            'bash', '-c', cmd,
            stdout=asyncio.subprocess.PIPE, stderr=stderr_log, env=env,
            limit=32 * 1024 * 1024,
        )
        asyncio.get_event_loop().create_task(self._pump())
        await hub.broadcast_text(json.dumps(self._state_msg()))
        return self.session_id

    async def _pump(self):
        assert self.proc and self.proc.stdout
        try:
            while True:
                line = await self.proc.stdout.readline()
                if not line:
                    break
                await hub.broadcast_text(line.decode('utf-8', 'replace').rstrip())
        finally:
            await self.proc.wait()
            await hub.broadcast_text(json.dumps(self._state_msg()))

    async def stop(self):
        if not self.running:
            return
        self.proc.send_signal(signal.SIGINT)
        try:
            await asyncio.wait_for(self.proc.wait(), timeout=20)
        except asyncio.TimeoutError:
            self.proc.terminate()
            try:
                await asyncio.wait_for(self.proc.wait(), timeout=5)
            except asyncio.TimeoutError:
                self.proc.kill()

    def _state_msg(self):
        return {'kind': 'feature_state', 'feature': self.fid,
                'running': self.running, 'session_id': self.session_id,
                'started_at': self.started_at}

    def state(self):
        return {'running': self.running, 'session_id': self.session_id,
                'started_at': self.started_at,
                'uptime_s': round(time.time() - self.started_at, 1)
                if self.running and self.started_at else 0}


runners = {fid: Runner(fid) for fid in FEATURES}


# --------------------------------------------------------------------------
# ZED konteyner yonetimi
# --------------------------------------------------------------------------
def _docker(*args, timeout=60):
    try:
        out = subprocess.run(['docker', *args], capture_output=True, text=True,
                             timeout=timeout)
        return out.returncode, out.stdout.strip(), out.stderr.strip()
    except Exception as exc:  # noqa: BLE001
        return 1, '', str(exc)


def zed_status():
    rc, out, _ = _docker('inspect', '-f', '{{.State.Status}}', 'auv-zed',
                         timeout=10)
    return out if rc == 0 else 'yok'


async def zed_status_async():
    """docker cagrisi event loop'u kilitlemesin (dockerd yavassa sunucu donmasin)."""
    return await asyncio.to_thread(zed_status)


async def ensure_zed():
    if await zed_status_async() == 'running':
        return True
    proc = await asyncio.create_subprocess_exec(
        'docker', 'compose', '-f', COMPOSE, 'up', '-d', 'zed',
        stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)
    await proc.wait()
    return await zed_status_async() == 'running'


# --------------------------------------------------------------------------
# REST API
# --------------------------------------------------------------------------
@app.get('/api/features')
def api_features():
    return {fid: {**meta, **runners[fid].state()}
            for fid, meta in FEATURES.items()}


@app.post('/api/features/{fid}/start')
async def api_start(fid: str):
    if fid == 'all':
        if not await ensure_zed():
            return JSONResponse({'ok': False,
                                 'error': 'ZED konteyneri baslatilamadi'}, 500)
        out = {}
        for f in FEATURES:
            out[f] = await runners[f].start()
        return {'ok': True, 'sessions': out}
    if fid not in FEATURES:
        return JSONResponse({'ok': False, 'error': 'bilinmeyen ozellik'}, 404)
    ok = await ensure_zed()
    if not ok:
        return JSONResponse({'ok': False,
                             'error': 'ZED konteyneri baslatilamadi'}, 500)
    sid = await runners[fid].start()
    return {'ok': True, 'session_id': sid}


@app.post('/api/features/{fid}/stop')
async def api_stop(fid: str):
    if fid == 'all':
        for f in FEATURES:
            await runners[f].stop()
        return {'ok': True}
    if fid not in FEATURES:
        return JSONResponse({'ok': False, 'error': 'bilinmeyen ozellik'}, 404)
    await runners[fid].stop()
    return {'ok': True}


@app.get('/api/sessions/{fid}')
def api_sessions(fid: str):
    base = os.path.join(RECORDINGS_DIR, fid)
    sessions = []
    if os.path.isdir(base):
        for sid in sorted(os.listdir(base), reverse=True):
            meta_p = os.path.join(base, sid, 'meta.json')
            if not os.path.isfile(meta_p):
                continue
            try:
                with open(meta_p) as fh:
                    meta = json.load(fh)
            except Exception:  # noqa: BLE001
                continue
            sdir = os.path.join(base, sid)
            size = 0
            for root, _, files in os.walk(sdir):
                size += sum(os.path.getsize(os.path.join(root, f))
                            for f in files)
            meta['size_mb'] = round(size / 1e6, 1)
            meta['has_analytics'] = os.path.isfile(
                os.path.join(sdir, 'analytics.json'))
            # 'running' bayragini gercek process durumuyla uzlastir: finalize
            # calismadan olen oturumlar (kill/elektrik kesintisi) sonsuza dek
            # 'kayitta' gorunmesin.
            r = runners.get(fid)
            if meta.get('running') and not (
                    r and r.running and r.session_id == sid):
                meta['running'] = False
                meta['interrupted'] = True
            sessions.append(meta)
    return sessions


@app.get('/api/zed/status')
async def api_zed_status():
    return {'status': await zed_status_async()}


@app.post('/api/zed/start')
async def api_zed_start():
    ok = await ensure_zed()
    return {'ok': ok, 'status': await zed_status_async()}


@app.post('/api/zed/stop')
async def api_zed_stop():
    proc = await asyncio.create_subprocess_exec(
        'docker', 'compose', '-f', COMPOSE, 'stop', 'zed',
        stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)
    await proc.wait()
    return {'ok': True, 'status': await zed_status_async()}


@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await hub.add(ws)
    try:
        while True:
            await ws.receive_text()   # istemciden veri beklemiyoruz (ping olabilir)
    except WebSocketDisconnect:
        hub.remove(ws)
    except Exception:  # noqa: BLE001
        hub.remove(ws)


@app.get('/')
def index():
    return FileResponse(os.path.join(STATIC_DIR, 'index.html'))


app.mount('/static', StaticFiles(directory=STATIC_DIR), name='static')
app.mount('/recordings', StaticFiles(directory=RECORDINGS_DIR),
          name='recordings')


@app.on_event('shutdown')
async def on_shutdown():
    for r in runners.values():
        await r.stop()
