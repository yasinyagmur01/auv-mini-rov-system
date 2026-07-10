"""Jetson mission node WebSocket istemcisi (otomatik yeniden baglanma)."""
import asyncio
import json
import threading
import time
from collections import deque


class WsClient:
    def __init__(self, url):
        self.url = url
        self.connected = False
        self.state = {}            # son 'state' mesaji
        self.log = deque(maxlen=200)
        self._loop = None
        self._ws = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def send(self, obj: dict):
        if self._loop and self._ws and self.connected:
            asyncio.run_coroutine_threadsafe(
                self._ws.send(json.dumps(obj)), self._loop)
        else:
            self.log.append('! Gonderilemedi (baglanti yok): ' + str(obj.get('cmd')))

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._main())

    async def _main(self):
        import websockets
        while True:
            try:
                async with websockets.connect(self.url, open_timeout=3) as ws:
                    self._ws = ws
                    self.connected = True
                    self.log.append('Jetson baglantisi kuruldu')
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        if msg.get('type') == 'state':
                            self.state = msg
                        elif msg.get('type') in ('error', 'info'):
                            self.log.append(f"[{msg['type']}] {msg.get('message')}")
                        elif msg.get('type') == 'missions':
                            self.state['missions'] = msg.get('missions', [])
            except Exception:
                pass
            self.connected = False
            self._ws = None
            await asyncio.sleep(2.0)
