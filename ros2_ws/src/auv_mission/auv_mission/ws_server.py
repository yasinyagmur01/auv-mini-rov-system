"""Panel icin WebSocket JSON sunucusu (ayri thread + asyncio dongusu).

Gelen mesajlar thread-guvenli kuyruga konur; mission_node kendi zamanlayicisinda
isler. Durum yayini broadcast() ile yapilir.
"""
import asyncio
import json
import queue
import threading

import websockets


class WsServer:
    def __init__(self, host='0.0.0.0', port=8765, logger=None):
        self.host = host
        self.port = port
        self.logger = logger
        self.inbox = queue.Queue()
        self._clients = set()
        self._loop = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def _log(self, msg):
        if self.logger:
            self.logger.info(msg)

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())
        self._loop.run_forever()

    async def _serve(self):
        await websockets.serve(self._handler, self.host, self.port)
        self._log(f'WebSocket sunucusu: ws://{self.host}:{self.port}')

    async def _handler(self, ws):
        self._clients.add(ws)
        self._log(f'Panel baglandi: {ws.remote_address}')
        try:
            async for raw in ws:
                try:
                    self.inbox.put(json.loads(raw))
                except json.JSONDecodeError:
                    await ws.send(json.dumps({'type': 'error',
                                              'message': 'gecersiz JSON'}))
        finally:
            self._clients.discard(ws)
            self._log('Panel baglantisi kapandi')

    def broadcast(self, obj: dict):
        """Herhangi bir thread'den cagrilabilir."""
        if self._loop is None or not self._clients:
            return
        data = json.dumps(obj)
        asyncio.run_coroutine_threadsafe(self._send_all(data), self._loop)

    async def _send_all(self, data: str):
        dead = []
        for ws in self._clients:
            try:
                await ws.send(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self._clients.discard(ws)
