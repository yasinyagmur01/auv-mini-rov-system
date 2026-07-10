"""--demo modu: arac olmadan panel gelistirme/deneme icin sahte veri uretici.

MavListener ve WsClient ile ayni arayuzu sunan sahte siniflar.
"""
import math
import threading
import time
from collections import deque


class DemoMav:
    connected = True

    def __init__(self):
        self.t0 = time.monotonic()

    def start(self):
        pass

    def get(self, sysid):
        t = time.monotonic() - self.t0
        if sysid == 1:
            return {
                'last_seen': time.monotonic(),
                'armed': (int(t) // 20) % 2 == 1,
                'depth_m': 0.5 + 0.1 * math.sin(t / 3),
                'altitude_m': 2.0 + 0.2 * math.cos(t / 5),
                'yaw_deg': (t * 8) % 360,
                'heading_deg': (t * 8) % 360,
                'gps_fix': 3, 'gps_sats': 9,
                'lat': 37.4160, 'lon': 38.7940,
                'battery_v': 15.8 - 0.001 * t,
            }
        return {
            'last_seen': time.monotonic(),
            'armed': False, 'depth_m': 0.0, 'battery_v': 16.2,
        }


class DemoWs:
    connected = True

    def __init__(self):
        self.t0 = time.monotonic()
        self.log = deque(maxlen=200)
        self.log.append('DEMO modu aktif - gercek arac YOK')
        self._mission_on = False
        self._mt0 = 0.0

    def start(self):
        pass

    @property
    def state(self):
        t = time.monotonic() - self.t0
        mt = time.monotonic() - self._mt0 if self._mission_on else 0.0
        steps = ['geri sayim', 'arm', 'mod ALT_HOLD', 'derinlik 0.5m',
                 'duz 16s', 'don +90deg', 'duz 16s', 'daire 1.25 tur',
                 'duz 16s', 'don +90deg', 'duz 16s', 'yuzeye cik']
        idx = min(int(mt // 8), len(steps) - 1) if self._mission_on else 0
        r = 15.0
        return {
            'type': 'state',
            'mission': {
                'state': 2 if self._mission_on else 0,
                'state_name': 'VIDEO_PATTERN' if self._mission_on else 'IDLE',
                'step_name': steps[idx] if self._mission_on else '',
                'step_index': idx, 'step_count': len(steps),
                'step_elapsed_s': round(mt % 8, 1),
                'mission_elapsed_s': round(mt, 1),
                'message': 'demo verisi',
            },
            'telemetry': {
                'depth_m': 0.5, 'altitude_m': 2.0, 'altitude_valid': True,
                'heading_deg': (t * 8) % 360, 'armed': self._mission_on,
                'mode': 'ALT_HOLD',
                'dr': {
                    'origin_valid': True,
                    'x_east_m': r * math.cos(t / 10),
                    'y_north_m': r * math.sin(t / 10),
                    'est_lat': 37.4160 + 0.0001 * math.sin(t / 10),
                    'est_lon': 38.7940 + 0.0001 * math.cos(t / 10),
                    'speed_mps': 0.5,
                },
                'lane_status': 1,
            },
        }

    def send(self, obj):
        c = obj.get('cmd')
        self.log.append(f'DEMO komut: {c}')
        if c == 'start_mission':
            self._mission_on = True
            self._mt0 = time.monotonic()
        elif c == 'stop':
            self._mission_on = False
