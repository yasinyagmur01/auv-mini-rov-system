"""MAVLink dinleyici thread'i (14552 panel ucu).

Her sysid icin son telemetri sozlugunu tutar; GUI zamanlayiciyla okur (poll).
"""
import threading
import time

from pymavlink import mavutil


class MavListener:
    def __init__(self, url):
        self.url = url
        self._lock = threading.Lock()
        self._vehicles = {}   # sysid -> dict
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.connected = False

    def start(self):
        self._thread.start()

    def get(self, sysid) -> dict:
        with self._lock:
            return dict(self._vehicles.get(sysid, {}))

    def _veh(self, sysid):
        if sysid not in self._vehicles:
            self._vehicles[sysid] = {}
        return self._vehicles[sysid]

    def _run(self):
        while True:
            try:
                conn = mavutil.mavlink_connection(self.url)
                self.connected = True
                self._loop(conn)
            except Exception:
                self.connected = False
                time.sleep(2.0)

    def _loop(self, conn):
        import math
        while True:
            m = conn.recv_match(blocking=True, timeout=2.0)
            if m is None:
                continue
            sysid = m.get_srcSystem()
            t = m.get_type()
            with self._lock:
                v = self._veh(sysid)
                v['last_seen'] = time.monotonic()
                if t == 'HEARTBEAT' and m.get_srcComponent() == 1:
                    v['armed'] = bool(m.base_mode & 128)
                    v['custom_mode'] = m.custom_mode
                elif t == 'VFR_HUD':
                    v['depth_m'] = max(0.0, -m.alt)
                    v['heading_deg'] = m.heading
                elif t == 'ATTITUDE':
                    v['yaw_deg'] = math.degrees(m.yaw) % 360.0
                elif t == 'DISTANCE_SENSOR':
                    v['altitude_m'] = m.current_distance / 100.0
                elif t == 'GPS_RAW_INT':
                    v['gps_fix'] = m.fix_type
                    v['gps_sats'] = m.satellites_visible
                    v['lat'] = m.lat / 1e7
                    v['lon'] = m.lon / 1e7
                elif t == 'SYS_STATUS':
                    v['battery_v'] = m.voltage_battery / 1000.0
                elif t == 'NAMED_VALUE_FLOAT':
                    name = m.name if isinstance(m.name, str) else m.name.decode()
                    v.setdefault('named', {})[name.strip('\x00')] = m.value
                elif t == 'STATUSTEXT':
                    text = m.text if isinstance(m.text, str) else m.text.decode(errors='ignore')
                    v.setdefault('statustext', []).append(text)
                    v['statustext'] = v['statustext'][-20:]
