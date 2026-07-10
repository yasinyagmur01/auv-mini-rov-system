#!/usr/bin/env python3
"""AUV Kontrol Paneli (QGroundControl'un YANINDA calisir).

    python main.py            gercek baglanti (config.py'deki adresler)
    python main.py --demo     arac olmadan sahte veriyle deneme

QGC pilotaj/parametre/joystick icindir; bu panel izleme + gorev kontroludur.
"""
import sys
import time

from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel,
                               QGridLayout, QHBoxLayout, QVBoxLayout,
                               QPushButton, QComboBox, QGroupBox, QLineEdit,
                               QPlainTextEdit, QDoubleSpinBox, QMessageBox)

import config
from widgets import VerticalProfileWidget, DRMapWidget


def fmt(v, spec='.2f', suffix=''):
    if v is None:
        return '—'
    try:
        return f'{v:{spec}}{suffix}'
    except (TypeError, ValueError):
        return str(v)


class MainWindow(QMainWindow):
    def __init__(self, demo=False):
        super().__init__()
        self.demo = demo
        self.setWindowTitle('AUV Kontrol Paneli' + (' [DEMO]' if demo else ''))
        self.resize(1500, 900)

        if demo:
            from demo import DemoMav, DemoWs
            self.mav = DemoMav()
            self.ws = DemoWs()
        else:
            from mav_listener import MavListener
            from ws_client import WsClient
            self.mav = MavListener(config.MAVLINK_LISTEN)
            self.ws = WsClient(config.WS_URL)
        self.mav.start()
        self.ws.start()

        from video_widget import VideoWidget
        self.vid_auv = VideoWidget('AUV / D435', config.D435_SOURCES, demo=demo)
        self.vid_rov = VideoWidget('Mini ROV', config.MINIROV_SOURCES, demo=demo)

        self.profile = VerticalProfileWidget()
        self.drmap = DRMapWidget()

        self._targets_latlon = {}
        self._log_len = 0

        self._build_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)  # 5 Hz

    # ------------------------------------------------ UI kurulumu
    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)

        # SOL: videolar
        vids = QVBoxLayout()
        vids.addWidget(self.vid_auv, 1)
        vids.addWidget(self.vid_rov, 1)
        layout.addLayout(vids, 5)

        # ORTA: dikey profil + harita
        mid = QVBoxLayout()
        gb_p = QGroupBox('Dikey Eksen (Bar30 + Sonar)')
        lp = QVBoxLayout(gb_p)
        lp.addWidget(self.profile)
        mid.addWidget(gb_p, 3)
        gb_m = QGroupBox('Olu Hesap Haritasi')
        lm = QVBoxLayout(gb_m)
        lm.addWidget(self.drmap)
        mid.addWidget(gb_m, 4)
        layout.addLayout(mid, 3)

        # SAG: telemetri + gorev kontrol + log
        right = QVBoxLayout()

        gb_t = QGroupBox('Telemetri')
        gt = QGridLayout(gb_t)
        self.lbl = {}
        rows = [('AUV mod', 'mode'), ('AUV arm', 'armed'),
                ('Derinlik', 'depth'), ('Dipten yukseklik', 'alt'),
                ('Pruva', 'hdg'), ('GPS', 'gps'),
                ('AUV batarya', 'batt1'), ('ROV batarya', 'batt2'),
                ('AUV link', 'link1'), ('ROV link', 'link2'),
                ('Jetson WS', 'ws')]
        for i, (title, key) in enumerate(rows):
            gt.addWidget(QLabel(title + ':'), i, 0)
            self.lbl[key] = QLabel('—')
            self.lbl[key].setStyleSheet('font-weight:bold')
            gt.addWidget(self.lbl[key], i, 1)
        right.addWidget(gb_t)

        gb_g = QGroupBox('Gorev')
        gg = QGridLayout(gb_g)
        self.cmb_mission = QComboBox()
        self.cmb_mission.addItems(['video_pattern', 'lane_follow', 'task2_nav'])
        gg.addWidget(self.cmb_mission, 0, 0, 1, 2)
        self.lbl_mission = QLabel('IDLE')
        self.lbl_mission.setStyleSheet('font-size:15px; font-weight:bold; color:#2a7')
        gg.addWidget(self.lbl_mission, 1, 0, 1, 2)
        self.lbl_step = QLabel('—')
        gg.addWidget(self.lbl_step, 2, 0, 1, 2)

        b_arm = QPushButton('ARM')
        b_arm.clicked.connect(lambda: self.ws.send({'cmd': 'arm'}))
        b_dis = QPushButton('DISARM')
        b_dis.clicked.connect(lambda: self.ws.send({'cmd': 'disarm'}))
        gg.addWidget(b_arm, 3, 0)
        gg.addWidget(b_dis, 3, 1)

        b_start = QPushButton('START')
        b_start.setStyleSheet('background:#2a7; color:white; font-weight:bold; padding:8px')
        b_start.clicked.connect(self.on_start)
        gg.addWidget(b_start, 4, 0)
        b_stop = QPushButton('STOP')
        b_stop.setStyleSheet('background:#c33; color:white; font-weight:bold; padding:8px')
        b_stop.clicked.connect(lambda: self.ws.send({'cmd': 'stop'}))
        gg.addWidget(b_stop, 4, 1)

        b_origin = QPushButton('GPS Origin Al')
        b_origin.clicked.connect(lambda: self.ws.send({'cmd': 'capture_origin'}))
        gg.addWidget(b_origin, 5, 0, 1, 2)
        right.addWidget(gb_g)

        gb_c = QGroupBox('Gorev 2 Koordinatlari (ipucu)')
        gc = QGridLayout(gb_c)
        self.ed = {}
        for i, (label, key) in enumerate([('Donme lat', 'turn_lat'),
                                          ('Donme lon', 'turn_lon'),
                                          ('Bitis lat', 'finish_lat'),
                                          ('Bitis lon', 'finish_lon')]):
            gc.addWidget(QLabel(label), i, 0)
            self.ed[key] = QLineEdit()
            self.ed[key].setPlaceholderText('derece (orn. 37.416000)')
            gc.addWidget(self.ed[key], i, 1)
        b_tgt = QPushButton('Hedefleri Gonder')
        b_tgt.clicked.connect(self.on_send_targets)
        gc.addWidget(b_tgt, 4, 0, 1, 2)

        gc.addWidget(QLabel('Akinti dogu (m/s)'), 5, 0)
        self.sp_be = QDoubleSpinBox()
        self.sp_be.setRange(-2, 2)
        self.sp_be.setSingleStep(0.05)
        gc.addWidget(self.sp_be, 5, 1)
        gc.addWidget(QLabel('Akinti kuzey (m/s)'), 6, 0)
        self.sp_bn = QDoubleSpinBox()
        self.sp_bn.setRange(-2, 2)
        self.sp_bn.setSingleStep(0.05)
        gc.addWidget(self.sp_bn, 6, 1)
        b_bias = QPushButton('Akintiyi Gonder')
        b_bias.clicked.connect(lambda: self.ws.send({
            'cmd': 'set_current_bias',
            'east_mps': self.sp_be.value(), 'north_mps': self.sp_bn.value()}))
        gc.addWidget(b_bias, 7, 0, 1, 2)
        right.addWidget(gb_c)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        right.addWidget(self.log, 1)

        layout.addLayout(right, 3)

    # ------------------------------------------------ olaylar
    def on_start(self):
        name = self.cmb_mission.currentText()
        if name == 'task2_nav' and not self._targets_latlon:
            QMessageBox.warning(self, 'Eksik hedef',
                                'Once ipucu koordinatlarini gonderin!')
            return
        self.ws.send({'cmd': 'start_mission', 'mission': name})

    def on_send_targets(self):
        try:
            vals = {k: float(self.ed[k].text().replace(',', '.'))
                    for k in self.ed}
        except ValueError:
            QMessageBox.warning(self, 'Hatali giris',
                                'Koordinatlari ondalik derece olarak girin.')
            return
        self._targets_latlon = {
            'turn': (vals['turn_lat'], vals['turn_lon']),
            'finish': (vals['finish_lat'], vals['finish_lon']),
        }
        self.ws.send({'cmd': 'set_targets', **vals})
        self.log.appendPlainText('Hedef koordinatlar gonderildi')

    # ------------------------------------------------ periyodik guncelleme
    def refresh(self):
        now = time.monotonic()
        auv = self.mav.get(config.AUV_SYSID)
        rov = self.mav.get(config.MINIROV_SYSID)
        st = self.ws.state or {}
        tel = st.get('telemetry', {})
        mis = st.get('mission', {})

        # telemetri etiketleri (WS varsa onu, yoksa MAVLink'i kullan)
        depth = tel.get('depth_m', auv.get('depth_m'))
        alt = tel.get('altitude_m', auv.get('altitude_m'))
        alt_valid = tel.get('altitude_valid', alt is not None)
        hdg = tel.get('heading_deg', auv.get('yaw_deg'))

        self.lbl['mode'].setText(str(tel.get('mode') or '—'))
        armed = tel.get('armed', auv.get('armed'))
        self.lbl['armed'].setText('ARMED' if armed else 'disarmed')
        self.lbl['armed'].setStyleSheet(
            'font-weight:bold; color:%s' % ('#c33' if armed else '#2a7'))
        self.lbl['depth'].setText(fmt(depth, '.2f', ' m'))
        self.lbl['alt'].setText(fmt(alt, '.2f', ' m') if alt_valid else '—')
        self.lbl['hdg'].setText(fmt(hdg, '.0f', '°'))
        self.lbl['gps'].setText(
            f"fix {auv.get('gps_fix', '—')} / {auv.get('gps_sats', '—')} uydu")
        self.lbl['batt1'].setText(fmt(auv.get('battery_v'), '.1f', ' V'))
        self.lbl['batt2'].setText(fmt(rov.get('battery_v'), '.1f', ' V'))

        def link_text(v):
            if not v or 'last_seen' not in v:
                return ('YOK', '#c33')
            age = now - v['last_seen']
            return (f'{age:.0f} sn once', '#2a7') if age < 3 else (f'KOPUK ({age:.0f} sn)', '#c33')

        for key, v in (('link1', auv), ('link2', rov)):
            text, color = link_text(v)
            self.lbl[key].setText(text)
            self.lbl[key].setStyleSheet(f'font-weight:bold; color:{color}')
        self.lbl['ws'].setText('bagli' if self.ws.connected else 'KOPUK')
        self.lbl['ws'].setStyleSheet(
            'font-weight:bold; color:%s' % ('#2a7' if self.ws.connected else '#c33'))

        # gorev durumu
        self.lbl_mission.setText(mis.get('state_name', 'IDLE'))
        step_name = mis.get('step_name') or '—'
        self.lbl_step.setText(
            f"adim {mis.get('step_index', 0) + 1}/{mis.get('step_count', 0)}: "
            f"{step_name}  ({mis.get('step_elapsed_s', 0)}s / "
            f"gorev {mis.get('mission_elapsed_s', 0)}s)"
            if mis.get('step_count') else '—')

        # ozel widget'lar
        self.profile.set_data(depth or 0.0, alt or 0.0,
                              depth is not None, bool(alt_valid and alt))
        dr = tel.get('dr')
        if dr:
            self.drmap.set_state(dr)
            self.drmap.set_heading(hdg or 0.0)
            if self._targets_latlon and dr.get('origin_valid'):
                # origin = est - offset; dogrudan est uzerinden yaklasik cizim
                import math
                o_lat = dr['est_lat'] - (dr['y_north_m'] / 6371000.0) * 180.0 / math.pi
                o_lon = dr['est_lon'] - (dr['x_east_m'] / (
                    6371000.0 * math.cos(math.radians(dr['est_lat'])))) * 180.0 / math.pi
                self.drmap.set_targets_latlon(o_lat, o_lon, self._targets_latlon)

        # log akisi
        entries = list(self.ws.log)
        if len(entries) > self._log_len:
            for line in entries[self._log_len:]:
                self.log.appendPlainText(line)
            self._log_len = len(entries)
        msg = mis.get('message')
        if msg and getattr(self, '_last_msg', '') != msg:
            self._last_msg = msg
            self.log.appendPlainText(f'[gorev] {msg}')


def main():
    demo = '--demo' in sys.argv
    app = QApplication(sys.argv)
    win = MainWindow(demo=demo)
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
