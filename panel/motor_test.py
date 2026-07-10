#!/usr/bin/env python3
"""Motor Test ve Diagnostik Araci (BAGIMSIZ - ROS gerektirmez).

Dogrudan CUAV V6X USB'sine pymavlink ile baglanir; MAV_CMD_DO_MOTOR_TEST ile
motorlari TEK TEK dondurur (ARM gerektirmez, ArduSub karistiricisini baypas
eder). Guc modulu akimindan otomatik "calisiyor" tespiti yapar ve sonuclari
rapor dosyasina kaydeder.

    python motor_test.py            gercek (COM portu secilir)
    python motor_test.py --demo     donanimsiz onizleme

ONEMLI: Test edilen numara ArduSub'in KENDI cikis numarasidir; senin renk
etiketinle (kirmizi=M1 vb.) ayni OLMAYABILIR. Her testte hangi fiziksel
motorun donduğunu "Gozlem" kutusuna yaz - amac tam bu eslemeyi cikarmak.

UYARI: Iticiler suyla sogur. Susuz test KISA (birkac sn) ve DUSUK gucte
(%5-15) yapilmali. "TUMUNU DURDUR" butonu her an motorlari durdurur.
"""
import sys
import threading
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QShortcut, QKeySequence
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QLabel,
                               QGridLayout, QHBoxLayout, QVBoxLayout,
                               QPushButton, QComboBox, QGroupBox, QLineEdit,
                               QPlainTextEdit, QSpinBox, QSlider, QFileDialog,
                               QFrame)

# ArduSub cikis numarasi -> senin fiziksel etiketin (DOGRULANACAK ipucu)
MOTORS = [
    (1, 'std: sag on'),
    (2, 'std: sol on'),
    (3, 'std: sag arka'),
    (4, 'std: sol arka'),
    (5, 'dikey'),
    (6, 'dikey'),
    (7, 'dikey'),
    (8, 'dikey'),
]

CURRENT_DELTA_THRESHOLD_A = 0.15  # bu kadar akim artisi = "calisiyor"


# ============================================================ MAVLink katmani
class MavLink:
    """Dogrudan seri (USB) baglanti + MOTOR_TEST gonderimi + telemetri okuma."""

    def __init__(self):
        self.master = None
        self._lock = threading.Lock()
        self._rx = None
        self._run = False
        self._streams_requested = False
        # paylasilan durum
        self.connected = False
        self.last_hb = 0.0
        self.armed = False
        self.mode = 0
        self.voltage = None
        self.current_a = None
        self.target_sys = 1
        self.target_comp = 1
        self.acks = []            # (cmd, result, t)
        self.sensors = {}         # msg_type -> last_seen
        self.statustexts = []     # FC mesajlari (arming sebepleri vb.)
        self.error = ''
        self._hb = None

    @staticmethod
    def list_ports():
        try:
            from serial.tools import list_ports
            return [(p.device, f'{p.device} - {p.description}')
                    for p in list_ports.comports()]
        except Exception:
            return []

    def connect(self, device, baud=115200):
        from pymavlink import mavutil
        self.disconnect()
        try:
            self.master = mavutil.mavlink_connection(device, baud=baud,
                                                     source_component=194)
        except Exception as e:
            self.error = f'Baglanti acilamadi: {e}'
            return False
        self._run = True
        self._streams_requested = False
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()
        # GCS heartbeat (arming ve GCS-failsafe icin gerekli)
        self._hb = threading.Thread(target=self._hb_loop, daemon=True)
        self._hb.start()
        return True

    def _hb_loop(self):
        from pymavlink import mavutil
        while self._run and self.master:
            try:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
            time.sleep(1.0)

    def arm(self, do_arm=True):
        from pymavlink import mavutil
        if not self.master:
            return
        with self._lock:
            self.master.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1.0 if do_arm else 0.0, 0, 0, 0, 0, 0, 0)

    def _request_streams(self):
        """SYS_STATUS + BATTERY_STATUS akimini hizlandir (akim tespiti icin)."""
        from pymavlink import mavutil
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
                       mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS):
            try:
                self.master.mav.command_long_send(
                    self.target_sys, self.target_comp,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    float(msg_id), 200000.0, 0, 0, 0, 0, 0)  # 5 Hz
            except Exception:
                pass
        try:  # eski firmware'ler icin yedek
            self.master.mav.request_data_stream_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5, 1)
        except Exception:
            pass

    def disconnect(self):
        self._run = False
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
        self.master = None
        self.connected = False

    def _rx_loop(self):
        from pymavlink import mavutil
        while self._run and self.master:
            try:
                m = self.master.recv_match(blocking=True, timeout=1.0)
            except Exception as e:
                self.error = f'rx: {e}'
                time.sleep(0.5)
                continue
            if m is None:
                if time.time() - self.last_hb > 3:
                    self.connected = False
                continue
            t = m.get_type()
            now = time.time()
            self.sensors[t] = now
            if t == 'HEARTBEAT' and m.get_srcComponent() == 1:
                self.connected = True
                self.last_hb = now
                self.target_sys = m.get_srcSystem()
                self.armed = bool(m.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = m.custom_mode
                if not self._streams_requested:
                    self._streams_requested = True
                    self._request_streams()
            elif t == 'SYS_STATUS':
                self.voltage = m.voltage_battery / 1000.0
                self.current_a = (m.current_battery / 100.0
                                  if m.current_battery != -1 else None)
            elif t == 'BATTERY_STATUS':
                if m.voltages and m.voltages[0] != 65535:
                    self.voltage = m.voltages[0] / 1000.0
                if m.current_battery != -1:
                    self.current_a = m.current_battery / 100.0
            elif t == 'COMMAND_ACK':
                self.acks.append((m.command, m.result, now))
                self.acks = self.acks[-30:]
            elif t == 'STATUSTEXT':
                txt = m.text if isinstance(m.text, str) else m.text.decode(errors='ignore')
                self.statustexts.append(txt)
                self.statustexts = self.statustexts[-40:]

    def motor_test(self, motor_num, throttle_pct, duration_s):
        from pymavlink import mavutil
        if not self.master:
            return
        with self._lock:
            self.master.mav.command_long_send(
                self.target_sys, self.target_comp,
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
                float(motor_num),   # param1: motor cikis no (1 tabanli)
                0,                  # param2: 0 = yuzde
                float(throttle_pct),
                float(duration_s),  # param4: sure sonunda otomatik durur
                0,                  # param5: tek motor
                0, 0)

    def stop_all(self):
        """Tum motorlara guc 0 / sure 0 gonder (acil durdurma)."""
        for n, _ in MOTORS:
            self.motor_test(n, 0, 0)


# ============================================================ Demo katmani
class DemoMav(MavLink):
    def __init__(self):
        super().__init__()
        self._base = 0.4
        self._boost = 0.0
        self._boost_until = 0.0

    def connect(self, device, baud=115200):
        self.connected = True
        self.last_hb = time.time()
        self.target_sys = 1
        for s in ('HEARTBEAT', 'SYS_STATUS', 'ATTITUDE', 'SCALED_PRESSURE2',
                  'DISTANCE_SENSOR', 'GPS_RAW_INT'):
            self.sensors[s] = time.time()
        return True

    def disconnect(self):
        self.connected = False

    @property
    def voltage(self):
        return 15.9

    @voltage.setter
    def voltage(self, v):
        pass

    @property
    def current_a(self):
        self.last_hb = time.time()
        extra = self._boost if time.time() < self._boost_until else 0.0
        return round(self._base + extra, 2)

    @current_a.setter
    def current_a(self, v):
        pass

    def arm(self, do_arm=True):
        self.armed = bool(do_arm)
        self.statustexts.append('ARMED' if do_arm else 'DISARMED')

    def motor_test(self, motor_num, throttle_pct, duration_s):
        if throttle_pct > 0:
            self._boost = 0.5 + throttle_pct * 0.05
            self._boost_until = time.time() + duration_s
            self.acks.append((209, 0, time.time()))

    def stop_all(self):
        self._boost_until = 0.0


# ============================================================ GUI
def dot(ok):
    lbl = QLabel('●')
    lbl.setStyleSheet('color:%s; font-size:16px' % ('#2a2' if ok else '#a33'))
    return lbl


class MotorTestWindow(QMainWindow):
    def __init__(self, demo=False):
        super().__init__()
        self.demo = demo
        self.mav = DemoMav() if demo else MavLink()
        self.setWindowTitle('Motor Test & Diagnostik' + (' [DEMO]' if demo else ''))
        self.resize(920, 720)

        # aktif test takibi (akim delta tespiti)
        self._test = None   # dict: motor, baseline, peak, end_t
        self._obs = {}      # motor -> QLineEdit (gozlem)
        self._result_lbl = {}
        self._sensor_dots = {}
        self._last_ack_len = 0
        self._last_st_len = 0

        self._build_ui()

        # Klavye acil durdurma: ESC (metin kutusuyla cakismaz).
        # WindowShortcut -> pencere aktifken her yerden calisir.
        for keyseq in (Qt.Key_Escape,):
            sc = QShortcut(QKeySequence(keyseq), self)
            sc.activated.connect(self._emergency_stop)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(150)

    # ---------------------------------------------------------- UI
    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        L = QVBoxLayout(root)

        # --- uyari banneri ---
        warn = QLabel('⚠  DUR: buyuk kirmizi buton veya ESC tusu (her an). '
                      'Motor testi girilen sure sonunda OTOMATIK durur (USB kopsa '
                      'bile FC durdurur). Guc DUSUK/sure KISA tut (%10, 2 sn). '
                      'En son guvenlik: donanim acil-stop / batarya fisini elde tut. '
                      'Test edilen numara ArduSub cikisidir; donen fiziksel motoru '
                      'Gozlem\'e yaz.')
        warn.setWordWrap(True)
        warn.setStyleSheet('background:#3a2f10; color:#ffd; padding:8px; '
                           'border:1px solid #7a6; border-radius:4px')
        L.addWidget(warn)

        # --- baglanti bari ---
        conn = QHBoxLayout()
        conn.addWidget(QLabel('COM portu:'))
        self.cmb_port = QComboBox()
        self.cmb_port.setMinimumWidth(280)
        conn.addWidget(self.cmb_port)
        b_ref = QPushButton('Yenile')
        b_ref.clicked.connect(self._refresh_ports)
        conn.addWidget(b_ref)
        self.b_conn = QPushButton('Baglan')
        self.b_conn.clicked.connect(self._toggle_conn)
        conn.addWidget(self.b_conn)
        self.lbl_conn = QLabel('kopuk')
        self.lbl_conn.setStyleSheet('font-weight:bold; color:#a33')
        conn.addWidget(self.lbl_conn)
        conn.addStretch(1)
        L.addLayout(conn)
        self._refresh_ports()

        # --- canli durum ---
        st = QGroupBox('Durum')
        g = QHBoxLayout(st)
        self.lbl_arm = QLabel('—')
        self.lbl_volt = QLabel('—')
        self.lbl_cur = QLabel('—')
        for title, w in [('ARM:', self.lbl_arm), ('Voltaj:', self.lbl_volt),
                         ('Akim:', self.lbl_cur)]:
            g.addWidget(QLabel(title))
            w.setStyleSheet('font-weight:bold; font-size:14px')
            g.addWidget(w)
            g.addSpacing(20)
        g.addStretch(1)
        L.addWidget(st)

        # --- ARM / DISARM (bu ArduSub surumu motor testi icin ARM ister) ---
        arm_row = QHBoxLayout()
        self.b_arm = QPushButton('ARM ET')
        self.b_arm.setStyleSheet('background:#284; color:white; font-weight:bold; padding:6px')
        self.b_arm.clicked.connect(lambda: self.mav.arm(True))
        self.b_disarm = QPushButton('DISARM')
        self.b_disarm.clicked.connect(lambda: self.mav.arm(False))
        arm_row.addWidget(self.b_arm)
        arm_row.addWidget(self.b_disarm)
        arm_row.addWidget(QLabel('  (motor testi ARM GEREKTIRMEZ - DISARM iken calisir. '
                                 'ARM sadece ileride manuel surus icin.)'))
        arm_row.addStretch(1)
        L.addLayout(arm_row)

        # --- global ayarlar ---
        gs = QHBoxLayout()
        gs.addWidget(QLabel('Varsayilan guc %:'))
        self.sp_pow = QSpinBox()
        self.sp_pow.setRange(1, 100)
        self.sp_pow.setValue(10)
        gs.addWidget(self.sp_pow)
        gs.addWidget(QLabel('Sure (sn):'))
        self.sp_dur = QSpinBox()
        self.sp_dur.setRange(1, 15)
        self.sp_dur.setValue(2)
        gs.addWidget(self.sp_dur)
        gs.addStretch(1)
        L.addLayout(gs)

        # --- motor tablosu ---
        mt = QGroupBox('Motorlar (ArduSub cikis no)')
        grid = QGridLayout(mt)
        headers = ['Motor', 'İpucu', 'Test', 'Akim sonucu', 'Gozlem (donen fiziksel motor)']
        for c, h in enumerate(headers):
            lab = QLabel(h)
            lab.setStyleSheet('font-weight:bold')
            grid.addWidget(lab, 0, c)
        for r, (num, hint) in enumerate(MOTORS, start=1):
            grid.addWidget(QLabel(f'M{num}'), r, 0)
            grid.addWidget(QLabel(hint), r, 1)
            btn = QPushButton('Test Et')
            btn.clicked.connect(lambda _=False, n=num: self._start_test(n))
            grid.addWidget(btn, r, 2)
            res = QLabel('—')
            self._result_lbl[num] = res
            grid.addWidget(res, r, 3)
            obs = QLineEdit()
            obs.setPlaceholderText('orn. kirmizi / sag arka')
            self._obs[num] = obs
            grid.addWidget(obs, r, 4)
        L.addWidget(mt)

        # --- butonlar ---
        bb = QHBoxLayout()
        self.b_stop = QPushButton('TÜMÜNÜ DURDUR  (ESC)')
        self.b_stop.setStyleSheet('background:#c33; color:white; font-weight:bold; '
                                  'font-size:16px; padding:12px')
        self.b_stop.clicked.connect(self._emergency_stop)
        bb.addWidget(self.b_stop, 2)
        b_seq = QPushButton('Sirayla 1→8 Test')
        b_seq.clicked.connect(self._sequence)
        bb.addWidget(b_seq, 1)
        b_rep = QPushButton('Raporu Kaydet')
        b_rep.clicked.connect(self._save_report)
        bb.addWidget(b_rep, 1)
        L.addLayout(bb)

        # --- sensor durumu (bonus) ---
        sg = QGroupBox('Sensor baglanti durumu (mesaj geliyor mu)')
        sgl = QHBoxLayout(sg)
        for name, msg in [('Bar30', 'SCALED_PRESSURE2'), ('Ping', 'DISTANCE_SENSOR'),
                          ('GPS', 'GPS_RAW_INT'), ('IMU', 'ATTITUDE')]:
            sgl.addWidget(QLabel(name))
            d = dot(False)
            self._sensor_dots[msg] = d
            sgl.addWidget(d)
            sgl.addSpacing(16)
        sgl.addStretch(1)
        L.addWidget(sg)

        # --- log ---
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(400)
        self.log.setMaximumHeight(140)
        L.addWidget(self.log)

    # ---------------------------------------------------------- olaylar
    def _refresh_ports(self):
        self.cmb_port.clear()
        ports = MavLink.list_ports()
        if not ports and self.demo:
            ports = [('DEMO', 'DEMO - sahte port')]
        for dev, desc in ports:
            self.cmb_port.addItem(desc, dev)
        if not ports:
            self.cmb_port.addItem('(port bulunamadi)', None)

    def _toggle_conn(self):
        if self.mav.connected:
            self.mav.disconnect()
            self.b_conn.setText('Baglan')
            self._log('Baglanti kesildi')
            return
        dev = self.cmb_port.currentData()
        if dev is None and not self.demo:
            self._log('! Once gecerli bir COM portu secin')
            return
        if self.mav.connect(dev or 'DEMO'):
            self.b_conn.setText('Kes')
            self._log(f'Baglaniyor: {dev}')
        else:
            self._log('! ' + self.mav.error)

    def _start_test(self, motor):
        if not self.mav.connected:
            self._log('! Once baglan')
            return
        if self.mav.armed:
            self._log('! Motorlar ARMED. Motor testi DISARM iken calisir '
                      '(QGC boyle yapiyor). Once DISARM et.')
            return
        pw = self.sp_pow.value()
        dur = self.sp_dur.value()
        base = self.mav.current_a   # None olabilir (akim raporlanmiyor)
        self._test = {'motor': motor, 'baseline': base,
                      'peak': base if base is not None else None,
                      'got_current': base is not None,
                      'end_t': time.time() + dur + 1.0}
        self._result_lbl[motor].setText('test ediliyor...')
        self._result_lbl[motor].setStyleSheet('color:#cc0')
        self.mav.motor_test(motor, pw, dur)
        base_s = f'{base:.2f} A' if base is not None else 'YOK'
        self._log(f'M{motor}: %{pw}, {dur}s test gonderildi (baz akim {base_s})')

    def _finish_test(self):
        t = self._test
        self._test = None
        motor = t['motor']

        # Durum 1: hic akim verisi gelmedi -> tespit yapilamaz (gozle dogrula)
        if not t['got_current'] or t['baseline'] is None or t['peak'] is None:
            self._result_lbl[motor].setText('akim verisi YOK — gozle dogrula')
            self._result_lbl[motor].setStyleSheet('color:#c90; font-weight:bold')
            self._log(f'M{motor}: akim raporlanmiyor. Guc modulu akim kablosu/'
                      'ayari? Motorun dondugunu gozle/sesle dogrulayip Gozlem\'e yaz.')
            return

        # Durum 2: akim verisi var -> delta hesapla
        delta = t['peak'] - t['baseline']
        detail = f'baz {t["baseline"]:.2f}→tepe {t["peak"]:.2f} A (Δ{delta:+.2f})'
        if delta >= CURRENT_DELTA_THRESHOLD_A:
            self._result_lbl[motor].setText(f'✓ calisiyor ({detail})')
            self._result_lbl[motor].setStyleSheet('color:#2a2; font-weight:bold')
            self._log(f'M{motor}: OK, {detail}')
        else:
            self._result_lbl[motor].setText(f'✗ degisim yok ({detail})')
            self._result_lbl[motor].setStyleSheet('color:#a33; font-weight:bold')
            self._log(f'M{motor}: akim degismedi, {detail}. Guc dusukse artir '
                      '(%15-20) veya kablo/ESC/guc kontrol et; gozle de bak.')

    def _stop_all(self):
        if self.mav.connected:
            self.mav.stop_all()
        self._test = None
        self._log('TUMU DURDURULDU')

    def _emergency_stop(self):
        """Acil durdurma: butondan veya ESC'den. Birden cok kez guc-0 gonderir
        (paket kaybina karsi) ve varsa sirali testi iptal eder."""
        self._seq_i = len(MOTORS)  # sirali test dongusunu durdur
        if self.mav.connected:
            for _ in range(3):
                self.mav.stop_all()
            self.mav.arm(False)     # motorlari tamamen kes (disarm)
        self._test = None
        for lbl in self._result_lbl.values():
            if 'test ediliyor' in lbl.text():
                lbl.setText('durduruldu')
                lbl.setStyleSheet('color:#a33')
        self._log('*** ACIL DURDURMA - tum motorlara guc 0 gonderildi ***')

    def keyPressEvent(self, event):
        # ESC'e ek olarak, metin kutusunda DEGILKEN bosluk da acil durdurur
        if event.key() == Qt.Key_Space and not isinstance(
                self.focusWidget(), QLineEdit):
            self._emergency_stop()
        else:
            super().keyPressEvent(event)

    def _sequence(self):
        if not self.mav.connected:
            self._log('! Once baglan')
            return
        # basit sirali: her motoru sure+0.8s arayla test et
        dur = self.sp_dur.value()
        self._seq_i = 0
        gap = int((dur + 1.2) * 1000)

        def step():
            if self._seq_i >= len(MOTORS):
                self._log('Sirali test bitti')
                return
            n = MOTORS[self._seq_i][0]
            self._start_test(n)
            self._seq_i += 1
            QTimer.singleShot(gap, step)
        step()

    def _save_report(self):
        from datetime import datetime
        lines = ['# Motor Test Raporu', f'# {datetime.now():%Y-%m-%d %H:%M}',
                 '# ArduSub_cikis, ipucu, akim_sonucu, gozlem(fiziksel motor)']
        for num, hint in MOTORS:
            res = self._result_lbl[num].text()
            obs = self._obs[num].text()
            lines.append(f'M{num},{hint},{res},{obs}')
        text = '\n'.join(lines) + '\n'
        path, _ = QFileDialog.getSaveFileName(
            self, 'Raporu Kaydet', 'motor_test_raporu.txt', 'Metin (*.txt)')
        if path:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text)
            self._log(f'Rapor kaydedildi: {path}')

    def _log(self, msg):
        self.log.appendPlainText(msg)

    # ---------------------------------------------------------- periyodik
    def _refresh(self):
        m = self.mav
        # baglanti
        conn = m.connected and (time.time() - m.last_hb) < 3
        self.lbl_conn.setText('bagli' if conn else 'kopuk')
        self.lbl_conn.setStyleSheet('font-weight:bold; color:%s'
                                    % ('#2a2' if conn else '#a33'))
        # durum
        self.lbl_arm.setText('ARMED' if m.armed else 'disarmed')
        self.lbl_arm.setStyleSheet('font-weight:bold; font-size:14px; color:%s'
                                   % ('#c33' if m.armed else '#2a2'))
        self.lbl_volt.setText(f'{m.voltage:.1f} V' if m.voltage else '—')
        cur = m.current_a
        self.lbl_cur.setText(f'{cur:.2f} A' if cur is not None else '—')

        # aktif test: akim tepe takibi + bitis
        if self._test is not None:
            if cur is not None:
                self._test['got_current'] = True
                if self._test['baseline'] is None:
                    self._test['baseline'] = cur
                if self._test['peak'] is None:
                    self._test['peak'] = cur
                else:
                    self._test['peak'] = max(self._test['peak'], cur)
            if time.time() >= self._test['end_t']:
                self._finish_test()

        # ack'ler (reddedilen komutlari bildir)
        if len(m.acks) > self._last_ack_len:
            for cmd, result, _ in m.acks[self._last_ack_len:]:
                if cmd == 209 and result != 0:  # MAV_CMD_DO_MOTOR_TEST
                    self._log(f'! Motor testi reddedildi (sonuc={result}) - '
                              'genelde ARM degil veya batarya/arming-check sorunu')
            self._last_ack_len = len(m.acks)

        # FC mesajlari (arming sebepleri: "Battery unhealthy" vb.)
        if len(m.statustexts) > self._last_st_len:
            for txt in m.statustexts[self._last_st_len:]:
                self._log('FC: ' + txt)
            self._last_st_len = len(m.statustexts)

        # sensor noktalari
        now = time.time()
        for msg, d in self._sensor_dots.items():
            ok = (now - m.sensors.get(msg, 0)) < 3
            d.setStyleSheet('color:%s; font-size:16px' % ('#2a2' if ok else '#a33'))


def main():
    demo = '--demo' in sys.argv
    app = QApplication(sys.argv)
    win = MotorTestWindow(demo=demo)
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
