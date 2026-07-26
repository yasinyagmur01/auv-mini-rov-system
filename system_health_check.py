#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
system_health_check.py — AUV + Mini ROV sistem sağlık testi (SALT-OKUNUR)

Fiber/Ethernet topolojisi, ROS 2 haberleşmesi ve sensör veri akışını tek seferde
denetler. Terminalde renkli PASS/FAIL/WARN basar, logs/ altına zaman damgalı log yazar.

=======================  GÜVENLİK SÖZLEŞMESİ  =======================
Bu script HİÇBİR ŞEKİLDE:
  * ROS 2 publisher OLUŞTURMAZ (kodda tek bir create_publisher çağrısı yoktur),
  * arm/disarm, motor_test, direct_output, manual_control komutu göndermez,
  * MAVLink yazmaz, FC parametresi değiştirmez,
  * HTTP POST atmaz (yalnız GET / TCP connect yapar).
Sadece dinler, ping atar ve okur. FORBIDDEN_TOPICS listesi yalnızca "başkası
yayın yapıyor mu" diye SAYMAK için kullanılır.
=====================================================================

Kullanım:
    python3 system_health_check.py                  # haberleşme + sensör (auto)
    python3 system_health_check.py --comms-only     # SADECE haberleşme/topoloji
    python3 system_health_check.py -i               # "hangi sensörler takılı?" diye sor
    python3 system_health_check.py --only depth,sonar,zed   # sadece bunları test et
    python3 system_health_check.py --no-ros         # sadece ağ + HTTP (ROS'suz PC)
    python3 system_health_check.py --sample 10      # sensör örnekleme penceresi (sn)
    python3 system_health_check.py --extra-ip 192.168.2.10:Pixhawk-ETH
    python3 system_health_check.py --json rapor.json

Aşama akışı:
    1) Şimdi (haberleşme kritik):   python3 system_health_check.py --comms-only
    2) Sensör(ler) takılınca:       python3 system_health_check.py -i
       (menüden takılı olanları seç → yalnız onlar test edilir, veri yoksa FAIL)
"""

import argparse
import json
import os
import platform
import re
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(REPO_ROOT, "logs")

# Tüm testler için mutlak üst sınır (gereksinim: 60 sn'yi aşma)
GLOBAL_BUDGET_S = 55.0

# ---------------------------------------------------------------------------
# A) AĞ TOPOLOJİSİ — repodaki konfigürasyonlardan toplanan sabit IP'ler
#    Kaynaklar: config/netplan/01-auv-static.yaml, config/mavlink-router/main.conf,
#               docs/ag-kurulumu.md, docs/kablolama.md, jetson/ros2_web_bridge.py
# ---------------------------------------------------------------------------
# (ip, rol, kritik_mi, kaynak_dosya)
HOSTS = [
    ("192.168.2.135", "Jetson Orin NX (AUV) — enP8p1s0 statik",
     True,  "config/netplan/01-auv-static.yaml:8"),
    ("192.168.2.2",   "Mini ROV — RPi4 + Navigator (BlueOS)",
     True,  "docs/ag-kurulumu.md:8"),
    ("192.168.2.1",   "Kontrol PC (planlanan / mavlink-router hedefi)",
     False, "config/mavlink-router/main.conf:19"),
    ("192.168.2.196", "Kontrol PC (sahada gözlenen adres)",
     False, "docs/DURUM-DEVAM.md:10"),
    ("192.168.2.4",   "Yedek laptop (opsiyonel)",
     False, "docs/ag-kurulumu.md:10"),
    ("127.0.0.1",     "Loopback (yerel yığın)",
     True,  "config/mavlink-router/main.conf:23"),
]

# Fiber medya dönüştürücüler L2-şeffaftır: IP'leri YOKTUR, ping atılamaz.
# Fiber hattın sağlığı yerel arayüzün carrier/link durumundan okunur.
MEDIA_CONVERTERS = [
    ("Fiber dönüştürücü A", "AUV gövde switch'i ↔ omurga", "docs/kablolama.md:12"),
    ("Fiber dönüştürücü B", "Mini ROV RPi4 ETH ↔ omurga", "docs/kablolama.md:22"),
]
NET_IFACES = ["enP8p1s0"]  # Jetson'daki tek yapılandırılmış arayüz

# CUAV V6X bu repoda ETHERNET ile DEĞİL, USB seri ile bağlı (udev: /dev/ttyFC).
# Ethernet portu kullanılıyorsa --extra-ip ile IP'sini ver.
FC_SERIAL_PATHS = ["/dev/ttyFC", "/dev/ttyACM0"]

# ---------------------------------------------------------------------------
# HTTP / TCP servis uçları (yalnız GET ve TCP-connect; POST YOK)
# ---------------------------------------------------------------------------
# (ad, tip, host, port, yol, kritik_mi, kaynak)
SERVICES = [
    ("ros2_web_bridge /sensors", "http", "192.168.2.135", 8000, "/sensors",
     True,  "jetson/ros2_web_bridge.py:896"),
    ("auv_mission WebSocket",    "tcp",  "192.168.2.135", 8765, "",
     True,  "ros2_ws/src/auv_mission/auv_mission/ws_server.py:15"),
    ("ui/ FastAPI paneli",       "tcp",  "192.168.2.135", 8080, "",
     False, "ui/run_ui.sh:6"),
    ("Mini ROV mavlink2rest",    "http", "192.168.2.2",   6040, "/mavlink",
     False, "jetson/ros2_web_bridge.py:761"),
    ("Mini ROV WebRTC (MCM)",    "tcp",  "192.168.2.2",   6021, "",
     False, "jetson/minirov.html:104"),
    ("BlueOS web arayüzü",       "tcp",  "192.168.2.2",   80,   "",
     False, "docs/ardusub-kurulum.md:45"),
]

# ---------------------------------------------------------------------------
# B/C) ROS 2 topic envanteri
#      DİKKAT: Bu depoda MAVROS KULLANILMAZ. auv_mav_bridge doğrudan pymavlink
#      ile konuşur (ros2_ws/src/auv_mav_bridge/package.xml:6). Gerçek isim alanı
#      /mav/... — /mavros/... yalnızca uyumluluk kontrolü olarak sorgulanır.
# ---------------------------------------------------------------------------
# key: (topic, beklenen_tip, kategori, kritik_mi, açıklama, alan_yolu)
#   alan_yolu: örnekten sayısal değer çıkarmak için (min/max/ort için); None ise sadece Hz
TOPICS = [
    # --- B) Haberleşme / durum ---
    ("/mav/link_ok",      "std_msgs/msg/Bool",              "B", True,
     "FC MAVLink bağlantısı", "data"),
    ("/mav/armed",        "std_msgs/msg/Bool",              "B", False,
     "Arm durumu (salt-okunur)", "data"),
    ("/mav/mode",         "std_msgs/msg/String",            "B", False,
     "Uçuş modu", None),
    ("/mav/attitude",     "geometry_msgs/msg/Vector3Stamped", "B", False,
     "Attitude (roll/pitch/yaw, rad)", "vector.z"),
    ("/mav/heading_deg",  "std_msgs/msg/Float32",           "B", False,
     "Pusula başlığı (derece)", "data"),
    ("/mav/statustext",   "std_msgs/msg/String",            "B", False,
     "FC STATUSTEXT akışı", None),
    ("/mav/servo_out",    "std_msgs/msg/UInt16MultiArray",  "B", False,
     "SERVO_OUTPUT_RAW (salt-okunur telemetri)", None),
    ("/vertical_state",   "auv_msgs/msg/VerticalState",     "B", False,
     "Füzyonlanmış dikey durum", None),
    ("/dr/state",         "auv_msgs/msg/DeadReckonState",   "B", False,
     "Dead reckoning konumu", None),
    ("/mission/state",    "auv_msgs/msg/MissionState",      "B", False,
     "Görev durum makinesi", None),

    # --- C) Sensör veri akışı ---
    ("/mav/depth",        "std_msgs/msg/Float32",           "C", True,
     "Bar30 derinlik (m, aşağı +)", "data"),
    ("/mav/water_temp",   "std_msgs/msg/Float32",           "C", False,
     "Bar30 su sıcaklığı (°C)", "data"),
    ("/mav/board_temp",   "std_msgs/msg/Float32",           "C", False,
     "CUAV ADC kart sıcaklığı (°C) — SCALED_PRESSURE3", "data"),
    ("/mav/rangefinder",  "sensor_msgs/msg/Range",          "C", True,
     "Ping sonar mesafe (m)", "range"),
    ("/mav/gps",          "sensor_msgs/msg/NavSatFix",      "C", False,
     "GPS NavSatFix", "latitude"),
    ("/mav/gps_info",     "auv_msgs/msg/GpsInfo",           "C", False,
     "GPS/RTK fix tipi + uydu sayısı", "fix_type"),
    ("/mav/leak",         "std_msgs/msg/Bool",              "C", False,
     "Sızıntı (MANDALLI, yalnız True yayınlanır)", "data"),
    ("/mav/battery",      "sensor_msgs/msg/BatteryState",   "C", False,
     "Batarya gerilimi (V)", "voltage"),
    ("/zed/zed_node/rgb/color/rect/image/compressed",
     "sensor_msgs/msg/CompressedImage",                     "C", True,
     "ZED 2i RGB compressed kare akışı", None),
    ("/zed/zed_node/imu/data", "sensor_msgs/msg/Imu",       "C", False,
     "ZED 2i IMU", None),
    ("/zed/zed_node/odom",     "nav_msgs/msg/Odometry",     "C", False,
     "ZED 2i görsel odometri", None),
]

# Sensör topic'lerini mantıksal gruplara bağlayan anahtarlar (interaktif / --only seçimi).
TOPIC_KEY = {
    "/mav/depth": "depth",
    "/mav/water_temp": "water_temp",
    "/mav/board_temp": "board_temp",
    "/mav/rangefinder": "sonar",
    "/mav/gps": "gps",
    "/mav/gps_info": "gps",
    "/mav/leak": "leak",
    "/mav/battery": "battery",
    "/zed/zed_node/rgb/color/rect/image/compressed": "zed",
    "/zed/zed_node/imu/data": "zed",
    "/zed/zed_node/odom": "zed",
}

# İnteraktif menü / --only için sensör grupları (anahtar, etiket).
SENSOR_MENU = [
    ("depth",      "Bar30 derinlik/basınç   (/mav/depth + I2C 0x76)"),
    ("water_temp", "Bar30 su sıcaklığı       (/mav/water_temp)"),
    ("board_temp", "CUAV ADC kart sıcaklığı  (/mav/board_temp)"),
    ("sonar",      "Ping sonar mesafe        (/mav/rangefinder)"),
    ("gps",        "GPS / RTK                (/mav/gps, /mav/gps_info)"),
    ("leak",       "Sızıntı (leak)           (/mav/leak)"),
    ("battery",    "Batarya                  (/mav/battery)"),
    ("zed",        "ZED 2i kamera            (rgb/imu/odom)"),
    ("mrov",       "Mini ROV telemetri       (.2 mavlink2rest)"),
]
SENSOR_KEYS = [k for k, _ in SENSOR_MENU]

# Spesifikasyonda istenen MAVROS topic'leri — bu depoda YOKTUR (bkz. yukarıdaki not).
# Yine de graftan sorgulanır; bulunmazsa FAIL değil WARN üretir.
MAVROS_COMPAT_TOPICS = [
    "/mavros/state",
    "/mavros/imu/data",
    "/mavros/global_position/global",
]

# Beklenen ROS 2 düğümleri: (düğüm_adı, kritik_mi, paket)
EXPECTED_NODES = [
    ("mav_bridge",      True,  "auv_mav_bridge"),
    ("mission",         True,  "auv_mission"),
    ("ros2_web_bridge", True,  "jetson/ros2_web_bridge.py"),
    ("vertical_state",  False, "auv_vertical_state"),
    ("dead_reckoning",  False, "auv_dead_reckoning"),
    ("zed_node",        False, "zed_wrapper (Docker)"),
]

# ASLA yayın yapılmayacak topic'ler. Burada YALNIZCA publisher SAYILIR.
FORBIDDEN_TOPICS = [
    "/mav/manual_control",
    "/mav/cmd/motor_test",
    "/mav/cmd/direct_output",
    "/mav/cmd/arm",
    "/mav/cmd/mode",
]
# direct_output çökme kurtarma işareti (bridge_node.py:162)
DIRECT_MODE_MARKER = os.path.expanduser("~/auv/.direct_mode_active")

IS_WINDOWS = platform.system().lower().startswith("win")


# ===========================================================================
# Renk + log altyapısı
# ===========================================================================
class C:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    DIM = "\033[2m"
    GREEN = "\033[32m"
    RED = "\033[31m"
    YELLOW = "\033[33m"
    BLUE = "\033[36m"
    GREY = "\033[90m"


_ANSI_RE = re.compile(r"\033\[[0-9;]*m")


def _setup_console():
    """Konsolu UTF-8'e al (Türkçe + kutu karakterleri için). Jetson'da zaten UTF-8."""
    if IS_WINDOWS:
        try:
            import ctypes
            ctypes.windll.kernel32.SetConsoleOutputCP(65001)
        except Exception:  # noqa: BLE001
            pass
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:  # noqa: BLE001
            pass


def _enable_colors():
    """Windows terminalinde ANSI'yi aç; desteklenmiyorsa renkleri boşalt."""
    if not sys.stdout.isatty():
        _strip_colors()
        return
    if IS_WINDOWS:
        try:
            import ctypes
            k = ctypes.windll.kernel32
            k.SetConsoleMode(k.GetStdHandle(-11), 7)  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
        except Exception:
            _strip_colors()


def _strip_colors():
    for attr in dir(C):
        if not attr.startswith("_"):
            setattr(C, attr, "")


class Logger:
    """Terminale renkli, dosyaya düz metin yazar."""

    def __init__(self, path):
        self.path = path
        self._lock = threading.Lock()
        self._fh = open(path, "w", encoding="utf-8")

    def line(self, text=""):
        with self._lock:
            try:
                print(text)
            except UnicodeEncodeError:
                # Konsol kod sayfası dar — terminalde ASCII'ye düş (log dosyası UTF-8 kalır)
                print(text.encode("ascii", "replace").decode("ascii"))
            self._fh.write(_ANSI_RE.sub("", text) + "\n")
            self._fh.flush()

    def file_only(self, text):
        with self._lock:
            self._fh.write(_ANSI_RE.sub("", text) + "\n")
            self._fh.flush()

    def close(self):
        self._fh.close()


# ===========================================================================
# Sonuç modeli
# ===========================================================================
PASS, FAIL, WARN, SKIP = "PASS", "FAIL", "WARN", "SKIP"

# İnsan-okur sensör veri-akış durumları (kullanıcı isteği)
STATE_FLOW = "BAĞLI · VERİ AKIYOR"
STATE_NODATA = "BAĞLANTI VAR AMA VERİ GELMİYOR"
STATE_NOCONN = "BAĞLANTI YOK"


class Result:
    __slots__ = ("group", "name", "status", "duration", "detail", "source", "state")

    def __init__(self, group, name, status, duration, detail="", source="", state=""):
        self.group = group
        self.name = name
        self.status = status
        self.duration = duration
        self.detail = detail
        self.source = source
        self.state = state  # sensörler için insan-okur akış durumu (STATE_*)

    def as_dict(self):
        return {
            "group": self.group, "name": self.name, "status": self.status,
            "duration_s": round(self.duration, 3), "detail": self.detail,
            "source": self.source, "state": self.state,
        }


def badge(status):
    return {
        PASS: C.GREEN + "[ PASS ]" + C.RESET,
        FAIL: C.RED + "[ FAIL ]" + C.RESET,
        WARN: C.YELLOW + "[ WARN ]" + C.RESET,
        SKIP: C.GREY + "[ SKIP ]" + C.RESET,
    }[status]


# ===========================================================================
# A) AĞ TESTLERİ
# ===========================================================================
def ping_once(ip, timeout_s=2.0):
    """Tek ping. (basarili, rtt_ms|None, ham_ozet) döner."""
    if IS_WINDOWS:
        cmd = ["ping", "-n", "1", "-w", str(int(timeout_s * 1000)), ip]
    else:
        cmd = ["ping", "-c", "1", "-W", str(int(max(1, timeout_s))), ip]
    try:
        p = subprocess.run(cmd, capture_output=True, text=True,
                           timeout=timeout_s + 3.0,
                           errors="replace")
        out = (p.stdout or "") + (p.stderr or "")
    except subprocess.TimeoutExpired:
        return False, None, "ping zaman aşımı"
    except FileNotFoundError:
        return False, None, "ping komutu bulunamadı"
    except Exception as e:  # noqa: BLE001
        return False, None, "ping hatası: %s" % e

    # Windows ping, "Destination host unreachable" durumunda da 0 dönebilir;
    # bu yüzden TTL= imzası aranır (tüm dillerde literal).
    ok = (p.returncode == 0) and ("TTL=" in out or "ttl=" in out)
    m = re.search(r"(?:time|süre|zaman)[<=]\s*([\d.,]+)\s*ms", out, re.IGNORECASE)
    rtt = None
    if m:
        try:
            rtt = float(m.group(1).replace(",", "."))
        except ValueError:
            rtt = None
    return ok, rtt, out.strip().splitlines()[-1] if out.strip() else ""


def test_hosts(hosts, timeout_s=2.0):
    results = []

    def one(entry):
        ip, role, critical, src = entry
        t0 = time.monotonic()
        ok, rtt, raw = ping_once(ip, timeout_s)
        dt = time.monotonic() - t0
        if ok:
            detail = "yanıt var" + (" · rtt %.1f ms" % rtt if rtt is not None else "")
            return Result("A/Ağ", "%-15s %s" % (ip, role), PASS, dt, detail, src)
        status = FAIL if critical else WARN
        detail = "yanıt YOK (%.1fs timeout)" % timeout_s
        if raw:
            detail += " · " + raw[:80]
        if not critical:
            detail += " · opsiyonel node"
        return Result("A/Ağ", "%-15s %s" % (ip, role), status, dt, detail, src)

    with ThreadPoolExecutor(max_workers=max(4, len(hosts))) as ex:
        results = list(ex.map(one, hosts))
    return results


def test_interfaces():
    """Yerel arayüz link/carrier durumu — fiber hattın tek ölçülebilir göstergesi."""
    out = []
    for iface in NET_IFACES:
        t0 = time.monotonic()
        base = "/sys/class/net/%s" % iface
        if IS_WINDOWS or not os.path.isdir(base):
            out.append(Result(
                "A/Ağ", "Arayüz %s link durumu" % iface, SKIP,
                time.monotonic() - t0,
                "bu makinede yok (yalnız Jetson'da anlamlı)",
                "config/netplan/01-auv-static.yaml:3"))
            continue
        try:
            with open(os.path.join(base, "carrier")) as f:
                carrier = f.read().strip()
            with open(os.path.join(base, "operstate")) as f:
                oper = f.read().strip()
            speed = ""
            try:
                with open(os.path.join(base, "speed")) as f:
                    speed = " · %s Mb/s" % f.read().strip()
            except Exception:  # noqa: BLE001
                pass
            ok = carrier == "1" and oper == "up"
            out.append(Result(
                "A/Ağ", "Arayüz %s link durumu" % iface,
                PASS if ok else FAIL, time.monotonic() - t0,
                "carrier=%s operstate=%s%s" % (carrier, oper, speed),
                "config/netplan/01-auv-static.yaml:3"))
        except Exception as e:  # noqa: BLE001
            out.append(Result("A/Ağ", "Arayüz %s link durumu" % iface, WARN,
                              time.monotonic() - t0, "okunamadı: %s" % e))
    # Medya dönüştürücüler: L2-şeffaf, IP yok — kayda geçir.
    for name, role, src in MEDIA_CONVERTERS:
        out.append(Result(
            "A/Ağ", "%s (%s)" % (name, role), SKIP, 0.0,
            "L2-şeffaf cihaz, IP'si yok — link yukarıdaki carrier ile ölçülür", src))
    return out


def test_fc_serial():
    """CUAV V6X USB seri portu (bu repoda FC ethernet ile değil USB ile bağlı)."""
    t0 = time.monotonic()
    if IS_WINDOWS:
        return Result("A/Ağ", "CUAV V6X seri portu (/dev/ttyFC)", SKIP,
                      time.monotonic() - t0, "Linux dışı makine",
                      "scripts/99-auv-serial.rules:5")
    for path in FC_SERIAL_PATHS:
        if os.path.exists(path):
            real = os.path.realpath(path)
            return Result("A/Ağ", "CUAV V6X seri portu", PASS,
                          time.monotonic() - t0, "%s → %s" % (path, real),
                          "config/mavlink-router/main.conf:13")
    return Result("A/Ağ", "CUAV V6X seri portu", FAIL, time.monotonic() - t0,
                  "hiçbiri yok: %s (udev kuralı / USB kablo)" % ", ".join(FC_SERIAL_PATHS),
                  "scripts/99-auv-serial.rules:5")


# ===========================================================================
# B) SERVİS UÇLARI (HTTP GET / TCP connect — POST YOK)
# ===========================================================================
def check_tcp(host, port, timeout_s=2.0):
    t0 = time.monotonic()
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True, time.monotonic() - t0, "TCP açık"
    except Exception as e:  # noqa: BLE001
        return False, time.monotonic() - t0, "TCP kapalı: %s" % e


def check_http_get(host, port, path, timeout_s=3.0):
    url = "http://%s:%d%s" % (host, port, path)
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as r:
            body = r.read(65536)
            return True, r.status, body
    except urllib.error.HTTPError as e:
        return True, e.code, b""          # sunucu ayakta, yol farklı olabilir
    except Exception as e:  # noqa: BLE001
        return False, 0, str(e).encode("utf-8", "replace")


def test_services(services):
    def one(svc):
        name, kind, host, port, path, critical, src = svc
        t0 = time.monotonic()
        if kind == "tcp":
            ok, _, msg = check_tcp(host, port)
            dt = time.monotonic() - t0
            return Result("B/Servis", "%s (%s:%d)" % (name, host, port),
                          PASS if ok else (FAIL if critical else WARN), dt, msg, src)
        ok, code, body = check_http_get(host, port, path)
        dt = time.monotonic() - t0
        if not ok:
            return Result("B/Servis", "%s (%s:%d%s)" % (name, host, port, path),
                          FAIL if critical else WARN, dt,
                          "erişilemedi: %s" % body.decode("utf-8", "replace")[:90], src)
        detail = "HTTP %d · %d bayt" % (code, len(body))
        # /sensors ise sözleşme anahtarlarını da doğrula (salt-okunur)
        if path == "/sensors" and body:
            try:
                data = json.loads(body.decode("utf-8", "replace"))
                # Köprü /sensors sözleşmesindeki gerçek anahtarlar (ros2_web_bridge.py)
                keys = ["connected", "link_ok", "depth_m", "water_temp_c",
                        "board_temp_c", "leak", "armed", "mode", "yaw", "gps_fix"]
                missing = [k for k in keys if k not in data]
                detail += " · anahtar %d/%d" % (len(keys) - len(missing), len(keys))
                if missing:
                    return Result("B/Servis", "%s (%s:%d%s)" % (name, host, port, path),
                                  WARN, dt, detail + " · eksik: " + ",".join(missing), src)
            except Exception as e:  # noqa: BLE001
                return Result("B/Servis", "%s (%s:%d%s)" % (name, host, port, path),
                              WARN, dt, detail + " · JSON çözülemedi: %s" % e, src)
        return Result("B/Servis", "%s (%s:%d%s)" % (name, host, port, path),
                      PASS if code < 400 else WARN, dt, detail, src)

    with ThreadPoolExecutor(max_workers=max(4, len(services))) as ex:
        return list(ex.map(one, services))


# ===========================================================================
# B/C) ROS 2 SONDASI — yalnız abone olur, ASLA yayın yapmaz
# ===========================================================================
def _get_field(msg, path):
    """'vector.z' gibi nokta yollu alan okuma; başarısızsa None."""
    if path is None:
        return None
    cur = msg
    for part in path.split("."):
        cur = getattr(cur, part, None)
        if cur is None:
            return None
    try:
        return float(cur)
    except (TypeError, ValueError):
        return None


class RosProbe:
    """Tek düğüm, çok abonelik. Örnekleme penceresi boyunca spin eder."""

    def __init__(self, sample_s, log):
        self.sample_s = sample_s
        self.log = log
        self.stats = {}      # topic -> {"n":int, "vals":[float], "bytes":int, "first":t, "last":t}
        self.graph = {}      # topic -> [type,...]
        self.nodes = []
        self.forbidden_pub = {}
        self.available = False
        self.error = ""

    def run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                                   DurabilityPolicy)
            from rosidl_runtime_py.utilities import get_message
        except Exception as e:  # noqa: BLE001
            self.error = "rclpy içe aktarılamadı (%s). ROS 2 ortamı source edilmemiş " \
                         "olabilir: source /opt/ros/humble/setup.bash && " \
                         "source ros2_ws/install/setup.bash" % e
            return

        try:
            rclpy.init(args=None)
        except Exception as e:  # noqa: BLE001
            self.error = "rclpy.init başarısız: %s" % e
            return

        node = None
        try:
            node = Node("auv_health_check_listener")
            # Tüm yayıncılarla eşleşsin diye BEST_EFFORT + VOLATILE (en gevşek abone).
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
            )

            for topic, type_str, _cat, _crit, _desc, field in TOPICS:
                self.stats[topic] = {"n": 0, "vals": [], "bytes": 0,
                                     "first": None, "last": None, "sub": False,
                                     "err": ""}
                try:
                    msg_cls = get_message(type_str)
                except Exception as e:  # noqa: BLE001
                    self.stats[topic]["err"] = "mesaj tipi çözülemedi (%s): %s" % (type_str, e)
                    continue

                def make_cb(tname, fpath):
                    st = self.stats[tname]

                    def cb(msg):
                        now = time.monotonic()
                        st["n"] += 1
                        if st["first"] is None:
                            st["first"] = now
                        st["last"] = now
                        data = getattr(msg, "data", None)
                        if isinstance(data, (bytes, bytearray)):
                            st["bytes"] += len(data)
                        v = _get_field(msg, fpath)
                        if v is not None and len(st["vals"]) < 5000:
                            st["vals"].append(v)
                    return cb

                try:
                    # >>> SADECE create_subscription — hiçbir yerde create_publisher YOK <<<
                    node.create_subscription(msg_cls, topic, make_cb(topic, field), qos)
                    self.stats[topic]["sub"] = True
                except Exception as e:  # noqa: BLE001
                    self.stats[topic]["err"] = "abone olunamadı: %s" % e

            deadline = time.monotonic() + self.sample_s
            graph_at = time.monotonic() + min(3.0, self.sample_s * 0.4)
            graph_done = False
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if not graph_done and time.monotonic() >= graph_at:
                    graph_done = True
                    try:
                        self.graph = {t: ts for t, ts in node.get_topic_names_and_types()}
                    except Exception:  # noqa: BLE001
                        self.graph = {}
                    try:
                        self.nodes = list(node.get_node_names())
                    except Exception:  # noqa: BLE001
                        self.nodes = []
                    for ft in FORBIDDEN_TOPICS:
                        try:
                            self.forbidden_pub[ft] = node.count_publishers(ft)
                        except Exception:  # noqa: BLE001
                            self.forbidden_pub[ft] = -1
            if not graph_done:
                try:
                    self.graph = {t: ts for t, ts in node.get_topic_names_and_types()}
                    self.nodes = list(node.get_node_names())
                except Exception:  # noqa: BLE001
                    pass
            self.available = True
        except Exception as e:  # noqa: BLE001
            self.error = "ROS 2 sondası hata verdi: %s" % e
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass


def _summarize(topic, desc, st, window_s):
    """Bir topic için (durum, detay) üret."""
    n = st["n"]
    hz = 0.0
    if n > 1 and st["first"] is not None and st["last"] is not None:
        span = st["last"] - st["first"]
        hz = (n - 1) / span if span > 0 else 0.0
    parts = ["%d mesaj" % n]
    if hz > 0:
        parts.append("%.1f Hz" % hz)
    if st["bytes"]:
        parts.append("%.1f kB" % (st["bytes"] / 1024.0))
    if st["vals"]:
        v = st["vals"]
        parts.append("min=%.3f ort=%.3f max=%.3f" % (min(v), sum(v) / len(v), max(v)))
    return n, hz, " · ".join(parts)


def evaluate_ros(probe, window_s, sensors_mode="auto", only_keys=None):
    results = []

    if not probe.available:
        results.append(Result("B/ROS2", "ROS 2 ortamı", SKIP, 0.0,
                              probe.error or "kullanılamıyor"))
        for topic, _t, cat, _c, desc, _f in TOPICS:
            results.append(Result("%s/ROS2" % cat, "%s — %s" % (topic, desc),
                                  SKIP, 0.0, "ROS 2 yok"))
        return results

    results.append(Result("B/ROS2", "ROS 2 ortamı", PASS, 0.0,
                          "rclpy hazır · graftaki topic sayısı: %d · düğüm: %d"
                          % (len(probe.graph), len(probe.nodes))))

    # --- Düğümler ---
    for name, critical, pkg in EXPECTED_NODES:
        found = any(name == n or n.endswith("/" + name) for n in probe.nodes)
        results.append(Result(
            "B/ROS2", "Düğüm '%s'" % name,
            PASS if found else (FAIL if critical else WARN), 0.0,
            ("çalışıyor" if found else "graf'ta yok") + " · %s" % pkg))

    # --- Topic'ler ---
    #   Kategori B = haberleşme/durum (FAIL kapısı). Kategori C = sensör:
    #   bu aşamada sensörler takılı olmayabilir → auto modda yokluk WARN (FAIL değil).
    for topic, type_str, cat, critical, desc, _f in TOPICS:
        is_sensor = (cat == "C")
        label = "%s/ROS2" % cat
        name = "%s — %s" % (topic, desc)

        # Seçili sensör kümesi varsa: bu topic'in grubu seçilmediyse atla.
        if is_sensor and only_keys is not None:
            if TOPIC_KEY.get(topic) not in only_keys:
                results.append(Result(label, name, SKIP, 0.0,
                                      "seçilmedi — takılı değil kabul (bu turda test yok)"))
                continue
        elif is_sensor and sensors_mode == "skip":
            results.append(Result(label, name, SKIP, 0.0,
                                  "sensör testi atlandı (--sensors skip / --comms-only)"))
            continue

        # Seçilen (veya require) sensörler için yokluk = FAIL/WARN kapısı.
        sensor_require = (only_keys is not None) or (sensors_mode == "require")

        st = probe.stats.get(topic, {"n": 0, "vals": [], "bytes": 0,
                                     "first": None, "last": None, "err": ""})
        n, hz, detail = _summarize(topic, desc, st, window_s)
        in_graph = topic in probe.graph
        graph_types = probe.graph.get(topic, [])

        if st.get("err"):
            results.append(Result(label, name, WARN, window_s, st["err"]))
            continue

        # /mav/link_ok: mesaj gelmesi yetmez, DEĞERİ True olmalı (FC heartbeat canlı).
        if topic == "/mav/link_ok":
            if n > 0:
                up = bool(st["vals"]) and st["vals"][-1] >= 0.5
                results.append(Result(
                    label, name, PASS if up else FAIL, window_s,
                    "FC HEARTBEAT canlı — link UP" if up else
                    "topic akıyor ama link_ok=False — FC HEARTBEAT YOK "
                    "(FC bağlı değil / beslemesiz / mavlink-router ayakta değil)"))
            else:
                results.append(Result(
                    label, name, FAIL, window_s,
                    "link_ok hiç yayınlanmıyor (mav_bridge / köprü çalışmıyor)"))
            continue

        if n > 0:
            # Akış var — min/ort/max önizlemesiyle PASS (asıl 'kaydet' testi sonra).
            results.append(Result(label, name, PASS, window_s, detail,
                                  state=STATE_FLOW if is_sensor else ""))
            continue

        # Veri yok — sebebi kategoriye göre yorumla.
        extra = ""
        if in_graph and graph_types and type_str not in graph_types:
            extra = " · TİP UYUŞMUYOR: graf=%s beklenen=%s" % (
                ",".join(graph_types), type_str)

        state = ""
        if is_sensor:
            # Üç insan-okur durum: bağlantı yok / var-ama-veri-yok / (akış üstte)
            state = STATE_NODATA if in_graph else STATE_NOCONN
            if sensor_require:
                # Sensör takılı kabul edildi → veri yoksa gerçek arıza.
                status = FAIL if critical else WARN
                msg = ("düğüm yayında ama %.0f sn'de veri gelmedi (TAKILI kabul edildi)"
                       % window_s if in_graph else
                       "topic graf'ta yok — düğüm kapalı / sensör bağlı değil") + extra
            else:  # auto — sensör takılı olmayabilir, bu aşamada beklenen
                status = WARN
                msg = (("düğüm yayında, sensör verisi yok — takılı değil "
                        "(bu aşamada beklenen)") if in_graph else
                       ("topic graf'ta yok — sensör/düğüm kapalı "
                        "(bu aşamada beklenen)")) + extra
        else:
            status = FAIL if critical else WARN
            msg = ("topic kayıtlı ama %.0f sn'de veri gelmedi%s" % (window_s, extra)
                   if in_graph else
                   "topic graf'ta YOK (düğüm çalışmıyor olabilir)")
        results.append(Result(label, name, status, window_s, msg, state=state))

    # --- MAVROS uyumluluk kontrolü ---
    for topic in MAVROS_COMPAT_TOPICS:
        present = topic in probe.graph
        results.append(Result(
            "B/ROS2", "%s (MAVROS uyumluluk)" % topic,
            PASS if present else WARN, 0.0,
            "yayında" if present else
            "yok — BEKLENEN: bu depo MAVROS kullanmaz, pymavlink köprüsü /mav/... yayınlar "
            "(ros2_ws/src/auv_mav_bridge/package.xml:6)"))

    # --- GÜVENLİK: motor/arm topic'lerinde beklenmedik yayıncı var mı? ---
    for topic in FORBIDDEN_TOPICS:
        cnt = probe.forbidden_pub.get(topic, -1)
        if cnt < 0:
            results.append(Result("E/Güvenlik", "Yayıncı sayımı %s" % topic, SKIP, 0.0,
                                  "sayılamadı"))
        elif cnt == 0:
            results.append(Result("E/Güvenlik", "Yayıncı sayımı %s" % topic, PASS, 0.0,
                                  "0 yayıncı (aktüatör komutu üretilmiyor)"))
        else:
            results.append(Result(
                "E/Güvenlik", "Yayıncı sayımı %s" % topic, WARN, 0.0,
                "%d yayıncı var — normalde yalnız 'mission' düğümüdür; "
                "beklenmiyorsa araç yanında kimse olmadığından emin ol" % cnt))
    return results


# ===========================================================================
# C) ROS DIŞI SENSÖR YEDEK YOLLARI (salt-okunur)
# ===========================================================================
def test_i2c_bar30():
    """Bar30 (MS5837) I2C 0x76 varlık taraması — yalnız algılama, yazma yok."""
    t0 = time.monotonic()
    if IS_WINDOWS:
        return Result("C/Sensör", "Bar30 I2C yedek yolu (0x76)", SKIP,
                      time.monotonic() - t0, "Linux dışı makine")
    buses = [p for p in ("/dev/i2c-%d" % i for i in range(0, 9)) if os.path.exists(p)]
    if not buses:
        return Result("C/Sensör", "Bar30 I2C yedek yolu (0x76)", SKIP,
                      time.monotonic() - t0, "I2C veri yolu yok")
    try:
        from smbus2 import SMBus
    except Exception:  # noqa: BLE001
        return Result("C/Sensör", "Bar30 I2C yedek yolu (0x76)", SKIP,
                      time.monotonic() - t0,
                      "smbus2 kurulu değil (pip install smbus2) · veri yolları: %s"
                      % ", ".join(buses))
    for path in buses:
        idx = int(path.rsplit("-", 1)[1])
        try:
            with SMBus(idx) as bus:
                bus.read_byte(0x76)      # salt-okunur varlık taraması
            return Result("C/Sensör", "Bar30 I2C yedek yolu (0x76)", PASS,
                          time.monotonic() - t0,
                          "MS5837 %s üzerinde yanıt veriyor" % path,
                          state=STATE_FLOW)
        except Exception:  # noqa: BLE001
            continue
    return Result("C/Sensör", "Bar30 I2C yedek yolu (0x76)", WARN,
                  time.monotonic() - t0,
                  "0x76 hiçbir veri yolunda yanıt vermedi (%s) — Bar30 FC'ye bağlı "
                  "olabilir, bu normaldir" % ", ".join(buses),
                  state=STATE_NOCONN)


def test_bridge_link_status():
    """Köprü /sensors üzerinden FC + Mini ROV MAVLink link bayraklarını oku (salt-okuma).

    Bu, ROS 2 kurulu OLMAYAN kontrol PC'sinden bile FC↔Jetson MAVLink linkini
    doğrulamayı sağlar (köprü zaten /mav/link_ok'i özetliyor).
    """
    host, port = "192.168.2.135", 8000
    src = "jetson/ros2_web_bridge.py (/sensors)"
    ok, code, body = check_http_get(host, port, "/sensors", 3.0)
    out = []
    if not ok:
        return out  # /sensors erişilemezliği zaten B/Servis'te raporlanıyor
    try:
        d = json.loads(body.decode("utf-8", "replace"))
    except Exception:  # noqa: BLE001
        return out
    fc = d.get("link_ok")
    out.append(Result(
        "B/ROS2", "FC MAVLink linki (/sensors.link_ok)",
        PASS if fc else FAIL, 0.0,
        "link_ok=%s · köprü connected=%s" % (fc, d.get("connected")), src))
    ml = d.get("mrov_link")
    out.append(Result(
        "B/Servis", "Mini ROV MAVLink linki (/sensors.mrov_link)",
        PASS if ml else WARN, 0.0, "mrov_link=%s" % ml, src))
    return out


def test_minirov_telemetry():
    """Mini ROV telemetrisi — mavlink2rest üzerinden GET (POST YOK)."""
    t0 = time.monotonic()
    ok, code, body = check_http_get("192.168.2.2", 6040, "/mavlink", timeout_s=3.0)
    dt = time.monotonic() - t0
    src = "jetson/ros2_web_bridge.py:761"
    if not ok:
        return [Result("C/Sensör", "Mini ROV mavlink2rest telemetrisi", WARN, dt,
                       "erişilemedi: %s" % body.decode("utf-8", "replace")[:80], src,
                       state=STATE_NOCONN)]
    try:
        data = json.loads(body.decode("utf-8", "replace"))
    except Exception as e:  # noqa: BLE001
        return [Result("C/Sensör", "Mini ROV mavlink2rest telemetrisi", WARN, dt,
                       "HTTP %d ama JSON çözülemedi: %s" % (code, e), src,
                       state=STATE_NODATA)]

    found = []

    def walk(obj, depth=0):
        if depth > 6 or not isinstance(obj, dict):
            return
        for k, v in obj.items():
            if k in ("SCALED_PRESSURE2", "SCALED_PRESSURE3", "SYS_STATUS",
                     "HEARTBEAT", "STATUSTEXT", "GPS_RAW_INT"):
                found.append(k)
            walk(v, depth + 1)

    walk(data)
    detail = "HTTP %d · bulunan mesajlar: %s" % (
        code, ", ".join(sorted(set(found))) or "yok")
    return [Result("C/Sensör", "Mini ROV mavlink2rest telemetrisi",
                   PASS if found else WARN, dt, detail, src,
                   state=STATE_FLOW if found else STATE_NODATA)]


def test_direct_mode_marker():
    t0 = time.monotonic()
    if os.path.exists(DIRECT_MODE_MARKER):
        return Result("E/Güvenlik", "direct_output kurtarma işareti", WARN,
                      time.monotonic() - t0,
                      "%s VAR — önceki oturum doğrudan çıkış modunda çökmüş olabilir; "
                      "SERVOx_FUNCTION değerlerini QGC'den doğrula"
                      % DIRECT_MODE_MARKER,
                      "ros2_ws/src/auv_mav_bridge/auv_mav_bridge/bridge_node.py:162")
    return Result("E/Güvenlik", "direct_output kurtarma işareti", PASS,
                  time.monotonic() - t0, "işaret dosyası yok (temiz durum)")


# ===========================================================================
# Raporlama
# ===========================================================================
# Haberleşme (FAIL kapısı) grupları · sensör (bu aşamada bilgilendirici) grupları
COMM_GROUPS = ("A/Ağ", "B/Servis", "B/ROS2")
SENSOR_GROUPS = ("C/ROS2", "C/Sensör")

# Özet tablosunda kart-kart gösterilecek "board"lar (sensör değil, fiziksel kart)
SUMMARY_BOARDS = [
    ("192.168.2.135", "Jetson Orin NX (AUV)"),
    ("192.168.2.2",   "Mini ROV (RPi4+Navigator)"),
]

GROUP_TITLES = [
    ("A/Ağ",        "A) FİBER / ETHERNET TOPOLOJİSİ"),
    ("B/Servis",    "B) SERVİS UÇLARI (HTTP/WS köprüleri)"),
    ("B/ROS2",      "B) ROS 2 HABERLEŞMESİ"),
    ("C/ROS2",      "C) SENSÖR VERİ AKIŞI (ROS 2)"),
    ("C/Sensör",    "C) SENSÖR VERİ AKIŞI (ROS dışı yollar)"),
    ("E/Güvenlik",  "E) GÜVENLİK DENETİMİ (aktüatör dokunulmadı)"),
]


def _find(results, sub):
    for r in results:
        if sub in r.name:
            return r
    return None


def _find_in_group(results, group, sub):
    for r in results:
        if r.group == group and sub in r.name:
            return r
    return None


# Statik topoloji — üst bilgi bloğunda gösterilir (repodaki konfiglerden)
NET_INFO_LINES = [
    "Ağ         : 192.168.2.0/24 (düz L2, DHCP yok, gateway yok)",
    "Jetson AUV : 192.168.2.135  (enP8p1s0, statik)  · HTTP 8000 · WS 8765 · UI 8080",
    "Mini ROV   : 192.168.2.2    (RPi4+Navigator/BlueOS) · mavlink2rest 6040 · WebRTC 6021",
    "Kontrol PC : 192.168.2.1    (QGC 14550, panel 14552)",
    "FC (CUAV)  : USB seri /dev/ttyFC → mavlink-router → ROS köprüsü (udp 14551)",
]


def print_comms_verdict(log, results):
    """ÜST BLOK: statik IP/genel bilgi + kart-kart haberleşme + hata varsa işaretle."""
    comm_fail = [r for r in results
                 if r.group in COMM_GROUPS and r.status == FAIL]
    ros_skipped = any(r.name == "ROS 2 ortamı" and r.status == SKIP for r in results)

    log.line("")
    log.line(C.BOLD + "╔" + "═" * 72 + C.RESET)
    log.line(C.BOLD + "║  HABERLEŞME DURUMU  (statik ağ + kart erişilebilirliği)" + C.RESET)
    log.line(C.BOLD + "╚" + "═" * 72 + C.RESET)

    log.line(C.GREY + "  ── Genel bilgi ──" + C.RESET)
    for ln in NET_INFO_LINES:
        log.line(C.GREY + "  " + ln + C.RESET)
    log.line("")

    def row(label, r, fallback="test yok"):
        st = r.status if r else SKIP
        det = r.detail if r else fallback
        log.line("  %s  %-30s %s%s%s" % (badge(st), label[:30], C.GREY, det[:42], C.RESET))

    for ip, label in SUMMARY_BOARDS:
        # Kart erişilebilirliği = ping sonucu (A/Ağ grubu), servis değil.
        row("%s (%s)" % (label.split(" (")[0], ip),
            _find_in_group(results, "A/Ağ", ip))
    # FC MAVLink linki: ROS varsa /mav/link_ok, yoksa köprü /sensors.link_ok, o da
    # yoksa seri port varlığı.
    fc = _find(results, "/mav/link_ok")
    if fc and fc.status != SKIP:
        row("FC MAVLink linki", fc)
    else:
        fc2 = _find(results, "link_ok)")  # /sensors.link_ok türevi
        if fc2:
            row("FC MAVLink linki", fc2)
        else:
            row("CUAV V6X seri portu", _find(results, "seri portu"))
    row("ROS köprüsü (8000)", _find(results, "/sensors"))
    row("Görev WS (8765)", _find(results, "WebSocket"))
    row("ui/ panel (8080)", _find(results, "FastAPI"))

    log.line("")
    if comm_fail:
        log.line(C.RED + C.BOLD + "  ✖ HABERLEŞME HATASI — %d kritik bağlantı yok:"
                 % len(comm_fail) + C.RESET)
        for r in comm_fail:
            log.line(C.RED + "     • %s — %s" % (r.name.strip()[:40],
                     r.detail[:44]) + C.RESET)
    else:
        log.line(C.GREEN + C.BOLD
                 + "  ✔ TÜM KRİTİK KARTLARLA HABERLEŞME VAR" + C.RESET)
    if ros_skipped:
        log.line(C.YELLOW + "  ⚠ NOT: ROS 2 topic'leri bu makineden dinlenemedi (rclpy yok); "
                            "FC-link köprü /sensors'tan okundu. Topic-seviyesi doğrulama "
                            "için Jetson'da çalıştır." + C.RESET)


# state -> işaret (renk çalışma anında seçilir; --no-color ile doğru sıfırlansın diye)
STATE_MARK = {STATE_FLOW: "●", STATE_NODATA: "◐", STATE_NOCONN: "○"}


def _state_color(state):
    return {STATE_FLOW: C.GREEN, STATE_NODATA: C.YELLOW,
            STATE_NOCONN: C.RED}.get(state, C.GREY)


def print_sensor_verdict(log, results):
    """ALT BLOK: her sensör adı + insan-okur veri-akış durumu (3 hal)."""
    sres = [r for r in results if r.group in SENSOR_GROUPS]
    if not sres:
        return
    tested = [r for r in sres if r.state]  # state atanmış = gerçekten değerlendirilmiş

    log.line("")
    log.line(C.BOLD + "╔" + "═" * 72 + C.RESET)
    log.line(C.BOLD + "║  SENSÖRLER  (ad + veri akış durumu)" + C.RESET)
    log.line(C.BOLD + "╚" + "═" * 72 + C.RESET)
    log.line(C.GREY + "  ● BAĞLI·VERİ AKIYOR   ◐ BAĞLANTI VAR AMA VERİ GELMİYOR   "
             "○ BAĞLANTI YOK" + C.RESET)
    log.line("")

    if not tested:
        log.line(C.GREY + "  Bu turda sensör test edilmedi "
                          "(--comms-only / seçim yapılmadı)." + C.RESET)
        return

    for r in tested:
        friendly = r.name.split(" — ")[-1] if " — " in r.name else r.name
        color = _state_color(r.state)
        mark = STATE_MARK.get(r.state, "·")
        reading = r.detail if r.state == STATE_FLOW else ""
        log.line("  %s%s %-34s%s %s%-32s%s %s%s%s" % (
            color, mark, friendly[:34], C.RESET,
            color, r.state, C.RESET,
            C.GREY, reading[:40], C.RESET))

    flow = sum(1 for r in tested if r.state == STATE_FLOW)
    nod = sum(1 for r in tested if r.state == STATE_NODATA)
    noc = sum(1 for r in tested if r.state == STATE_NOCONN)
    log.line("")
    log.line("  Toplam: %s%d akıyor%s · %s%d bağlı-veri yok%s · %s%d bağlantı yok%s"
             % (C.GREEN, flow, C.RESET, C.YELLOW, nod, C.RESET, C.RED, noc, C.RESET))


def print_report(log, results, total_s, args, log_path):
    for key, title in GROUP_TITLES:
        group = [r for r in results if r.group == key]
        if not group:
            continue
        log.line("")
        log.line(C.BOLD + C.BLUE + "── " + title + " " + "─" * max(0, 62 - len(title))
                 + C.RESET)
        for r in group:
            log.line("  %s %-58s %s%6.2fs%s" % (
                badge(r.status), r.name[:58], C.GREY, r.duration, C.RESET))
            if r.detail:
                log.line("         %s%s%s" % (C.GREY, r.detail, C.RESET))
            if r.source:
                log.file_only("         kaynak: %s" % r.source)

    counts = {PASS: 0, FAIL: 0, WARN: 0, SKIP: 0}
    for r in results:
        counts[r.status] += 1
    total = len(results)

    log.line("")
    log.line(C.BOLD + "═" * 74 + C.RESET)
    log.line(C.BOLD + "  ÖZET" + C.RESET)
    log.line("  " + "─" * 70)
    log.line("  %sPASS%s : %3d      %sFAIL%s : %3d      %sWARN%s : %3d      %sSKIP%s : %3d"
             % (C.GREEN, C.RESET, counts[PASS],
                C.RED, C.RESET, counts[FAIL],
                C.YELLOW, C.RESET, counts[WARN],
                C.GREY, C.RESET, counts[SKIP]))
    log.line("  Toplam test  : %d" % total)
    log.line("  Toplam süre  : %.2f sn  (bütçe %.0f sn)" % (total_s, GLOBAL_BUDGET_S))
    log.line("  Log dosyası  : %s" % log_path)
    log.line(C.BOLD + "═" * 74 + C.RESET)

    print_comms_verdict(log, results)
    print_sensor_verdict(log, results)

    if counts[FAIL]:
        log.line("")
        log.line(C.RED + C.BOLD + "  BAŞARISIZ TESTLER:" + C.RESET)
        for r in results:
            if r.status == FAIL:
                log.line("   • %s — %s" % (r.name.strip(), r.detail))
    log.line("")
    log.line(C.GREY + "  Not: Bu çalıştırmada hiçbir motor komutu, arm komutu veya "
                      "MAVLink yazması yapılmadı." + C.RESET)
    return counts


# ===========================================================================
# Sensör seçimi (interaktif / --only / --comms-only)
# ===========================================================================
def _parse_keys(raw):
    keys = set()
    for tok in raw.replace(";", ",").split(","):
        tok = tok.strip().lower()
        if not tok:
            continue
        if tok in ("all", "hepsi", "*"):
            keys |= set(SENSOR_KEYS)
        elif tok in SENSOR_KEYS:
            keys.add(tok)
        else:
            print("  ! bilinmeyen sensör anahtarı yok sayıldı: %s "
                  "(geçerli: %s)" % (tok, ", ".join(SENSOR_KEYS)))
    return keys


def _prompt_sensors():
    """Terminalde 'hangi sensörler takılı?' diye sorar. (mode, only_keys) döndürür."""
    if not sys.stdin.isatty():
        print("  ! --interactive terminal (tty) gerektirir. Sensör anahtarını "
              "komut satırından ver: --only depth,sonar,zed  ·  şimdilik atlanıyor.")
        return "skip", set()
    print("")
    print("Takılı sensörleri seç — SADECE bunlar test edilir (veri yoksa FAIL).")
    print("Diğerleri 'takılı değil' kabul edilip atlanır.")
    for k, label in SENSOR_MENU:
        print("   %-11s %s" % (k, label))
    print("Virgülle gir (ör: depth,sonar,zed) · 'hepsi' · boş = hiçbiri")
    try:
        raw = input("Takılı sensörler > ").strip()
    except (EOFError, KeyboardInterrupt):
        print("")
        return "skip", set()
    if not raw:
        return "skip", set()
    return "require", _parse_keys(raw)


def resolve_sensor_selection(args):
    """(sensors_mode, only_keys) döndürür.

    only_keys None → sensors_mode (auto/require/skip) TÜM sensörlere uygulanır.
    only_keys bir küme → yalnız o anahtarlar 'require' ile test edilir, gerisi SKIP.
    """
    if args.comms_only:
        return "skip", set()
    if args.only:
        return "require", _parse_keys(args.only)
    if args.interactive:
        return _prompt_sensors()
    return args.sensors, None


# ===========================================================================
# main
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(
        description="AUV + Mini ROV sistem sağlık testi (salt-okunur)")
    ap.add_argument("--sample", type=float, default=10.0,
                    help="Sensör örnekleme penceresi, saniye (varsayılan 10)")
    ap.add_argument("--ping-timeout", type=float, default=2.0,
                    help="Ping zaman aşımı, saniye (varsayılan 2)")
    ap.add_argument("--no-ros", action="store_true",
                    help="ROS 2 testlerini atla (sadece ağ + servis)")
    ap.add_argument("--no-net", action="store_true",
                    help="Ağ/ping testlerini atla")
    ap.add_argument("--sensors", choices=["auto", "require", "skip"], default="auto",
                    help="Sensör kanalları: auto=veri yoksa WARN 'takılı değil' "
                         "(VARSAYILAN, bu aşamada doğru) · require=veri yoksa FAIL "
                         "(sensörler takılıyken) · skip=hiç test etme")
    ap.add_argument("--comms-only", action="store_true",
                    help="Kısayol: sadece haberleşme/topoloji (sensörleri tamamen atla)")
    ap.add_argument("--only", metavar="ANAHTAR[,...]", default=None,
                    help="Sadece bu sensörleri test et (takılı kabul → veri yoksa FAIL). "
                         "Anahtarlar: %s · 'hepsi'" % ",".join(SENSOR_KEYS))
    ap.add_argument("-i", "--interactive", action="store_true",
                    help="Başlarken 'hangi sensörler takılı?' diye sor; sadece "
                         "seçilenleri test et. Terminal gerektirir.")
    ap.add_argument("--extra-ip", action="append", default=[],
                    metavar="IP[:ROL]",
                    help="Listeye ek IP ekle (ör. 192.168.2.10:Pixhawk-ETH). "
                         "Tekrarlanabilir.")
    ap.add_argument("--json", metavar="DOSYA", default=None,
                    help="Sonuçları JSON olarak da yaz")
    ap.add_argument("--no-color", action="store_true", help="Renkleri kapat")
    args = ap.parse_args()

    _setup_console()
    if args.no_color:
        _strip_colors()
    else:
        _enable_colors()

    if args.sample > 30:
        args.sample = 30.0  # 60 sn bütçesini koru

    sensors_mode, only_keys = resolve_sensor_selection(args)

    def sensor_selected(key):
        """Bu sensör grubu bu turda test edilecek mi?"""
        if only_keys is not None:
            return key in only_keys
        return sensors_mode != "skip"

    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(LOG_DIR, "health_check_%s.log" % stamp)
    log = Logger(log_path)

    t_start = time.monotonic()
    log.line("")
    log.line(C.BOLD + "AUV + Mini ROV — SİSTEM SAĞLIK TESTİ" + C.RESET)
    log.line(C.GREY + "  Zaman   : %s" % datetime.now().isoformat(timespec="seconds")
             + C.RESET)
    log.line(C.GREY + "  Makine  : %s %s / Python %s"
             % (platform.system(), platform.release(),
                platform.python_version()) + C.RESET)
    log.line(C.GREY + "  Depo    : %s" % REPO_ROOT + C.RESET)
    log.line(C.GREY + "  Mod     : SALT-OKUNUR — motor/arm/MAVLink yazması YOK" + C.RESET)
    if only_keys is not None:
        _sensmsg = "seçili [%s] (takılı kabul → veri yoksa FAIL)" % (
            ", ".join(sorted(only_keys)) or "hiçbiri")
    else:
        _sensmsg = {"auto": "auto (veri yoksa 'takılı değil' = WARN)",
                    "require": "require (veri yoksa FAIL)",
                    "skip": "skip (sensörler atlandı)"}[sensors_mode]
    log.line(C.GREY + "  Odak    : HABERLEŞME/TOPOLOJİ kritik · sensör modu: %s"
             % _sensmsg + C.RESET)
    log.line(C.GREY + "  Pencere : örnekleme %.0f sn · ping timeout %.0f sn"
             % (args.sample, args.ping_timeout) + C.RESET)

    hosts = list(HOSTS)
    for spec in args.extra_ip:
        if ":" in spec:
            ip, role = spec.split(":", 1)
        else:
            ip, role = spec, "ek node (--extra-ip)"
        hosts.append((ip.strip(), role.strip(), False, "--extra-ip"))

    results = []
    lock = threading.Lock()

    def collect(items):
        with lock:
            if isinstance(items, list):
                results.extend(items)
            else:
                results.append(items)

    # --- Paralel iş parçacıkları: ağ, servisler, ROS sondası aynı anda koşar ---
    probe = RosProbe(args.sample, log) if not args.no_ros else None
    threads = []

    def t_net():
        if args.no_net:
            return
        collect(test_hosts(hosts, args.ping_timeout))
        collect(test_interfaces())
        collect(test_fc_serial())

    def t_svc():
        collect(test_services(SERVICES))
        collect(test_bridge_link_status())
        if sensor_selected("mrov"):
            collect(test_minirov_telemetry())

    def t_ros():
        probe.run()

    def t_misc():
        if sensor_selected("depth"):
            collect(test_i2c_bar30())
        collect(test_direct_mode_marker())

    for fn in (t_net, t_svc, t_misc):
        th = threading.Thread(target=fn, daemon=True)
        th.start()
        threads.append(th)
    if probe is not None:
        th = threading.Thread(target=t_ros, daemon=True)
        th.start()
        threads.append(th)

    deadline = t_start + GLOBAL_BUDGET_S
    for th in threads:
        remaining = max(0.1, deadline - time.monotonic())
        th.join(timeout=remaining)
    stragglers = [th for th in threads if th.is_alive()]
    if stragglers:
        collect(Result("E/Güvenlik", "Zaman bütçesi", WARN,
                       time.monotonic() - t_start,
                       "%d test iş parçacığı %.0f sn içinde bitmedi, sonuçları eksik"
                       % (len(stragglers), GLOBAL_BUDGET_S)))

    if probe is not None:
        collect(evaluate_ros(probe, args.sample, sensors_mode, only_keys))
    else:
        collect(Result("B/ROS2", "ROS 2 testleri", SKIP, 0.0, "--no-ros ile atlandı"))

    total_s = time.monotonic() - t_start
    counts = print_report(log, results, total_s, args, log_path)

    if args.json:
        payload = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "host": platform.node(),
            "total_duration_s": round(total_s, 3),
            "counts": counts,
            "results": [r.as_dict() for r in results],
        }
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        log.line(C.GREY + "  JSON raporu: %s" % args.json + C.RESET)

    log.close()
    return 1 if counts[FAIL] else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nKullanıcı iptal etti.")
        sys.exit(130)
