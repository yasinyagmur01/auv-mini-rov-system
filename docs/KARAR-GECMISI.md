# Karar Geçmişi

Dokümanlar/kod arasında tespit edilen çelişkilerin **tek doğruluk kaynağı**. Bir konuda
şüphe olursa önce buraya bak. Her satır: hangi bilgiyi doğru kabul ettik, neden, hangi
kanıta dayanarak. Yeni bir karar çıktığında en üste ekle (en yeni üstte).

> Not: Bu dosya "neyin doğru olduğunu" söyler; "nasıl yapılır" bilgisi ilgili dokümanda kalır.

---

## 2026-07-26 — Analiz sonrası 7 çelişki çözümü

### 1) Motor testi — web'den mi, yalnız QGC'den mi?
| Alan | İçerik |
|---|---|
| **Konu** | AUV motor testinin hangi yoldan yapıldığı |
| **Eski bilgi** | `docs/tezgah-testi.md` §C: "Bu firmware'de motor testi = QGC; ham `MAV_CMD_DO_MOTOR_TEST` reddediliyor (bad test type 0.00)". Özel `panel/motor_test.py` çalışmıyor. |
| **Doğru bilgi** | **Web arayüzünden çalışıyor.** "bad test type" firmware bug'ı değilmiş: ArduSub test type'ı param6 (`MOTOR_TEST_ORDER`) alanından okuyor ve **2 (BOARD)** bekliyor. Çözüm: `y=2` + komutu **20 Hz** tekrarla (500 ms watchdog) + **her testten önce ARM** (ArduSub test bitince oto-disarm). QGC şart değil. |
| **Gerekçe** | Canlı `COMMAND_ACK` + `/mav/servo_out` ile doğrulandı; kod bunu uyguluyor. tezgah-testi.md bu bilgiyle güncellenmemişti (daha eski). |
| **Kaynak** | `ros2_ws/src/auv_mav_bridge/auv_mav_bridge/bridge_node.py` (motor_test handler, ~satır 272-304) · `docs/DURUM-DEVAM.md` (10 Tem açıklaması) |

### 2) `FS_LEAK_ENABLE` — 2 mi, 1 mi?
| Alan | İçerik |
|---|---|
| **Konu** | Sızıntı failsafe parametresi değeri |
| **Eski bilgi** | Çelişki gibi görünüyordu: baseline `=2` (yüzeye çık), saha okuması `=1` (yalnız uyar). |
| **Doğru bilgi** | **Çelişki değil — hedef ↔ canlı farkı.** Hedef değer `2` (baseline'da). FC'de 2026-07-16'da `=1` okundu çünkü **baseline param henüz FC'ye yüklenmedi**. Ayrıca `FS_LEAK_ENABLE` tek başına yetmez; `LEAK1_PIN` tanımlı olmalı (FC'de şu an `-1`/kapalı). |
| **Gerekçe** | baseline "yüklenecek hedef"i, saha okuması "o anki canlı durumu" temsil eder; ikisi farklı şeydir. |
| **Kaynak** | `ardusub_params/auv_v6x_baseline.param:41` · `docs/kamera-leak-sicaklik-arac-dogrulama.md` (FC mevcut değerleri, ~satır 133) |

### 3) Kontrol PC IP — .196 mı, .1 mi?
| Alan | İçerik |
|---|---|
| **Konu** | Kontrol PC statik IP |
| **Eski bilgi** | `docs/DURUM-DEVAM.md` (10 Tem): PC = `192.168.2.196` |
| **Doğru bilgi** | **`192.168.2.1`.** BlueOS karayı `.1` varsayar; `mavlink-router` QGC/panel'i `.1`'e push eder. |
| **Gerekçe** | README (07-23, en yeni) + ag-kurulumu + tezgah-testi hepsi `.1` diyor; DURUM-DEVAM bayat. |
| **Kaynak** | `docs/ag-kurulumu.md` (IP tablosu) · `README.md` (07-23 ağ satırı) |

### 4) Ping Sonar portu — SERIAL1/TELEM1 mi, SERIAL2/TELEM2 mi?
| Alan | İçerik |
|---|---|
| **Konu** | Ping sonarın bağlı olduğu seri port |
| **Eski bilgi** | `docs/DURUM-DEVAM.md` (10 Tem): "TELEM1'e takılı; `SERIAL1_PROTOCOL=9`" |
| **Doğru bilgi** | **SERIAL2 / TELEM2** (`SERIAL2_PROTOCOL,9`, 115200). |
| **Gerekçe** | Yüklenen baseline param'ı SERIAL2 kullanıyor (yerleşim gerçeği) ve kablolama.md de TELEM2 diyor; DURUM-DEVAM'daki TELEM1 erken/bayat bilgi. |
| **Kaynak** | `ardusub_params/auv_v6x_baseline.param:15-16` · `docs/kablolama.md:9` |

### 5) `GPS_TYPE` — 1 mi, 5 mi?
| Alan | İçerik |
|---|---|
| **Konu** | Helical FP9 GNSS sürücü tipi |
| **Eski bilgi** | kablolama `GPS_TYPE=1`, ardusub-kurulum `GPS_TYPE=5` (NMEA) — çelişki gibi. |
| **Doğru bilgi** | **Çelişki değil: `1` (AUTO) birincil, kilitlenmezse `5` (NMEA) yedek.** Dokümanlar zaten tutarlı; baseline yorumu bunu açıkça yazıyor. |
| **Gerekçe** | baseline `GPS_TYPE,1 # AUTO; kilitlenmezse 5 (NMEA) deneyin` — iki değer sıralı strateji, alternatif değil. |
| **Kaynak** | `ardusub_params/auv_v6x_baseline.param:24` · `docs/kablolama.md:10` · `docs/ardusub-kurulum.md:35` |

### 6) Web sunucu teknolojisi — saf `http.server` mı, Flask mı?
| Alan | İçerik |
|---|---|
| **Konu** | `jetson/ros2_web_bridge.py` hangi web çatısı |
| **Eski bilgi** | `CLAUDE.md`: "saf Python `http.server`" |
| **Doğru bilgi** | **Flask** (+ rclpy + PIL). |
| **Gerekçe** | Kaynakta `from flask import Flask` ve `app = Flask(__name__)` mevcut. |
| **Kaynak** | `jetson/ros2_web_bridge.py:2, 38, 582` |

### 7) Mini ROV NAV_SYSID — 1 mi, 2 mi? **(AÇIK — donanım teyidi gerek)**
| Alan | İçerik |
|---|---|
| **Konu** | Köprünün mavlink2rest'te sorguladığı Mini ROV sysid |
| **Durum** | **AÇIK / ÇÖZÜLMEDİ.** Param `SYSID_THISMAV=2` diyor; köprü kodu `NAV_SYSID=1` sorguluyor. Kodun kendi yorumu bile "ARAÇTA DOĞRULA" diyor. |
| **Karar** | Fonksiyonel kod **değiştirilmedi** (yanlış değer Mini ROV telemetri/motor rölesini bozabilir). Donanımda `mavlink2rest .../vehicles` uç noktasından aracın ilan ettiği gerçek sysid teyit edilecek, sonra `NAV_SYSID` ona göre ayarlanacak. |
| **Kaynak** | `ardusub_params/minirov_navigator.param:47` (SYSID_THISMAV=2) · `jetson/ros2_web_bridge.py:74-77` (NAV_SYSID=1) |
