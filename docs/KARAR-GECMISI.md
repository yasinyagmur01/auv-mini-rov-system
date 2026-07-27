# Karar Geçmişi

Dokümanlar/kod arasında tespit edilen çelişkilerin **tek doğruluk kaynağı**. Bir konuda
şüphe olursa önce buraya bak. Her satır: hangi bilgiyi doğru kabul ettik, neden, hangi
kanıta dayanarak. Yeni bir karar çıktığında en üste ekle (en yeni üstte).

> Not: Bu dosya "neyin doğru olduğunu" söyler; "nasıl yapılır" bilgisi ilgili dokümanda kalır.

---

## 2026-07-27 — CUAV V6X USB→Ethernet geçişi (TAMAMLANDI)

| Alan | İçerik |
|---|---|
| **Konu** | AUV FC (CUAV V6X) ile Jetson arasındaki MAVLink bağlantısı |
| **Eski durum** | USB seri (`/dev/ttyFC`, udev symlink) → mavlink-router. Titrek: USB kopmaları. |
| **Kök neden** | (1) Regülatör yandı → FC'ye güç yok (tamir edildi). (2) FMU USB'si araç hub'ı üzerinden Jetson'a hiç ulaşmıyordu — o portta aslında u-blox GPS vardı. FMU doğrudan Jetson'a takılınca `1209:5740 CUAV-V6X-v2` enumerate oldu, `/dev/ttyFC→ttyACM1`. |
| **Yeni/doğru** | **FC ethernet ile bağlı: `192.168.2.20:14550`** (NET_ENABLE=1, NET_DHCP=0, NET_IPADDR=192.168.2.20, NET_NETMASK=24, NET_P1_TYPE=2 UDP Server, PORT=14550, PROTOCOL=MAVLink2). Bu ayarlar FC'de zaten mevcuttu (`.10` değil `.20`). mavlink-router `main-ethernet.conf` (UDP .20:14550) ile bağlanıyor. |
| **Doğrulama** | 2026-07-27: USB çekiliyken `link_ok=True` (3/3), `.20` ping 0.15ms, sağlık testi "TÜM KRİTİK KARTLARLA HABERLEŞME VAR". |
| **Not** | Şablon `.10` varsayıyordu → gerçeğe (`.20`) göre güncellendi. USB (`/dev/ttyFC`, `main.conf`) acil yedek olarak durur. systemd override `ttyFC bekle`→`.20 ping bekle`. |
| **Kaynak** | `config/mavlink-router/main-ethernet.conf` · `config/mavlink-router/override-ethernet.conf` · canlı FC NET_ params |

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

### 7) Mini ROV NAV_SYSID — 1 mi, 2 mi? **(ÇÖZÜLDÜ 2026-07-27 — koşullu)**
| Alan | İçerik |
|---|---|
| **Konu** | Köprünün mavlink2rest'te sorguladığı Mini ROV sysid |
| **Donanım teyidi** | 2026-07-27, Mini ROV canlı: mavlink2rest `/v1/mavlink` ağacında araç **sysid=1** ilan ediyor (HEARTBEAT: MAV_TYPE_SUBMARINE, ArduPilotMega). sysid=2 boş. |
| **Doğru bilgi (şimdilik)** | **`NAV_SYSID=1` şu an DOĞRU.** Neden: `minirov_navigator.param` (SYSID_THISMAV=2) Navigator'a **henüz yüklenmedi** → ArduSub varsayılanı 1'de kalıyor. |
| **⚠ Çakışma riski** | AUV de sysid=1 (`auv_v6x_baseline.param`). İki araç **aynı anda** açılınca QGC/mavlink'te **çakışırlar**. Mini ROV'un 2 olmasının sebebi buydu. |
| **Kalıcı çözüm (takip)** | Mini ROV'a `minirov_navigator.param` yüklenip SYSID_THISMAV=2 yapılınca `NAV_SYSID`'i **2** yap; iki araç birlikte açıkken çakışmasız çalıştığını doğrula. `docs/AKSIYON-PLANI.md`'de takip. |
| **Karar** | Kod değeri 1'de bırakıldı (canlıyla uyumlu); yorum bloğu bulgu + çakışma bağımlılığıyla güncellendi. |
| **Kaynak** | canlı mavlink2rest `192.168.2.2:6040/v1/mavlink` · `ardusub_params/minirov_navigator.param:47` · `jetson/ros2_web_bridge.py:74-77` |
