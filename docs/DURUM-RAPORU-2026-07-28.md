# Sistem Durum Raporu — 2026-07-28

AUV'nin şartname görev isterlerine karşı **nerede olduğu**, **yapılanlar** ve
**önündeki engeller** (sudaki işler dahil). Kaynak: bu tarihli saha-öncesi doğrulama
oturumu (Faz 0/3/4 + GPS + sensör) ve repo denetimi. Tek doğruluk kaynağı çelişkide
[KARAR-GECMISI.md](KARAR-GECMISI.md); yol haritası [AKSIYON-PLANI.md](AKSIYON-PLANI.md).

---

## BÖLÜM A — YAPILANLAR (mevcut yetenekler)

### A1. Haberleşme & Ağ
- **FC↔Jetson ethernet**: CUAV V6X `192.168.2.20:14550` (UDP server, MAVLink2). USB `/dev/ttyFC`
  acil yedek. `mavlink-router` seri/ethernet'i QGC + ROS köprüsü arasında paylaştırır.
- **Statik L2 ağ** (DHCP/gateway yok): Jetson `.135`, Mini ROV/BlueOS `.2`, Kontrol PC `.1`, FC `.20`.
- **Boot servisleri** aktif+enabled: `mavlink-router`, `auv-stack`, `auv-ui`.
- **Mini ROV entegrasyonu**: mavlink2rest (`.2:6040`), WebRTC (`.2:6021`), BlueOS (`.2:80`) erişilebilir;
  Mini ROV telemetri `/sensors`'a taşınıyor (`mrov_*`).
- ✅ Faz 0'da tam yeşil: FC link_ok, 10 ROS düğümü, tüm köprüler açık.

### A2. Uçuş Kontrol / FC (ArduSub)
- ArduSub 4.7; EKF3 (`AHRS_EKF_TYPE=3`), yatay konum kaynağı GPS (`EK3_SRC1_POSXY=3`).
- Canlı tam param dump: `ardusub_params/auv_v6x_20260727.param`.
- ⚠️ Bilinen açıklar (bkz Bölüm B): `BATT_MONITOR=0` (tezgah), `ARMING_SKIPCHK` çok geniş,
  `MOTn_`/yön parametreleri henüz eşlenmemiş, `SERIAL3/4_PROTOCOL=5` atıl (GPS CAN'de).

### A3. Hareket & Görev Mantığı — **Durum Makinesi (FSM) VAR**
- `ros2_ws/src/auv_mission/mission_node.py` — sonlu durum makinesi:
  **IDLE / MANUAL / VIDEO_PATTERN / LANE_FOLLOW / AUTONOMOUS_NAV / ABORTED**.
- `/mav/manual_control`'un **tek yazarı** bu düğüm (yetki çakışması yok). IDLE/MANUAL'de **sessiz**
  (pilot QGC'den sürer); görev başlayınca komut üretir, STOP her an nötre alır (2 sn güvenli teslim).
- **Kontrol primitifleri** (`primitives.py`): `wait, countdown, arm, disarm, set_mode, set_depth,
  turn, drive, spin, orbit, capture_origin, goto_target, surface, lane_follow` + `HeadingPID`.
- **Görev tanımları** (`config/missions/`): `video_pattern.yaml` (Görev 1), `task2_nav.yaml` (Görev 2),
  `lane_follow.yaml`.
- **Panel WebSocket** `:8765` (`ws_server.py`): start/stop/arm/disarm/set_mode/goto/set_targets/
  motor_test/direct_output komutları.
- ✅ Faz 3'te ARM'sız dry-run ile doğrulandı: eksen yönleri (x+ ileri, z<500 dal, z>500 yüzey,
  r+ sağa) köprü→MAVLink 1:1 eşlemesiyle tutarlı. Araç: `scripts/faz3_mission_dryrun.py`.

### A4. Sensörler (pipeline'lar canlı, veri akıyor)
- **Bar30 (I2C)**: `/mav/depth` 2 Hz + `/mav/water_temp` 2 Hz. Kuru tezgahta derinlik ~0 (doğru).
- **Ping sonar (SERIAL1, protokol 9)**: `/mav/rangefinder` 3 Hz. `sonar_valid` mantığı doğru
  (havada geçersiz işaretliyor). `RNGFND1_ORIENT=25` aşağı bakış.
- **IMU (CUAV içi)**: `/mav/attitude` + `/mav/heading_deg` 10 Hz. Harici IMU yok (tasarım).
- **GPS (u-blox F9P, DroneCAN/CAN1)**: `GPS1_TYPE=9`, `/mav/gps` (NavSatFix) + `/mav/gps_info` 5 Hz.
  `GPS_RAW_INT` 5 Hz garantisi (SET_MESSAGE_INTERVAL, commit `33e562c`). Sahada açık gök fix≥3 +
  uydu doğrulandı (kullanıcı). RTK kurulu ama bilinçli KULLANILMIYOR.
- **P0 güvenlik telemetrisi**: `/mav/battery`, `/mav/statustext`, `/mav/leak` abonelikleri;
  NaN→None güvenlemesi (`_safe_num`).
- ✅ Faz 5'te akış doğrulandı. ⚠️ **board_temp akmıyor** (bkz B — FC config).

### A5. Konumlama & Füzyon
- **Dead reckoning** (`auv_dead_reckoning`): `/dr/state`, hız tablosu (`SpeedTable`), `capture_origin`
  servisi (yüzeyde GPS origin), `geo_local` (kerteriz/mesafe).
- **Dikey durum füzyonu** (`auv_vertical_state`): `/vertical_state` (derinlik + irtifa).

### A6. Web Panel & Güvenlik (P0)
- Sayfalar: kokpit `/`, kontrol `/control`, motor test `/test`, Mini ROV `/minirov`.
- `/sensors` JSON (10 anahtar), tümü null-korumalı (**NaN çıkması yapısal olarak imkânsız**).
- **P0 güvenlik**: leak çipi mandallı ("izleniyor" / "⚠ SIZINTI", asla sahte "kuru"),
  haberleşme-kaybı kenar-tetikli alarm, 90°C sıcaklık alarmı, bayat (stale) göstergesi.
- ✅ Faz 4'te canlı doğrulandı (leak=None→"izleniyor", link/mode/attitude doğru).

### A7. Video / Kamera
- **ZED 2i** → Jetson ROS2 → `/video/zed` MJPEG (Docker sarmalayıcı; şu an oturumda kapalıydı).
- **Mini ROV** → tarayıcı WebRTC → BlueOS `.2:6021`.

### A8. Test & Doğrulama Araçları
- `scripts/health_check.sh` — salt-okunur sistem sağlık testi (haberleşme + sensör modları).
- `scripts/faz3_mission_dryrun.py` — **YENİ**: görev mantığı ARM'sız prova (donanımsız).
- `scripts/calibrate_speed.py` — hız tablosu kalibrasyonu (havuz).
- `scripts/sim_video_pattern.py`, `scripts/gps_web.py`, `scripts/saha_menu.sh`.

### A9. Bu oturumda (2026-07-28) yapılanlar
| Faz | Sonuç |
|---|---|
| **Faz 0** — Sağlık testi | ✅ PASS 31 / FAIL 0; FC link, 10 düğüm, köprüler yeşil |
| **Faz 4** — Web telemetri + P0 | ✅ leak "izleniyor", NaN yok, link/mode/attitude doğru |
| **Faz 3** — Mission mantığı (ARM'sız) | ✅ dry-run doğrulandı; araç repoya eklendi (`276077f`) |
| **ÇATI-GPS** | ✅ config (TYPE=9 DroneCAN) + pipeline (5 Hz) doğrulandı; saha fix'i kullanıcıda |
| **Sensör doğrulama** | ✅ Bar30/sonar/IMU akıyor; board_temp kök nedeni teşhis edildi |

---

## BÖLÜM B — ŞARTNAME İSTERLERİNİN ÖNÜNDEKİ ENGELLER

### Görev 1 — Otonom Video Deseni (şartname 2.4.3.3, İleri Kategori)
*Gerek: 1×1 m kareden başla, ≥15 sn düz → sağa 90° → ≥15 sn düz → kendi etrafında ≥1 tur ≥1 m
çaplı daire → ≥15 sn düz → sağa 90° → ≥15 sn düz, bitişte tamamen kare içinde, kesintisiz su altı.*

**Yazılım mantığı HAZIR** (`video_pattern.yaml` + FSM). Engeller:
1. 🔴 **Motor kanal eşleme + yön** — 8 motor tek tek döndürülüp panel/QGC ↔ fiziksel eşlenmedi,
   ters dönenlere `MOT_x_DIRECTION=-1` yazılmadı. **Bu yapılmadan araç doğru hareket edemez.**
   *(Motor oturumu — motor erişimi gerekir; bu oturumda atlandı.)* → param dump + commit.
2. 🔴 **Derinlik/pruva PID tuning** (suda) — `ATC_*`/`PSC_*` hâlâ ArduSub varsayılanı. Kabul:
   ALT_HOLD ±0.15 m, pruva tutma ±5°. → param dump + commit.
3. 🔴 **Hız tablosu kalibrasyonu** (suda) — `calibrate_speed.py` → `config/speed_table.csv`;
   `video_pattern.yaml` bacak süreleri (16 sn) ve daire çapı (≥1 m) gerçek hızla ayarlanır.
4. 🟠 **Sızdırmazlık testi** + **DC-dereceli E-stop** (fiziksel) — mevcut E-stop AC-tipi, güvenlik açığı.
5. 🟠 **CompassMot** (suda/fiziksel) — motor akımının pusulaya etkisi kalibre edilmeli.
6. 🟡 **`BATT_MONITOR=0`** → geri aç (8), düşük-voltaj alarmı ekle. **`ARMING_SKIPCHK` çok geniş**
   → operasyon öncesi gözden geçir. Batarya bir kez dibe indi (BMS) — düşük voltaj koruması şart.

### Görev 2 — Otonom Navigasyon / İntikal (task2_nav)
*Gerek: yüzeyde GPS fix → ipucu koordinatına git → şamandıra etrafında tur → bitiş koordinatı → yüzey.*

**Yazılım mantığı HAZIR** (`task2_nav.yaml`: capture_origin → goto_target → orbit → goto_target).
Görev 1'in TÜM engelleri + ek:
7. 🔴 **DR doğruluğu** (suda) — hız tablosu + akıntı biası kalibre edilmeli; `orbit` yarıçapı
   beklenen DR hatasından büyük seçilmeli. Yüzeyde `capture_origin` GPS fix'e bağlı (sahada çalışıyor).
8. 🟡 Yarışma alanında açık gök GPS fix'i (bodrumda test edilemez; saha koşulu).

### Şerit Takibi (otonom hat) — İleri Kategori bileşeni
9. 🔴 **`auv_lane_follow` üretici düğümü YAZILMADI.** Arayüz tam hazır (`LaneStatus.msg`, `/lane/cmd_vel`,
   `/lane/status`, `LaneFollowStep`) ama kameradan şerit çıkaran düğüm yok → **otonom şerit uçtan uca
   çalışmaz.** *(AKSIYON-PLANI §3: suya atıldıktan sonra da yapılabilir.)*

### Mini ROV (ayrı araç, AUV'yi bloke etmez)
10. 🟠 **sysid çakışması** — Navigator varsayılan `sysid=1`, AUV de 1 → birlikte açılınca çakışır.
    `minirov_navigator.param` (sysid=2) yükle → `ros2_web_bridge.py:NAV_SYSID=2` → doğrula.
11. 🟡 Mini ROV motor montajı + param doğrulama.

### Diğer / Yardımcı
12. 🟠 **Leak devresi** — 5V↔3.3V gerilim böler/uyum devresi düzeltilmeden **ıslatma riskli**.
    Sonra: probu ısla → "Leak Detected" + kokpit "⚠ SIZINTI" kabul testi.
13. 🟡 **board_temp** — `TEMP1_TYPE=1` (TSYS01 dijital), ama `TEMP1_SRC=0` (kapalı) → FC
    `SCALED_PRESSURE3` üretmiyor → `/mav/board_temp` boş. ROS+web+90°C alarm hazır; sadece FC'de
    `TEMP1_SRC` yönlendirilmeli (+ TSYS01 fiziksel bağlı mı doğrula). Havuz öncesi baseline'da.
14. 🟡 **ZED USB3 dayanıklılık** (fiziksel: kablo/güç/titreşim).
15. ⚪ EKF/gelişmiş füzyon (robot_localization, D* Lite) — ARCHITECTURE.md'de "planlı", opsiyonel.

**Renk kodu:** 🔴 kritik/bloklayıcı · 🟠 önemli güvenlik/işlev · 🟡 iyileştirme/config · ⚪ opsiyonel.

---

## BÖLÜM C — SUDA YAPILACAKLAR ÖZETİ (sıra önerisi)
1. (Kuru, motorlu) Motor eşleme + yön → param dump + commit.
2. Sızdırmazlık + DC E-stop + halat + BATT_MONITOR geri aç.
3. (Havuz) Derinlik/pruva PID tuning → dump + commit.
4. (Havuz) Hız tablosu kalibrasyonu → `speed_table.csv` + `video_pattern.yaml` → commit.
5. (Havuz) CompassMot → dump + commit.
6. (Havuz) Görev 1 deseni: 3 temiz koşu (kare + ≥1 m daire).
7. (Havuz/saha) GPS fix + DR doğruluğu → Görev 2 koşusu.
8. Baseline'ı 4.7 isimleriyle TAM yükle (bu sırada `TEMP1_SRC`, `SERIAL3/4` temizliği) → dump + commit.
