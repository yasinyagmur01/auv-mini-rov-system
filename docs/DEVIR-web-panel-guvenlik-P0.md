# DEVİR/BRİF — Web Panelini Bağımsız İzleme + Görev-Kontrol İstasyonuna Taşıma

**Hedef oturum:** CLAUDE.md sahibi, donanıma erişimi olan ekip
**Kaynak:** GCS (yer kontrol istasyonu) mühendisliği — tasarım/bilgi-mimarisi katmanı
**Tarih:** 2026-07-12
**Kapsam:** `jetson/` web paneli (:8000) + `ros2_web_bridge.py` + `auv_mission` + `auv_mav_bridge`

> **Doğrulama notu:** Bu dokümandaki tüm dosya:satır referansları ve sözleşmeler koda karşı
> okunup adversaryal olarak denetlendi. Denetimde düzeltilen hatalar: `BATT_MONITOR` baseline
> `.param`'da değil (bench notunda + yedek json'da); bazı iframe satır referansları; eksik
> `set_targets` komutu ve `altitude_valid` anahtarı; `info` tipini sunucunun üretmediği; NaN'ın
> `-1` değil doğrudan `NaN` geldiği (guard `math.isnan` olmalı).

---

## 0. GÜNCELLEME — reviewer kararları + uygulama durumu (2026-07-12)

CLAUDE.md sahibi (donanım sahibi) oturum bu brifi **onayladı** ve şu ayarları verdi; hepsi
uygulandı:

- **Sahiplik kapısı:** Backend eklemelerini (bridge/mission) donanımsız taraf **yazar**;
  **araç-üzeri doğrulama donanım sahibindedir**. Demo'da "yeşil" görünmek "bitti" DEĞİLDİR.
  Her P0 maddesinin bir **kabul testi (araçta X görülmeli)** satırı vardır (aşağıda).
- **Leak ayrı, donanım-kapılı görev** yapıldı (P0-LEAK). Varsayılan **`None`/bilinmiyor**;
  asla sahte "sızıntı yok" gösterilmez. Gerçek FC doğrulaması donanım sahibinde.
- **GPS → P1'e taşındı** (P0'dan çıktı; araç su altında GPS'i sürekli kaybeder, güvenlik-kritik
  değil). **P0 = leak + batarya V/A/% + failsafe/STATUSTEXT + stale.**
- **Sıra:** önce P0-3 (stale) + P0-2 istemci/demo → **P0-1 Operasyon ekranı en son** yapıldı.
- **CLAUDE.md'ye dokunulmadı** (donanım sahibi merge sonrası düzeltecek).

### Uygulama durumu

| İş | Kod | Demo'da doğrulandı? | Araç-üzeri kabul testi (donanım sahibi) |
|---|---|---|---|
| **P0-3 stale/kopuk** | panel.css `.linkbar`/`body.stale`; dashboard/control/motortest/ops JS | ✅ (sunucu durdurulunca kırmızı şerit + soluklaşma) | Telemetri kesilince ≤2 sn'de "veri güncel değil" görülmeli |
| **P0-2 /sensors güvenlik** | ros2_web_bridge.py: `current_a, battery_pct, armed, mode, statustext, leak` | ✅ (demo `/sensors` yeni anahtarlar; dashboard Güvenlik kartı) | Gerçek FC'de A/% NaN→None, arm/mod doğru; leak=None kalmalı (sızıntı yokken) |
| **P0-LEAK (bridge)** | bridge_node.py `/mav/leak` (STATUSTEXT tabanlı, mandallı, yalnız True) | ⚠ yalnız yazıldı (demo'da leak=None) | Leak sensörü **ıslatılınca** `/mav/leak True` + panelde ALARM + banner |
| **P0-2 mission telemetri** | mission_node.py `battery/statustext/leak` + `_safe_num` NaN guard | ⚠ yalnız yazıldı (demo'da :8765 yok) | Görev sırasında control/ops'ta batarya + leak + statustext canlı |
| **P0-1 Operasyon ekranı** | jetson/ops.html + `/ops` rotası + nav linki | ✅ (video+HUD+STOP tek pano, scroll yok) | STOP → araç nötrlenir; canlı video+telemetri tek ekranda |

### Kabul testleri (araçta görülmeli)

- **P0-3:** Ağ kablosunu çek / mission node'u durdur → **≤2 sn** içinde her ekranda kırmızı
  "veri güncel değil" şeridi + içerik soluklaşır. Yeniden bağlanınca şerit kaybolur.
- **P0-2 batarya:** `BATT_MONITOR,8` yüklüyken `/sensors` ve `state.telemetry`'de gerçekçi
  V/A/% görülür; FC yokken alan **None** (asla `NaN` → JSON kırılmaz).
- **P0-2 statustext:** ARM reddi tetikle → panelde FC prearm metni görülür.
- **P0-LEAK:** Leak sensörünü ıslat → `/mav/leak True`, panelde **⚠ SIZINTI** + ops'ta alarm
  banner + kırmızı pulse. Sızıntı yokken alan **bilinmiyor** kalır (yeşil "kuru" gösterilmez).
- **P0-1:** Otonom görevde ops ekranında video + derinlik/heading/batarya + görev adımı aynı
  anda görülür; **STOP** basılınca araç nötrlenir (`stop` komutu :8765'e gider).

---

## 0.2 GÜNCELLEME — backend entegrasyon turu (kokpit) + araç-üzeri doğrulama

Kokpit kabuğuna (`jetson/cockpit.html`) şunlar eklendi; **frontend demo'da doğrulandı**, **backend araç-üzeri doğrulanacak**:

- **Acil durdurma AUV/Mini ROV ayrıldı ve GERÇEKTEN durdurur:** AUV → WS `stop` + **`disarm`** (motor keser); Mini ROV → `/minirov/motor?motor=0`. *(Yine de fiziksel acil-durdurma butonu birincil kalır.)*
- **Per-araç sıcaklık + sızıntı:** header'da AUV (`leak`, `water_temp_c`) ve Mini ROV (`mrov_leak`, `mrov_temp_c`) ayrı.
- **Haberleşme watchdog + acil notlar:** AUV (`connected`/stale — demo'da doğrulandı), Mini ROV (`mrov_link`), tether (ikisi birden düşerse **çıkarımsal** "olası"). Kesilince `commsbar` + olay günlüğü ("AUV HABERLEŞME KESİLDİ", "Mini ROV/Navigator SİNYAL YOK", "KABLO/TETHER VERİ İLETMİYOR").
- **Drag-to-resize** sol/orta/sağ paneller; **GPS büyük harita** (orta-üstten, yerel iz — internetsiz, harita karosu yok); **çevre-algısı (SLAM) bölgesi büyütüldü**.

### Mini ROV telemetrisi — mavlink2rest GET (araçta DOĞRULA)
`ros2_web_bridge.py` gerçek modda `BridgeNode._mrov_loop` ile 1 Hz poll:
`GET http://192.168.2.2:6040/mavlink/vehicles/<NAV_SYSID>/components/1/messages/{SYS_STATUS,HEARTBEAT,SCALED_PRESSURE2,STATUSTEXT}` → `mrov_voltage/current_a/battery_pct/armed/temp_c/leak/link`. Alan çıkarımı `bridge_node.py` formülleriyle aynı, her erişim `try/except` korumalı.
**Kabul testleri / araç-üzeri doğrulama:**
- **`NAV_SYSID`** (şu an `1`): `GET .../mavlink` ağacına bakıp Mini ROV'un gerçek sysid'sini teyit et (docs `SYSID_THISMAV=2`; motor POST `target_system=1`). Yanlışsa mrov_* boş gelir.
- **JSON alan şekli** sürüme bağlı: `base_mode` int mi `{bits}` mı, `STATUSTEXT.text` liste mi string mi — `/mavlink` çıktısına bakıp doğrula.
- `mrov_temp_c` yalnız Mini ROV'da **Bar30 fiilen takılıysa** gelir; `mrov_battery_pct/current` yalnız `BATT_MONITOR` etkinse.
- **Kabul:** Navigator'ı ıslat → `mrov_leak True` + panelde ⚠; kabloyu çek → `mrov_link False` + "Navigator SİNYAL YOK".

### ZED2i çevre-algısı (3D VO/SLAM) — hâlâ araç-üzeri açık iş
Köprü ZED'den **yalnız RGB** alıyor (`/zed/.../rect/image/compressed`). 3D için gerçek veri yok; sağ panelde **DEMO/sentetik VO izi** (açıkça etiketli) + gerçek modda "veri bekleniyor". Bağlamak için araçta:
1. `ros2 topic list | grep zed` → gerçek `odom/pose/point_cloud` topic adlarını bul (konteyner→host UDP-only profilinden geçtiğini doğrula).
2. `ros2_web_bridge.py`'ye abonelik + `/sensors.zed_pose`'a (pose) veya yeni bir binary/WS kanalına (point cloud) bağla; büyük mesajlar için bant genişliğine dikkat.
3. Nokta bulutu 3D render için tarayıcıya **yerel-bundle** three.js gerekir (Jetson internetsiz). — Ayrı Faz.

---

## 0.3 GÜNCELLEME — kokpit ek entegrasyonlar (yazıldı; araç-üzeri doğrula)

- **AUV FC link watchdog:** `bridge_node.py` `/mav/link_ok` (Bool) — HEARTBEAT 3 sn'den taze mi (`_link_tick`, `h_heartbeat._last_hb`). `ros2_web_bridge` → `/sensors.link_ok`; kokpit `evalComms` artık AUV kopmasını bundan (yoksa `connected`'ten) belirler. **Kabul:** FC kablosunu çek → ≤3 sn'de `link_ok False` + "AUV HABERLEŞME KESİLDİ".
- **Mini ROV çift-yön doğrudan çıkış:** `ros2_web_bridge` `POST /minirov/direct?pwms=a,b,c,d` (1100-1900) → mavlink2rest `DO_SET_SERVO` (kanal 1-4); `POST /minirov/direct/stop` → hepsi 1500. Kokpit 4-sürgü (PWM) → 500 ms periyodik. **KRİTİK araç-üzeri:** `DO_SET_SERVO` mikser tarafından ezilirse `SERVO1-4_FUNCTION` geçici **Disabled/RCPassThru** yapılmalı (AUV bridge'deki gibi) + kanal eşlemesi doğrulanmalı. Aksi halde sürgüler motoru sürmez.
- **ZED VO/SLAM izi:** `ros2_web_bridge` `ZED_ODOM_TOPIC` (varsayılan **boş=kapalı**). Araçta `ros2 topic list | grep zed` ile odom topic'ini (nav_msgs/Odometry; tipik `/zed/zed_node/odom`) bulup bu sabite yaz → gerçek `zed_pose` (x/y/yaw) sağ panele akar (aksi halde demo-sentetik/DEMO-etiketli iz).
- **Hat-takibi simülasyonu:** control_node.py (localhost) `/stream` (MJPEG) + `/data` (JSON: `offset,yaw,pwm|pwm_left/pwm_right,line_lost`) sunacak. Kokpit Kontrol modu sol-altına gömülü; `SIM_URL` varsayılan `http://127.0.0.1:8091` (URL'de `?sim=host:port` ile değiştir). **control_node `/data`'ya `Access-Control-Allow-Origin: *` eklemeli** (panel farklı origin'den fetch ediyor); MJPEG `<img>` CORS istemez.

### ZED2i RGB GÖRÜNTÜ — araçta kontrol adımları (görüntü geliyor mu?)
Köprü RGB'yi `/zed/zed_node/rgb/color/rect/image/compressed` topic'inden alır → `/video/zed`. Sırayla:
1. **Topic var mı:** `ros2 topic list | grep zed` → `.../rgb/color/rect/image/compressed` görünüyor mu? Yoksa ZED node/launch (konteyner) yayınlamıyor.
2. **Akıyor mu:** `ros2 topic hz /zed/zed_node/rgb/color/rect/image/compressed` → >0 Hz mi?
3. **Konteyner→host:** ZED Docker'da FastDDS UDP-only profili aktif mi (`webpanel_ros2.sh`; `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `FASTRTPS_DEFAULT_PROFILES_FILE`). Host'ta `ros2 topic echo --once` ile kare geliyor mu?
4. **Köprü görüyor mu:** panelde `/sensors.cameras.zed` = `canli (ROS2)` mi yoksa `topic yok` mu?
5. Topic adı farklıysa `ros2_web_bridge.py` `COMPRESSED_TOPICS['zed']` değerini gerçek ada güncelle.

---

## 1. Bağlam & Amaç

Web paneli bugüne kadar QGroundControl'e yardımcı, ikincil bir arayüz olarak konumlandı.
TEKNOFEST görevinde ise panelin, güvenlik-kritik izleme için QGC'ye bağımlı olmadan tek başına
kullanılabilen, güvenilir bir izleme + görev-kontrol istasyonu olması gerekiyor. Bu devir,
panelin **görsel tutarlılık katmanının tamamlandığını** kabul edip, eksik kalan
**işlevsel/bilgi-mimarisi** ve **güvenlik telemetrisi** boşluklarını denize inmeden önce
kapatmayı hedefler. Tüm eklemeler geriye-uyumlu (additive) olacak; mevcut sözleşmeler kırılmayacak.

---

## 2. Mevcut Durum (tamamlandı — görsel katman)

- `jetson/panel.css`: ortak tasarım sistemi (tema, header, 4-linkli nav, kart, buton, pill/dot, tile, uyarı). Köprü `/panel.css` rotasından sunuyor (`ros2_web_bridge.py:411-419`).
- 4 sayfa (dashboard/control/motortest/minirov) ortak CSS'e bağlandı; nav'a Mini ROV linki eklendi.
- `control.html`, `web/templates/`'ten `jetson/`'a **taşındı** (eski kopya silindi — doğrulandı: `web/templates/` altında yalnız `index.html` kaldı); artık `/control` demo'da çalışıyor (`ros2_web_bridge.py:427-436`).
- `minirov.html`: WebRTC mantığına dokunulmadan HUD temaya uyduruldu + "‹ Panel" dönüş bağı eklendi (bağ `minirov.html:75`; iframe içinde gizleyen script `minirov.html:76`).

**Sonuç:** Görsel/tutarlılık katmanı bitti. İşlevsel eksikler duruyor.

---

## 3. Neden Yetersiz (kanıt) — "denize bırakınca ne yaşarız"

### 3.1 Tek-pano yok → operatör görev anında sayfa değiştirmek zorunda
Kamera, telemetri ve komutlar şu an ayrı sayfalarda dağılmış:
- **Video/HUD:** ZED (MJPEG `img` `dashboard.html:53`) + derinlik profili/IMU HUD dashboard'da; Mini ROV WebRTC videosu `minirov.html`'de (dashboard'a iframe ile gömülü, `dashboard.html:60`).
- **Görev/komut + motor + STOP:** `control.html`'de (harita/waypoint, arm/disarm/mode/mission, motor testi, STOP). **`control.html`'de kamera/video YOK** — yalnız `/minirov`'a nav linki var (`control.html:25`).
- **Motor testi + doğrudan çıkış:** `motortest.html`.

Kritik anda (araç kaçıyor, sızıntı var) operatörün canlı videoyu görürken STOP'a ulaşması için sekme değiştirmesi gecikme ve hata riski demek. Görev ekranının **tek sayfada** (video + telemetri + STOP) olması gerekir.

### 3.2 Güvenlik telemetrisi yüzeye çıkmıyor — boşluklar
Panel iki ayrı telemetri kaynağından besleniyor: `/sensors` (dashboard) ve WS `:8765` `state.telemetry` (control/motortest). İkisinde de güvenlik alanları eksik:

| Alan | `/sensors` durumu | mission `state.telemetry` durumu | Denizde riski |
|---|---|---|---|
| **Leak / sızıntı** | Yok | Yok | Su alımı fark edilmez → araç kaybı. `FS_LEAK_ENABLE,2` FC'de etkin (`auv_v6x_baseline.param:41`) ama panele hiç taşınmıyor |
| **Batarya akım (A) + %** | Sadece `voltage` (`_batt_cb`, `ros2_web_bridge.py:195-198`) | Yok | Kalan süre/akım çekişi görünmez → beklenmedik güç kesintisi |
| **Failsafe / STATUSTEXT** | Yok | Yok | FC "neden" söylüyor ama operatör görmüyor (arm reddi, failsafe tetiği) |
| **GPS fix + uydu sayısı** | Yok (hiçbir abonelik GPS taşımıyor) | Yok | Yüzeyde konum kalitesi bilinmez |
| **Arm + mod** | Yok | **Var** (`mission_node.py:435-436`, `armed`+`mode`; kaynak `/mav/armed`+`/mav/mode`) | `/sensors` tarafında yok → dashboard'da arm durumu görünmez |
| **Roll + pitch** | **Var** (`_att_cb`, `ros2_web_bridge.py:189-193`) | Yok (mission `/mav/attitude`'a abone değil; yalnız `/mav/heading_deg`) | Görev tarafında araç yatışı görünmez |

**Kaynak durumu (kritik ayrım):**
- `/sensors` tarafı (`ros2_web_bridge.py`): batarya akım/% için **yeni abonelik gerekmez** (mevcut `/mav/battery` yeterli, `:141`); leak/GPS/statustext/arm/mod için mav_bridge'in yayınladığı topic'lere **yeni abonelik** gerekir.
- mission tarafı (`mission_node.py`): batarya/GPS/statustext/roll-pitch topic'leri bridge tarafından **zaten yayınlanıyor** (`bridge_node.py:104-115`); yalnız leak topic'i hiç yok. Yani mission için çoğu iş = 1 abonelik + 1 callback + telemetry alanı.

### 3.3 "Veri bayat / bağlantı koptu" göstergesi yok
Not: `/sensors` tarafında `connected` bayrağı var (`_sensor_snapshot`, `ros2_web_bridge.py:104`, `SENSOR_FRESH_S=3.0`) ama bu yalnız **sunucu→FC** tazeliğini yansıtır ve dashboard'da yalnızca üstteki pill'i etkiler; tek tek metrikler bayatken de "canlı" gibi görünmeye devam eder. **İstemci→sunucu** kopması (WS veya HTTP poll sessizce kesilir) hiçbir sayfada izlenmiyor. Kesildiğinde ekranda **eski değerler donuk kalır** ve operatör bunu canlı sanır — güvenlik açısından en tehlikeli senaryo. Ayrı, istemci-taraflı "son güncellemeden bu yana geçen süre" göstergesi şart.

### 3.4 NaN serileştirme riski (mission tarafı, gizli bomba)
`bridge_node.py`'de `battery.current`/`percentage` (`bridge_node.py:519-520`) ve `gps.hdop` (`bridge_node.py:510`) FC'den `-1`/`65535` gelince **NaN** oluyor. `json.dumps(nan)` → geçersiz `NaN` üretir → tarayıcıda `JSON.parse` patlar → **tüm panel akışı kırılır** (tüm istemciler için). Bu alanlar telemetry'ye/`/sensors`'a eklenmeden önce her float'a `NaN→None` guard zorunlu. **Önemli:** bridge bu alanları `-1` değil doğrudan `NaN` olarak yayınladığı için, tüketen callback'lerdeki koruma `> 0` kontrolü değil `math.isnan()` kontrolü olmalı.

---

## 4. Yapılacaklar

Format: **[NE] / [NEDEN] / [NEREDE] / [SÖZLEŞME ETKİSİ] / [DONANIMSIZ MI]**

### P0 — denizden önce, güvenlik-kritik

**P0-1. Tek "Operasyon" ekranı**
- **[NE]** ZED + Mini ROV video + HUD (derinlik, heading, batarya V/A/%, görev durumu+adım, geçen süre) + belirgin STOP + üstte alarm şeridi — hepsi tek sayfada.
- **[NEDEN]** Kritik anda sekme değiştirmeyi ortadan kaldırmak (§3.1).
- **[NEREDE]** Yeni sayfa (öneri `jetson/ops.html` + yeni GET rotası) ya da dashboard'ın yeniden düzeni — bkz. açık soru §6-A. Video için mevcut `/video/zed` (MJPEG, dashboard'da `:53`) ve `/minirov` iframe'i (WebRTC, dashboard'da `:60`) yeniden kullanılır. STOP → WS `stop` komutu (`control.html:462`). Motor diyagramı gerekiyorsa `MOTORS` dizisi (`control.html:247-256`) referans.
- **[SÖZLEŞME ETKİSİ]** Additive: mevcut sayfalar/rotalar korunur; yeni rota eklenir. Yeni rota eklenirse nav (4 link) ve `panel.css` header desenine uyulur.
- **[DONANIMSIZ MI]** Düzen/stil/akış demo'da tam görünür. Canlı video + STOP davranışı araç-üzeri doğrulama ister (demo'da WS `:8765` yok, MJPEG sahte kare).

**P0-2. Güvenlik telemetrisini uçtan uca yüzeye çıkar**
- **[NE]** leak, failsafe/STATUSTEXT metni, batarya V+A+%, GPS fix+uydu, arm+mod, roll+pitch → hem `/sensors` hem mission `state.telemetry`'ye **ek anahtar** olarak.
- **[NEDEN]** §3.2 boşlukları.
- **[NEREDE]**
  - **`/sensors` (`ros2_web_bridge.py`):**
    - Yeni anahtarları `_blank_sensors()`'a (`:90-93`) ekle. **Güvenlik kuralı:** absent (henüz veri gelmemiş) durum için varsayılan **"güvenli görünen" değer OLMAMALI**. Özellikle `leak` varsayılanı `False` (sızıntı yok) değil `None` (bilinmiyor) olmalı; GPS fix `0`/None; statustext `None`. Değerler ancak gerçek callback geldiğinde dolar, aksi halde P0-3 stale/bilinmiyor göstergesine düşer.
    - **Batarya A+%:** yalnız `_batt_cb` (`:195-198`) genişlet — `current_a`, `battery_pct` (BatteryState `.current`, `.percentage×100`). **Guard:** bridge bu alanları veri yokken **`NaN`** yayınlar (`bridge_node.py:519-520`), `-1` değil → `math.isnan(v)` ise `None`. **Yeni abonelik yok** (`/mav/battery` zaten abone).
    - **Roll+pitch:** zaten var (`:189-193`), iş yok.
    - **GPS / leak / statustext / arm / mod:** yeni sabit (`:50-55` civarı), yeni import (`:115-117`), yeni `create_subscription` (`:137-141` bloğu), yeni callback (`:175-198` bloğu). Bu topic'ler mav_bridge tarafından yayınlanmalı (arm/mode/gps/statustext zaten yayınlanıyor; leak için bkz. P0-2c).
    - Demo değerleri: `DemoSource._run()` `self.sensors.update(...)` (`:249-255`) — ör. `current_a=8.4, battery_pct=87.0, gps_fix_type=3, gps_satellites=11, leak=False, statustext='OK', armed=True, mode='MANUAL'`. **Uyarı:** Bunlar sentetik değerlerdir; demo'da leak/GPS "sağlıklı" görünmesi operatörü yanıltmasın diye ekran demo modunda açıkça "DEMO" etiketi taşımalı ve bu alanların gerçek davranışı yalnız araç-üzeri doğrulanmış sayılmalı.
  - **mission `state.telemetry` (`mission_node.py`):**
    - Her ekleme = 1 abonelik (`:153-160` bloğu) + 1 callback (`:184-195` bloğu) + telemetry dict alanı (`:430-447`).
    - Batarya → `/mav/battery` (yeni abonelik); GPS → `/mav/gps_info` (auv_msgs/GpsInfo: `fix_type, satellites, hdop, lat, lon` — `bridge_node.py:506-513`); statustext → `/mav/statustext` (son metni sakla); roll/pitch → `/mav/attitude` (Vector3Stamped, radyan → `math.degrees()`).
    - **Zorunlu:** her float alanda **NaN→None** (`math.isnan`, §3.4).
  - **Leak kaynağı — bridge (`bridge_node.py`):** ArduSub'da ayrı leak MAVLink mesajı **yok**. Şu an `h_sys_status` (`:515-522`) yalnız voltage/current/remaining alıyor; `onboard_control_sensors_health` bitmask'ine bakmıyor. `h_statustext` (`:524-527`) ham metni geçiriyor ama "Leak" ayrıştırması yok. **Öneri:** `SYS_STATUS.onboard_control_sensors_health` leak health-bit'i (deterministik) birincil kanal + STATUSTEXT "Leak" eşleşmesi yedek. Yeni topic (ör. `/mav/leak` Bool veya `/mav/sys_health`) → sonra mission/web abone olur.
- **[SÖZLEŞME ETKİSİ]** Tümü additive. `/sensors` mevcut 11 anahtar korunur; dashboard yalnız `d.X!=null` ile okuduğu için fazla anahtar bozmaz. mission `state.telemetry` mevcut anahtarları (armed/mode/depth_m/altitude_m/**altitude_valid**/heading_deg/motors/dr/lane_status) korunur; control/motortest bilinmeyen anahtarı yok sayar.
- **[DONANIMSIZ MI]** `/sensors` batarya A+% ve demo değerleri **demo'da tam yapılabilir/görülebilir**. GPS/leak/statustext/arm-mod gerçek verisi + bridge yayını + NaN davranışı **araç-üzeri doğrulama** ister.

**P0-3. "Veri bayat / bağlantı koptu" göstergesi**
- **[NE]** Her ekranda son telemetri yaşı > 1-2 sn ise kırmızı uyarı; ayrıca güvenlik alanları `None`/bilinmiyor iken açıkça "—/bilinmiyor" göster (asla boş bırakıp son değeri canlı gibi tutma).
- **[NEDEN]** §3.3 — donuk eski değer canlı sanılmasın; `None` güvenlik alanı "güvenli" sanılmasın.
- **[NEREDE]** İstemci JS: `/sensors` poll ve WS `state` mesajı zaman damgasını tut, eşik aşılınca `panel.css` uyarı stilini tetikle. Tamamen ön-yüz. (Sunucudaki `connected` bayrağı FC→sunucu tazeliğidir; istemci→sunucu kopmasını kapsamaz — ikisi ayrı gösterilmeli.)
- **[SÖZLEŞME ETKİSİ]** Yok (yalnız istemci mantığı + CSS). Sunucu sözleşmesi değişmez.
- **[DONANIMSIZ MI]** Demo'da yapılabilir (kaynağı durdurup davranış test edilir).

**P0-4. Operasyonel hatırlatmalar (UI değil, dokümana not)**
- **[NE]** `BATT_MONITOR` 8'e geri (şu an bench için 0); fiziksel acil-durdurma butonu (yarışma şartı).
- **[NEDEN]** Batarya telemetrisi ancak `BATT_MONITOR` etkinken anlamlı; fiziksel STOP yazılım STOP'unun yedeği.
- **[NEREDE]** BATT_MONITOR baseline `.param`'da **yok**; bench'te `0`'a çekildiği `docs/DURUM-DEVAM.md:30-31`'de belgeli, orijinal `8` değeri `ardusub_params/bench_yedek_orijinal.json:2`'de yedekli. Yarış öncesi bu yedekten `8` geri yüklenmeli. Fiziksel acil-durdurma → donanım montajı.
- **[SÖZLEŞME ETKİSİ]** —
- **[DONANIMSIZ MI]** Araç/FC üzeri.

### P1

**P1-1. GPS kalitesi / uydu sayısı gösterimi** — [NEREDE] `/mav/gps_info` (`bridge_node.py:506-513`, `hdop` NaN guard `:510`) → mission telemetry + Operasyon HUD. Additive. Demo'da düzen, araç-üzeri gerçek.
**P1-2. ARM reddi sebebi (STATUSTEXT)** — [NEREDE] `/mav/statustext` (`bridge_node.py:524-527`) son metni HUD'da göster. Additive. Araç-üzeri.
**P1-3. EKF/estimator sağlığı** — [NEREDE] `EKF_STATUS_REPORT` bridge'de hiç dinlenmiyor; yeni handler (`rx_loop` tablosu `bridge_node.py:406-417`) + `/mav/ekf_status` gerekir. Additive. Araç-üzeri.
**P1-4. İstemci heartbeat/watchdog + "bağlı operatör yok" görünürlüğü** — [NEREDE] `auv_mission` WS sunucusu (`ws_server.py`) + Operasyon ekranı. Additive. Demo + araç.
**P1-5. Renkli eşik + sesli alarm** — [NEREDE] İstemci JS + `panel.css`. Additive, sözleşme etkisi yok. Demo'da.

### P2

- **:8765 tek-operatör kilidi/kimlik** — izole kablolu ağda düşük öncelik. `auv_mission` WS. Additive.
- **İç sıcaklık / titreşim / CPU** — bridge'de okunmuyor: `SCALED_PRESSURE2.temperature` atlanıyor (`bridge_node.py:468-482`, en kolay ekleme), `VIBRATION` dinlenmiyor, `SYS_STATUS.load` alınmıyor (`:515-522`). Yeni handler/alan. Araç-üzeri.
- **Mobil düzen** — `panel.css`. Additive.
- **Doküman çelişkilerini gider** — Jetson IP `.3` vs `.135` (`motortest.html:96`'da varsayılan `192.168.2.135`); motor testi "QGC" vs "web" (çözüldü). Yalnız dokümantasyon.

---

## 5. Bozulmayacak Sözleşmeler (kesin liste)

1. **HTTP rotaları:** `/` (`:389`), `/video/<name>` (`:399`, `name∈CAM_NAMES`), `/stream/color` (`:406`, zed alias), `/panel.css` (`:411`), `/sensors` (`:422`), `/control` (`:427`), `/minirov/motor` **POST** (`:459`, gövde: system_id=255/component_id=240/MAV_CMD_DO_MOTOR_TEST/param2=0/target_system=1/target_component=1 → `192.168.2.2:6040/mavlink`), `/test` (`:475`), `/minirov` (`:486`). `CAM_NAMES=['zed']` (IMAGE_TOPICS boş, COMPRESSED_TOPICS yalnız `zed`).
2. **`/sensors` mevcut 11 anahtar:** `depth_m, altitude_m, yaw, roll, pitch, voltage, depth_valid, sonar_valid, water_column_m, connected, cameras`. Tümü korunur; yalnız yeni anahtar eklenir. `water_column_m` yalnız `depth_valid && sonar_valid && depth_m!=None && altitude_m!=None` iken hesaplanır (`:99-103`). `connected` = son sensör damgası `SENSOR_FRESH_S=3.0` sn içindeyse (`:104`).
3. **WS `:8765` panel→sunucu komutları:** `list_missions, arm, disarm, stop, set_mode(mode), capture_origin, start_mission(mission), set_targets(turn_lat,turn_lon,finish_lat,finish_lon), set_current_bias(east_mps,north_mps), goto(lat,lon,speed_mps,arrive_radius_m), motor_test(motor,throttle,duration), motor_test_stop, direct_output(pwms[8]), direct_output_stop` (`mission_node.py:228-290`).
4. **WS `:8765` sunucu→panel tipleri:** `state, missions, error(message)`. `state.telemetry` mevcut anahtarları: `depth_m, altitude_m, altitude_valid, heading_deg, armed, mode, motors[8], dr{...}, lane_status` (`mission_node.py:430-447`). **Not:** `info(message)` tipi *istemci* tarafında işleniyor (`control.html:197`) ama sunucu şu an bu tipi **hiç üretmiyor** (rezerve). Yeni alanlar yalnız eklenir.
5. **MOTORS dizisi + yerleşim:** `control.html:247-256` ve `motortest.html:111-116`'da birebir aynı (M1-M8 koordinat/renk/açı, viewBox `0 0 260 280`). **PWM anlamı: 1100 geri · 1500 dur · 1900 ileri** (`motortest.html:78-82,192,197,199-202,224`; `control.html:296-300`). `direct_output_stop` → SERVO1-8_FUNCTION 33-40 geri yüklenir (`bridge_node.py:294-298`).
6. **minirov WebRTC protokolü:** ham WS `ws://<HOST>:<PORT>` path'siz (`minirov.html:105`, host `192.168.2.2` `:103`, port `6021` `:104`); iki katmanlı `{type,content}` zarf; dış type `question|answer|negotiation`; el sıkışma sırası (`:168-236` civarı); PRODUCER offer / istemci yalnız `createAnswer`; `mediaNegotiation`+`iceNegotiation`; snake_case alanlar; `endSession` işleme; `bundlePolicy:max-bundle`, `iceServers:[]`, `recvonly`; `TARGET_ID=f0098545-3af2-40c1-a84b-13d9d58bfd94` (`:108`). **iframe entegrasyonu yalnız `dashboard.html:60`'ta** (ayrıca ZED `img` `:53`); `control.html`'de Mini ROV iframe'i **yoktur**, orada yalnız `/minirov` nav linki var (`control.html:25`). iframe'de "‹ Panel" gizleme (`minirov.html:76`) korunur.
7. **Demo modu** (`--demo`) tüm eklemelerden sonra da çalışmaya devam etmeli.
8. **NaN kuralı:** telemetry/`/sensors`'a giden hiçbir float NaN olamaz (JSON.parse'ı kırar). Kaynak: bridge `-1`/`65535` gelen alanları `NaN`'a çeviriyor → tüketen callback `math.isnan(v)` ile `None`'a indirmeli.

---

## 6. Açık Sorular / Ekip Kararı Gerektirenler

- **A. Operasyon ekranı: ayrı sayfa mı, dashboard yeniden mi?** Ayrı `/ops` rotası eklemek mevcut dashboard'ı bozmadan yeniden başlar (additive, geri dönüş kolay); dashboard'ı yeniden yapmak nav'ı 4 linkte tutar ama regresyon riski. Öneri: ayrı sayfa + nav'a link. Ekip karar versin.
- **B. Leak kanalı: STATUSTEXT mi, SYS_STATUS health-bit mi?** Kanıt: STATUSTEXT dile/severity'ye bağlı, kırılgan; `SYS_STATUS.onboard_control_sensors_health` deterministik. Öneri: health-bit birincil + STATUSTEXT yedek. Ama health-bit'in bu FC/ArduSub sürümünde leak için doğru set edildiği **araç-üzeri doğrulanmalı**.
- **C. GPS ihtiyacı:** Araç su altında GPS fix'i sürekli kaybedecek. GPS HUD'u yalnız yüzey/kalibrasyon aşaması için mi? Yoksa dead-reckoning origin teyidi için mi? Gösterim mantığı buna göre.
- **D. Batarya %:** `BATT_MONITOR,0` iken `battery_remaining` anlamsız/-1 (→ NaN) gelir. `BATT_MONITOR,8` (yedekten) yüklenmeden % göstergesi doğrulanamaz — P0-4 ile bağımlı.
- **E. auv_msgs:** GPS/EKF/vibration için `auv_msgs`'e yeni mesaj tipi mi, yoksa `diagnostic_msgs`/`Float32MultiArray` mı? Mevcut `GpsInfo.msg` var (`fix_type, satellites, hdop, lat, lon`); diğerleri için karar.
- **F. mission vs /sensors çift kaynak:** İki telemetri yolu (dashboard→`/sensors`, Operasyon→WS) uzun vadede tek kaynağa mı indirgenecek? Şimdilik ikisini de beslemek gerekiyor.

---

## 7. CLAUDE.md'ye Önerilen Güncellemeler

1. **Aktif görev bölümü:** "Web arayüzünü baştan yeniden tasarla (görsel katman)" **tamamlandı** olarak işaretlenip, yeni aktif görev "Güvenlik telemetrisi + tek-pano Operasyon ekranı (P0)" olarak eklenmeli.
2. **"BOZMA — arka uç sözleşmeleri" bölümüne ekle:**
   - `/sensors` **11** anahtarın tam listesi (§5-2).
   - WS panel→sunucu komut listesi — özellikle `set_targets` (§5-3) — ve `state.telemetry` mevcut anahtar listesi (`altitude_valid` dahil, §5-4).
   - `/panel.css`, `/stream/color`, `/video/<name>` rotaları (mevcut CLAUDE.md listesinde eksik).
   - **NaN kuralı** (§5-8) — yeni float telemetri eklerken `math.isnan` guard zorunlu.
3. **Mimari notu düzelt:** "saf Python `http.server`" ifadesi yanlış — kod gerçekte **Flask** kullanıyor (`ros2_web_bridge.py:38` import, `:271` `app = Flask(__name__)`). Metin güncellensin.
4. **Yeni not — telemetri katmanlaması:** İki ayrı telemetri yolu olduğu (dashboard `/sensors` polling vs control/motortest WS `:8765`) ve güvenlik alanlarının **her ikisine de** eklenmesi gerektiği yazılsın.
5. **Leak notu:** ArduSub'da ayrı leak mesajı olmadığı; `SYS_STATUS.onboard_control_sensors_health` + STATUSTEXT kanalları; `FS_LEAK_ENABLE,2` (`auv_v6x_baseline.param:41`) etkin ama bridge'in taşımadığı kaydedilsin.
6. **Ops uyarıları:** `BATT_MONITOR` baseline `.param`'da değil; bench'te `0` (`docs/DURUM-DEVAM.md:30-31`), orijinal `8` yedeği `ardusub_params/bench_yedek_orijinal.json:2` — yarış öncesi `8` geri yüklensin; fiziksel acil-durdurma butonu şartı — kalıcı checklist notu.

---

**İlgili dosyalar (mutlak yol):**
- `C:\Users\yasin\auv-mini-rov-system\jetson\ros2_web_bridge.py`
- `C:\Users\yasin\auv-mini-rov-system\jetson\dashboard.html`
- `C:\Users\yasin\auv-mini-rov-system\jetson\control.html`
- `C:\Users\yasin\auv-mini-rov-system\jetson\motortest.html`
- `C:\Users\yasin\auv-mini-rov-system\jetson\minirov.html`
- `C:\Users\yasin\auv-mini-rov-system\jetson\panel.css`
- `C:\Users\yasin\auv-mini-rov-system\ros2_ws\src\auv_mission\auv_mission\mission_node.py`
- `C:\Users\yasin\auv-mini-rov-system\ros2_ws\src\auv_mission\auv_mission\ws_server.py`
- `C:\Users\yasin\auv-mini-rov-system\ros2_ws\src\auv_mav_bridge\auv_mav_bridge\bridge_node.py`
- `C:\Users\yasin\auv-mini-rov-system\ardusub_params\auv_v6x_baseline.param`
- `C:\Users\yasin\auv-mini-rov-system\ardusub_params\bench_yedek_orijinal.json`
- `C:\Users\yasin\auv-mini-rov-system\docs\DURUM-DEVAM.md`
