# Aksiyon Planı — Suya Atma Yol Haritası

Bu plan **yazılım + kalibrasyon** işlerini önceliklendirir. Kaynak: 2026-07-26 sistem durum
analizi. Güncel/çelişkili konularda [KARAR-GECMISI.md](KARAR-GECMISI.md) tek doğruluk kaynağıdır.

> **Fiziksel kontrol listesi (ayrıca takip edilecek, bu planın DIŞINDA):** halat (≥50 m),
> sızdırmazlık testi, acil durdurma butonu, BATT_MONITOR geri açma (0→8), CompassMot.
> Bunlar donanım ekibinin fiziksel iş kalemidir; burada tekrar takip edilmez, yalnız hatırlatılır.

---

## 1) Suya atılmadan ZORUNLU (yazılım / kalibrasyon)

Bunlar bitmeden araç suya atılmamalı. Öncelik sırasıyla:

- [x] **FC↔Jetson bağlantısı (TAMAMLANDI 2026-07-27):** CUAV V6X ethernet'e taşındı
      (`192.168.2.20:14550`, UDP server). USB-only'den kurtulundu; USB acil yedek.
      Doğrulama: USB çekiliyken `link_ok=True`, sağlık testi tam yeşil. Bkz KARAR-GECMISI.

- [ ] **Motor kanal eşleme + yön (QGC veya panel `/test`).** 8 motoru tek tek döndür, panel/QGC
      numarası ↔ fiziksel motor eşle; ters dönenlere `MOT_x_DIRECTION=-1`. → **Tam param dump'ı
      `ardusub_params/auv_v6x_YYYYAAGG.param` olarak repoya geri yaz** (şu an param'da `MOTn_`/yön yok).
- [ ] **Sensör MAVLink doğrulaması** (QGC → MAVLink Inspector): `SCALED_PRESSURE2` (Bar30),
      `DISTANCE_SENSOR` (Ping, SERIAL2), `GPS_RAW_INT` (FP9), `ATTITUDE` (IMU) akıyor.
- [ ] **Telemetri + güvenlik göstergesi araç-üzeri doğrulama:** kokpitte leak=`None`/"izleniyor"
      (asla sahte "kuru"), stale/kopuk göstergesi, NaN guard. (P0 kuralları — DEVIR dokümanı.)
- [ ] **ROS2 yığını sağlıklı:** `cd ~/auv/ros2_ws && colcon build --symlink-install` hatasız;
      boot servisleri (`mavlink-router`, `auv-stack`, `auv-ui`) aktif; `/mav/*` + `/vertical_state`
      topic'leri akıyor.
- [ ] **Yazılım kuru koşusu (MOTORSUZ):** görev durum makinesi doğru `manual_control` üretiyor
      (ARM yok → dönmez); START→adımlar→STOP nötr. (tezgah-testi.md §F.)

## 2) Havuzda yapılacak (tuning + param dump + repo'ya geri yazma)

Havuz erişimi gerektirir; her param değişikliğinden sonra **dump alıp commit et**.

- [ ] **Hız tablosu kalibrasyonu:** `scripts/calibrate_speed.py` → `config/speed_table.csv`
      güncelle + commit (dead reckoning bu tabloya bağlı).
- [ ] **Derinlik / pruva tuning:** ALT_HOLD ±0.15 m, pruva tutma ±5° (ATC_/PSC_ hiç ayarlanmadı,
      şu an ArduSub varsayılanları). Kabul kriteri tutturulunca param dump → commit.
- [ ] **CompassMot** sonrası (fiziksel adım) → tam param dump → `auv_v6x_YYYYAAGG.param` commit.
- [ ] **NAV_SYSID donanım teyidi (#7 OPEN):** `mavlink2rest .../vehicles` ile Mini ROV'un ilan
      ettiği sysid'yi oku; `ros2_web_bridge.py:NAV_SYSID`'i ona göre ayarla + commit.
- [ ] **Leak / sıcaklık kabul testi:** (devre fiziksel — bkz §3) pin+logic ayarlı ise probu ısla →
      "Leak Detected" + kokpit "⚠ SIZINTI"; sensörü ısıt → 90°C uyarısı gerçek veriyle.
- [ ] **Görev deseni koşusu:** video pattern (kare + ≥1 m daire) 3 temiz koşu; param/süre
      ayarları `config/missions/video_pattern.yaml`'da → commit.

## 3) Sonraya bırakılabilir (suya atıldıktan sonra da yapılabilir)

- [ ] `auv_lane_follow` node'u (şerit takip görüntü işleme). Arayüz hazır (`LaneStatus.msg`,
      `/lane/*`, `LaneFollowStep`) ama üretici node yazılmadı → otonom şerit şu an uçtan uca çalışmaz.
- [ ] EKF / gelişmiş sensör füzyonu (robot_localization, D* Lite) — ARCHITECTURE.md'de "planlı".
- [ ] Mini ROV motor montajı + `minirov_navigator.param` doğrulama (ayrı araç, AUV'yi bloke etmez).
- [ ] **Mini ROV sysid çakışması (#7):** Navigator şu an varsayılan **sysid=1** (param yüklenmemiş);
      AUV de sysid=1 → iki araç birlikte açılınca çakışır. `minirov_navigator.param` yükle
      (SYSID_THISMAV=2) → `jetson/ros2_web_bridge.py:NAV_SYSID`'i **2** yap → iki araç birlikte
      açıkken QGC'de çakışmasız göründüğünü doğrula. (Teyit: 2026-07-27 mavlink2rest.)
- [ ] ZED USB3 dayanıklılık (fiziksel kök neden: kablo/güç/titreşim).
- [ ] Leak 5V↔3.3V gerilim uyumu devresi (gerilim böler / 6.6V ADC girişi) — çözülmeden ıslatma riskli.

---

## Workflow hatırlatması (geliştir → commit → deploy)
1. **Geliştir (donanımsız):** `cd jetson && python3 ros2_web_bridge.py --demo`.
2. **Commit + push** (kontrol PC deploy etmez, sadece push).
3. **Deploy (donanım ekibi, Jetson):** cerrahi `scp` (asla `rsync --delete` değil) → `colcon build`
   → `systemctl restart auv-stack`.
4. **Saha kalibrasyonu → param dump → geri commit** (döngüyü kapat; kod↔FC ayrışmasın).
