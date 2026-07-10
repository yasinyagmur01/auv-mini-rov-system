# TEKNOFEST 2026 İnsansız Su Altı — AUV + Mini ROV Yazılım Sistemi

İleri Kategori için AUV (8 motor, Jetson Orin NX + CUAV V6X) ve Mini ROV (RPi4 + Navigator)
yazılım sistemi. Otopilot: **ArduSub**. Şirket bilgisayarı tarafı: **QGroundControl + özel panel**.

## Mimari Özeti

```
Kontrol PC (192.168.2.1)  [QGC :14550, Panel :14552, WebSocket→Jetson:8765]
   └─ Ethernet ── fiber ── AUV anahtarı
                             ├─ Jetson Orin NX (192.168.2.3)
                             │    ├─ mavlink-router ── USB ── CUAV V6X (ArduSub, sysid 1)
                             │    │    (Bar30 I2C, Ping Sonar TELEM2, FP9 GNSS GPS1)
                             │    └─ ROS 2 Humble (görev, şerit takibi, ölü hesap, D435)
                             └─ fiber ── Mini ROV: RPi4 + Navigator, BlueOS (192.168.2.2, sysid 2)
```

## Dizinler

| Dizin | İçerik |
|---|---|
| `ros2_ws/src/` | Jetson üzerinde koşan ROS 2 Humble paketleri |
| `panel/` | Kontrol PC özel arayüzü (PySide6) — Windows/Linux |
| `ardusub_params/` | ArduSub parametre dosyaları (AUV + Mini ROV) |
| `config/` | mavlink-router, netplan, görev betikleri (YAML), hız tablosu |
| `scripts/` | udev, systemd, Jetson kurulum, kalibrasyon yardımcıları |
| `docs/` | Kablolama, ağ, havuz kontrol listeleri, video görev planı |

## ROS 2 Paketleri

| Paket | Görev |
|---|---|
| `auv_msgs` | Özel mesajlar (VerticalState, MissionState, DeadReckonState, LaneStatus, ManualControl, GpsInfo) |
| `auv_mav_bridge` | pymavlink ↔ ROS 2 köprüsü (MAVROS kullanılmıyor) |
| `auv_vertical_state` | Bar30 + Ping → net dikey eksen verisi (derinlik / dipten yükseklik / su sütunu) |
| `auv_lane_follow` | Şerit takibi (D435 renk görüntüsü → hız komutu) |
| `auv_dead_reckoning` | Yüzey GPS fix'i + pusula + hız tablosu → su altı konum kestirimi |
| `auv_mission` | Görev durum makinesi + kontrol primitifleri + panel WebSocket sunucusu |
| `auv_video` | D435 görüntüsünü H.264/RTP ile kontrol PC'ye akıtma |
| `auv_bringup` | Tek komutla tüm sistemi başlatan launch dosyaları |

## Hızlı Başlangıç

**Jetson (ilk kurulum):** `scripts/setup_jetson.sh` içindeki adımları izle, sonra:

```bash
cd ros2_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch auv_bringup bringup.launch.py
```

**Kontrol PC (panel):**

```bash
cd panel
pip install -r requirements.txt
python main.py            # görev paneli — gerçek bağlantı
python main.py --demo     # araç olmadan sahte veriyle deneme
```

**Motor test / diagnostik aracı** (bağımsız, ROS gerektirmez — tezgah için):

```bash
cd panel
python motor_test.py      # V6X USB'ye doğrudan bağlanır, motorları tek tek test
python motor_test.py --demo
```

**Görev başlatma:** Panel → görev seç (`video_pattern`) → ARM → START (10 sn geri sayım).

## Kritik Kurallar (şartname)

- Video teslim: **27 Temmuz 2026** — otonom desen: 15 sn düz → 90° sağ → 15 sn → daire (≥1 tur,
  ≥1 m çap) → 15 sn → 90° sağ → 15 sn → başlangıç karesine dönüş. Tamamı su altında, müdahalesiz.
- Görev 2'de kablo bilgisayardan sökülür; araç yüzeye yalnız bitiş karesinde çıkabilir.
- Mini ROV sinyali ana araç üzerinden gider (mevcut fiber zinciri kurala uygun).

Ayrıntılar: `docs/` klasörü ve plan dosyası.
