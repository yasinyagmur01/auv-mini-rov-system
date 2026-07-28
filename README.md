# AUV + Mini ROV — Ana Bilgisayar Yazılımı (Jetson Orin NX 16GB)

TEKNOFEST insansız su altı yarışması yazılımı. Tek depo, iki alt-sistem:

- **Algı (ZED 2i / ROS 2 / Docker):** görsel-ataletsel odometri, derinlik, 3D haritalama,
  lokalizasyon. → `docker/`, `ros2_ws/src/zed-ros2-wrapper`, `ros2_ws/src/auv_bringup`, `maps/`.
  (Detay aşağıda "ZED 2i Algı Alt-sistemi".)
- **Kontrol / Panel / Otonom (MAVLink / web):** ArduSub↔ROS2 köprüsü, görev durum makinesi,
  ölü-hesap navigasyon, dikey eksen, ve **iki web arayüzü**. → `ros2_ws/src/auv_*`, `jetson/`, `ui/`.

> Sürüm: **L4T r36.5 (JetPack 6.2) · Ubuntu 22.04 · ROS 2 Humble · ZED SDK 5.4.0 ·
> CUDA 12.6 + cuDNN 9.3 + TensorRT 10.3**. Kök disk: NVMe SSD. Carrier: AVerMedia D115S
> (device-tree `scripts/apply_d115s_dtb.sh` ile uygulanır — USB3/ZED için gerekli).

## İki web arayüzü (ikisi de korunuyor — kanonik seçim ekipçe sonra)
| Arayüz | Konum | Port | Teknoloji |
|---|---|---|---|
| **Kokpit** | `jetson/cockpit.html` + `jetson/ros2_web_bridge.py` | **:8000** | Flask; tek-sayfa (Operasyon/Kontrol/Motor Test); ZED MJPEG + sonar + 3D SLAM nokta bulutu + manuel sürüş |
| **ui/** | `ui/server/main.py` (+ `static/`) | **:8080** | FastAPI + three.js (3D) + uplot; görev kontrol paneli |

Yerelde donanımsız test: `python3 jetson/mock_host.py` → http://127.0.0.1:8001/ (sahte veri, "MOCK VERİ" bandı).

## Ağ
Düz L2 (`192.168.2.0/24`, DHCP yok). Jetson **192.168.2.135** (statik, arayüz `enP8p1s0`; canlı doğrulandı 2026-07-23),
BlueOS/Mini ROV **.2**, kontrol PC **.1** (`mavlink-router` QGC/panel'i `.1`'e push eder). Panel: `http://192.168.2.135:8000/`.

## Saha kontrolü — sağlık testi (tek komut)
Kontrol PC'sinden (Windows / PowerShell) **tek kelime**; SSH'ı kendisi yapar, Jetson'da menü açar:
```
saha
```
Menü: **1** Haberleşme · **2** Sensör verileri · **3** Sensör seç (`-i`) · **4** Tam test · **5** Jetson kabuğu · **q** Çıkış.
Her seçim raporu gösterir, `~/auv/logs/health_check_*.log`'a yazılır, sonra menüye döner.
(`saha/` klasörü PATH'te — yeni terminalde `saha` her dizinden çalışır. Ayrıntı: [`saha/README.md`](saha/README.md).)

Doğrudan **Jetson'da** (SSH'liyken):
```bash
bash ~/auv/scripts/saha_menu.sh                  # menü
bash ~/auv/scripts/health_check.sh               # haberleşme + sensör
bash ~/auv/scripts/health_check.sh --comms-only  # sadece haberleşme
bash ~/auv/scripts/health_check.sh -i            # takılı sensörleri seç
```
Salt-okunur (motor/arm/MAVLink yazması YOK). Sensör durumları: `BAĞLI·VERİ AKIYOR` /
`BAĞLANTI VAR AMA VERİ GELMİYOR` / `BAĞLANTI YOK`. Haberleşme: kart ping + FC MAVLink linki
(`.20`) + köprüler (8000/8765/8080); sorun varsa nedeni yazılır.

## Deploy + Boot (tek yol)
```bash
# DEPLOY (Jetson'da): repo tek doğruluk-kaynağı; ~/webpanel EMEKLİ (köprü ~/auv/jetson'dan koşar)
cd ~/auv && git pull origin main
cd ros2_ws && colcon build --symlink-install     # yalnız ROS2 değiştiyse
sudo systemctl restart auv-stack

# BOOT servisleri (bir kez kur): mavlink-router (FC seri paylaşımı) + auv-stack (çekirdek+cockpit) + auv-ui
sudo cp ~/auv/scripts/auv-stack.service ~/auv/ui/auv-ui.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mavlink-router auv-stack auv-ui
```
- **Router = mavlink-router:** FC seri portunu QGC (:14550) + mav_bridge (:14551) + panel arasında
  EŞZAMANLI paylaştırır. Yedek (router yoksa): `run_stack.sh` mav_bridge'i doğrudan seri açar.
- Tek başlatıcı: `scripts/auv_stack_start.sh` (auv-stack.service çağırır).

## Yedekler / tarihçe
Bu depo iki çatalın birleşimidir (`main` = jetson-master araç kodu + web-kokpit graft, 2026-07-13).
Eski durumlar: `origin/jetson-master`, `origin/web-kokpit-devir` dalları + `pre-unify-*` tag'leri.
Tam proje talimatı: **`CLAUDE.md`**.

---

# ZED 2i Algı Alt-sistemi

İlk entegre sensör: **ZED 2i** stereo kamera (görsel-ataletsel odometri, derinlik, 3D haritalama).
Taban imaj `stereolabs/zed:5.4.0-devel-l4t-r36.5` · zed-ros2-wrapper v5.4.0.

```
docker/                  # Konteyner mimarisi (Dockerfile.zed, compose.yaml, ros_entrypoint.sh)
ros2_ws/src/auv_bringup/ # ZED launch + config + map_saver düğümü (zed2i_auv.launch.py)
ros2_ws/src/zed-ros2-wrapper/  # Resmi Stereolabs sarmalayıcı (v5.4.0)
maps/                    # .area (lokalizasyon) + .ply (3D harita) çıktıları
scripts/install_host.sh · check_zed_usb.sh · build_native.sh · run_native.sh
docs/ARCHITECTURE.md     # Mimari kararlar + sensör füzyon planı
```

## Kurulum (bir kez)
```bash
sudo bash scripts/install_host.sh   # CUDA/TensorRT + Docker + nvidia-container-toolkit (sonra oturum aç/kapa)
bash scripts/check_zed_usb.sh       # ZED 2i USB 3.0 tanısı
docker compose -f docker/compose.yaml build zed
```

## Çalıştırma + doğrulama
```bash
docker compose -f docker/compose.yaml up -d zed
# Host'ta (network_mode: host): konteyner root ↔ host kullanıcı → FastDDS SHM uyuşmaz, UDP zorunlu:
source /opt/ros/humble/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic hz /zed/zed_node/rgb/color/rect/image
ros2 topic echo /zed/zed_node/odom --once
ros2 topic hz /zed/zed_node/mapping/fused_cloud
```

## Harita / lokalizasyon
```bash
ros2 service call /map_saver/save_map std_srvs/srv/Trigger          # 3D .ply kaydet (60 sn'de otomatik de)
ros2 service call /zed/zed_node/reset_odometry std_srvs/srv/Trigger # odometriyi sıfırla
```

### ⚠️ Tracking modu takası (SDK 5.4.0'da doğrulandı)
| Mod | VIO/Odometri | 3D haritalama | Area memory (relocalization) |
|---|---|---|---|
| `GEN_1` (varsayılan) | ✅ | ✅ | ❌ ("FILE EMPTY") |
| `GEN_3` | ✅ | ❌ (hep 0 nokta — SDK) | ✅ |
```bash
ros2 launch auv_bringup zed2i_auv.launch.py                    # haritalama (GEN_1)
ros2 launch auv_bringup zed2i_auv.launch.py tracking_mode:=GEN_3  # relocalization
```
- Area dosyaları nesile özgü: GEN_3 kaydını GEN_1 yükleyemez (`mv maps/auv_area.area{,.bak}`).

## Notlar
- ZED 2i **USB 3.0** ister (USB 2.0'da kamera numaralanmaz — yalnız 2b03:f881 HID). `check_zed_usb.sh`.
- SDK 5.x derinlik modları NEURAL ailesi → TensorRT gerektirir (`install_host.sh` kurar; Docker'da hazır).
- İlk açılışta AI modeli optimize edilir (dakikalar; `zed_resources` volume'unda önbelleklenir).
- GNSS füzyonu hazır ama kapalı: `ros2 launch auv_bringup zed2i_auv.launch.py enable_gnss:=true`.
- Docker root yazar → `sudo chown -R $USER ~/auv/maps` (native'e geçmeden).
