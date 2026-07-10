# Durum ve Devam Notu (Handoff)

**Son güncelleme: 10 Temmuz 2026.** Bu dosya, işe başka bir PC'de git ile devam
edildiğinde "nerede kaldık" bilgisini taşır.

## Genel mimari
- **AUV:** 8 motor + Jetson Orin NX + CUAV V6X (ArduSub) + Bar30 + Ping sonar +
  Helical FP9 GNSS + RealSense D435. Ayrıca Jetson'da **ZED 2i** + ROS2 SLAM.
- **Mini ROV:** RPi4 + BlueOS/Navigator + low-light USB kamera (4 motor takılacak).
- **Ağ (192.168.2.0/24, fiber):** Kontrol PC (Ethernet .196), BlueOS/Mini ROV .2,
  **Jetson .135 (DHCP; statik .3 uygulanmadı)**. DHCP var ama **internet yok**.
- Otopilot: **ArduSub 4.7.0 beta, hedef CUAV-V6X-v2** (kart v2, stable yok — DONDUR).

## Şu an ÇALIŞAN
- **PC web paneli** (`web/app.py`, Flask :5000): Bar30 derinlik + Ping sonar +
  IMU (V6X COM7'den) + D435 + Mini ROV kamera. `python web/app.py` (demo: `--demo`).
- **Motor test:** ArduSub beta'da MAV_CMD_DO_MOTOR_TEST bozuk ("bad test type");
  motorlar **QGC → Motors** sayfasından dönüyor (8'i de sağlam, çift yönlü).
- **Ping sonar:** TELEM1'e takılı; `SERIAL1_PROTOCOL=9, RNGFND1_TYPE=23` ile açıldı.
- **Mini ROV kamera:** BlueOS'ta RTSP stream ("MiniROV-Web", `rtsp://192.168.2.2:8554/minirov`).

## ⚠️ Geri alınacak bench ayarı
Motor testi için ARM açılabilsin diye V6X'te **`BATT_MONITOR=0`** yapıldı (bench
güç kaynağı var, batarya yok). Orijinal `BATT_MONITOR=8`, yedek:
`ardusub_params/bench_yedek_orijinal.json`. **Gerçek bataryadan/yarıştan önce geri aç.**

## 🔧 KALDIĞIMIZ YER — ROS2 geçişi (Faz 1, YARIM)
Hedef: tüm veriler ROS2 Humble'dan geçip web'de görünsün.
Karar: internet yok + PC'de admin yok → rosbridge/web_video_server apt ile
kurulamadı; yerine **custom ROS2→web köprüsü** (Flask+rclpy+PIL) seçildi.

Jetson `~/webpanel/`'e deploy edilenler (kaynak: repo `jetson/`):
- `d435_ros2_node.py` — pyrealsense2 → `/camera/color/image_raw` **(çalışıyor ✓)**
- `ros2_web_bridge.py` — :8000, ROS2 topic → MJPEG+JSON (`/stream/color` rsweb-uyumlu)
- `webpanel_ros2.sh` — rsweb'i durdur + d435 node + köprüyü başlat

**Takılan nokta:** `ros2_web_bridge` **:8000'e bağlanamıyor** — `rsweb` (Jetson'daki
eski D435 web app'i, `~/rsweb/app.py`) pkill sonrası yeniden doğuyor (muhtemelen
systemd servisi/supervisor) ve portu tutuyor.

### Sıradaki adımlar
1. **rsweb'i kalıcı durdur:** `systemctl` ile servisini bul/durdur/disable et
   (`systemctl list-units | grep -i rsweb`), ya da köprüyü başka porta al
   (`ros2_web_bridge.py --port 8001`) ve PC panel `web/config.py` D435 kaynağını güncelle.
2. Köprüyü doğrula: `http://192.168.2.135:8000/` (D435 ROS2 + ZED) ve `/stream/color`.
3. **Faz 2:** V6X USB'yi **PC'den Jetson'a taşı** → Jetson'da `mav_bridge` ROS2
   node'u (repo `ros2_ws/src/auv_mav_bridge`) Bar30/sonar/IMU/GPS'i topic'lere yayınlasın;
   köprü zaten `/mav/depth`, `/mav/rangefinder`, `/mav/heading_deg`'e abone.
4. Web arayüzünü tamamen ROS2'den besle; Mini ROV kamerasını da (gscam RTSP→topic) ROS2'ye al.

## Jetson erişimi
- SSH: `qayra@192.168.2.135` — **şifre repo'ya yazılmadı, takımdan al.**
- ROS2 Humble kurulu (`source /opt/ros/humble/setup.bash`), librealsense 2.58 kaynaktan.
- ZED2i SLAM çalışıyor (`auv_bringup zed2i_auv.launch.py`) — **dokunma.**
- Başka PC'den bağlanmak için: `pip install paramiko`, ya da normal `ssh qayra@192.168.2.135`.

## Başka PC'de devam (git)
```bash
git clone https://github.com/yasinyagmur01/auv-mini-rov-system.git
cd auv-mini-rov-system
# PC paneli:  cd web && pip install -r requirements.txt && python app.py
# Jetson dosyaları jetson/ altında; SFTP ile ~/webpanel/'e kopyalanır.
```
