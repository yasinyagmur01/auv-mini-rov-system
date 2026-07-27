# Durum ve Devam Notu (Handoff)

**Son güncelleme: 10 Temmuz 2026.** Bu dosya, işe başka bir PC'de git ile devam
edildiğinde "nerede kaldık" bilgisini taşır.

> ⚠️ **Tarihsel snapshot.** Bazı satırlar sonradan güncellendi (satır-içi `DÜZELTİLDİ`
> notlarıyla işaretli). Güncel/çelişkili konularda **tek doğruluk kaynağı** için
> [KARAR-GECMISI.md](KARAR-GECMISI.md)'ye bakın.

## Genel mimari
- **AUV:** 8 motor + Jetson Orin NX + CUAV V6X (ArduSub) + Bar30 + Ping sonar +
  Helical FP9 GNSS + RealSense D435. Ayrıca Jetson'da **ZED 2i** + ROS2 SLAM.
- **Mini ROV:** RPi4 + BlueOS/Navigator + low-light USB kamera (4 motor takılacak).
- **Ağ (192.168.2.0/24, fiber):** Kontrol PC (Ethernet **.1**), BlueOS/Mini ROV .2,
  **Jetson .135 (statik, enP8p1s0)**. **İnternet yok.**
  <!-- DÜZELTİLDİ 2026-07-26: eski "PC .196" → .1 ; eski "Jetson .135 DHCP, statik .3
       uygulanmadı" → statik .135 uygulandı (canlı doğrulandı 07-23). Bkz KARAR-GECMISI.md #3. -->
- <!-- (bu satır tarihseldir; en güncel ağ bilgisi için ag-kurulumu.md / KARAR-GECMISI.md) -->
- Otopilot: **ArduSub 4.7.0 beta, hedef CUAV-V6X-v2** (kart v2, stable yok — DONDUR).

## Şu an ÇALIŞAN
- **PC web paneli** (`web/app.py`, Flask :5000): Bar30 derinlik + Ping sonar +
  IMU (V6X COM7'den) + D435 + Mini ROV kamera. `python web/app.py` (demo: `--demo`).
- **Motor test:** ✅ ÇÖZÜLDÜ — web arayüzünden (`/test` + `/control`) çalışıyor, 8'i de
  fiziksel dönüyor. "bad test type 0.00" hatası **firmware bug'ı DEĞİLMİŞ**: ArduSub
  DO_MOTOR_TEST'te "test type"ı param6 (COMMAND_INT `y` = MOTOR_TEST_ORDER) alanından
  okuyor ve **2 (MOTOR_TEST_ORDER_BOARD)** olmasını şart koşuyor; biz 0 gönderdiğimiz için
  reddediyordu. Düzeltme (`mav_bridge`): y=2 + komutu **20Hz** tekrarla (500ms watchdog,
  yoksa "timed out"→disarm) + **her testten önce ARM** (ArduSub test bitince oto-disarm).
  Ayrıca test sıra no **0-tabanlı**: seq k → SERVO çıkış k+1. Bridge UI "Motor N"→seq(N-1)→
  fiziksel çıkış N eşliyor. Testler arası **~10s cooldown** (ArduSub kuralı; UI'da beklemeli).
  QGC'ye artık gerek yok. Canlı COMMAND_ACK + /mav/servo_out ile doğrulandı.
- **Ping sonar:** TELEM1'e takılı; `SERIAL1_PROTOCOL=9, RNGFND1_TYPE=23` ile açıldı.
  <!-- DOĞRULANDI 2026-07-27: canli FC dump SERIAL1_PROTOCOL=9 -> bu satir BASTAN DOGRUYMUS.
       (26 Tem'de yanlislikla SERIAL2'ye cevrilmisti, geri alindi.) Bkz KARAR-GECMISI.md #4. -->
- **Mini ROV kamera:** BlueOS'ta RTSP stream ("MiniROV-Web", `rtsp://192.168.2.2:8554/minirov`).

## ⚠️ Geri alınacak bench ayarı
Motor testi için ARM açılabilsin diye V6X'te **`BATT_MONITOR=0`** yapıldı (bench
güç kaynağı var, batarya yok). Orijinal `BATT_MONITOR=8`, yedek:
`ardusub_params/bench_yedek_orijinal.json`. **Gerçek bataryadan/yarıştan önce geri aç.**

## 🔧 ROS2 geçişi — KÖPRÜ TAMAMLANDI (kod tarafı), Jetson'da DEPLOY bekliyor
Hedef: tüm veriler ROS2 Humble'dan geçip web'de görünsün.
Karar: internet yok + PC'de admin yok → rosbridge/web_video_server apt ile
kurulamadı; yerine **custom ROS2→web köprüsü** (Flask+rclpy+PIL) seçildi.

**Repoda tamamlanan (10 Tem 2026):**
- `jetson/webpanel_ros2.sh` — rsweb'i artık **kalıcı emekli ediyor** (systemd
  stop/disable/**mask** + supervisor + pkill), `:8000`'i `fuser`/`ss` ile garanti
  boşaltıyor, boşalmazsa `8080`'e düşüyor. `--with-mav` ile `mav_bridge`'i de başlatır.
- `jetson/ros2_web_bridge.py` — `/mav/attitude` + `/mav/battery` aboneliği eklendi;
  `/sensors` artık `roll/pitch/voltage/connected` da veriyor (web paneliyle **birebir**
  sözleşme). Yeni **`--demo`** bayrağı: rclpy'siz sentetik veri (yerelde test).
- `web/config.py` + `web/app.py` — `SENSOR_SOURCE='bridge'` seçilince panel sensörleri
  köprünün `/sensors`'ından çeker (`BridgeReader`). Böylece V6X Jetson'a taşınınca panel
  tamamen ROS2'den beslenir; V6X PC'deyken `'mavlink'` (varsayılan) çalışmaya devam eder.
- `scripts/webpanel-ros2.service` — köprüyü açılışta başlatan systemd birimi (`qayra`).

**Jetson'da yapılacak DEPLOY + doğrulama:**
1. `jetson/` dosyalarını `~/webpanel/`'e kopyala (SFTP). rsweb'i bir kez emekli et:
   `sudo systemctl disable --now rsweb; sudo systemctl mask rsweb`
   (unit adı farklıysa `systemctl list-units --all | grep -i rsweb`).
2. Kamera-only test: `bash ~/webpanel/webpanel_ros2.sh` → çıktıda `:8000` köprüde,
   `http://192.168.2.135:8000/` (D435 + ZED) ve `/stream/color` açılıyor mu.
3. **Faz 2** (sensörler): V6X USB'yi **PC'den Jetson'a taşı**, `mavlink-router`'ı
   çalıştır (`/dev/ttyFC → udpin 14551`), `ros2_ws`'i `colcon build` et, sonra
   `bash ~/webpanel/webpanel_ros2.sh --with-mav` → `curl localhost:8000/sensors`
   gerçek derinlik/sonar/pruva/roll/pitch/batarya veriyor mu.
   Not: köprü `mav_bridge`'i `depth_source=pressure2` ile başlatır (Bar30 paritesi).
4. PC panelini ROS2'ye al: `SENSOR_SOURCE=bridge python web/app.py` (D435 zaten köprüden).
   Kalıcı için: `webpanel-ros2.service`'i kur.
5. Kalan iş: Mini ROV kamerasını da (gscam RTSP→topic) ROS2'ye al.

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
