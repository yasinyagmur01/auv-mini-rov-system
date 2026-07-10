# Web Sensör & Kamera Paneli

Tek tarayıcı sayfasında iki kamera + Bar30 derinlik + Ping sonar + IMU.

## Çalıştırma

```bash
cd web
pip install -r requirements.txt
python app.py            # gerçek (config.py'deki kaynaklar)
python app.py --demo     # donanımsız önizleme
```

Sonra tarayıcıda: **http://localhost:5000**

## Kaynak ayarı (`config.py`)

- **MAVLINK_URL**: Bench'te V6X USB portu (Windows `COM7`, Linux `/dev/ttyACM0`).
  QGC ile aynı anda kullanılamaz (seri port tek programa açılır). Ağlı sistemde
  `udpin:0.0.0.0:14552` (mavlink-router gerekli).
- **D435_SOURCES / MINIROV_SOURCES**: Kameralar sırayla denenir. Windows'ta
  `gst` çalışmaz (pip OpenCV GStreamer'sız) → `ffmpeg` + SDP kullanılır. Kamera
  bu makineye doğrudan takılıysa `('device', 0)` satırını aç.

## Nerede çalıştırmalı?

- **Kontrol PC'de**: MAVLink = COM7 (USB). Kameralar Jetson/RPi'den RTP ile
  gelmeli (Jetson'da `auv_video` node'u 5601'e, BlueOS 5600'e akıtır).
- **Jetson'da**: D435 doğrudan (`('device', ...)` veya realsense), MAVLink =
  `udpin:127.0.0.1:14551` (mavlink-router), Mini ROV = BlueOS RTP.

## Notlar

- Bar30 bench'te ~0 m okur (atmosfer); sensöre bastırınca/suya sokunca değişir.
- Ping sonar bir yüzeye tutulunca mesafe değişmeli.
- Veri gelmiyorsa: Bar30 için `SCALED_PRESSURE2`, sonar için `DISTANCE_SENSOR`
  mesajları QGC MAVLink Inspector'da görünüyor mu kontrol et (RNGFND/BARO params).
