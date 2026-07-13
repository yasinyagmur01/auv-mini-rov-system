# ArduSub Kurulumu

## AUV (CUAV V6X **v2** — KESİNLEŞTİ)

Kartımız **V6X v2** (orijinal V6X bulunamadığı için v2 alındı). ArduSub'da bu
revizyonun **stable sürümü YOK**; yalnız beta'da var (9 Tem 2026'da doğrulandı):

| | Değer |
|---|---|
| Firmware hedefi | **CUAV-V6X-v2** |
| Sürüm | **ArduSub 4.7.0 beta** (git `97775f82`, 12 Haz 2026) |
| Dosya | `firmware.ardupilot.org/Sub/beta/CUAV-V6X-v2/ardusub.apj` |

**Flaşlama:** QGC → Vehicle Setup → Firmware → "Advanced settings" →
"Custom firmware file" → yukarıdaki `.apj`'yi indirip seçin.

> **KRİTİK — firmware'i DONDURUN.** Bu tek seçenek olduğu için, flaşlayıp tüm
> sensörlerin çalıştığını doğruladıktan sonra sürümü not edin ve **yarışmaya
> kadar güncellemeyin**. Beta sürümler değişebilir; çalışan sürümü kaybetmemek
> için `.apj` dosyasının bir kopyasını `ardusub_params/` yanına yedekleyin.
> Sensörler görünmezse (beta hatası): son çare olarak Pixhawk6X uyumlu başka bir
> FC (CubeOrange vb.) veya farklı bir beta commit'i denenir.

**Flaşlama:** QGC → Vehicle Setup → Firmware → "Custom firmware file" → apj seçin.

## Kurulum sırası (M0)

1. Firmware flaşla, QGC bağlan.
2. `ardusub_params/auv_v6x_baseline.param` yükle (Parameters → Tools → Load).
3. Frame doğrula: `FRAME_CONFIG=2` (Vectored_6DOF). QGC Motor Setup'ta 8 motoru
   tek tek test et; ters dönenlere `MOT_x_DIRECTION=-1`.
4. Sensör kontrol (MAVLink Inspector):
   - `SCALED_PRESSURE2` → Bar30 ✓ (görünmüyorsa `BARO_PROBE_EXT` MS5837 biti)
   - `DISTANCE_SENSOR` → Ping ✓ (önce PingViewer ile firmware ≥ 3.28)
   - `GPS_RAW_INT` → FP9 ✓ (açık alanda; kilitlenmezse `GPS_TYPE=5` NMEA)
   - `ATTITUDE` → IMU ✓
5. Kalibrasyonlar: ivmeölçer (QGC), pusula (montajlı gövdede, demirden uzakta),
   joystick. Havuzda: **CompassMot** (motor girişimi).
6. Su tipi: QGC Frame sayfası → tatlı/tuzlu su (havuz/deniz günü değiştir).
7. Tam parametre dökümü al → `ardusub_params/auv_v6x_YYYYAAGG.param` (her havuz günü!).

## Mini ROV (RPi4 + Navigator)

1. BlueOS stable imajını SD'ye yaz (Raspberry Pi Imager / balena).
2. Ethernet ile bağlan → `http://192.168.2.2` → BlueOS arayüzü.
3. ArduSub'ı BlueOS içinden kur (Navigator Linux build'i otomatik).
4. `ardusub_params/minirov_navigator.param` yükle (`FRAME_CONFIG=5`, `SYSID_THISMAV=2`).
5. Motorları SimpleROV-4 yerleşimine göre TAK (2 ileri yatay + 2 dikey), Motor Setup'ta doğrula.
6. Video Streams → USB kamera → UDP 5600 → 192.168.2.1 (+5602 ikinci akış panele).
7. MAVLink Endpoints: varsayılan `udpout 192.168.2.1:14550` kalsın.

## Bilinen riskler

- **librealsense / JetPack 6:** sorun çıkarsa librealsense v2.55+ RSUSB backend
  ile kaynaktan derleyin; `realsense2_camera` humble dalını eşleştirin.
- **GUIDED modu kullanmıyoruz:** su altında EKF konum kestirimi yok; tüm otonomi
  ALT_HOLD + MANUAL_CONTROL üzerinden (mission node).
- **FS_GCS_ENABLE=0:** Görev 2'de kablo söküleceği için bilinçli kapalı. Güvenlik,
  görev betiği zaman aşımları + FS_BATT/FS_LEAK ile sağlanıyor.
