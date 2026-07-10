# Kablolama ve Bağlantı Şeması

## AUV

| Cihaz | Bağlantı | Not |
|---|---|---|
| CUAV V6X ↔ Jetson | USB (Type-C) | udev ile `/dev/ttyFC` (bkz. `scripts/99-auv-serial.rules`; `lsusb` ile VID/PID doğrulayın) |
| Bar30 | V6X **harici I2C** (GPS1/GPS2 konektörünün I2C pinleri) | MS5837 otomatik algılanır; QGC'de `SCALED_PRESSURE2` görünmeli |
| Ping Sonar | V6X **TELEM2** (SERIAL2), 115200 | Ping firmware **≥ 3.28** (PingViewer ile güncelle). Aşağı bakacak şekilde monte |
| Helical FP9 GNSS | V6X **GPS1** (SERIAL3) | Anten su üstündeyken fix alır; `GPS_TYPE=1` |
| RealSense D435 | Jetson USB 3.0 | Şerit takibi için aşağı-ileri açıyla, düz port arkasında |
| Fiber çevirici A | AUV Ethernet anahtarına | Jetson + Mini ROV fiber çifti aynı anahtarda |
| 8 motor ESC | V6X MAIN OUT 1-8 | **Vectored_6DOF (BlueROV2-Heavy) yerleşimine göre monte edin** — yön düzeltmesi `MOT_x_DIRECTION` ile |

## Mini ROV

| Cihaz | Bağlantı | Not |
|---|---|---|
| Navigator | RPi4 üstüne şapka | BlueOS imajı SD karta |
| 4 motor ESC | Navigator PWM 1-4 | **SimpleROV-4 yerleşimi:** 2 ileri yatay (diferansiyel yaw) + 2 dikey |
| Low-light USB kamera | RPi4 USB | BlueOS Video Streams → UDP 5600 (+5602 kopya akış panele) |
| Fiber çevirici B | RPi4 Ethernet | Sinyal ana araç üzerinden karaya (şartname kuralı) |
| (Önerilen) Bar30 | Navigator I2C | Boru içinde ALT_HOLD kullanabilmek için |

## Kritik montaj notları

- V6X'i güç hatlarından/ESC'lerden uzağa monte edin (pusula girişimi). Güç kabloları burgulu çift.
- Ping Sonar ile Bar30 aynı su hacmine açık olmalı; Bar30 port yüzeyi hava kabarcığı tutmasın.
- Acil durdurma butonu batarya hattını **donanımsal** kesmeli (yazılımdan bağımsız — video şartı).
- Pervaneler nozül içinde, keskin uç yok (güvenlik kontrolü şartı).
