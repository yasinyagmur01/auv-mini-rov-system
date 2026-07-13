# Tezgah (Bench) Testi — Havuz Öncesi

Mevcut durum: kartlar ve sistem masa üstünde kurulu, motorlar bir kovanın
içinde. Havuza girmeden **şimdi** yapılabilecek her şey burada. Sırayla gidin;
her adım bir sonrakinin ön koşuludur.

> ⚠️ Motorlar kovada su içindeyken çalıştırılabilir ama **pervaneler su altında
> olmalı** (havada uzun süre çalıştırmayın, ESC/motor ısınır). El/kablo motordan
> uzak dursun. Acil durdurma butonu her zaman erişilebilir olsun.

---

## A. ArduSub kurulumu (masa üstü, motor gerekmez)

- [ ] **Firmware flaşla:** ArduSub **4.7.0 beta, CUAV-V6X-v2** hedefi
      (bkz. [ardusub-kurulum.md](ardusub-kurulum.md)). `.apj`'yi yedekle, sürümü dondur.
- [ ] QGC ile USB üzerinden bağlan, `ardusub_params/auv_v6x_baseline.param` yükle.
- [ ] `FRAME_CONFIG=2` doğrula (QGC motor diyagramı ekran görüntünüzle aynı olmalı:
      1-4 yatay çapraz, 5-8 dikey dairesel).
- [ ] **İvmeölçer kalibrasyonu** (QGC → Sensors → Accelerometer). Kartı masaya
      sabit koyup 6 yönde çevirerek yapın.
- [ ] **Pusula kalibrasyonu** (QGC → Sensors → Compass). **Motorlardan ve
      demir/çelik masadan uzakta** yapın — el ile döndürerek. (Not: CompassMot =
      motor akım girişimi kalibrasyonu havuz işi, tezgahta anlamlı olmaz.)

## B. Sensör doğrulama (QGC → Analyze → MAVLink Inspector)

Her sensörün mesajının aktığını gözle doğrulayın:

- [ ] `SCALED_PRESSURE2` → **Bar30**. Masada ~atmosfer basıncı okur; parmakla
      hafif bastırınca `press_abs` değişmeli. Görünmüyorsa I2C bağlantısı + gerekirse
      `BARO_PROBE_EXT` MS5837 biti.
- [ ] `DISTANCE_SENSOR` → **Ping Sonar**. Önce PingViewer ile firmware ≥ 3.28.
      Sonarı bir yüzeye tutunca `current_distance` değişmeli.
- [ ] `GPS_RAW_INT` → **FP9**. Anteni **pencere kenarına/dışarı** koyun; `fix_type`
      3'e ulaşmalı, `satellites_visible` artmalı. (Masa ortasında fix almaz — normal.)
- [ ] `ATTITUDE` → IMU. Kartı elinizle eğince roll/pitch/yaw değişmeli.

## C. Motor sırası ve yönü (motorlar kovada — EN ÖNEMLİ tezgah işi)

> **BU FIRMWARE'DE (ArduSub 4.7.0 beta, CUAV-V6X-v2) MOTOR TESTİ = QGC.**
> Ham `MAV_CMD_DO_MOTOR_TEST` (özel `panel/motor_test.py` dahil) reddediliyor:
> DISARM iken "Arm motors before testing motors", elle ARM edince "bad test
> type 0.00". Sadece **QGC Motors sayfası çalışıyor** (arka planda otomatik
> arm/disarm + kendi komut biçimi). Motor bring-up/eşleme QGC ile yapılır.

**QGC ile motor eşleme:**
- QGC → Vehicle Setup → **Motors** sayfası.
- Motorlar suda, e-stop elde. Bir motorun slider'ını hafifçe yukarı çek → döner.
- QGC slider no = ArduSub motor no. Hangi fiziksel motor (renk/konum) döndü not et.
- 8 motoru tek tek yap → ArduSub çıkışı ↔ fiziksel motor eşleme tablosu (KTR için de).
- **Test edilen numara ArduSub çıkışıdır, senin renk etiketinle aynı olmayabilir.**

(Özel `panel/motor_test.py`'nin bu firmware'de çalışması için QGC'nin gönderdiği
tam komutun köprüyle yakalanıp kopyalanması gerekir — ileride yapılabilir.)

### Gerçek test — güç ve güvenlik (ÖNEMLİ)

- **Motorların dönmesi için batarya (motor gücü) AÇIK olmalı.** USB yalnız uçuş
  kontrolcünün beynini besler, ESC/motor güç hattını beslemez. Voltaj 0 / akım
  "-1" görüyorsan batarya bağlı değildir — motorlar komutu alır ama dönmez.
- **Akım tabanlı "çalışıyor" tespiti** ancak güç modülü voltaj/akım raporlarsa
  çalışır (batarya + güç modülü bağlıyken).
- Port: V6X genelde **COM7** (değişebilir; heartbeat ile doğrula). Tek seferde
  porta **tek program** bağlanabilir — QGC veya başka araç açıksa COM'u bırakmaz.

**Elle durdurma katmanları (sorun anında):**
1. **ESC tuşu** veya büyük kırmızı **TÜMÜNÜ DURDUR** butonu → tüm motorlara güç 0.
2. **Otomatik zaman aşımı:** MOTOR_TEST girilen süre sonunda kendiliğinden durur —
   USB kopsa/yazılım donsa bile FC motoru durdurur. Bu yüzden süreyi KISA tut.
3. **Son güvenlik = donanım:** acil-stop butonu / batarya fişi elde. Yazılım ve
   USB tamamen çökse bile bu keser. Suya girmeden bunu elinin altında tut.

Her iki yolla da:

- [ ] Her motoru (1-8) tek tek düşük güçte döndür. Ekrandaki numara ile fiziksel
      motor eşleşiyor mu? (Yanlışsa ESC sinyal kablolarını doğru MAIN OUT'a takın.)
- [ ] Her motorun **itki yönü** diyagramdaki yeşil/mavi renk kodu ile eşleşiyor mu?
      - Yatay (1-4): kovada su akışı doğru yöne mi itiyor?
      - Dikey (5-8): yukarı/aşağı itiş doğru mu?
- [ ] Ters dönen motoru **kablo değiştirmeden** `MOT_x_DIRECTION = -1` ile düzelt
      (x = motor no). Değişiklik sonrası tekrar test et.
- [ ] Sonucu not et: hangi motorlarda DIRECTION=-1 gerekti → parametre dökümüne girer.

> İpucu: itki yönü kontrolü için motoru çalıştırırken kovadaki suyun/kabarcığın
> hareket yönüne bakın, ya da eli (dikkatlice, uzaktan) akış önüne tutun.

## D. Ağ + telemetri (Jetson, BlueOS, panel)

- [ ] Statik IP'ler ayarlı (PC .1, BlueOS .2, Jetson .3). `ping 192.168.2.3` ve
      `ping 192.168.2.2` çalışıyor.
- [ ] Jetson'da `sudo systemctl status mavlink-router` → aktif, FC (`/dev/ttyFC`) bağlı.
- [ ] QGC'de AUV (araç 1) görünüyor; Mini ROV hazırsa araç 2 de.
- [ ] Panel: `python main.py` → telemetri etiketleri doluyor, "Jetson WS: bağlı",
      dikey profil ve harita çiziyor. (Kamera yoksa video "kaynak açılamadı" der, normal.)

## E. ROS 2 yığını (Jetson)

- [ ] `cd ~/auv/ros2_ws && colcon build --symlink-install` hatasız.
- [ ] `ros2 launch auv_bringup bringup.launch.py with_camera:=false` (kamera yoksa).
- [ ] Topic'ler akıyor:
      ```
      ros2 topic echo /mav/attitude        # IMU
      ros2 topic echo /vertical_state       # Bar30+Ping fuzyonu
      ros2 topic echo /mav/gps_info
      ```

## F. Yazılım kuru koşusu — MOTORSUZ (en güvenli mantık testi)

Görev durum makinesinin doğru komutları ürettiğini **motorları döndürmeden**
doğrulayın:

- [ ] Bir terminalde komut çıkışını izle (ARM YOK → motorlar dönmez):
      ```
      ros2 topic echo /mav/manual_control
      ```
- [ ] Panelden `video_pattern` seç → START. `capture_origin` yok, ilk adım geri
      sayım. `/mav/manual_control` içindeki x/y/z/r dizisini izle:
      geri sayım (0,0,500,0) → düz (x>0) → dönüş (r≠0) → daire → ...
- [ ] Pruva bağımlı adımlar (turn/drive düzeltmesi) IMU gerçek yaw verdiği için
      tezgahta **mantıklı** çalışır: kartı elinizle çevirince `turn` adımının r
      komutu buna tepki vermeli.
- [ ] STOP → çıkış nötr (0,0,500,0) olmalı.

> **Not:** `set_depth` adımı tezgahta anlamsızdır — Bar30 masada ~0 m okur, hedef
> derinliğe inmek için sürekli aşağı itki komutlar (30 sn sonra zaman aşımıyla
> geçer). Bu yüzden gerçek desen koşusunu ancak havuzda değerlendirin.

## G. İSTEĞE BAĞLI — motorlu kuru koşu (motorlar kovada, DİKKAT)

Sadece motor tepkilerini görmek isterseniz, **kısa süreli**:

- [ ] Emniyet: kimse kovaya dokunmuyor, e-stop elde.
- [ ] `MANUAL` modda ARM et, panelden bir `turn` içeren kısa görev başlat.
      Kartı çevirince motorların düzeltme yönünde döndüğünü gözle.
- [ ] Bittiğinde DISARM. Uzun süre çalıştırma (ESC ısınır).

---

## Bu aşamada üretilmesi gereken çıktılar

1. Çalışan parametre dökümü → `ardusub_params/auv_v6x_YYYYAAGG.param`
2. `MOT_x_DIRECTION` düzeltmeleri listesi (hangi motorlar)
3. Sensörlerin hepsinin MAVLink'te göründüğü teyidi
4. Panel + QGC + ROS 2 yığını aynı anda sorunsuz bağlanıyor teyidi

Bunlar tamamsa havuza [havuz-kontrol-listesi.md](havuz-kontrol-listesi.md) ile girin.
İlk havuz işi: hız tablosu kalibrasyonu (`scripts/calibrate_speed.py`) ve
DEPTH_HOLD/pruva doğrulaması — desen koşusu ancak bunlardan sonra anlamlı.
