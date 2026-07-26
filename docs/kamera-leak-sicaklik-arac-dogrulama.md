# Kamera · Sızdırmazlık · Sıcaklık — Araç Üzeri Doğrulama

Bu belge, panelde eklenen/düzeltilen üç konunun **araca bağlı PC'de** yapılması gereken
kısımlarını anlatır. Kodda yapılan kısımlar (kokpit UI + köprü) donanımsız demoda doğrulandı;
aşağıdakiler gerçek donanım gerektirir.

> Kural: Motor/aktüatör tarafına dokunmaz. FC parametre değişiklikleri QGC'den yapılır,
> her değişiklikten sonra tam parametre dump'ı `ardusub_params/`'a kaydedilir.

---

## 1) Kameralar — "ikisi de görüntü vermiyor"

İki kamera **bağımsız** yollardan gelir; ikisinin birden gitmesi genelde ortak bir
sebeptir (panel yüklenmiyor / ağ). Panel artık siyah ekran yerine **nedeni** gösteriyor:
ZED gelmiyorsa merkezde "ZED 2i görüntüsü yok + durum" katmanı çıkar.

### AUV / ZED 2i (Jetson → ROS2 → `/video/zed` MJPEG)
Kokpitteki ZED penceresi `/sensors`'daki `cameras.zed` durumunu gösterir:
`canli (ROS2)` = akıyor · `topic yok` = köprüye kare gelmiyor.

Araçta sırayla kontrol et:
1. Köprü gerçek modda mı? (`--demo` DEĞİL): `python3 ros2_web_bridge.py --port 8000`
2. ZED düğümü çalışıyor mu, topic yayında mı:
   ```bash
   ros2 topic hz /zed/zed_node/rgb/color/rect/image/compressed
   ```
   - Hiç çıktı yoksa: ZED wrapper/Docker konteyneri çalışmıyor ya da topic adı farklı.
     `ros2 topic list | grep zed` ile gerçek adı bul; farklıysa
     `ros2_web_bridge.py` içindeki `COMPRESSED_TOPICS['zed']`'i güncelle.
   - Konteynerden host'a **compressed** geçmiyorsa (ham 720p SHM sınırına takılır),
     ZED wrapper'da `rgb/color/rect/image` için `compressed` transport açık olmalı.
3. Panelde `http://<jetson>:8000/` aç; ZED penceresinde katman "topic yok" diyorsa sorun
   yukarıdaki 1-2 adımındadır (panel değil).

### Mini ROV (tarayıcı → WebRTC → BlueOS `.2:6021`)
`minirov.html` doğrudan `ws://192.168.2.2:6021`'e bağlanır (mavlink-camera-manager).
1. Kontrol PC'den `.2`'ye erişim var mı: `ping 192.168.2.2`
2. Signalling portu doğru mu? Varsayılan 6021. Farklıysa iframe'e parametre geç:
   panelde Mini ROV başlığındaki tam-ekran bağını `‹/minirov?port=XXXX›` ile aç ya da
   `cockpit.html`'de `#cam-minirov` iframe `src`'sine `?port=` ekle.
3. Producer adı/id: `minirov.html` içinde `TARGET_NAME="MiniROV-Web"`. BlueOS'ta yayın adı
   farklıysa tek kaynak varsa yine seçilir; birden çok kaynak varsa adı eşleştir.
4. Mini ROV penceresinin sol-üst HUD'u "Bağlanıyor / Oynatılıyor / Hata" durumunu gösterir —
   hangi adımda takıldığını oradan oku.

---

## 2) Sızdırmazlık (Leak) — AUX1, CUAV V6X

**Bulgu:** ArduSub sızıntıyı yalnız **olay anında** `STATUSTEXT "Leak Detected"` ile bildirir;
sürekli "kuru/ıslak" durumu **yoktur**. Köprü bunu zaten doğru yakalıyor (mandallı). Panel de
artık: sızıntı yoksa **"izleniyor"** (bağlantı var), sızıntıda **"⚠ SIZINTI"** gösterir —
"kuru garantisi" iddia etmez.

**Muhtemel kök neden:** `ardusub_params/auv_v6x_baseline.param`'da `FS_LEAK_ENABLE=2` var ama
`LEAK1_PIN` **tanımlı değildi**. Pin tanımlı değilse ArduSub hiç "Leak Detected" yollamaz.

Araçta (QGC > Parameters):
1. `LEAK1_PIN` = AUX1'in GPIO pin no'su (parametre açıklamasındaki listeden CUAV V6X/AUX1).
   AUX1'i GPIO yapmak için ilgili `SERVOx_FUNCTION=0` gerekebilir.
2. `LEAK1_LOGIC` = kuru durumdaki mantık seviyesi (Blue Robotics SOS: tipik kuru=LOW → `0`).
3. **Kabul testi (tezgah):** leak probunu ısla → QGC'de "Leak Detected" görülmeli **ve**
   kokpit header'da "⚠ SIZINTI" (mandallı, kırmızı) yanmalı, olay günlüğüne düşmeli.

Detaylı parametre notları: `ardusub_params/auv_v6x_baseline.param` (Leak bölümü).

---

## 3) Sıcaklık — CUAV ADC analog sensörü + 90°C uyarısı

**Kodda yapıldı (demoda doğrulandı):** kokpit header AUV sıcaklık çipi artık **CUAV ADC
iç/kart sıcaklığını** (`board_temp_c`, Bar30 su sıcaklığından ayrı) gösterir; hem AUV hem
Mini ROV için **≥90°C** olduğunda çip kırmızı + yanıp söner, üst banner çıkar, olay günlüğüne
kenar-tetikli kayıt düşer. Eşik `?tempwarn=NN` ile geçici değiştirilebilir (varsayılan 90).

Veri yolu: analog sensör → FC → `SCALED_PRESSURE3.temperature` →
`bridge_node.py h_pressure3` → `/mav/board_temp` → köprü `board_temp_c` → kokpit.

Araçta (QGC > Parameters):
1. `TEMP1_TYPE=5` (Analog), `TEMP1_PIN=<CUAV ADC pin no>` (açıklamadaki listeden doğrula).
2. `TEMP1_A0..A4` = sensör datasheet'ine göre volt→°C polinom katsayıları.
3. `TEMP1_SRC` = okumanın **MAVLink'e çıkması için** gerekli (varsayılanda yalnız loglanır,
   panele gelmez).
4. **Kaynak doğrulama (kritik):** sensörü elinle/fönle ısıt, QGC **MAVLink Inspector**'da
   hangi mesajın `temperature` alanının değiştiğini izle:
   - `SCALED_PRESSURE3` ise → hazır, köprü onu okuyor. Panelde AUV sıcaklık çipi yükselmeli.
   - Başka mesaj/alan ise → bana söyle, `bridge_node.py` içindeki `h_pressure3`'ü o kaynağa
     göre güncelleyeyim (tek satır handler + `handlers` sözlüğüne ekleme).
5. **Bar30 ayrımı:** `board_temp_c` = ADC iç sıcaklık (90°C mantıklı); `water_temp_c` =
   Bar30 su sıcaklığı (~15-25°C). İkisi popover'da ayrı görünür.

### Mini ROV sıcaklığı
`mrov_temp_c`, BlueOS mavlink2rest `SCALED_PRESSURE2.temperature`'dan gelir (Navigator/Bar30).
90°C uyarısı aynı mantıkla onun için de çalışır. Doğrulama: BlueOS'ta veya
`http://192.168.2.2:6040/mavlink/.../SCALED_PRESSURE2` GET ile `temperature` alanını gör.

---

## Hızlı demo doğrulaması (donanımsız, referans)
```bash
cd jetson && python3 ros2_web_bridge.py --demo --port 8000
# http://localhost:8000  → AUV sıcaklık çipi ~40 sn'de bir 90°C'yi aşar (üçgen dalga),
#   çip kırmızı+yanıp söner, üst banner + olay günlüğü kaydı görülür.
#   (Demo değeri kasıtlı; gerçekte /mav/board_temp'ten gelir.)
```

---

## DURUM & PLAN — 2026-07-16 saha oturumu

Canlı araçta (Jetson `.135`, BlueOS `.2`) teşhis + kısmi çözüm yapıldı.

### ✅ Tamamlandı ve canlı doğrulandı
- **ZED (AUV) kamera:** Kök neden USB kopması (`dmesg`: disconnect + USB2'ye düşme → `Argus: Cannot create camera provider` → node segfault). `nvargus-daemon` tazelendi + `ros2 launch auv_bringup zed2i_auv.launch.py` yeniden başlatıldı → compressed topic ~28 Hz, `cameras.zed="canli"`, panelde görüntü var.
- **Mini ROV leak parse bug:** `ros2_web_bridge.py` mavlink2rest `text`'i karakter-listesi (`["L","e",...]`) döndürünce eski kod (int bekliyordu) metni eliyordu → `mrov_leak` hep null. Düzeltildi, deploy edildi, doğrulandı (`mrov_leak=True`).
- **Kokpit UI:** 90°C aşım uyarısı + kamera arıza katmanı + leak "izleniyor" + `board_temp_c` alanı. Deploy edildi (`~/auv/jetson/`, yedek `.bak-20260716-220433`).

### 🅿️ BACKLOG — fiziksel/saha dokunuşu gerektirir (ES GEÇİLDİ)
1. **ZED USB3 dayanıklılık:** Şu an stabil ama kök neden fiziksel. Kısa/kaliteli USB3 kablo, hub yerine doğrudan Jetson portu, güç (brownout) kontrolü. Motor akımı/titreşimle tekrar düşebilir.
2. **Leak 5V↔3.3V uyumu:** LM358 çıkışı 5V; A2 = FMU PWM çıkış 2 = **3.3V GPIO** (ArduPilot pin ~51). 5V'u doğrudan vermek pini riske atar. Gerilim böler (5V→3.3V) ekle VEYA sinyali 6.6V ADC (pin 12) girişine al. **Bu yapılmadan leak pini etkinleştirilse bile ıslatma testi riskli.**
3. **Leak ıslatma testi:** Devre + pin + logic doğrulaması (probu ıslat → "Leak Detected" gelmeli).
4. **Sıcaklık ısıtma testi:** Sensörü ısıt, 90°C uyarısını gerçek veriyle gör; MAVLink Inspector'da hangi mesaja düştüğünü doğrula.

### ▶️ Sıradaki adımlar — donanım dokunuşu YOK (yapılabilir)
- **Mini ROV kamera yayını:** Kamera BlueOS'ta ALGILANIYOR ("H264 USB Camera"); sadece stream tanımsız. BlueOS → Video Streams'ten WebRTC yayını ekle (ad `MiniROV-Web`, H264, 1280x720@30). Fiziksel değil.
- **Bayat Mini ROV leak temizliği:** Navigator'ı BlueOS'tan reboot (mavlink2rest önbelleği temizlensin) → köprü restart → `mrov_leak` "izleniyor"a döner.
- **AUV leak pini:** QGC `LEAK1_PIN` = A2 karşılığı (şu an **-1/kapalı**), `LEAK1_LOGIC` = ters devre için ayarla. (Voltaj böler backlog'da.)
- **Sıcaklık:** Önce **sensörün tipini netleştir** (aşağı). Sonra QGC + `bridge_node.py` (SCALED_PRESSURE3 handler) rebuild + mav_bridge restart (aktüatör-komşusu — disarm/güvenli durumda, onayla).

### FC mevcut değerleri (2026-07-16 okundu, salt-okunur)
```
LEAK1_PIN = -1  (KAPALI)     LEAK1_LOGIC = 0     FS_LEAK_ENABLE = 1 (warn-only)
TEMP1_TYPE = 1 (TSYS01!)     TEMP1_PIN = (yok)   TEMP1_SRC = 0 (None)   TEMP1_SRC_ID = -1
```

### ❓ Açık soru (sıcaklık için kilit — fiziksel değil, tanımlama)
FC'de `TEMP1_TYPE=1` = **TSYS01 (I2C dijital)** ayarlı, ama sen "analog / ADC portu" demiştin. Sensör gerçekte hangisi?
- **I2C TSYS01/Celsius** (küçük kart, 4 telli I2C) ise → `TEMP1_SRC`'yi ayarla + I2C algılamasını doğrula.
- **Analog termistör/LM35/TMP36** (2-3 telli, ADC pini) ise → `TEMP1_TYPE=5`, `TEMP1_PIN`, polinom katsayıları.

