# CLAUDE.md — AUV + Mini ROV Yazılım Sistemi

TEKNOFEST 2026 insansız su altı yarışması yazılımı: **AUV** (8 motor, Jetson Orin NX +
CUAV V6X, ArduSub) ve **Mini ROV** (RPi4 + Navigator, BlueOS). Kontrol PC tarafı:
QGroundControl + özel **web paneli**.

---

## 📌 AKTİF GÖREV — Web arayüzünü baştan yeniden tasarla (DONANIM GEREKMEZ)

Bu görev tamamen **yerelde, donanımsız** yapılır (`--demo` modu). Kartlara/araca bağlanmaya
gerek yok. Amaç: web kontrol panelini **modern, tutarlı ve düzenli** biçimde baştan elden
geçirmek.

### ⚠️ ÖNCE BUNU YAP (zorunlu, kod değiştirmeden)
1. Bu depoyu ve bu görevi **nasıl anladığını Türkçe özetle** ve kullanıcıya (takım arkadaşı) sun.
2. Kısa bir **plan** ver: hangi sayfaları, hangi sırayla, hangi ortak stil/bileşenlerle
   yenileyeceğini.
3. **Açık onay bekle.** Kullanıcı "başla/onaylıyorum" demeden **hiçbir dosyayı değiştirme.**

### Kapsam — yenilenecek sayfalar (hepsi web paneli)
| Sayfa | Dosya | URL |
|---|---|---|
| Panel (dashboard) | `jetson/dashboard.html` | `/` |
| Kontrol | `web/templates/control.html` | `/control` |
| Motor Test | `jetson/motortest.html` | `/test` |
| Mini ROV | `jetson/minirov.html` | `/minirov` |

Sunucu: `jetson/ros2_web_bridge.py` (saf Python `http.server`, port 8000). Sayfalar aynı
klasörden servis edilir (`control.html` şu an `web/templates/`'te — `/control`'ü demo'da
görmek için `jetson/`'a kopyalaman gerekebilir).

### Nasıl yerelde çalıştırıp önizlersin (donanımsız)
```bash
pip install pillow          # demo sahte kamera görüntüsü için
cd jetson
python3 ros2_web_bridge.py --demo      # http://localhost:8000  (sahte sensör verisi)
```
- Saf düzen/CSS için `.html` dosyalarını doğrudan tarayıcıda da açabilirsin.
- Demo modda AUV WebSocket (`:8765`) YOK — Kontrol/Motor Test sayfalarındaki buton/sürgü
  aksiyonları yanıt vermez ama **düzen, stil, akış** tam görünür (tasarım için yeterli).

### 🔒 BOZMA — arka uç sözleşmeleri (deploy edilince canlı backend ile uyum şart)
Yalnız **sunum/tasarımı** değiştir; şu kontratları **AYNEN KORU** (mevcut JS'ten oku):
- `/sensors` JSON anahtarları (dashboard bunları çeker).
- WebSocket `:8765` komut adları: `motor_test`, `motor_test_stop`, `direct_output`,
  `direct_output_stop`, ayrıca control.html'deki arm/disarm/mode/goto/mission komutları.
- Endpoint yolları: `/`, `/control`, `/test`, `/minirov`, `/sensors`, `/minirov/motor`.
- Motor diyagramı yerleşimi (`MOTORS` dizisi) ve PWM anlamı (**1100 geri · 1500 dur ·
  1900 ileri**) değişmez.

### Hedefler (öneri)
- Tutarlı tema/tipografi/renk; ortak header+nav; tekrar eden CSS'i tek yerde topla.
- Duyarlı (responsive) + gerektiğinde **scrollsuz viewport** düzeni (`motortest.html`'de
  yapıldı: `body{height:100vh;display:flex;flex-direction:column;overflow:hidden}` +
  `main` grid + kart-içi `overflow:auto`). Aynı deseni diğer sayfalara uygula.
- Güvenlik uyarıları (motorlar fiziksel döner) korunur/iyileştirilir.

---

## Mimari (özet)
- **Jetson** (ArduSub'a USB ile bağlı CUAV V6X): ROS 2 Humble düğümleri + `mavlink-router`
  (seri portu QGC ve köprü arasında paylaştırır).
- **`ros2_web_bridge.py`** (:8000): ROS2 topic'lerini + kameraları web'e sunar; HTML sayfaları
  servis eder.
- **`auv_mav_bridge`** (`ros2_ws/src/...`): pymavlink ↔ ROS2 köprüsü (MAVROS yok). Motor test,
  doğrudan çıkış, sensör telemetri.
- **`auv_mission`**: görev durum makinesi + panel WebSocket sunucusu (`:8765`).
- Diğer: `auv_vertical_state`, `auv_dead_reckoning`, `auv_lane_follow`, `auv_video`, `auv_bringup`.

## Dizinler
| Dizin | İçerik |
|---|---|
| `jetson/` | Web paneli sayfaları + `ros2_web_bridge.py` (canlı arayüz, :8000) |
| `web/` | Flask tabanlı alternatif panel (`app.py`, `templates/`) + `config.py` |
| `ros2_ws/src/` | ROS 2 paketleri (Jetson'da koşar) |
| `panel/` | PySide6 masaüstü panel + bağımsız `motor_test.py` |
| `ardusub_params/`, `config/`, `scripts/`, `docs/` | Parametreler, ağ/router, kurulum, doküman |

## Kurallar / konvansiyonlar
- Kod yorumları ve UI metinleri **Türkçe**.
- Donanımsız görevlerde `ros2_ws` derlemesi/çalıştırması gerekmez; UI için sadece
  `ros2_web_bridge.py --demo` yeter.
- Donanıma dokunan işler (FC parametreleri, canlı motor/sensör testi, Jetson deploy) **bu
  depoda geliştirilir ama kartlara bağlı PC'de** yürütülür — sende donanım yoksa yapma.
