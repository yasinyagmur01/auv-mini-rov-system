# Takım Hızlı Başlangıç — Donanımsız Geliştirme (Arayüz)

Bu rehber, **kartlara/araca bağlı OLMAYAN** bir PC'de (arkadaşın PC'si) repodan çekip
arayüz (UI) geliştirmek içindir. Kartlara bağlı PC'de yapılacak donanım işleri ayrıdır.

## 1. Kurulum
```bash
git clone https://github.com/yasinyagmur01/auv-mini-rov-system.git
cd auv-mini-rov-system
pip install pillow        # web köprüsü demo modu için (sahte kamera görüntüsü)
```
> Python 3 yeter. ROS 2 / colcon **gerekmez** (UI işi için).

## 2. Web panelini demo modunda çalıştır (donanımsız)
```bash
cd jetson
python3 ros2_web_bridge.py --demo
```
Tarayıcıda aç:
- `http://localhost:8000/`         → Panel (dashboard)
- `http://localhost:8000/test`     → Motor Test
- `http://localhost:8000/minirov`  → Mini ROV
- `http://localhost:8000/control`  → Kontrol *(bu sayfa `web/templates/control.html`;
  demo'da görünmesi için dosyayı `jetson/control.html`'e kopyala)*

Demo modda **sahte sensör verisi** akar (düzen/stil için yeterli). AUV WebSocket (`:8765`)
demo'da yok; buton/sürgü aksiyonları yanıt vermez ama tasarım tam görünür.

Saf CSS/düzen için `.html` dosyalarını tarayıcıda **doğrudan** da açabilirsin.

## 3. Ne üzerinde çalışılacak (arayüz)
Sayfalar: `jetson/dashboard.html`, `jetson/motortest.html`, `jetson/minirov.html`,
`web/templates/control.html`. Hepsi tek dosyada (inline CSS+JS).

**Örnek alınacak desen:** `jetson/motortest.html` scrollsuz (viewport'a sığan) düzene
çevrildi — aynı yaklaşımı Panel ve Kontrol sayfalarına uygula:
`body{height:100vh;display:flex;flex-direction:column;overflow:hidden}` + `main` grid +
kart-içi `overflow:auto`.

## 4. BOZMA (canlı backend ile uyum için)
Sadece **görünüm/tasarımı** değiştir, şu kontratları **aynen koru**:
- `/sensors` JSON anahtarları, WebSocket `:8765` komut adları
  (`motor_test`, `direct_output`, `direct_output_stop`, arm/mode/goto...), endpoint yolları.
- Motor diyagramı (`MOTORS` dizisi) ve **PWM anlamı: 1100 geri · 1500 dur · 1900 ileri**.

## 5. Akış (nasıl canlıya gider)
```
sen (kendi PC) → geliştir + demo'da önizle → commit → git push
        → donanıma bağlı PC ekibi Jetson'a deploy eder → canlı doğrular
```
Sen Jetson'a **deploy edemezsin** (araç ağında değilsin) — push et, deploy'u donanım ekibi yapar.

## 6. Claude Code kullanıyorsan
Repo kökündeki **`CLAUDE.md`** görevi ve kuralları içerir. Claude ilk olarak **ne anladığını
özetleyip senden onay isteyecek** — planı gözden geçir, onayla, sonra başlasın.

## 7. Commit kuralı
Kendi git kimliğinle commit'le. (Örn. `git config user.name "Ad Soyad"`,
`git config user.email "..."`.)
