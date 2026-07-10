# Havuz Günü Kontrol Listeleri

## Dalış öncesi (her dalışta)

- [ ] O-ring/conta gözle kontrol, vakum testi (varsa)
- [ ] Acil durdurma butonu çalışıyor (motorlar kesiliyor)
- [ ] Batarya gerilimi > eşik, sıkı bağlantı
- [ ] Emniyet halatı bağlı (≥ 50 m kuralı — yarışmada zorunlu)
- [ ] Pervanelerde yosun/ip yok, nozüller sağlam
- [ ] QGC + panel bağlı, tüm telemetri akıyor
- [ ] `ros2 bag record -a` başlatıldı (her koşu kayıt!)
- [ ] Parametre dökümü alındı mı (gün başı)?

## M1 — Islak temel testleri

- [ ] Sızdırmazlık: 10 dk batık, kabarcık yok, nem/sızıntı yok
- [ ] Yüzerlik ~nötr, trim düzgün (hafif pozitif önerilir)
- [ ] ALT_HOLD derinlik tutma: hedef ±0.15 m (adım cevabını bag'den ölç)
- [ ] Pruva tutma: itki altında ±5° (kulvar çizgisiyle karşılaştır)
- [ ] CompassMot yapıldı; tam gazda pruva sapması < 10°
- [ ] Hız kalibrasyonu: gaz {200,300,400,500,600,700} × 10 m × 2 yön
      → `config/speed_table.csv` güncellendi
- [ ] Yaw hız kalibrasyonu: r=250, 500'de dönüş hızı (°/s) ölç
      → `yaw_rate_full_dps` parametresi güncellendi

## M2/M3 — Video deseni koşusu (şartname 2.4.3.3 puanlaması)

- [ ] Başlangıç karesi (1×1 m) yerleştirildi, kamera açısı tüm parkuru görüyor
- [ ] Video kesintisiz, 720p+, araç net görünüyor
- [ ] START sonrası sıfır müdahale (panel/QGC'ye dokunma!)
- [ ] Segmentler ≥ 15 sn (kronometreyle doğrula; betikte 16 sn)
- [ ] Daire ≥ 1 m çap, ≥ 1 tam tur
- [ ] Araç su yüzeyine HİÇ çıkmadı
- [ ] Bitişte araç TAMAMEN kare içinde
- [ ] 3 temiz koşu kaydedildi (yedek çekimler)
- Ayrı çekimler: sızdırmazlık gösterimi (kabarcık yok) + acil stop gösterimi

## Görev 2 provası

- [ ] İki yüzer işaret GPS ile ölçülü noktalara (~50 m ara) yerleştirildi
- [ ] Yüzeyde origin fix alındı (fix 3D, ≥ 6 uydu)
- [ ] Kablo sökme prosedürü prova edildi (geri sayımda ayır)
- [ ] ≥ 3 koşuda yüzeye çıkış hatası ölçüldü: ____ m, ____ m, ____ m
- [ ] Orbit yarıçapı R hataya göre güncellendi (R ≥ maks hata)
- [ ] Akıntı gözlendi ise bias vektörü panelden girildi, tekrar test

## Dalış sonrası

- [ ] Bag dosyaları PC'ye kopyalandı, adlandırıldı (tarih_test_no)
- [ ] Değişen parametreler `ardusub_params/` klasörüne kaydedildi
- [ ] Sorunlar/notlar bu dosyanın altına işlendi
