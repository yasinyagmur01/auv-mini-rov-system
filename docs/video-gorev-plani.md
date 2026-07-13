# 27 Temmuz Video Görevi Planı (İleri Kategori)

**Son teslim: 27 Temmuz 2026, 17:00 — YouTube'a yüklenmiş olacak.**

## Şartname gereksinimleri (2.4.3)

1. **Sızdırmazlık:** araç tamamen batırılır, yüzeye kabarcık çıkmadığı gösterilir.
2. **Acil durdurma:** butona basılınca tüm motorların durduğu ve sistemin
   kapandığı gösterilir (bataryalı araç zorunlu).
3. **Otonom görev deseni** (kesintisiz, müdahalesiz, tamamı su altında):
   - 1×1 m kare başlangıç alanı videoda net görünmeli
   - ≥15 sn düz ileri → sağa 90° → ≥15 sn düz → **kendi etrafında ≥1 tur,
     ≥1 m çaplı daire** → ≥15 sn düz → sağa 90° → ≥15 sn düz
   - Bitişte araç **tamamen** başlangıç karesi içinde
4. Video: kesintisiz (kurgu/kesme yok), ≥720p, 1–5 dk, YouTube.

## Rota geometrisi (neden 1.25 tur?)

2 adet 90° dönüş + 4 eş süreli düz bacakla rota ancak dairenin kendisi de
+90° net dönüş bırakırsa kapanır (şartnamedeki şema kare parkur gösteriyor).
`spin turns: 1.25` = 450° — "en az 1 tur" kuralını sağlar ve rotayı kapatır.
**Provada hakem yorumuna göre gerekirse `turns: 1.0` + ayrı `turn 90` yapılabilir
(bu durumda daire ile düz gidiş arasına dönüş girer — kural metnine göre riskli,
tercihen 1.25).**

## Çekim planı

- Tek kesintisiz çekimde: acil stop gösterimi → sızdırmazlık → desen koşusu
  (veya kurallara göre ayrı videolar kabul ediliyorsa böl — şartname tek
  kesintisiz video diyor, tek planda çekin).
- Kamera: havuz kenarından geniş açı; su berraksa su altı aksiyon kamerası
  İKİNCİ açı olarak (ana video kesintisiz kalmalı).
- START panelden verilir, 10 sn geri sayım vardır → eller çekilir; bataryalı
  ve müdahalesiz koşu. Güvenlik halatı gevşek bağlanabilir (kurala uygun).
- Süre ayarı: `config/missions/video_pattern.yaml` içinde 16 sn (15 sn kural
  + 1 sn pay). Derinlik 0.5 m — havuz derinliğine göre ayarlayın.

## Yayın kontrolü

- [ ] Video 720p+ ve 1-5 dk arasında mı?
- [ ] Kesme/kurgu YOK (tek plan)
- [ ] Kare, araç ve tüm manevralar net görünüyor
- [ ] YouTube'a yüklendi (liste dışı değil, erişilebilir), link KYS'ye girildi
- [ ] Yedek çekimler saklandı
