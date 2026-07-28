# Saha Kontrol — tek komut

Yarışma sahasında kontrol PC'sinden **tek kelime** ile her şeyi kontrol et:

```
saha
```

Bu komut Jetson'a **SSH'ı kendi yapar** ve bir **menü** açar. Sen sadece numara seçersin:

```
==================================================
            AUV  SAHA  KONTROL
==================================================
   1) Haberleşme kontrolü   (kartlar + FC + köprü)
   2) Sensör verileri       (var/yok + neden)
   3) Sensör — takılı olanları seç (-i)
   4) Tam test              (haberleşme + sensör + log)
   5) Jetson kabuğu         (Linux komutları)
   q) Çıkış
==================================================
Seçim:
```

- **1** → haberleşme raporu → Enter → menüye döner
- **2** → sensör verileri (BAĞLI·VERİ AKIYOR / BAĞLANTI VAR AMA VERİ GELMİYOR / BAĞLANTI YOK)
- **q** → çıkış (kontrol PC'ye döner)

`cd` yok, `.\` yok, elle `ssh` yok. Her rapor Jetson'da otomatik loglanır (`~/auv/logs/`).

## Kurulum (bir kez yapıldı)
- `saha\` klasörü Windows PATH'ine eklendi → her dizinde `saha` çalışır.
- **Önemli:** PATH değişikliği sadece **yeni açılan** terminallerde geçerli. Mevcut
  pencerede çalışmazsa, yeni bir PowerShell/Windows Terminal aç ve `saha` yaz.

## Nasıl çalışır
```
Kontrol PC:  saha
     │  (saha.cmd → ssh -t qayra@192.168.2.135 "bash ~/auv/scripts/saha_menu.sh")
     ▼
Jetson'da menü açılır → seçim → rapor → Enter → menü → q
     ▼
Kontrol PC'ye dönersin
```

## Ayarlar
- Jetson adresi: `saha.cmd` içinde (`qayra@192.168.2.135`).
- Menü ve testler Jetson'da: `~/auv/scripts/saha_menu.sh` + `~/auv/scripts/health_check.sh`.

## Geri alma (istersen)
PATH'ten çıkarmak için PowerShell'de:
```powershell
$d="C:\Users\yasin\auv-mini-rov-system\saha"; $u=[Environment]::GetEnvironmentVariable('Path','User'); [Environment]::SetEnvironmentVariable('Path', (($u -split ';' | Where-Object {$_ -ne $d}) -join ';'),'User')
```
