# Ağ Kurulumu

Tek düz L2 ağ (fiber çeviriciler şeffaf), alt ağ `192.168.2.0/24`, DHCP yok.

| Cihaz | IP | Ayar yeri |
|---|---|---|
| Kontrol PC | 192.168.2.1 | Windows: Ethernet adaptörü → statik IP (BlueOS karayı .1 varsayar) |
| Mini ROV (BlueOS) | 192.168.2.2 | BlueOS varsayılanı — değiştirmeyin |
| Jetson | 192.168.2.135 | `config/netplan/01-auv-static.yaml` (arayüz `enP8p1s0`) → `/etc/netplan/` |
| Yedek laptop | 192.168.2.4 | opsiyonel |

## Portlar

| Port | Ne | Yön |
|---|---|---|
| UDP 14550 | MAVLink → QGC (AUV sysid 1 + Mini ROV sysid 2, tek QGC çoklu-araç) | Jetson+RPi4 → PC |
| UDP 14551 | MAVLink → ROS köprüsü (yerel) | Jetson içi |
| UDP 14552 | MAVLink → özel panel | Jetson → PC |
| UDP 5600 | Mini ROV kamera (QGC otomatik açar) | RPi4 → PC |
| UDP 5601 | D435 H.264 → panel | Jetson → PC |
| UDP 5602 | Mini ROV kamera kopyası → panel (BlueOS'ta 2. akış tanımla) | RPi4 → PC |
| TCP 8765 | Panel ↔ mission node WebSocket | PC → Jetson |

## Doğrulama sırası (tezgahta)

1. `ping 192.168.2.135` (Jetson) ve `ping 192.168.2.2` (BlueOS) — fiber zinciri sağlam.
2. Jetson'da `systemctl status mavlink-router` — FC bağlı, uçlar açık.
3. Kontrol PC'de QGC aç → araç 1 (AUV) ve araç 2 (Mini ROV) otomatik görünmeli.
4. Panel `python main.py` → telemetri etiketleri dolmalı, WS "bağlı".
5. Windows Güvenlik Duvarı: QGC ve Python için UDP 14550/14552, 5600-5602 gelen izni.

## Görev 2 (kablo sökme) prosedürü

1. Panelden hedef koordinatları gönder, `task2_nav` seç, START (10 sn geri sayım başlar).
2. Geri sayım sırasında fiberi **bilgisayar tarafından** ayır (şartname: kablo bilgisayardan sökülür).
3. Araç görevi tamamen Jetson üzerinde yürütür; FS_GCS kapalı olduğundan kopma failsafe tetiklemez.
4. Görev bitince (yüzeye çıkış + disarm) kabloyu tekrar tak; telemetri geri gelir.
