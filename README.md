# AUV Ana Bilgisayar Yazılımı (Jetson Orin NX 16GB)

Otonom su altı aracının (AUV) ana bilgisayar yazılımı. ROS 2 Humble + Docker
mimarisi üzerine kuruludur. İlk entegre sensör: **ZED 2i** stereo kamera
(görsel-ataletsel odometri, derinlik algılama, 3D haritalama, lokalizasyon).

Sürüm matrisi (resmi kaynaklardan doğrulandı):
**L4T r36.5 (JetPack 6.2.x) · ZED SDK 5.4.0 · zed-ros2-wrapper v5.4.0 ·
ROS 2 Humble · Taban imaj `stereolabs/zed:5.4.0-devel-l4t-r36.5`**

## Dizin Yapısı

```
auv/
├── docker/                  # Konteyner mimarisi
│   ├── Dockerfile.zed       # ZED 2i + ROS 2 Humble imajı
│   ├── compose.yaml         # Servisler (zed; ileride: pixhawk, sonar, navigation)
│   └── ros_entrypoint.sh
├── ros2_ws/src/
│   ├── auv_bringup/         # AUV'a özel launch + config + map_saver düğümü
│   └── zed-ros2-wrapper/    # Resmi Stereolabs sarmalayıcı (v5.4.0)
├── maps/                    # Kalıcı çıktılar: .area (lokalizasyon) + .ply (3D harita)
├── scripts/
│   ├── install_host.sh      # SUDO: CUDA/TensorRT + Docker + nvidia-container-toolkit
│   ├── check_zed_usb.sh     # ZED 2i USB3 bağlantı tanısı
│   ├── build_native.sh      # Docker'sız derleme (host kurulumundan sonra)
│   └── run_native.sh        # Docker'sız çalıştırma
└── docs/ARCHITECTURE.md     # Mimari kararlar ve sensör füzyon planı
```

## Kurulum (bir kez)

```bash
# 1. Host bileşenleri (şifre ister)
sudo bash scripts/install_host.sh
#    ardından oturumu kapat/aç (docker grubu için)

# 2. ZED 2i'nin USB 3.0'a bağlı olduğunu doğrula
bash scripts/check_zed_usb.sh

# 3. İmajı derle (ilk sefer imaj indirme dahil uzun sürer)
docker compose -f docker/compose.yaml build zed
```

## Çalıştırma

```bash
docker compose -f docker/compose.yaml up -d zed
docker compose -f docker/compose.yaml logs -f zed
```

### Doğrulama (host'ta, network_mode: host sayesinde)

```bash
source /opt/ros/humble/setup.bash
# Konteyner root, host kullanıcı → FastDDS paylaşımlı bellek uyuşmaz;
# host tarafında UDP taşıması zorunlu (yoksa topic listelenir ama veri akmaz):
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic list | grep zed
ros2 topic hz  /zed/zed_node/rgb/color/rect/image        # RGB (v5.1.0+ isim şeması)
ros2 topic hz  /zed/zed_node/depth/depth_registered      # derinlik haritası
ros2 topic echo /zed/zed_node/depth/depth_info --once    # derinlik istatistikleri
ros2 topic hz  /zed/zed_node/confidence/confidence_map   # piksel hata payı haritası
ros2 topic echo /zed/zed_node/pose --once                # konum (map çerçevesi)
ros2 topic echo /zed/zed_node/odom --once                # odometri (drift analizi)
ros2 topic hz  /zed/zed_node/mapping/fused_cloud         # 3D harita (fused point cloud)
```

### Harita / lokalizasyon işlemleri

```bash
# 3D haritayı .ply olarak kaydet (auv_bringup map_saver düğümü; ayrıca her 60 sn'de
# maps/fused_map_latest.ply otomatik güncellenir):
ros2 service call /map_saver/save_map std_srvs/srv/Trigger

# Area memory'yi (lokalizasyon haritası) elle kaydet:
ros2 service call /zed/zed_node/save_area_memory zed_msgs/srv/SaveAreaMemory \
  "{area_file_path: '/data/maps/auv_area.area'}"

# Odometriyi sıfırla / pozisyon takibini resetle:
ros2 service call /zed/zed_node/reset_odometry std_srvs/srv/Trigger
ros2 service call /zed/zed_node/reset_pos_tracking std_srvs/srv/Trigger
```

### ⚠️ Tracking modu takası (SDK 5.4.0'da deneysel olarak doğrulandı)

| Mod | VIO/Odometri | 3D haritalama (fused cloud) | Area memory (relocalization) |
|---|---|---|---|
| `GEN_1` (varsayılan) | ✅ | ✅ | ❌ ("FILE EMPTY") |
| `GEN_3` | ✅ | ❌ (hep 0 nokta — SDK sorunu) | ✅ |

```bash
# Haritalama görevi (varsayılan): 3D piksel haritası üretir
ros2 launch auv_bringup zed2i_auv.launch.py
# Önceden haritalanmış ortamda kendini konumlama:
ros2 launch auv_bringup zed2i_auv.launch.py tracking_mode:=GEN_3
```

- GEN_3 modunda kapanışta area memory `maps/auv_area.area`'ya otomatik kaydedilir,
  sonraki GEN_3 açılışında otomatik yüklenir → relocalization.
- **Area dosyaları nesile özgüdür**: GEN_3 ile kaydedilen dosyayı GEN_1 yükleyemez;
  node "Unable to load area file" ile çöker ve konteyner yeniden başlatma
  döngüsüne girer. Çözüm: `mv maps/auv_area.area maps/auv_area.area.bak`

## Bilinen Gereksinimler / Notlar

- ZED 2i **USB 3.0** ister; USB 2.0'da kamera arayüzü hiç numaralanmaz
  (yalnızca 2b03:f881 HID görünür). `scripts/check_zed_usb.sh` ile test edin.
- SDK 5.x'te tüm derinlik modları NEURAL ailesidir (NONE/NEURAL_LIGHT/NEURAL/
  NEURAL_PLUS) ve TensorRT gerektirir → `install_host.sh` kurar (native için);
  Docker imajında hazır gelir.
- İlk açılışta AI modeli cihaz için optimize edilir (dakikalar sürebilir);
  sonuç `zed_resources` volume'unda önbelleklenir, tek seferliktir.
- Native derleme `find_package(CUDA 12.6 REQUIRED)` nedeniyle CUDA toolkit ister
  (resmi README, JP6 notu: `nvidia-jetpack` + `nvidia-jetpack-dev`).
- GNSS füzyonu (global localization) hazır ama **kapalı**. Açmak YAML'dan
  DEĞİL launch argümanından yapılır (launch argümanları YAML'ı ezer):
  `ros2 launch auv_bringup zed2i_auv.launch.py enable_gnss:=true`
  (compose'da `command:` satırına eklenir). F9P GPS sürücüsü entegre olunca.
- Docker servisi root olarak yazdığı için `maps/` içindeki dosyalar root sahipli
  olur; native çalıştırmaya geçmeden önce: `sudo chown -R $USER ~/auv/maps`
