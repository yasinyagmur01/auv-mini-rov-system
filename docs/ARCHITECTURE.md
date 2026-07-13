# AUV Yazılım Mimarisi

## Hedef Sistem

| Bileşen | Rol |
|---|---|
| Jetson Orin NX 16GB | Ana bilgisayar: AI, rota planlama (D* Lite), sensör füzyonu |
| CUAV Pixhawk V6X | Uçuş kontrol kartı: 3× IMU + manyetometre, motor/ESC sürme |
| ZED 2i (4mm) | Stereo kamera: VIO, derinlik, 3D haritalama, dahili IMU/baro |
| Blue Robotics Ping Sonar | Akustik mesafe (zemin/engel) |
| Blue Robotics Bar30 | Batimetre — su derinliği (Y ekseni ground truth) |
| DroneCAN F9P GPS | Yüzeyde son konum sabitleme (dalış öncesi) |
| 8× ESC'li itici, 2× fener, sıcaklık + sızıntı sensörleri | Tahrik ve güvenlik |

## Katmanlı Konteyner Mimarisi (discrete servisler)

Her sensör/işlev grubu kendi konteynerinde çalışır; ortak DDS (host ağı)
üzerinden ROS 2 topic'leriyle konuşurlar. Bu, hata ayıklamayı ve tek tek
yeniden başlatmayı kolaylaştırır:

```
┌─────────────────────────── Jetson Orin NX ───────────────────────────┐
│  docker compose (docker/compose.yaml)                                │
│  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌──────────────────┐  │
│  │ zed        │ │ pixhawk    │ │ sonar      │ │ navigation       │  │
│  │ (BU AŞAMA) │ │ (planlı)   │ │ (planlı)   │ │ (planlı)         │  │
│  │ VIO, depth │ │ MAVROS/    │ │ Ping +     │ │ robot_localization│  │
│  │ 3D harita  │ │ uXRCE-DDS  │ │ Bar30      │ │ EKF + D* Lite    │  │
│  └─────┬──────┘ └─────┬──────┘ └─────┬──────┘ └────────┬─────────┘  │
│        └───────────────┴─── ROS 2 Humble / DDS ────────┘            │
└──────────────────────────────────────────────────────────────────────┘
```

## ZED 2i Modül Eşlemesi (bu aşamada etkin)

| İstenen yetenek | ZED SDK modülü | ROS 2 karşılığı |
|---|---|---|
| Depth sensing (Sonar/Bar30 destekli hata payı analizi) | Depth (NEURAL_LIGHT) + confidence | `~/depth/depth_registered`, `~/confidence/confidence_map`, `~/depth/depth_info` |
| Visual odometry (haberleşmesiz seyir) | Positional Tracking (VIO: stereo + dahili IMU) | `~/odom`, `~/pose`, `~/pose_with_covariance`, TF `odom→base` |
| Localization (yeniden konumlanma) | Area Memory (`.area` dosyası, loop closure) | `save_area_memory` servisi, `area_file_path` parametresi |
| Global localization | GNSS Fusion (**şimdilik kapalı**; F9P entegre olunca NavSatFix beslenecek) | `gnss_fusion.gnss_fusion_enabled` |
| 3D piksel haritalama | Spatial Mapping — FUSED_POINT_CLOUD (5cm çözünürlük) | `~/mapping/fused_cloud` |

## Sensör Füzyon Planı (sonraki aşamalar)

1. **pixhawk servisi**: CUAV V6X → micro-XRCE-DDS/MAVROS ile 3 IMU +
   manyetometre verisi ROS 2'ye akar.
2. **navigation servisi**: `robot_localization` EKF —
   girdiler: ZED odom (+kovaryans), CUAV IMU'ları, Bar30 (Z ekseni mutlak),
   Ping sonar (zemin mesafesi). ZED ve CUAV IMU'ları birbirinin driftini
   sıfırlayacak şekilde tamamlayıcı filtre yapısı.
3. **D* Lite planlayıcı**: dalış öncesi GPS sabiti + hedef koordinatı ile
   ilk rota; seyir sırasında fused odometri ile sürüklenme telafisi.
4. **Derinlik teyit sistemi**: ZED depth vs Sonar/Bar30 karşılaştırması →
   piksel başına hata payı/kayma metriği (ZED confidence haritası temel alınır).

## Kritik Teknik Kararlar

- **Taban imaj `stereolabs/zed:5.4.0-devel-l4t-r36.5`**: Resmi Stereolabs imajı;
  host L4T (r36.5.0) ile birebir eşleşir (Stereolabs şartı) ve hosttaki SDK ile
  aynı sürümü (5.4.0) + CUDA 12.6 + TensorRT 10.3'ü içerir. ROS 2 Humble üzerine
  apt binarileriyle kurulur (Ubuntu 22.04 tabanı Humble ile doğal uyumlu).
  Alternatif (dustynv/ros r36.4 tabanı) araştırıldı ve elendi: L4T minor sürüm
  uyumsuzluğu riski taşıyor.
- **Harita depolama `map_saver` düğümü**: wrapper v5.4.0'da save_3d_map servisi
  bulunmadığından (kaynak koddan doğrulandı) fused point cloud'u .ply'ye yazan
  özel düğüm eklendi: Trigger servisi + 60 sn'de bir atomik otomatik kayıt.
- **Area memory dosyası volume'da** (`maps/`): konteyner yeniden kurulsa bile
  lokalizasyon haritası kaybolmaz.
- **AI model önbelleği volume'da** (`zed_resources`): model optimizasyonu
  (dakikalar sürer) tek sefer yapılır.
- **ZED 2i su geçirmez DEĞİLDİR** (IP66): muhafaza içine düz optik pencere ile
  yerleştirilecek; su-cam-hava kırılması stereo derinlikte ölçek hatası yaratır —
  su altında kamera yeniden kalibrasyonu (custom calibration) planlanmalıdır.
