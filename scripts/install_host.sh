#!/usr/bin/env bash
# ============================================================================
# AUV host kurulumu (Jetson Orin NX, L4T r36.5 / JetPack 6.2.x)
#
# SUDO GEREKTIRIR:  sudo bash scripts/install_host.sh
# Idempotenttir; guncellemelerden sonra yeniden calistirilabilir.
#
# Yapilanlar (resmi kaynaklardan dogrulanmis paket adlariyla):
#  1. Celisen Ubuntu CUDA 11.5 paketlerini kaldir (nvidia-cuda-toolkit)
#  2. JetPack bilesenleri: CUDA 12.6 toolkit + cuDNN 9.3 + TensorRT 10.3
#     (zed-ros2-wrapper README JP6 notu: nvidia-jetpack nvidia-jetpack-dev)
#  3. Docker + compose + nvidia-container-toolkit (varsayilan runtime: nvidia)
#  4. Kullaniciyi docker grubuna ekleme
#  5. USB bellek limiti — usbcore Jetson cekirdeginde BUILTIN oldugu icin
#     modprobe.d ise yaramaz; kalicilik systemd oneshot servisiyle saglanir.
#  6. Native derleme/kullanim icin ROS 2 Humble bagimliliklari
#
# Kaynaklar:
#  - https://raw.githubusercontent.com/stereolabs/zed-ros2-wrapper/master/README.md
#  - https://docs.nvidia.com/jetson/jetpack/install-setup/index.html
#  - https://docs.stereolabs.com/docs/integrations/docker/install-on-nvidia-jetson.md
# ============================================================================
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Bu script sudo ile calistirilmalidir: sudo bash $0" >&2
    exit 1
fi

REAL_USER="${SUDO_USER:-qayra}"

echo "==> [1/6] Celisen Ubuntu CUDA 11.5 paketleri kaldiriliyor..."
apt-get purge -y nvidia-cuda-toolkit nvidia-cuda-dev 2>/dev/null || true
apt-get autoremove -y || true

echo "==> [2/6] JetPack bilesenleri kuruluyor (CUDA 12.6, cuDNN, TensorRT)..."
apt-get update
apt-get install -y nvidia-jetpack nvidia-jetpack-dev

echo "==> [3/6] Docker + nvidia-container-toolkit kuruluyor..."
apt-get install -y docker.io docker-compose-v2
if ! command -v nvidia-ctk >/dev/null 2>&1; then
    apt-get install -y nvidia-container-toolkit
fi
# Tek resmi komut hem runtime'i kaydeder hem varsayilan yapar
nvidia-ctk runtime configure --runtime=docker --set-as-default
systemctl daemon-reload
systemctl restart docker

echo "==> [4/6] '$REAL_USER' docker grubuna ekleniyor..."
usermod -aG docker "$REAL_USER"

echo "==> [5/6] USB bellek limiti (ZED USB3 tamponlari; usbcore builtin ->"
echo "          kalicilik systemd oneshot ile)..."
echo 2000 > /sys/module/usbcore/parameters/usbfs_memory_mb || true
cat > /etc/systemd/system/zed-usbfs-memory.service <<'EOF'
[Unit]
Description=ZED kameralar icin usbfs bellek limitini yukselt
DefaultDependencies=no
After=sysinit.target
Before=basic.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo 2000 > /sys/module/usbcore/parameters/usbfs_memory_mb'
RemainAfterExit=yes

[Install]
WantedBy=basic.target
EOF
systemctl daemon-reload
systemctl enable zed-usbfs-memory.service
# Eski, etkisiz modprobe.d dosyasini temizle (usbcore builtin oldugu icin okunmuyordu)
rm -f /etc/modprobe.d/usbcore-zed.conf

echo "==> [6/6] Native ROS 2 bagimliliklari kuruluyor (wrapper derlemesi +"
echo "          host'tan zed_msgs servis cagrilari icin)..."
apt-get install -y \
    ros-humble-zed-msgs ros-humble-zed-description ros-humble-xacro \
    ros-humble-robot-state-publisher ros-humble-diagnostic-updater \
    ros-humble-robot-localization ros-humble-nmea-msgs \
    ros-humble-geographic-msgs ros-humble-angles ros-humble-backward-ros \
    ros-humble-image-transport ros-humble-image-transport-plugins \
    ros-humble-point-cloud-transport ros-humble-point-cloud-transport-plugins \
    nlohmann-json3-dev python3-colcon-common-extensions python3-numpy

echo ""
echo "================================================================"
echo " Kurulum tamamlandi. Dogrulama komutlari:"
echo "   /usr/local/cuda/bin/nvcc --version   -> CUDA 12.6"
echo "   dpkg -l | grep libnvinfer10          -> TensorRT 10.3"
echo "   ldd /usr/local/zed/lib/libsl_ai.so | grep 'not found'  -> bos cikmali"
echo "   docker info | grep -i 'default runtime'  -> nvidia"
echo "   cat /sys/module/usbcore/parameters/usbfs_memory_mb      -> 2000"
echo ""
echo " NOT: docker grubunun etkinlesmesi icin oturumu kapatip acin"
echo "      (veya 'newgrp docker' calistirin)."
echo "================================================================"
