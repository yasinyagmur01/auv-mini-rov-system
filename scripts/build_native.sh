#!/usr/bin/env bash
# Native (Docker'siz) derleme — hizli gelistirme dongusu icin.
# ONKOSUL: scripts/install_host.sh calistirilmis olmali (CUDA 12.6 toolkit +
# ROS bagimliliklari; zed-config.cmake 'find_package(CUDA 12.6 REQUIRED)' icerir).
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/ros2_ws"

if [ ! -d /usr/local/cuda ]; then
    echo "HATA: /usr/local/cuda yok. Once 'sudo bash scripts/install_host.sh' calistirin." >&2
    exit 1
fi

# Wrapper'in derleme bagimliliklarini kontrol et (install_host.sh kurar)
MISSING=()
for pkg in zed_msgs nmea_msgs point_cloud_transport diagnostic_updater \
           robot_localization geographic_msgs angles backward_ros xacro \
           zed_description; do
    [ -d "/opt/ros/humble/share/$pkg" ] || MISSING+=("ros-humble-${pkg//_/-}")
done
if [ ${#MISSING[@]} -gt 0 ]; then
    echo "HATA: Eksik ROS bagimliliklari: ${MISSING[*]}" >&2
    echo "Kurmak icin: sudo apt install -y ${MISSING[*]}" >&2
    echo "(veya 'sudo bash scripts/install_host.sh' yeniden calistirin)" >&2
    exit 1
fi

# ROS setup.bash nounset-guvenli degildir; -u kullanilmiyor.
source /opt/ros/humble/setup.bash

cd "$WS"
colcon build --symlink-install \
  --packages-skip zed_debug \
  --parallel-workers "$(nproc)" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --no-warn-unused-cli

echo ""
echo "Derleme tamam. Calistirmak icin: bash scripts/run_native.sh"
