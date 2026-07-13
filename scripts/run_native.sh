#!/usr/bin/env bash
# Native calistirma (Docker'siz hizli gelistirme dongusu).
# Harita dizini tek kaynaktan yonetilir: launch 'maps_dir' argumani
# (area_file_path'i zed_camera.launch.py param_overrides'ina cevirir).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"
MAPS_DIR="$ROOT/maps"
mkdir -p "$MAPS_DIR"

# ROS setup.bash dosyalari nounset-guvenli degildir (AMENT_TRACE_SETUP_FILES);
# bu nedenle -u kullanilmiyor.
source /opt/ros/humble/setup.bash
source "$WS/install/local_setup.bash"

exec ros2 launch auv_bringup zed2i_auv.launch.py maps_dir:="$MAPS_DIR"
