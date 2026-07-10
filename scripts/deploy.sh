#!/usr/bin/env bash
# Gelistirme PC'sinden Jetson'a depo kopyalama (Linux/WSL/Git Bash)
# Kullanim: ./deploy.sh [jetson-ip]
JETSON=${1:-192.168.2.3}
rsync -av --delete \
  --exclude 'ros2_ws/build' --exclude 'ros2_ws/install' --exclude 'ros2_ws/log' \
  --exclude '__pycache__' --exclude '.git' \
  ./ jetson@${JETSON}:~/auv/
echo "Kopyalandi. Jetson'da: cd ~/auv/ros2_ws && colcon build --symlink-install"
