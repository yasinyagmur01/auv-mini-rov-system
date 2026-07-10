#!/usr/bin/env bash
# Jetson'da ROS2 -> web koprusunu baslatir (internetsiz, mevcut paketlerle):
#   - rsweb'i durdurur (D435'i serbest birakir)
#   - D435 ROS2 node'u (pyrealsense2) -> /camera/color/image_raw
#   - ros2_web_bridge (:8000) ROS2 topic'lerini okuyup MJPEG+JSON servis eder
#     (/stream/color rsweb UYUMLU - mevcut PC paneli bozulmaz)
# ZED2i launch'ina DOKUNMAZ (ayni ROS graph'inda kalir; kopru ZED goruntusunu de gosterir).
#
# Kullanim: bash ~/webpanel/webpanel_ros2.sh
# NOT: ROS setup.bash tanimsiz degisken kullandigi icin 'set -u' YOK.
source /opt/ros/humble/setup.bash
[ -f ~/auv/ros2_ws/install/setup.bash ] && source ~/auv/ros2_ws/install/setup.bash
LOG=~/webpanel/logs; mkdir -p "$LOG"

echo "[1/3] rsweb durduruluyor (D435 serbest)..."
pkill -f 'rsweb/app.py' 2>/dev/null && sleep 2 || echo "  rsweb zaten kapali"

echo "[2/3] D435 ROS2 node..."
pkill -f 'd435_ros2_node.py' 2>/dev/null; sleep 1
nohup python3 ~/webpanel/d435_ros2_node.py --width 640 --height 480 --fps 30 \
  > "$LOG/d435.log" 2>&1 &
echo "  PID $!"; sleep 4

echo "[3/3] ros2_web_bridge (:8000)..."
pkill -f 'ros2_web_bridge.py' 2>/dev/null; sleep 1
nohup python3 ~/webpanel/ros2_web_bridge.py --port 8000 \
  > "$LOG/bridge.log" 2>&1 &
echo "  PID $!"; sleep 4

echo "--- Durum ---"
echo "ROS2 topic'ler:"; timeout 6 ros2 topic list 2>/dev/null | grep -E 'camera|image|mav' || echo "  (yok)"
echo "Port 8000:"; ss -tlnp 2>/dev/null | grep ':8000' || echo "  (dinlemiyor)"
echo "D435 log:"; tail -3 "$LOG/d435.log"
echo "Bridge log:"; tail -3 "$LOG/bridge.log"
echo
echo "Panel: http://192.168.2.135:8000/   (D435 rengi: /stream/color)"
