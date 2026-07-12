#!/bin/bash
# AUV çekirdek yığınını başlatır: MAVLink router + mav_bridge + mission_node + web köprüsü.
# systemd 'auv-stack.service' tarafından çağrılır (Type=oneshot + RemainAfterExit=yes).
# Kullanıcı: qayra. Yollar Jetson deploy düzenine göredir; farklıysa güncelle.
#
# NOT: Bu bir TASLAKTIR — Jetson'da (kartlara bağlı) test edilip etkinleştirilmelidir.
set -u
LOGDIR=/home/qayra/auv/logs
mkdir -p "$LOGDIR"

echo "[auv-stack] eski örnekler temizleniyor..."
pkill -f 'mav_router.py'    2>/dev/null
pkill -f 'auv_mav_bridge'   2>/dev/null
pkill -f 'auv_mission'      2>/dev/null
pkill -f 'ros2_web_bridge.py' 2>/dev/null
sleep 2

# 1) MAVLink router: V6X seri <-> QGC(:14550) + mav_bridge(:14551) eşzamanlı.
#    (Alternatif: sistemin 'mavlink-router' servisi. Burada scratchpad router'ı referans.)
echo "[auv-stack] router başlatılıyor..."
nohup python3 /home/qayra/mav_router.py > "$LOGDIR/router.log" 2>&1 &
sleep 2

# 2) ROS 2 ortamı
source /opt/ros/humble/setup.bash
source /home/qayra/auv/ros2_ws/install/setup.bash

# 3) mav_bridge — sensör telemetri + motor test/doğrudan çıkış (udpout:14551)
echo "[auv-stack] mav_bridge başlatılıyor..."
nohup ros2 run auv_mav_bridge bridge_node --ros-args \
  -p connection_url:=udpout:127.0.0.1:14551 \
  -p depth_source:=pressure2 -p water_density_kgm3:=997.0 \
  > "$LOGDIR/mav_bridge.log" 2>&1 &

# 4) mission_node — görev durum makinesi + panel WebSocket sunucusu (:8765)
echo "[auv-stack] mission_node başlatılıyor..."
nohup ros2 run auv_mission mission_node > "$LOGDIR/mission.log" 2>&1 &
sleep 2

# 5) web köprüsü — panel + kameralar + /sensors (:8000)
echo "[auv-stack] web köprüsü başlatılıyor..."
nohup python3 /home/qayra/webpanel/ros2_web_bridge.py --port 8000 > "$LOGDIR/web.log" 2>&1 &

sleep 1
echo "[auv-stack] başlatıldı: router + mav_bridge + mission + web. Loglar: $LOGDIR"
