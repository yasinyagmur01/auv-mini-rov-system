#!/bin/bash
# =============================================================================
# AUV çekirdek yığını + web köprüsü — TEK başlatıcı (systemd 'auv-stack.service' çağırır).
# Kullanıcı: qayra. REPO checkout'undan çalışır: ~/auv (deploy = `git pull`; ~/webpanel EMEKLİ).
#
# Router: 'mavlink-router' systemd servisi FC seri portunu tutar ve UDP'ye böler
#   → mav_bridge udpout:127.0.0.1:14551'e, QGC :14550'ye EŞZAMANLI bağlanır.
#   mavlink-router kurulu DEĞİLSE yedek: scripts/../run_stack.sh (mav_bridge'i doğrudan seri açar).
# ZED (Docker konteyner) topic'leri host'a FastDDS UDP-only profiliyle geçer.
# ui/ (three.js panel, :8080) AYRI 'auv-ui.service' ile yönetilir — burada başlatılmaz.
#
# ÖNKOŞUL (Jetson'da doğrula): FC USB bağlı (`ls /dev/ttyACM*`), mavlink-router aktif,
#   ros2_ws derli (`colcon build --symlink-install`).
# =============================================================================
set -u
AUV="$HOME/auv"
LOG="$AUV/logs"; mkdir -p "$LOG"

echo "[auv-stack] eski örnekler temizleniyor..."
pkill -f 'bridge_node'          2>/dev/null
pkill -f 'vertical_state_node'  2>/dev/null
pkill -f 'dr_node'              2>/dev/null
pkill -f 'mission_node'         2>/dev/null
pkill -f 'ros2_web_bridge.py'   2>/dev/null
sleep 2
# stale .pyc temizle (symlink-install kaynak değişimini eski .pyc ile gölgeleyebiliyor)
find "$AUV/ros2_ws" -path '*auv_*' -name '*.pyc' -delete 2>/dev/null
find "$AUV/ros2_ws" -path '*auv_*' -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null

# ZED konteyner->host topic geçişi için FastDDS UDP-only profili
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$AUV/config/fastdds/udp_only.xml"

source /opt/ros/humble/setup.bash
source "$AUV/ros2_ws/install/setup.bash"

echo "[auv-stack] mav_bridge (udpout:14551 <- mavlink-router)..."
nohup ros2 run auv_mav_bridge bridge_node --ros-args \
  -p connection_url:=udpout:127.0.0.1:14551 \
  -p depth_source:=pressure2 -p water_density_kgm3:=997.0 \
  > "$LOG/mav_bridge.log" 2>&1 &
sleep 4

echo "[auv-stack] vertical_state..."
nohup ros2 run auv_vertical_state vertical_state_node > "$LOG/vertical.log" 2>&1 &

echo "[auv-stack] dead_reckoning..."
nohup ros2 run auv_dead_reckoning dr_node --ros-args \
  -p speed_table_csv:="$AUV/config/speed_table.csv" > "$LOG/dr.log" 2>&1 &

echo "[auv-stack] mission (WS :8765)..."
nohup ros2 run auv_mission mission_node --ros-args \
  -p missions_dir:="$AUV/config/missions" \
  -p speed_table_csv:="$AUV/config/speed_table.csv" \
  -p ws_port:=8765 > "$LOG/mission.log" 2>&1 &
sleep 3

echo "[auv-stack] web köprüsü (cockpit :8000) — repodan (~/auv/jetson)..."
nohup python3 "$AUV/jetson/ros2_web_bridge.py" --port 8000 > "$LOG/web.log" 2>&1 &

sleep 2
echo "[auv-stack] başlatıldı."
echo "  panel  : http://<jetson-ip>:8000/    (cockpit)"
echo "  ui/    : http://<jetson-ip>:8080/    (three.js — ayrı auv-ui.service)"
echo "  loglar : $LOG"
timeout 8 ros2 topic list 2>/dev/null | grep -E '/mav|/vertical|/dr|/mission' | sort
ss -tln 2>/dev/null | grep -qE ':8765' && echo "  WS :8765 dinliyor" || echo "  UYARI: :8765 YOK"
ss -tln 2>/dev/null | grep -qE ':8000' && echo "  web :8000 dinliyor" || echo "  UYARI: :8000 YOK"
