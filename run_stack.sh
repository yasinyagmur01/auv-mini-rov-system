source /opt/ros/humble/setup.bash
source ~/auv/ros2_ws/install/setup.bash
LOG=~/auv/logs; mkdir -p "$LOG"
FC=/dev/serial/by-id/usb-ArduPilot_CUAV-V6X-v2_260040000551333033393439-if00

echo "eski instance'lar kapatiliyor..."
pkill -f 'bridge_node' 2>/dev/null; pkill -f 'vertical_state_node' 2>/dev/null
pkill -f 'dr_node' 2>/dev/null; pkill -f 'mission_node' 2>/dev/null; sleep 2
# stale .pyc temizle (symlink-install bazen kaynak degisimini eski .pyc ile golgeliyor)
find ~/auv/ros2_ws -path '*auv_*' -name '*.pyc' -delete 2>/dev/null
find ~/auv/ros2_ws -path '*auv_*' -name '__pycache__' -type d -exec rm -rf {} + 2>/dev/null

echo "mav_bridge (V6X seri)..."
nohup ros2 run auv_mav_bridge bridge_node --ros-args \
  -p connection_url:="$FC" -p depth_source:=pressure2 -p water_density_kgm3:=997.0 \
  > "$LOG/mav_bridge.log" 2>&1 &
echo "  PID $!"; sleep 4

echo "vertical_state..."
nohup ros2 run auv_vertical_state vertical_state_node > "$LOG/vertical.log" 2>&1 &
echo "  PID $!"

echo "dead_reckoning..."
nohup ros2 run auv_dead_reckoning dr_node --ros-args \
  -p speed_table_csv:="$HOME/auv/config/speed_table.csv" > "$LOG/dr.log" 2>&1 &
echo "  PID $!"

echo "mission (WS :8765)..."
nohup ros2 run auv_mission mission_node --ros-args \
  -p missions_dir:="$HOME/auv/config/missions" \
  -p speed_table_csv:="$HOME/auv/config/speed_table.csv" \
  -p ws_port:=8765 > "$LOG/mission.log" 2>&1 &
echo "  PID $!"; sleep 5

echo "--- ilgili topic'ler ---"
timeout 8 ros2 topic list 2>/dev/null | grep -E '/mav|/vertical|/dr|/mission' | sort
echo "--- :8765 ---"; ss -tln 2>/dev/null | grep ':8765' && echo "  WS dinliyor" || echo "  8765 YOK"
echo "--- mav_bridge log ---"; tail -6 "$LOG/mav_bridge.log"
echo "--- mission log ---"; tail -4 "$LOG/mission.log"
