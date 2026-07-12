#!/usr/bin/env bash
# Jetson kamera+sensor WEB koprusunu (:8000) baslatir.
#   - rsweb'i KALICI durdurur (systemd stop/disable/mask + supervisor + pkill).
#   - :8000'i garanti bosaltir; bosalmazsa 8080'e duser.
#   - FastDDS UDP-only profili yazar (ZED Docker konteynerinden veri host'a SHM
#     ile GECMEZ; UDP-only ile gecer -> ZED goruntusu gelir).
#   - ros2_web_bridge: ZED (compressed) + /mav/* -> MJPEG + /sensors JSON.
# NOT: Sensor/otonom ROS yigini (mav_bridge/vertical/dr/mission) AYRI: run_stack.sh.
#      D435 kaldirildi (artik kullanilmiyor).
# NOT: ROS setup.bash tanimsiz degisken kullandigi icin 'set -u' YOK.
source /opt/ros/humble/setup.bash
[ -f ~/auv/ros2_ws/install/setup.bash ] && source ~/auv/ros2_ws/install/setup.bash
LOG=~/webpanel/logs; mkdir -p "$LOG"
PORT=${PORT:-8000}

port_free() { ! ss -tln 2>/dev/null | grep -q ":$1 "; }
free_port() {
  local p=$1 pids
  pids=$(ss -tlnp 2>/dev/null | grep ":$p " | grep -oP 'pid=\K[0-9]+' | sort -u)
  [ -z "$pids" ] && command -v fuser >/dev/null 2>&1 && pids=$(fuser "$p/tcp" 2>/dev/null)
  for pid in $pids; do kill -9 "$pid" 2>/dev/null || sudo kill -9 "$pid" 2>/dev/null; done
  [ -n "$pids" ] && sleep 2
}

echo "[1/4] rsweb kalici durduruluyor..."
unit=$(systemctl list-units --all --type=service --no-legend 2>/dev/null | awk '{print $1}' | grep -i rsweb | head -1)
[ -z "$unit" ] && unit=$(systemctl list-unit-files --no-legend 2>/dev/null | awk '{print $1}' | grep -i rsweb | head -1)
if [ -n "$unit" ]; then
  sudo systemctl stop "$unit" 2>/dev/null; sudo systemctl disable "$unit" 2>/dev/null; sudo systemctl mask "$unit" 2>/dev/null
  echo "  $unit -> stop/disable/mask"
fi
pkill -f 'rsweb/app.py' 2>/dev/null && echo "  rsweb process kapatildi" || true

echo "[2/4] FastDDS UDP-only profili (ZED konteyner->host veri gecisi icin)..."
cat > ~/webpanel/udp_only.xml <<'XMLEOF'
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor><transport_id>udp_tr</transport_id><type>UDPv4</type></transport_descriptor>
    </transport_descriptors>
    <participant profile_name="udp_only" is_default_profile="true">
      <rtps>
        <userTransports><transport_id>udp_tr</transport_id></userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
XMLEOF
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/webpanel/udp_only.xml

echo "[3/4] port :$PORT hazirlaniyor..."
port_free "$PORT" || free_port "$PORT"
if ! port_free "$PORT"; then
  echo "  UYARI: :$PORT bosalmadi -> 8080 (web/config.py URL portunu guncelle)"; PORT=8080; free_port "$PORT"
fi

echo "[4/4] ros2_web_bridge (:$PORT, UDP-only)..."
pkill -9 -f 'ros2_web_bridge.py' 2>/dev/null; sleep 1
nohup python3 ~/webpanel/ros2_web_bridge.py --port "$PORT" > "$LOG/bridge.log" 2>&1 &
echo "  PID $!"; sleep 5

echo "--- Durum ---"
echo "Port $PORT:"; ss -tlnp 2>/dev/null | grep ":$PORT " || echo "  (dinlemiyor)"
echo "Bridge log:"; tail -4 "$LOG/bridge.log"
echo "/sensors:"; curl -s "http://localhost:$PORT/sensors" | head -c 400; echo
echo "Panel: http://192.168.2.135:$PORT/   |  Kontrol: /control  |  Mini ROV: /minirov"
