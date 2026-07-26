#!/usr/bin/env bash
# =============================================================================
# AUV + Mini ROV sistem sağlık testi — başlatıcı
# ROS 2 ortamını source eder ve system_health_check.py'yi çalıştırır.
# Salt-okunur: motor/arm/MAVLink yazması YOK.
#
# Kullanım (Jetson'da):
#   bash scripts/health_check.sh                 # haberleşme + sensör (auto)
#   bash scripts/health_check.sh --comms-only    # sadece haberleşme
#   bash scripts/health_check.sh -i              # "hangi sensörler takılı?" sorar
#   bash scripts/health_check.sh --only depth,sonar,zed
#
# Kontrol PC'sinden tek satır (Jetson'a SSH ile):
#   ssh qayra@192.168.2.135 'bash ~/auv/scripts/health_check.sh --comms-only'
# =============================================================================
# DİKKAT: 'set -u' KULLANMA — ROS setup.bash unbound değişken kullanır ve
# nounset altında sourcing anında shell'i öldürür (test hiç çalışmaz).

# Depo kökü (bu script scripts/ altında)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS 2 ortamı (varsa) — hata verse de devam et; script ROS yoksa da çalışır
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f "$HERE/ros2_ws/install/setup.bash" ]; then
  source "$HERE/ros2_ws/install/setup.bash" 2>/dev/null || true
fi

cd "$HERE"
echo "[health_check] depo: $HERE"
echo "[health_check] ROS_DISTRO=${ROS_DISTRO:-yok}  python: $(command -v python3)"
exec python3 system_health_check.py "$@"
