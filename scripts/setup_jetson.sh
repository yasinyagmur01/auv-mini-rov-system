#!/usr/bin/env bash
# Jetson Orin NX ilk kurulum (JetPack 6.x / Ubuntu 22.04 uzerinde)
# Adim adim calistirin; her blok bagimsizdir.
set -e

echo "== 1) ROS 2 Humble =="
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null || \
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-cv-bridge \
  ros-humble-realsense2-camera python3-colcon-common-extensions python3-rosdep
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

echo "== 2) Python bagimliliklari =="
pip3 install --user pymavlink websockets pyyaml opencv-python

echo "== 3) mavlink-router =="
sudo apt install -y git meson ninja-build pkg-config gcc g++ systemd
git clone --recurse-submodules https://github.com/mavlink-router/mavlink-router.git /tmp/mavlink-router
cd /tmp/mavlink-router
meson setup build . && ninja -C build && sudo ninja -C build install
sudo mkdir -p /etc/mavlink-router
sudo cp ~/auv/config/mavlink-router/main.conf /etc/mavlink-router/main.conf
sudo systemctl enable --now mavlink-router

echo "== 4) udev + statik IP =="
sudo cp ~/auv/scripts/99-auv-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo cp ~/auv/config/netplan/01-auv-static.yaml /etc/netplan/
echo "DIKKAT: 'sudo netplan apply' internet erisimini kesebilir - en son yapin."

echo "== 5) Workspace derleme =="
cd ~/auv/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
echo 'source ~/auv/ros2_ws/install/setup.bash' >> ~/.bashrc

echo "== 6) Acilista baslatma (istege bagli) =="
echo "sudo cp ~/auv/scripts/auv-bringup.service /etc/systemd/system/"
echo "sudo systemctl daemon-reload && sudo systemctl enable auv-bringup"

echo "KURULUM TAMAM. Notlar:"
echo " - librealsense JetPack 6 uyumu: sorun cikarsa RSUSB backend ile"
echo "   kaynaktan derleyin (docs/ardusub-kurulum.md, Riskler bolumu)."
echo " - Depoyu Jetson'a ~/auv olarak kopyalayin (deploy.sh)."
