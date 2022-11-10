cd ~/ros2_ws && colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 run rc_car send_ip_address