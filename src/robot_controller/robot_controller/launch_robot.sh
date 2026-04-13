cat << 'EOF' > ~/launch_robot.sh
#!/bin/bash
echo "=== LANCEMENT ROBOT ==="

# Tuer les anciens processus
pkill -f v4l2_camera
pkill -f robot_controller
sleep 1

# Lancer tous les noeuds en background
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" &
sleep 2
ros2 run robot_controller gopigo3_driver &
sleep 1
ros2 run robot_controller cube_detector &
sleep 1
ros2 run robot_controller imu_node &
sleep 2
ros2 run robot_controller cube_strategy &

echo "=== TOUS LES NOEUDS LANCES ==="
echo "Ctrl+C pour tout arreter"
wait
