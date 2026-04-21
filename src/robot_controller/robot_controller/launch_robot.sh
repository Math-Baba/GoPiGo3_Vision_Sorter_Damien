#!/bin/bash
echo "=== LANCEMENT ROBOT ==="

cleanup() {
    echo "Arret..."
    pkill -f v4l2_camera
    pkill -f "ros2 run robot_controller"
    sleep 1
    python3 -c "import easygopigo3; easygopigo3.EasyGoPiGo3().stop()" 2>/dev/null
    echo "Stop!"
    exit 0
}
trap cleanup SIGINT

pkill -f v4l2_camera 2>/dev/null
pkill -f "ros2 run robot_controller" 2>/dev/null
sleep 1

ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" &
sleep 2
ros2 run robot_controller gopigo3_driver &
sleep 1
ros2 run robot_controller cube_detector &
sleep 1
ros2 run robot_controller imu_node &
sleep 1
ros2 run robot_controller aruco_localizer &
sleep 1
ros2 run robot_controller dashboard_node &
sleep 2

echo "=== BASE LANCEE ==="
echo "Dashboard: http://$(hostname -I | awk '{print $1}'):8080"
echo "Lance la strat: ros2 run robot_controller cube_strategy"
echo "Ctrl+C pour tout arreter"
wait