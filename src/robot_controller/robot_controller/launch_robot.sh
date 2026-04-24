#!/bin/bash
# Script de lancement de tout le stack du robot.
#
# Ordre de demarrage (les sleep laissent le temps a chaque noeud d'initialiser
# son hardware avant que le suivant ne demarre):
#   1. v4l2_camera       - capture USB 640x480
#   2. gopigo3_driver    - moteurs + encodeurs + batterie
#   3. cube_detector     - detection HSV des cubes
#   4. imu_node          - lecture IMU BNO085 (UART-RVC)
#   5. dashboard_node    - serveur web port 8080
#   6. cube_strategy     - machine a etats (demarre en PAUSE, Start via UI)
#
# Ctrl+C -> cleanup: kill tous les noeuds ROS + stop moteurs via easygopigo3.
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

# Nettoyage prealable au cas ou un noeud tournerait encore d'un lancement precedent
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
ros2 run robot_controller dashboard_node &
sleep 1
# cube_strategy demarre avec active=False, le Start se declenche via le dashboard
ros2 run robot_controller cube_strategy &
sleep 2

echo "=== BASE LANCEE ==="
echo "Dashboard: http://$(hostname -I | awk '{print $1}'):8080"
echo "La strat est PAUSEE, clique Start sur le dashboard pour demarrer."
echo "Ctrl+C pour tout arreter"
wait
