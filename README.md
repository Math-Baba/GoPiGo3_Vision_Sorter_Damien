
# GoPiGo3 Vision Sorter

## Description
--------
Ce projet met en œuvre un robot mobile **GoPiGo3** piloté par **ROS2 Jazzy** sur Raspberry Pi. Placé au centre d'une arène délimitée par 4 murs, le robot tourne sur lui-même pour détecter des cubes colorés (vert, bleu, rouge, jaune), s'approche, les capture avec un grabber à servo, puis les transporte vers le mur de dépôt correspondant à leur couleur (vert/bleu → mur avant, rouge/jaune → mur arrière) en poussant droit contre la zone.
 
La stratégie repose essentiellement sur trois capteurs : **caméra Arducam Sony** pour la détection des cubes par seuillage HSV, **IMU BNO085** pour le cap absolu (rotation du robot), et **encodeurs des roues(Odométrie)** pour la distance parcourue (détection de patinage sur carreaux inégaux).
 
Un dashboard web affiche en direct le flux caméra annoté, l'état de la machine à états, la boussole IMU et les compteurs de cubes triés.
----------------------------------------

## Fonctionnalités
 
- **Détection multi-couleurs** des cubes via seuillage HSV + tracking temporel (lissage, persistance 300 ms, gestion blind-spot quand un cube passe sous la caméra).
- **Machine à états** gérant le cycle complet de tri : recherche par rotation → alignement visuel → approche PID → capture → rotation vers zone de dépôt → push droit → retour au centre.
- **Odométrie roue** pour mesurer les déplacements réels et détecter le patinage : si les roues tournent sans que le robot bouge, la stratégie abandonne l'état courant proprement.
- **Hystérésis sur cap IMU** pour éviter les transitions sur du bruit capteur.
- **Zone morte CHERCHER** (±25° autour des murs de dépôt) pour ignorer les cubes déjà triés.
- **Compensation HSV par couleur** : seuil d'aire ajustable (le jaune ressort plus petit en HSV que les autres couleurs).
- **Dashboard web** en temps réel sur port 8080 avec flux caméra annoté, état, IMU, compteurs et carte.
- **Localisation ArUco** (nœud séparé) capable de calculer la pose 6DOF du robot dans l'arène via 4 marqueurs muraux — prête à être réintégrée à la stratégie pour affiner les dépôts.
----------------------------------------

## Prérequis logiciel
 
- **Raspberry Pi OS** (64-bit) compatible Ubuntu 24.04 (Noble) ou **Ubuntu 24.04** directement
- **ROS 2 Jazzy Jalisco**
- **Python 3.12**
- **OpenCV 4.6+** avec module `aruco` (`opencv-contrib-python`)
- Paquets ROS 2 :
  - `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`
  - `v4l2_camera` (nœud caméra ROS)
  - `camera_calibration_parsers`
- Paquets Python (pip) :
  - `easygopigo3` — API haut niveau des moteurs GoPiGo3
  - `numpy`, `pyserial`
- **colcon** pour la compilation du workspace ROS 2


## Prérequis matériel

| Composant | Rôle |
|-----------|------|
| Raspberry Pi 4 (4 Go min recommandé) | Unité de calcul |
| Carte microSD 32 Go classe 10+ | Stockage du système |
| Plateforme **Dexter Industries GoPiGo3** | Base mobile à 2 roues motrices + encodeurs |
| Caméra USB 640×480 (ou Pi Camera) | Détection cubes + ArUco |
| IMU **BNO085** (UART-RVC sur `/dev/ttyAMA0`) | Mesure du cap yaw |
| Servo-moteur standard sur port **SERVO1** du GoPiGo3 | Grabber (pince/porte) |
| Batterie 9–12 V (pack AA ou Li-ion) | Alimentation motrice |
| Arène physique 2.4 m × 2.4 m avec 4 murs | Terrain de jeu |
| 4 marqueurs ArUco 4×4 (IDs 0-3), 6 cm, imprimés au centre de chaque mur | Repérage optionnel |
| Cubes colorés (vert, bleu, rouge, jaune), ~5–6 cm de côté | Objets à trier |


## Guide de configuration GoPiGo3
 
1. **Flasher** la carte SD avec Raspberry Pi OS 64-bit (ou Ubuntu Server 24.04).
2. **Installer la bibliothèque GoPiGo3** de Dexter Industries :
   ```bash
   curl -kL dexterindustries.com/update_gopigo3 | bash
   sudo reboot
   ```
3. **Installer `easygopigo3`** :
   ```bash
   sudo pip3 install easygopigo3 --break-system-packages
   ```
4. **Activer l'UART série** pour l'IMU BNO085 :
   ```bash
   sudo raspi-config
   # Interface Options → Serial Port
   # Login shell over serial : NO
   # Hardware serial enabled : YES
   sudo reboot
   ```
   Vérifier que `/dev/ttyAMA0` existe et est accessible par l'utilisateur `pi` (ajouter au groupe `dialout` si besoin : `sudo usermod -aG dialout pi`).
5. **Câblage IMU BNO085** (mode UART-RVC) :
   - `VIN` → 3.3 V Pi
   - `GND` → GND
   - `TX` de l'IMU → `RX` du Pi (GPIO 15)
   - `PS1` du BNO085 à `3.3V` (active le mode RVC)
6. **Brancher le servo** sur le port `SERVO1` de la carte GoPiGo3. Le code ouvre à 130° et ferme à 0° — ajuster ces valeurs dans `cube_strategy.py` (méthodes `servo_open` / `servo_close`) selon votre grabber.
7. **Brancher la caméra** (USB) au Pi. Vérifier avec :
   ```bash
   ls /dev/video*
   ```

----------------------------------------

- Package principal : `src/robot_controller`
	- `setup.py`                  : configuration Python / console_scripts
	- `package.xml`               : dépendances ROS (ex: rclpy)
	- `setup.cfg`                  
	- `resource/robot_controller` : package resource
	- `test/`                     : tests unitaires (flake8, pep257, copyright)

- Code du package : `src/robot_controller/robot_controller/`
	- `__init__.py`
	- `gopigo3_driver.py`         : driver / interface GoPiGo3 (entrée : `gopigo3_driver`)
	- `cube_detector.py`          : détecteur de cubes (entrée : `cube_detector`)
	- `cube_strategy.py`          : logique de tri / stratégie (entrée : `cube_strategy`)
	- `follow_test.py`            : script de test / suivi (entrée : `follow_test`)
	- `arena_map.py`              : gestion d'arène / cartographie (entrée : `arena_map`)
	- `imu_node.py`               : noeud IMU (entrée : `imu_node`)
	- `dashboard_node.py`         : noeud dashboard (entrée : `dashboard_node`)
	- `hsv_calibration.py`        : outil de calibration HSV
	- `launch_robot.sh`           : script d'aide au lancement (dans le dossier package)
	- `test_imu.py`               : test pour l'IMU

- Interface web (UI) :
	- `src/robot_controller/web2/index.html`  (UI dashboard)
	- NOTE : le `setup.py` référence `robot_controller/web` dans `data_files`, mais l'UI actuelle est dans `web2/`. Voir la section "Web / dashboard" ci-dessous.

## Points d'entrée (console_scripts)

Le `setup.py` expose plusieurs scripts installables via `ros2 run robot_controller <name>` :

- `gopigo3_driver = robot_controller.gopigo3_driver:main`
- `cube_detector = robot_controller.cube_detector:main`
- `cube_strategy = robot_controller.cube_strategy:main`
- `follow_test = robot_controller.follow_test:main`
- `arena_map = robot_controller.arena_map:main`
- `imu_node = robot_controller.imu_node:main`
- `dashboard_node = robot_controller.dashboard_node:main`

## Installation 
 
1. **Installer ROS 2 Jazzy** (si pas déjà fait) — suivre la doc officielle : https://docs.ros.org/en/jazzy/Installation.html
2. **Installer les paquets ROS 2 additionnels** :
   ```bash
   sudo apt install ros-jazzy-v4l2-camera ros-jazzy-camera-calibration-parsers
   ```
3. **Installer les dépendances Python** :
   ```bash
   sudo pip3 install opencv-contrib-python numpy pyserial --break-system-packages
   ```
4. **Créer le workspace ROS 2** et cloner le projet :
   ```bash
   mkdir -p ~/ROS2_WS/src
   cd ~/ROS2_WS/src
   git clone https://github.com/Math-Baba/GoPiGo3_Vision_Sorter_Damien.git robot_controller
   ```
5. **Compiler le package** :
   ```bash
   cd ~/ROS2_WS
   colcon build --packages-select robot_controller
   source install/setup.bash
   ```
6. **(Recommandé) Auto-source** : ajouter à `~/.bashrc` :
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ROS2_WS/install/setup.bash
   ```
7. **Rendre le script de lancement exécutable** :
   ```bash
   chmod +x ~/ROS2_WS/src/robot_controller/robot_controller/launch_robot.sh
   ```
 
## Structure du projet
 
```
robot_controller/
├── package.xml                              # Manifeste ROS 2
├── setup.py                                 # Configuration Python / entry points
├── setup.cfg
├── resource/robot_controller                # Marker ament
├── robot_controller/                        # Code source des nœuds
│   ├── launch_robot.sh                      # Script de lancement (tous les nœuds)
│   ├── gopigo3_driver.py                    # Pilote moteurs + encodeurs + batterie
│   ├── imu_node.py                          # Lecture IMU BNO085 UART-RVC
│   ├── cube_detector.py                     # Détection HSV des cubes → /cube_detections
│   ├── aruco_localizer.py                   # Pose 6DOF ArUco → /robot_position
│   ├── cube_strategy.py                     # Machine à états principale
│   └── dashboard_node.py                    # Serveur HTTP + annotations image
├── web2/
│   └── index.html                           # Dashboard web (servi sur port 8080)
└── test/                                    # Nœuds annexes et tests (camera_node,
                                             # motion_node, arena_map, follow_test,
                                             # hsv_calibration, test_flake8, etc.)
```
 
### Rôle de chaque nœud ROS 2
 
| Nœud | Publie | Souscrit | Fonction |
|------|--------|----------|----------|
| `v4l2_camera_node` | `/image_raw` | - | Flux caméra brut (640×480) |
| `gopigo3_driver` | `/battery`, `/odom_simple` | `/cmd_vel` | Pilote moteurs + odométrie roue |
| `imu_node` | `/imu/heading`, `/imu/data` | - | Cap yaw BNO085 en degrés |
| `cube_detector` | `/cube_detections` | `/image_raw` | Détection HSV 4 couleurs + tracking |
| `aruco_localizer` | `/robot_position` | `/image_raw`, `/imu/heading` | Pose arène via marqueurs muraux |
| `cube_strategy` | `/cmd_vel`, `/robot_status` | `/cube_detections`, `/imu/heading`, `/odom_simple` | Machine à états du tri |
| `dashboard_node` | - (HTTP:8080) | `/robot_status`, `/cube_detections`, `/imu/heading`, `/image_raw` | Interface web |
 
## Fonctionnement
 
### Démarrage
 
1. **Positionner** le robot au centre de l'arène, **face au mur A** (mur de dépôt vert/bleu). L'IMU se zérote à cette position au premier tick, donc `heading = 0°` correspond ensuite à "face au mur A".
2. **Lancer la base** (tous les nœuds sauf la stratégie) :
   ```bash
   bash ~/ROS2_WS/src/robot_controller/robot_controller/launch_robot.sh
   ```
   Le terminal affiche l'URL du dashboard (ex. `http://192.168.0.42:8080`).
3. **Lancer la stratégie** dans un second terminal :
   ```bash
   ros2 run robot_controller cube_strategy
   ```
4. Le robot commence automatiquement à rechercher des cubes.
 
### Machine à états
 
```
    ┌──────────────────────────────────────────────────────────────┐
    │                                                              │
    ▼                                                              │
CHERCHER ──(cube confirmé 3 frames)──> ALIGNER ──(centré)──> APPROCHER
    ▲                                     │                       │
    │                                     │                       │
    │                              (cube perdu)            (cube.area > close_area
    │                               distance_forward > 0.05       ou blind_spot)
    │                                     │                       │
    │                                     ▼                       ▼
    │                               RETOUR_CENTRE <────────── POUSSER (grab, 3.0s)
    │                                     │
    │                                     ▼
    │                              ORIENTER_ZONE (rotation vers 0° ou 180°)
    │                                     │
    │                                     ▼
    │                              POUSSER_DROIT (push vers mur, 6s)
    │                                     │
    │                                     ▼
    └─────────────────────────────── RETOUR (recul même distance)
```
 
### Détail des états
 
- **CHERCHER** : rotation lente (0.6 rad/s) sur place. Confirme un cube sur 3 frames consécutifs. Zone morte ±25° autour des caps de dépôt (ignore les cubes dans les zones A et B déjà triées).
- **ALIGNER** : centre horizontalement le cube dans le frame caméra via PID (kp=0.003). Tolérance ±40 px.
- **APPROCHER** : avance en conservant le centrage, vitesse modulée selon la taille du cube. Transition POUSSER quand `cube.area > close_area` (4000 par défaut, 2200 pour le jaune).
- **POUSSER** : fonce 3.0 s, ferme le servo à 2.7 s pour saisir le cube.
- **RETOUR_CENTRE** : recule exactement ce qui a été avancé (compteur `distance_forward`). Stall detection via odométrie : si les roues patinent pendant 2.5 s sans déplacement, on passe à la suite quand même.
- **ORIENTER_ZONE** : rotation IMU vers 0° (cubes verts/bleus → mur A) ou 180° (cubes rouges/jaunes → mur B). Hystérésis de 3 frames au cap cible pour ne pas transiter sur du bruit. Drop du cube si rotation bloquée.
- **POUSSER_DROIT** : push continu pendant `push_duration=6s` vers le mur de dépôt, servo ouvert à la fin.
- **RETOUR** : recule la distance poussée, puis retour à CHERCHER.
 
### Mapping couleurs → zones
 
| Cube | Cap de dépôt | Mur |
|------|--------------|-----|
| Vert | 0° | A (avant) |
| Bleu | 0° | A (avant) |
| Rouge | 180° | B (arrière) |
| Jaune | 180° | B (arrière) |
 
### Arrêt
 
`Ctrl+C` dans le terminal du `launch_robot.sh` arrête proprement tous les nœuds (handler `cleanup` qui `pkill` + stoppe les moteurs via l'API easygopig
 