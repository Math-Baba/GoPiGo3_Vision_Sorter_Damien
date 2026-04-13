
# GoPiGo3 Vision Sorter

## Objectif
--------
Ce dépôt contient le package ROS/Python `robot_controller` qui contient des noeuds et utilitaires pour piloter un GoPiGo3 (Raspberry Pi + caméra) et trier des objets par couleur.

Ce README décrit l'arborescence réelle trouvée dans le dépôt, les points d'entrée disponibles, et des instructions de build/exécution adaptées.

Découverte actuelle (fichiers importants)


## Prérequis
- Raspberry Pi (Raspberry Pi OS ou Ubuntu)
- Robot GoPiGo3
- Python 3.8+
- OpenCV (`opencv-python`)
- ROS2 Jazzy
- Git


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

- Interface web (UI trouvée) :
	- `src/robot_controller/web2/index.html`  (UI dashboard)
	- NOTE : le `setup.py` référence `robot_controller/web` dans `data_files`, mais l'UI actuelle est dans `web2/`. Voir la section "Web / dashboard" ci-dessous.

Points d'entrée (console_scripts)
---------------------------------
Le `setup.py` expose plusieurs scripts installables via `ros2 run robot_controller <name>` :

- `gopigo3_driver = robot_controller.gopigo3_driver:main`
- `cube_detector = robot_controller.cube_detector:main`
- `cube_strategy = robot_controller.cube_strategy:main`
- `follow_test = robot_controller.follow_test:main`
- `arena_map = robot_controller.arena_map:main`
- `imu_node = robot_controller.imu_node:main`
- `dashboard_node = robot_controller.dashboard_node:main`

Installation rapide
-------------------

1) Placer le dépôt dans un workspace ROS2 (exemple : `~/ROS2_WS/src`) :

```bash
cd ~/ROS2_WS/src
git clone https://github.com/Math-Baba/GoPiGo3_Vision_Sorter_Damien.git
```

2) Installer les dépendances Python nécessaires (exemples courants) :

```bash
sudo apt install opencv-python numpy
```

Le package Python n'indique pas explicitement OpenCV dans `install_requires` (il contient `setuptools`), mais le code utilise OpenCV/Numpy dans plusieurs modules.

3) Construire le workspace ROS2 et sourcer :

```bash
cd ~/ROS2_WS
colcon build 
source install/setup.bash
```

Exécution des noeuds
--------------------

Lancer individuellement les noeuds exposés par `setup.py` :

```bash
ros2 run robot_controller gopigo3_driver
ros2 run robot_controller cube_detector
ros2 run robot_controller cube_strategy
ros2 run robot_controller imu_node
ros2 run robot_controller dashboard_node
```

Vous pouvez aussi lancer le script de lancement trouvé dans le package (attention : il utilise `ros2 run` et tue des processus) :

```bash
bash src/robot_controller/robot_controller/launch_robot.sh
```

Web / Dashboard
----------------

- L'UI dashboard découverte se trouve dans `src/robot_controller/web2/index.html`.
- Attention : `setup.py` copie des fichiers depuis `robot_controller/web` lors de l'installation, or le dossier présent est `web2/`. Si vous souhaitez servir l'UI via le package installé, copiez/renommez `web2/` en `robot_controller/web/` avant de builder, ou servez la page localement :

```bash
# servir l'UI en local (port 8000)
cd src/robot_controller/web2
python3 -m http.server 8000
# puis ouvrir http://<IP>:8000/index.html
```

Utilitaires et debug
--------------------

- Vérifier les topics publiés par un noeud :

```bash
ros2 topic list
```

- Lancer un noeud avec log verbeux :

```bash
RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG ros2 run robot_controller cube_detector
```

- Tests unitaires : il existe des tests dans `src/robot_controller/test/`. Pour exécuter les tests Python :

```bash
cd src/robot_controller
pytest
```

Notes importantes 
------------------------------------------

- Le `setup.py` ajoute des fichiers web depuis `robot_controller/web/` mais l'UI présente dans le dépôt est sous `web2/`. Si vous voulez que l'UI soit installée automatiquement lors du build, déplacez/dupliquez `web2/` en `robot_controller/web/` ou mettez à jour `setup.py`.
- `package.xml` contient `rclpy` comme dépendance ROS : assurez-vous d'avoir ROS2 installé et le workspace sourcé avant d'exécuter les noeuds.

Checklist de la tâche demandée
-----------------------------

- [x] Scanner l'arborescence du workspace et détecter les fichiers existants.
- [x] Générer un `README.md` fidèle aux fichiers présents.
- [x] Indiquer les points d'entrée (console_scripts) réellement définis dans `setup.py`.
- [x] Signaler la différence entre `web` et `web2` et proposer une solution simple.

Vérifier localement
-------------------

Afficher les 200 premières lignes du README (pour validation) :

```bash
sed -n '1,200p' README.md
```

Souhaitez-vous que je :
- corrige `setup.py` pour inclure `web2/` automatiquement ?
- ou crée un `requirements.txt` en extrayant les imports du code ?
- ou génère un petit script systemd / service pour démarrer les noeuds au boot ?

Dites-moi la prochaine action à effectuer et je l'exécuterai.
