# Robot Trieur de Cubes

Robot autonome **ROS 2 Jazzy** qui détecte, capture et dépose des cubes colorés dans des zones configurables depuis un dashboard web temps réel.

![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Python 3.12](https://img.shields.io/badge/Python-3.12-green)
![Plateforme](https://img.shields.io/badge/plateforme-GoPiGo3-orange)

<p align="center">
  <img src="assets/robot_démo.gif" alt="Demo" width="600"/>
</p>

## Sommaire

- [Présentation](#présentation)
- [Fonctionnalités](#fonctionnalités)
- [Architecture](#architecture)
- [Prérequis logiciel](#prérequis-logiciel)
- [Prérequis matériel](#prérequis-matériel)
- [Installation](#installation)
- [Lancement](#lancement)
- [Fonctionnement](#fonctionnement)
- [Dashboard web](#dashboard-web)
- [Paramètres de tuning](#paramètres-de-tuning)
- [Notes de conception](#notes-de-conception)
- [Limitations connues](#limitations-connues)
- [Améliorations futures](#améliorations-futures)
- [Auteurs](#auteurs)

## Présentation

Ce projet met en œuvre un robot mobile **Dexter Industries GoPiGo3** piloté par **ROS 2 Jazzy** sur **Raspberry Pi** (Ubuntu 24.04 / Python 3.12). Placé au centre d'une arène 2.4 m × 2.4 m délimitée par 4 murs, le robot doit trier des cubes colorés (rouge, vert, bleu, jaune) en les capturant avec un grabber à servo puis en les poussant contre le mur de dépôt correspondant à leur couleur.

Le mapping **couleur → mur** (A = +Y, B = -Y) est **configuré à chaud depuis le dashboard** avant le démarrage : l'opérateur coche les couleurs à trier et sélectionne la paroi associée à chacune.

La stratégie s'appuie sur deux capteurs :
- **Pi Camera (CSI)** : détection HSV des cubes avec pipeline CLAHE + morpho agressive.
- **IMU BNO085 (UART-RVC)** : cap absolu, seule source de position fiable sur carrelage glissant.

Les encodeurs roue GoPiGo3 publient une odométrie indicative (affichée sur le dashboard) mais ne sont **pas** utilisés par la stratégie — ils mentent en cas de patinage.

Si un scan 360° au centre ne trouve aucun cube, le robot entre en mode **EXPLORER** : il avance vers le quadrant gauche puis droit de l'arène, scanne 180° sur chaque zone, et revient au centre (ou s'arrête si les 2 zones sont vides).

## Fonctionnalités

- **Détection multi-couleurs** : seuillage HSV à 4 couleurs + CLAHE sur canal L (LAB) pour robustesse aux ombres, morpho `erode(3×3, ×2) → dilate(7×7, ×2)` style PyImageSearch pour détecter les cubes lointains.
- **Tracking temporel** : EMA sur (x, y, aire) + mémoire ghost 300 ms pour ne pas perdre la cible sur un flicker ponctuel.
- **Machine à états complète** : `CHERCHER → ALIGNER → APPROCHER → POUSSER → RETOUR_CENTRE → ORIENTER_ZONE → POUSSER_DROIT → RETOUR`.
- **Branche EXPLORER** avec sous-phases GOTO_LEFT / ADV_LEFT / SCAN_L1 / SCAN_L2 / DONE_LEFT (et symétrique pour RIGHT), avec reprise de scan exacte après fausse détection (préservation de la position).
- **Pause scan 3 s** aux limites de rotation : donne le temps à la confirmation de cube d'aboutir même si le cube est en bord de champ.
- **Zone morte CHERCHER** ±30° autour des caps de dépôt (évite de re-viser un cube déjà trié au mur).
- **Dashboard web** temps réel sur port 8080 : flux caméra annoté, badge d'état, compas IMU, config couleurs/murs live, boutons de contrôle.
- **Configuration live** : Start / Pause / Reset / Reset IMU depuis l'UI, choix des couleurs et du mur de dépôt sans redémarrer.

## Architecture

### Vue d'ensemble des nœuds ROS 2

```
         ┌──────────────────┐
         │ v4l2_camera_node │  (paquet ROS système)
         └────────┬─────────┘
                  │ /image_raw (rgb8)
         ┌────────┴──────────┬───────────────────┐
         ▼                   ▼                   │
 ┌───────────────┐   ┌────────────────┐          │
 │ cube_detector │   │ dashboard_node │          │
 └──────┬────────┘   └───┬─────┬──────┘          │
        │                │     │                 │
        │ /cube_detec-   │     │ /robot_control  │
        │ tions (JSON)   │     │ /imu/reset      │
        │                │     │                 │
        ▼                │     ▼                 │
 ┌───────────────┐       │  ┌──────────┐         │
 │ cube_strategy │◄──────┘  │ imu_node │◄────────┘  (/imu/reset)
 └──┬─────────┬──┘          └────┬─────┘
    │ /cmd_vel│                  │ /imu/heading
    │         │                  │ /imu/data
    ▼         │                  ▼
 ┌──────────────┐       (dashboard_node subscribe aussi)
 │ gopigo3_     │
 │ driver       │
 └──────┬───────┘
        │ /odom_simple
        │ /battery
        ▼
    (stratégie + dashboard)
```

### Rôle de chaque nœud

| Nœud | Publie | Souscrit | Fonction |
|------|--------|----------|----------|
| `v4l2_camera_node` | `/image_raw` | — | Flux Pi Camera via V4L2 (640×480, rgb8) |
| `gopigo3_driver` | `/battery`, `/odom_simple` | `/cmd_vel` | Pilote moteurs (rampe 50 dps/tick) + odométrie |
| `imu_node` | `/imu/heading`, `/imu/data` | `/imu/reset` | Cap yaw BNO085 UART-RVC (100 Hz) |
| `cube_detector` | `/cube_detections` | `/image_raw` | Détection HSV 4 couleurs + tracking EMA + ghost |
| `cube_strategy` | `/cmd_vel`, `/robot_status` | `/cube_detections`, `/imu/heading`, `/odom_simple`, `/robot_control` | Machine à états du tri |
| `dashboard_node` | `/robot_control`, `/imu/reset` (+ HTTP:8080) | `/robot_status`, `/cube_detections`, `/imu/heading`, `/image_raw` | Interface web + relais des commandes UI |

### Structure du repo

```
robot_controller/
├── package.xml                      # Manifeste ROS 2
├── setup.py                         # Entry points Python
├── setup.cfg
├── README.md
├── resource/robot_controller        # Marker ament
├── robot_controller/                # Code source des nœuds
│   ├── launch_robot.sh              # Script de lancement complet
│   ├── gopigo3_driver.py            # Moteurs + odométrie + batterie
│   ├── imu_node.py                  # IMU BNO085 UART-RVC
│   ├── cube_detector.py             # Détection HSV → /cube_detections
│   ├── cube_strategy.py             # Machine à états
│   └── dashboard_node.py            # HTTP + annotations image
├── web2/
│   └── index.html                   # Dashboard web (servi sur :8080)
└── test/                            # Utilitaires (hsv_calibration, etc.)
```

## Prérequis logiciel

- **Raspberry Pi OS** 64-bit compatible Ubuntu 24.04 (Noble) ou **Ubuntu 24.04** directement
- **ROS 2 Jazzy Jalisco**
- **Python 3.12**
- **OpenCV 4.6+** (`opencv-python`)
- Paquets ROS 2 :
  - `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`
  - `v4l2_camera`
- Paquets Python (pip) :
  - `easygopigo3` — API haut niveau GoPiGo3 (hérite de `gopigo3`, pulle la dépendance)
  - `numpy`, `pyserial`
- **colcon** pour la compilation du workspace

## Prérequis matériel

| Composant | Rôle |
|-----------|------|
| Raspberry Pi 4 (≥ 4 Go recommandé) | Unité de calcul |
| Carte microSD 32 Go classe 10+ | Stockage |
| Plateforme **Dexter Industries GoPiGo3** | Base mobile 2 roues motrices + encodeurs |
| **Pi Camera** (module CSI, nappe ribbon) 640×480 | Détection cubes |
| IMU **BNO085** (UART-RVC sur `/dev/ttyAMA0`) | Cap yaw absolu |
| Servo-moteur standard sur **SERVO1** du GoPiGo3 | Grabber (pince) |
| Batterie 9–12 V | Alimentation motrice |
| Arène physique 2.4 m × 2.4 m avec 4 murs | Terrain de jeu |
| Cubes colorés (rouge, vert, bleu, jaune) ~5–6 cm | Objets à trier |

## Installation

### 1. Configuration du Raspberry Pi / GoPiGo3

```bash
# 1. Flasher Raspberry Pi OS 64-bit ou Ubuntu Server 24.04

# 2. Installer la bibliothèque GoPiGo3 (Dexter Industries)
curl -kL dexterindustries.com/update_gopigo3 | bash
sudo reboot

# 3. Installer easygopigo3 (wrapper haut niveau)
sudo pip3 install easygopigo3 --break-system-packages

# 4. Activer l'UART série (pour l'IMU BNO085)
sudo raspi-config
#   → Interface Options → Serial Port
#       Login shell over serial : NO
#       Hardware serial enabled : YES
sudo reboot

# Vérifier le port série (ajouter l'utilisateur au groupe dialout si besoin)
ls -l /dev/ttyAMA0
sudo usermod -aG dialout $USER   # puis logout/login

# 5. Activer la caméra (si Pi Camera CSI)
sudo raspi-config
#   → Interface Options → Camera → Enable
sudo reboot
ls /dev/video*
```

### 2. Câblage

- **IMU BNO085** en mode UART-RVC :
  - `VIN` → 3.3 V Pi
  - `GND` → GND
  - `TX` (IMU) → `RX` Pi (GPIO 15)
  - `PS1` → 3.3 V (active le mode RVC)
- **Servo** grabber → port `SERVO1` du GoPiGo3
  - Ouvert = 130°, fermé = 0° (ajuster dans `cube_strategy.py` si besoin)
- **Pi Camera** → port CSI Pi (nappe ribbon, contacts métal côté HDMI)

### 3. Installation du projet

```bash
# 1. Installer ROS 2 Jazzy — voir https://docs.ros.org/en/jazzy/Installation.html

# 2. Paquet ROS additionnel
sudo apt install ros-jazzy-v4l2-camera

# 3. Dépendances Python
sudo pip3 install opencv-python numpy pyserial --break-system-packages

# 4. Cloner + build
mkdir -p ~/ROS2_WS/src
cd ~/ROS2_WS/src
git clone <URL_DU_REPO> robot_controller
cd ~/ROS2_WS
colcon build --packages-select robot_controller
source install/setup.bash

# 5. Auto-source recommandé dans ~/.bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/ROS2_WS/install/setup.bash' >> ~/.bashrc

# 6. Permission exécution du launcher
chmod +x ~/ROS2_WS/src/robot_controller/robot_controller/launch_robot.sh
```

## Lancement

1. **Positionner** le robot au **centre** de l'arène, **face au mur A** (cap de référence 0°).
2. **Lancer la stack complète** :
   ```bash
   bash ~/ROS2_WS/src/robot_controller/robot_controller/launch_robot.sh
   ```
   Le terminal affiche l'URL du dashboard (ex. `http://192.168.0.42:8080`). La stratégie démarre en **PAUSE** (badge jaune).
3. **Ouvrir le dashboard** dans un navigateur, cliquer **Reset IMU** pour zéroter le cap à la position courante.
4. **Configurer les couleurs** : cocher les couleurs à trier, sélectionner le mur de dépôt (A ou B) pour chacune.
5. Cliquer **Start** → passage ACTIVE, le robot commence à chercher.

**Arrêt** : `Ctrl+C` dans le terminal → cleanup kill tous les nœuds + stop moteurs via `easygopigo3`.

## Fonctionnement

### Machine à états principale

```
                                         (cube confirmé)
                                              │
                                              ▼
           ┌───── CHERCHER ────────── ALIGNER ──────── APPROCHER
           │         │                                     │
           │  (360° sans cube)                     (area > close_area)
           │         │                                     ▼
           │         ▼                                 POUSSER
           │     EXPLORER                                  │
           │    (LEFT/RIGHT_QUAD)                          ▼
           │         │                             RETOUR_CENTRE
           │   (cube trouvé)                               │
           │         └──► cycle grab ──┐                   ▼
           │                           │            ORIENTER_ZONE
           │   (les 2 quads vides)     │                   │
           │         │                 │                   ▼
           │         ▼                 │            POUSSER_DROIT
           │     TERMINÉ               │                   │
           │                           │                   ▼
           └───────────────────── RETOUR ◄─────────────────┘
```

### Détail des états

| État | Action | Sortie |
|------|--------|--------|
| **CHERCHER** | Rotation lente 0.6 rad/s sur place, zone morte ±30° autour des caps de dépôt. | Cube confirmé (3 frames) → ALIGNER. 360° sans cube → EXPLORER. |
| **ALIGNER** | Centre horizontalement le cube (PID pixel, tolérance ±40 px). | Centré → APPROCHER. Cube perdu → CHERCHER (ou RETOUR_CENTRE si avancé). |
| **APPROCHER** | Avance en maintenant le centrage, ralentit en approchant. | `cube.area > close_area` → POUSSER. |
| **POUSSER** | Fonce 3 s, ferme le servo à 2.7 s pour saisir le cube. | Timer écoulé → RETOUR_CENTRE. |
| **RETOUR_CENTRE** | Recul symétrique de la distance accumulée en APPROCHER+POUSSER. | Distance consommée → ORIENTER_ZONE (ou EXPLORER_RETURN_CENTER si grab pendant EXPLORER). |
| **ORIENTER_ZONE** | Rotation IMU vers le cap du mur de dépôt pour la couleur (0° ou ±180°). Hystérésis 3 frames. | Cap atteint → POUSSER_DROIT. |
| **POUSSER_DROIT** | Push continu `push_duration` (6 s) vers le mur, servo ouvert en fin. | Timer écoulé → RETOUR. |
| **RETOUR** | Recul symétrique vers le centre. | Centre atteint → CHERCHER (nouveau cycle). |

### Branche EXPLORER

Déclenchée quand CHERCHER fait 360° sans cube visible depuis le centre. Le robot visite les 2 quadrants de l'arène pour élargir sa zone de détection.

**Sous-phases** (stockées dans `_explorer_phase`) :

| Phase | Action | Durée |
|-------|--------|-------|
| `GOTO_LEFT` | Rotation vers cap 90° | variable |
| `ADV_LEFT` | Avance tout droit (CENTRE → LEFT_QUAD) | `explorer_adv_left_s` (4.5 s) |
| `SCAN_L1` | Rotation lente 90° → 0°, détection en continu | variable |
| `SCAN_L2` | Rotation lente 0° → 180° | variable |
| `DONE_LEFT` | `_left_explored = True`, enchaîne RIGHT | instantané |
| `GOTO_RIGHT` | Rotation vers cap -90° | variable |
| `ADV_RIGHT` | Avance vers RIGHT_QUAD | `explorer_adv_left_s` depuis centre, `explorer_adv_right_s` (9 s) depuis LEFT_QUAD |
| `SCAN_R1`, `SCAN_R2` | Scan symétrique (180° → -180° → 0°) | variable |
| `DONE_RIGHT` | `_right_explored = True` | instantané |
| `GOTO_CENTER`, `ADV_CENTER` (4.5 s), `FINAL_ORIENT`, `DONE_ALL` | Retour au centre + face mur A + auto-stop TERMINÉ | — |

**Pause grace period** : à chaque fin de SCAN (heading target atteint), le robot s'arrête 3 s caméra immobile (`EX_SCAN_PAUSE_S`). Un cube en bord de scan continue d'accumuler son confirm_count sur une caméra sans motion blur → rattrape une détection qui aurait été perdue par la transition trop rapide.

**Filtres scan EXPLORER plus stricts que CHERCHER** :
- `EX_SCAN_MIN_AREA = 600 px²` (vs MIN_AREA=400 global) — exclut petits reflets
- `EX_SCAN_CONFIRM = 5 frames` (vs 3 en CHERCHER) — confirmation plus prudente

**Reprise après fausse détection** : si un cube est confirmé en EXPLORER mais s'avère faux (perdu en ALIGNER/APPROCHER/RETOUR_CENTRE sans capture), la phase de scan interrompue est mémorisée dans `_explorer_resume_phase` et restaurée par le helper `_abandon_target()` → le robot reprend le scan exactement où il en était, sans refaire tout l'EXPLORER depuis GOTO_LEFT.

**Grab EXPLORER réussi** : après un dépôt d'un cube attrapé en EXPLORER, les flags `_left_explored` / `_right_explored` sont reset → nouvelle exploration depuis le centre (au cas où d'autres cubes seraient apparus / auraient été déplacés).

### Mapping couleurs → zones

Configurable **à chaud depuis le dashboard** (checkbox couleur + select mur A/B). La stratégie filtre les détections selon `sort_colors` et envoie vers `depot_heading[color]` en ORIENTER_ZONE. Par défaut : vert/bleu → A (cap 0°), rouge/jaune → B (cap ±180°).

## Dashboard web

Accessible sur `http://<IP_du_Pi>:8080`. Le serveur HTTP tourne dans un thread dédié du `dashboard_node`.

**Affichage :**
- Flux caméra annoté temps réel (bounding boxes cubes, ligne centrale, label d'état, cap IMU, overlay PAUSE pendant grace period EXPLORER)
- Badge d'état : **ACTIVE** (vert), **PAUSE** (jaune), **TERMINÉ** (bleu pulsant)
- Compas IMU (cap degré)
- Carte config couleurs/murs (cochable live)
- Carte EXPLORER : phase courante + statut L / R explorés

**Boutons :**

| Bouton | Endpoint | Action sur le robot |
|--------|----------|---------------------|
| **Start** | `POST /api/control` `{action: "start"}` | active=True, badge ACTIVE vert |
| **Pause** | `POST /api/control` `{action: "pause"}` | active=False, badge PAUSE jaune |
| **Reset** | `POST /api/control` `{action: "reset"}` | Reset compteurs + flags EXPLORER, retour CHERCHER |
| **Reset IMU** | `POST /api/imu_reset` | Republie sur `/imu/reset` → imu_node re-zéroise l'offset au prochain tick |
| **Config couleurs** | `POST /api/control` `{action: "config", ...}` | Met à jour `sort_colors` et `depot_heading` |

## Paramètres de tuning

Tous dans `cube_strategy.py`, bloc `=== PARAMETRES ===` :

| Paramètre | Valeur par défaut | Effet si augmenté |
|-----------|-------------------|-------------------|
| `search_turn_speed` | 0.6 rad/s | Rotation CHERCHER plus rapide (risque motion blur) |
| `close_area` | 3500 px² | Robot doit s'approcher plus avant POUSSER |
| `close_area_per_color` | `{}` | Overrides pour couleurs avec masque HSV atypique |
| `push_duration` | 6.0 s | POUSSER_DROIT pousse plus longtemps (plus près du mur) |
| `align_tolerance` | 40 px | Plus tolérant sur le centrage avant APPROCHER |
| `auto_stop_rotation_deg` | 360° | Scan CHERCHER plus long avant EXPLORER |
| `explorer_forward_speed` | 0.7 | EXPLORER avance plus vite (baisser si dérapage) |
| `explorer_adv_left_s` | 4.5 s | Distance CENTRE → QUAD plus longue |
| `EX_SCAN_PAUSE_S` (local) | 3.0 s | Pause grace period plus longue aux fins de scan |
| `EX_SCAN_MIN_AREA` (local) | 600 px² | Filtre EXPLORER plus strict (moins de faux positifs) |
| `EX_SCAN_CONFIRM` (local) | 5 frames | Confirmation EXPLORER plus prudente |

### Calibration HSV

Les plages HSV de chaque couleur sont dans `cube_detector.py`, constante `COLORS`. Pour calibrer de nouvelles couleurs / conditions d'éclairage, utiliser `test/hsv_calibration.py` qui ouvre une fenêtre OpenCV avec sliders, puis reporter les valeurs trouvées.

**Principe HSV :**
- **V (Value) bas** = capture les pixels sombres → bon pour cubes à l'ombre
- **S (Saturation) haut** = rejette surfaces peu saturées (murs blancs/gris, ArUco)
- **H (Teinte)** = identifie la couleur elle-même (rouge enjambe 0/180 donc split en 2 plages)

## Notes de conception

### Pas de détection de patinage

Sur carrelage, les roues peuvent tourner sans que le châssis ne bouge ; les encodeurs comptent les tours et mentent donc en cas de slip. Sans ArUco ni accéléromètre utilisable, on ne peut pas recalibrer la position. Conséquence :

- **Phases linéaires** (POUSSER_DROIT, RETOUR, avances EXPLORER) → timers (`push_duration`, `explorer_adv_*_s`) au lieu d'une distance mesurée.
- **Phases rotation** (ORIENTER_ZONE, EXPLORER scan) → on laisse l'IMU converger vers le cap target sans timeout, quitte à patienter si la rotation est lente — un abandon prématuré casserait l'invariant "chaque état démarre depuis une position connue".

### Conservation de distance (principe hérité de v6)

En APPROCHER/POUSSER on accumule `distance_forward += v·dt`. En RETOUR_CENTRE on décompte le même compteur à `push_speed` → le robot recule **exactement** ce qu'il a avancé, même si les vitesses diffèrent (les intégrales ∫v·dt se compensent). C'est ce qui permet de revenir au point de scan initial sans GPS ni recalibration externe.

### Double API easygopigo3 / gopigo3

`easygopigo3.EasyGoPiGo3` hérite de `gopigo3.GoPiGo3`, donc la même instance expose les deux APIs :
- **Bas niveau** (`set_motor_dps`, `read_encoders`, `volt`, constantes `MOTOR_LEFT`) pour le contrôle fin (rampe DPS).
- **Haut niveau** (`init_servo`, `stop`, `set_speed`) pour les opérations pratiques (grabber).

## Limitations connues

- **Carrelage glissant** : les cubes peuvent glisser imprévisiblement pendant POUSSER_DROIT. Le timer 6 s de push compense en poussant longtemps, mais le cube peut ne pas atterrir pile dans la zone de dépôt.
- **Cubes en plein contre-jour** : CLAHE aide mais un cube très à l'ombre ou très éclairé peut être manqué (seuil V inévitablement borné).
- **Cubes très proches des murs** : la zone morte ±30° empêche le push contre un mur perpendiculaire, mais peut aussi ignorer un cube légitime qui vient d'être poussé-manqué.
- **Deux cubes de même couleur dans le frame** : la logique `smooth_detections` garde seulement le plus gros par couleur — risque de "switcher" entre deux cubes proches au fil des frames.
- **Fausses détections en EXPLORER** : durcies par le filtre 5 frames + aire 600 px², mais si ça arrive quand même, reprise de phase activée (pas de position perdue mais 1 cycle allongé).

## Améliorations futures

- **Recovery de rotation bloquée** : si l'IMU ne tourne plus (stall angulaire), au lieu de hang, essayer l'angular dans le sens inverse (soit bump-back 1 s + retry, soit flip définitif jusqu'au target). Préserve la position contrairement à un "abandon" classique.
- **Détection à distance** : intégrer un modèle ML léger (TFLite) en complément du HSV pour les cubes lointains ou partiellement occultés.
- **Recalibration IMU au dépôt** : quand le cube touche le mur, le robot connaît précisément sa position (collé au mur à cap 0° ou ±180°) → recaler une odométrie virtuelle à ce moment-là.
- **Localisation ArUco** : un localiseur ArUco avait été prototypé puis retiré pour cause d'ambiguïté de pose (IPPE). À reprendre avec une 2e caméra dédiée ou un filtre Kalman si besoin de position absolue.
- **Détection de patinage avec accéléromètre** : le BNO085 expose aussi des accélérations linéaires — intégrer pour détecter l'absence de mouvement physique en phase linéaire.

## Auteurs

Projet réalisé en groupe de 3 étudiants :

- **Damien SALAMERO** — intégration hardware, code ROS 2 complet (5 nœuds + dashboard), machine à états, calibration HSV, stratégie EXPLORER
- **Jean-Paul SALLAH** — modélisation 3D (châssis, pièces servo/grabber, cubes)
- **Mathieu BABA** — gestion du dépôt GitHub, documentation

## Licence

Apache License 2.0 — voir `package.xml`.
