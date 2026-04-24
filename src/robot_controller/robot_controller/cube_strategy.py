"""Machine a etats du robot trieur (noeud principal).

Orchestration du cycle complet detection -> capture -> depot en s'appuyant
sur les autres noeuds du systeme:
  - /cube_detections (cube_detector)  : detections HSV des cubes visibles
  - /imu/heading     (imu_node)       : cap absolu (seule source de position
                                        fiable, l'odometrie roue ment en
                                        patinage)
  - /odom_simple     (gopigo3_driver) : odometrie indicative (dashboard)
  - /robot_control   (dashboard_node) : boutons UI (Start/Pause/Reset/Config)

Publie:
  - /cmd_vel         -> gopigo3_driver pour piloter les moteurs
  - /robot_status    -> dashboard pour affichage temps reel

Le cycle principal (loop @10Hz via un timer) dispatche sur un handler
d'etat specifique. Chaque etat lit les capteurs, decide sa sortie et
commande /cmd_vel en consequence.

Voir la docstring de la classe CubeStrategy pour le detail du cycle.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import time
import math
from std_msgs.msg import Float32, String


class CubeStrategy(Node):
    """Machine a etats principale du robot trieur.

    CYCLE NORMAL (cube visible depuis le centre):
      CHERCHER -> ALIGNER -> APPROCHER -> POUSSER (capture)
      -> RETOUR_CENTRE -> ORIENTER_ZONE -> POUSSER_DROIT (depot)
      -> RETOUR -> CHERCHER

    BRANCHE EXPLORATION (aucun cube au centre apres 360deg):
      CHERCHER -360deg-> EXPLORER (GOTO_LEFT/ADV/SCAN_L1/SCAN_L2/DONE_LEFT
                                   GOTO_RIGHT/ADV/SCAN_R1/SCAN_R2/DONE_RIGHT
                                   GOTO_CENTER/ADV_CENTER/FINAL_ORIENT/DONE_ALL)
      Si cube trouve pendant un SCAN -> ALIGNER -> ... -> RETOUR
        -> re-entre EXPLORER (flags L/R resetes) -> continue
      Si tous les cotes vides -> DONE_ALL -> auto-stop (badge TERMINE).

    PRINCIPE DE CONSERVATION DE DISTANCE (herite de v6):
      Pendant APPROCHER/POUSSER on accumule distance_forward += v*dt. 
      Pendant RETOUR_CENTRE on decompte le meme compteur a la meme vitesse 
      -> le robot recule exactement ce qu'il a avance, meme si les vitesses v different,
      parce que les integrales (v_up * t_up) == (v_down * t_down) quand le compteur atteint 0.

    HYPOTHESE DEMARRAGE: robot pose au CENTRE face au MUR A, IMU remis a zero
    (bouton Reset IMU du dashboard). Convention cap:
      heading =  0   -> face mur A
      heading = 90   -> face +X (LEFT_QUAD direction)
      heading = 180  -> face mur B
      heading = -90  -> face -X (RIGHT_QUAD direction)
    Le mapping couleur->mur est configurable via le dashboard (depot_heading).

    CONTROLE PAR DASHBOARD:
      Le robot demarre avec active=False (en PAUSE). 
      L'utilisateur doit cliquer Start sur le dashboard pour autoriser les mouvements. 
      Les actions supportees (/robot_control JSON):   start / pause / reset / config
    Voir control_cb() pour le detail.
    """

    def __init__(self):
        super().__init__('cube_strategy')

        # ==================== Controle UI (Start/Pause/Reset/Config) ====================
        # Robot demarre toujours en pause: l'utilisateur clique Start sur le
        # dashboard pour autoriser les mouvements. Voir control_cb().
        self.state = 'CHERCHER'
        self.active = False
        # Distingue une pause MANUELLE (badge PAUSE) d'une fin automatique
        # apres exploration complete sans cube (badge TERMINE).
        self.auto_stopped = False
        self.state_start_time = time.time()
        self.last_loop_time = time.time()

        # Config configurable via /api/control action=config (dashboard).
        # Par defaut: trie les 4 couleurs, depot vert/bleu -> mur A (cap 0),
        # rouge/jaune -> mur B (cap 180). Le prof peut changer ce mapping
        # au runtime depuis l'UI.
        self.sort_colors = ['green', 'blue', 'red', 'yellow']
        self.depot_heading = {
            'green': 0.0, 'blue': 0.0,
            'red': 180.0, 'yellow': 180.0,
        }

        # ==================== Etat cube cible ====================
        self.target_color = None     # couleur capturee (verrouillee apres CHERCHER)
        self.confirm_count = 0       # compteur de frames consecutives avec meme couleur
        self.confirm_color = None
        self.confirm_needed = 3      # N frames consecutifs avant de valider une detection
        self.detections = []         # brut (peut contenir des ghosts)
        self.real_detections = []    # filtre: pas de ghosts
        self.frame_width = 640
        self.last_cube_area = 0      # taille pixel du dernier cube vu (pour blind_spot)
        self.last_cube_color = None
        self.last_cube_time = 0
        self.last_detection_time = 0

        # ==================== PID pixel (utilise ALIGNER + APPROCHER) ====================
        self.integral = 0.0
        self.prev_error = 0.0

        # ==================== Push timing / heading ====================
        self.push_start = None       # t0 d'un etat POUSSER/POUSSER_DROIT
        self.push_color = None       # couleur du cube actuellement dans le grabber
        self.push_heading = 0.0      # cap memorise pour maintenir la trajectoire

        # ==================== Compteurs "distance parcourue" (integrale v*dt) ====================
        # On accumule quand on avance, on decompte a la meme vitesse quand on
        # recule -> le robot revient a sa position de depart. En unites arbitraires
        # (Twist * seconde), pas des metres reels - le principe marche quand meme
        # car c'est la meme conversion dans les 2 sens.
        self.distance_forward = 0.0  # ALIGNER/APPROCHER/POUSSER (+) ; RETOUR_CENTRE (-)
        self.distance_push = 0.0     # POUSSER_DROIT (+) ; RETOUR (-)

        # ==================== CHERCHER: tracking de rotation + auto-stop ====================
        # _chercher_rotation_done accumule les deltas IMU en valeur absolue.
        # Auto-stop (ou entree en EXPLORER) quand il depasse auto_stop_rotation_deg.
        self._chercher_start_heading = None
        self._chercher_rotation_done = 0.0
        self._chercher_last_heading = 0.0
        self._chercher_last_diag_log = 0.0

        # ==================== EXPLORER (hors centre) ====================
        # Deux points d'observation: LEFT_QUAD (+X) et RIGHT_QUAD (-X).
        # Chaque cote fait 2 scans sur un demi-cercle puis passe a True quand
        # il a ete explore une fois sans cube. Les flags se RESET apres chaque
        # depot reussi (un cube vient de changer de place -> re-verifier).
        self._explorer_phase = None
        self._left_explored = False
        self._right_explored = False
        self._explorer_last_side = None       # 'LEFT' ou 'RIGHT' (pour re-centrer apres grab)
        self._explorer_was_grabbing = False   # True pendant un cycle grab interrompu d'EXPLORER
        self._explorer_resume_phase = None    # phase a restaurer si le grab echoue (fausse detect)
        self._explorer_adv_start = None       # t0 d'une phase d'avance
        self._explorer_sub_rotated = None     # sous-etat de EXPLORER_RETURN_CENTER
        self._ex_scan_pause_start = None      # t0 de la pause "grace period" en fin de scan
        # Duree de ADV_RIGHT: depuis CENTRE (apres avoir skip LEFT ou apres depot)
        # = explorer_adv_left_s; depuis LEFT_QUAD (transition LEFT -> RIGHT direct
        # via le centre) = explorer_adv_right_s (~2x la distance).
        self._explorer_start_from_center = True

        # ==================== Hardware: servo grabber ====================
        import easygopigo3
        self.gpg = easygopigo3.EasyGoPiGo3()
        self.servo = self.gpg.init_servo('SERVO1')
        self.servo.rotate_servo(130)  # 130 deg = ouvert, 0 deg = ferme
        self.servo_closed = False

        # ==================== Capteurs ====================
        self.imu_heading = 0.0
        self.has_imu = False

        # Odom: utile pour afficher la position dans le dashboard (MAIS non
        # fiable pour detecter un patinage -> les encodeurs comptent les tours
        # meme quand les roues glissent sans avancer).
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.has_odom = False
        self._state_start_x = 0.0
        self._state_start_y = 0.0

        # Hysteresis pour imu_at_target: on exige N frames consecutifs au target
        # avant de transitionner (evite que le bruit IMU declenche trop tot).
        self._at_target_count = 0

        # ==================== PARAMETRES (tuning expose) ====================
        # Vitesses Twist (valeurs adim, voir gopigo3_driver.TWIST_SCALE pour conv)
        self.search_turn_speed = 0.6      # rad/s pendant CHERCHER/SCAN (assez lent pour detecter)
        self.max_forward_speed = 0.5      # APPROCHER loin du cube
        self.min_forward_speed = 0.3      # APPROCHER proche du cube
        self.push_speed = 0.8             # POUSSER et POUSSER_DROIT

        # PID pixel pour centrer le cube dans le frame (ALIGNER/APPROCHER)
        self.kp = 0.003
        self.ki = 0.0001
        self.kd = 0.002
        self.max_angular = 0.4

        # Seuils de detection cube (aire en pixels)
        self.align_tolerance = 40         # px d'erreur accepte avant de passer APPROCHER
        self.close_area = 3500            # aire declenchant POUSSER (cube "proche")
        self.blind_spot_area = 3000       # aire pour considerer un cube "imminent" (blind push)
        # Overrides par couleur (dict vide = toutes les couleurs utilisent le
        # seuil global ci-dessus). Utile si un cube d'une couleur donnee
        # ressort systematiquement plus petit/grand que les autres dans la
        # detection HSV. Ex historique: le jaune avait un override a 2200 car
        # son masque HSV etait plus etroit.
        self.close_area_per_color = {}
        # Blind spot: le jaune garde son override (masque plus petit quand le
        # cube tombe sous la camera en fin d'APPROCHER).
        self.blind_spot_area_per_color = {'yellow': 1800}

        # Timings
        self.push_duration = 6.0          # duree POUSSER_DROIT (pousse jusqu'au mur)
        self.lost_timeout = 4.0           # secondes avant de declarer le cube perdu
        self.state_timeout = 30.0         # timeout general ALIGNER/APPROCHER

        # Auto-stop: 1 tour complet (360 deg) en CHERCHER sans cube declenche EXPLORER.
        # Si les 2 cotes ont ete explores aussi sans succes -> auto-stop final.
        self.auto_stop_rotation_deg = 360.0

        # Durees (timer) des phases d'avance d'EXPLORER. Ajuster si la geometrie
        # de l'arene change (actuellement ~2.4m x 2.4m).
        self.explorer_adv_left_s = 4.5    # CENTRE -> LEFT_QUAD
        self.explorer_adv_right_s = 9.0   # LEFT_QUAD -> RIGHT_QUAD (traverse centre, 2x)
        self.explorer_adv_center_s = 4.5  # RIGHT_QUAD -> CENTRE (fin exploration)
        self.explorer_return_adv_s = 4.5  # LEFT/RIGHT_QUAD -> CENTRE apres grab
        # Vitesse d'avance en EXPLORER. A 0.7 (~12 cm/s sol) les moteurs ont assez
        # de couple pour franchir les joints de carreaux inegaux. A 0.4, le robot
        # patine sur place.
        self.explorer_forward_speed = 0.7

        # ==================== Stats (affichees dans dashboard) ====================
        self.cubes_sorted = {}
        self.total_sorted = 0

        # ==================== ROS ====================
        self.create_subscription(String, '/cube_detections', self.detection_cb, 10)
        self.create_subscription(Float32, '/imu/heading', self.imu_cb, 10)
        self.create_subscription(String, '/odom_simple', self.odom_cb, 10)
        self.create_subscription(String, '/robot_control', self.control_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.create_timer(0.1, self.loop)  # boucle principale a 10Hz

        self.get_logger().info(
            '=== CUBE STRATEGY pret - EN PAUSE (Start via dashboard) ==='
        )

    # ============================================================ Callbacks

    def imu_cb(self, msg):
        """Met a jour le cap depuis imu_node. msg.data est en degres [-180, +180]."""
        self.imu_heading = float(msg.data)
        self.has_imu = True

    def detection_cb(self, msg):
        """Recoit les cubes detectes depuis cube_detector.

        Met a jour last_cube_* pour le cube "pertinent" du moment:
        - Si target_color est defini (on est deja en poursuite), on track
          seulement les cubes de cette couleur.
        - Sinon (CHERCHER), on track le premier cube vu (utilise pour blind_spot).
        """
        data = json.loads(msg.data)
        self.detections = data.get('detections', [])
        self.frame_width = data.get('frame_width', 640)
        # On exclut les "ghosts" (detections fantomes issues du tracking): seuls
        # les cubes reellement visibles dans le frame courant comptent pour les
        # decisions de controle.
        self.real_detections = [d for d in self.detections if not d.get('ghost', False)]
        if self.real_detections:
            self.last_detection_time = time.time()
            if self.target_color:
                targs = [d for d in self.real_detections if d['color'] == self.target_color]
                if targs:
                    best = targs[0]
                    self.last_cube_area = best['area']
                    self.last_cube_color = best['color']
                    self.last_cube_time = time.time()
            else:
                best = self.real_detections[0]
                self.last_cube_area = best['area']
                self.last_cube_color = best['color']
                self.last_cube_time = time.time()

    def odom_cb(self, msg):
        """Met a jour la position estimee depuis gopigo3_driver."""
        data = json.loads(msg.data)
        self.odom_x = data.get('x', 0.0)
        self.odom_y = data.get('y', 0.0)
        self.has_odom = True

    def control_cb(self, msg):
        """Commandes depuis le dashboard web (via /api/control -> /robot_control).

        Payload JSON:
          {"action": "start"}                       -> active=True
          {"action": "pause"}                       -> active=False, stop moteurs
          {"action": "reset"}                       -> active=False, clean state + compteurs
          {"action": "config", "sorting": [...],
           "mapping": {"green": 0, "blue": 180}}    -> couleurs + mur de depot
        """
        try:
            cmd = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'[CTRL] bad payload: {e}')
            return
        action = cmd.get('action', '')

        if action == 'start':
            self.active = True
            self.auto_stopped = False
            self.get_logger().info('[CTRL] START')
        elif action == 'pause':
            self.active = False
            self.auto_stopped = False  # pause manuelle, pas un auto-stop
            self.stop()
            self.get_logger().info('[CTRL] PAUSE')
        elif action == 'reset':
            self.active = False
            self.auto_stopped = False
            self.stop()
            self.servo_open()
            self.state = 'CHERCHER'
            self.state_start_time = time.time()
            self.target_color = None
            self.push_color = None
            self.confirm_count = 0
            self.confirm_color = None
            self.distance_forward = 0.0
            self.distance_push = 0.0
            self.cubes_sorted = {}
            self.total_sorted = 0
            self._chercher_start_heading = None
            self._chercher_rotation_done = 0.0
            # Reset explorer state aussi
            self._explorer_phase = None
            self._left_explored = False
            self._right_explored = False
            self._explorer_last_side = None
            self._explorer_was_grabbing = False
            self._explorer_adv_start = None
            self._explorer_sub_rotated = None
            self._explorer_start_from_center = True
            self.get_logger().info('[CTRL] RESET - clean state + compteurs 0')
        elif action == 'config':
            sorting = cmd.get('sorting')
            mapping = cmd.get('mapping')
            if isinstance(sorting, list) and all(isinstance(c, str) for c in sorting):
                self.sort_colors = [c for c in sorting if c in ('green', 'blue', 'red', 'yellow')]
            if isinstance(mapping, dict):
                for color, heading in mapping.items():
                    if color not in ('green', 'blue', 'red', 'yellow'):
                        continue
                    try:
                        h = float(heading)
                    except Exception:
                        continue
                    # Snap au plus proche (0 ou 180)
                    self.depot_heading[color] = 0.0 if abs(h) < 90 else 180.0
            self.get_logger().info(
                f'[CTRL] Config sort={self.sort_colors} mapping={self.depot_heading}'
            )
        else:
            self.get_logger().warn(f'[CTRL] action inconnue: {action}')

    # ============================================================ Utilities

    def cmd(self, linear=0.0, angular=0.0):
        """Publie une commande Twist sur /cmd_vel (relaye vers gopigo3_driver)."""
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.pub.publish(t)

    def stop(self):
        """Arret moteurs (Twist(0,0))."""
        self.cmd(0.0, 0.0)

    def servo_close(self):
        """Ferme le grabber (0 deg). Idempotent."""
        if not self.servo_closed:
            self.servo.rotate_servo(0)
            self.servo_closed = True

    def servo_open(self):
        """Ouvre le grabber (130 deg). Idempotent."""
        if self.servo_closed:
            self.servo.rotate_servo(130)
            self.servo_closed = False

    def close_area_for(self, color):
        """Seuil d'aire pour declencher POUSSER (override par couleur)."""
        return self.close_area_per_color.get(color, self.close_area)

    def _abandon_target(self):
        """Appele quand un cube confirme s'est avere faux (perdu en ALIGNER/
        APPROCHER ou recul RETOUR_CENTRE sans capture).

        Si on etait en pleine phase EXPLORER (flag _explorer_was_grabbing),
        on reprend direct le scan sur la sous-phase interrompue memorisee
        dans _explorer_resume_phase. Sinon retour CHERCHER normal.
        """
        self.target_color = None
        if self._explorer_was_grabbing:
            self._explorer_was_grabbing = False
            resume = self._explorer_resume_phase or 'GOTO_LEFT'
            self._explorer_resume_phase = None
            self._explorer_phase = resume
            self._explorer_adv_start = None
            self._ex_scan_pause_start = None
            self.get_logger().info(f'[ABANDON] fausse detect -> reprise EXPLORER phase={resume}')
            self.change_state('EXPLORER')
        else:
            self.change_state('CHERCHER')

    def blind_spot_area_for(self, color):
        """Seuil d'aire pour considerer qu'un cube est imminent meme s'il sort
        brievement du frame (cube tombe sous le champ de la camera)."""
        return self.blind_spot_area_per_color.get(color, self.blind_spot_area)

    def publish_status(self):
        """Publie l'etat courant sur /robot_status (consomme par le dashboard)."""
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'active': self.active,
            'auto_stopped': self.auto_stopped,
            'target': self.target_color,
            'sorted': self.cubes_sorted,
            'total': self.total_sorted,
            'last_area': self.last_cube_area,
            'imu': round(self.imu_heading, 1),
            'odom_x': round(self.odom_x, 2),
            'odom_y': round(self.odom_y, 2),
            'dist_fwd': round(self.distance_forward, 2),
            'dist_push': round(self.distance_push, 2),
            'chercher_rot': round(self._chercher_rotation_done, 0),
            'auto_stop_deg': self.auto_stop_rotation_deg,
            'explorer_phase': self._explorer_phase,
            'ex_pausing': self._ex_scan_pause_start is not None,
            'left_explored': self._left_explored,
            'right_explored': self._right_explored,
            'sort_colors': self.sort_colors,
            'depot_heading': self.depot_heading,
        })
        self.status_pub.publish(msg)

    def get_target_cube(self):
        """Retourne le cube a traiter dans la frame courante.
        - Si target_color est fixe: le cube de cette couleur (sinon None).
        - Sinon: le cube le plus proche du centre horizontal (pour CHERCHER)."""
        if not self.real_detections:
            return None
        if self.target_color:
            targs = [d for d in self.real_detections if d['color'] == self.target_color]
            return targs[0] if targs else None
        center = self.frame_width // 2
        return min(self.real_detections, key=lambda d: abs(d['x'] - center))

    def target_lost(self):
        """True si on a perdu le cube qu'on poursuivait depuis plus de
        lost_timeout secondes."""
        if not self.target_color:
            return True
        if any(d['color'] == self.target_color for d in self.real_detections):
            return False
        return time.time() - self.last_cube_time > self.lost_timeout

    def in_blind_spot(self):
        """True si le cube vient de disparaitre du frame MAIS il etait tres
        proche juste avant. Indique que le cube est tombe "sous" la camera
        (angle mort juste devant le robot) -> on peut pousser en aveugle."""
        if self.target_color:
            no_target = not any(d['color'] == self.target_color for d in self.real_detections)
        else:
            no_target = len(self.real_detections) == 0
        threshold = self.blind_spot_area_for(self.target_color or self.last_cube_color)
        was_close = self.last_cube_area > threshold
        recent = (time.time() - self.last_cube_time) < 2.0
        return no_target and was_close and recent

    def state_timed_out(self):
        return time.time() - self.state_start_time > self.state_timeout

    @staticmethod
    def _norm(a):
        """Normalise un angle en degres dans [-180, +180]."""
        while a > 180: a -= 360
        while a < -180: a += 360
        return a

    def change_state(self, new_state):
        """Transition propre vers un nouvel etat: reset des integrateurs PID,
        des ancres (odom/imu) pour les mesures "depuis debut d'etat", et des
        sous-etats specifiques aux etats EXPLORER*.

        Regles de reset contextuelles:
          - new=CHERCHER + old=RETOUR: reset du compteur de rotation CHERCHER
            (cycle de tri complet acheve, on part pour un nouveau tour).
          - new=CHERCHER quel que soit old: clear _explorer_was_grabbing
            (meme si on y arrive par un chemin parasite type lost_cube).
          - new=EXPLORER / EXPLORER_RETURN_CENTER: reset des timers/flags
            de sous-phase pour que la nouvelle entree reparte propre.
        """
        old = self.state
        self.state = new_state
        self.state_start_time = time.time()
        self.last_loop_time = time.time()
        self.integral = 0.0
        self.prev_error = 0.0
        self._state_start_x = self.odom_x
        self._state_start_y = self.odom_y
        self._at_target_count = 0

        if new_state == 'CHERCHER':
            # Reset du compteur de rotation UNIQUEMENT apres un cycle complet
            # (RETOUR -> CHERCHER). Les detours brefs ALIGNER->CHERCHER (faux
            # positifs) conservent le compteur pour que l'auto-stop 360deg
            # se declenche correctement.
            if old == 'RETOUR':
                self._chercher_start_heading = None
                self._chercher_rotation_done = 0.0
            # On ne veut jamais arriver en CHERCHER avec le flag "j'etais en
            # EXPLORER" encore actif: il pollueraient la suite.
            self._explorer_was_grabbing = False

        if new_state == 'EXPLORER_RETURN_CENTER':
            self._explorer_sub_rotated = False
            self._explorer_adv_start = None

        if new_state == 'EXPLORER':
            self._explorer_adv_start = None
            self._ex_scan_pause_start = None

        self.get_logger().info(
            f'[{old}] --> [{new_state}] target={self.target_color} '
            f'dist_fwd={self.distance_forward:.2f} dist_push={self.distance_push:.2f}'
        )
        self.publish_status()

    def pid_angular(self, error):
        """PID classique sur l'erreur pixel (centrage du cube dans le frame).
        Utilise par ALIGNER et APPROCHER. Integral borne pour eviter le windup."""
        self.integral = max(-1000, min(1000, self.integral + error))
        derivative = error - self.prev_error
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.max_angular, min(self.max_angular, out))

    def imu_error_to(self, target_deg):
        """Erreur de cap courante par rapport a target, normalisee [-180, +180]."""
        return self._norm(target_deg - self.imu_heading)

    def imu_at_target(self, target_deg, tolerance=5):
        """True si le cap courant est dans [target - tolerance, target + tolerance].
        Cas special 180deg: on accepte aussi les caps proches de -180 (meme
        orientation physique, ecriture angulaire differente)."""
        if target_deg == 180.0 and abs(self.imu_heading) > (180 - tolerance):
            return True
        return abs(self.imu_error_to(target_deg)) < tolerance

    def imu_turn_toward(self, target_deg):
        """Pilote la rotation vers target_deg. Vitesse rapide (2.0) si loin,
        lente (0.7) en approche finale.
        Convention v6 (cable de ce robot): erreur positive -> angular negatif."""
        error = self.imu_error_to(target_deg)
        turn_speed = 2.0 if abs(error) > 45 else 0.7
        self.cmd(angular=-turn_speed if error > 0 else turn_speed)

    # ---- Stall detection angulaire (filet de securite) ----

    # ============================================================ Main loop

    def loop(self):
        """Boucle principale appelee a 10Hz par le timer.

        Publie TOUJOURS le status (pour l'UI) mais ne dispatche la machine
        a etats QUE si active=True. En pause, on ne publie aucun /cmd_vel
        -> gopigo3_driver coupe les moteurs apres 1s d'inactivite (timeout
        dans motor_loop).
        """
        self.publish_status()
        if not self.active:
            return
        handler = {
            'CHERCHER': self.state_chercher,
            'ALIGNER': self.state_aligner,
            'APPROCHER': self.state_approcher,
            'POUSSER': self.state_pousser,
            'RETOUR_CENTRE': self.state_retour_centre,
            'ORIENTER_ZONE': self.state_orienter_zone,
            'POUSSER_DROIT': self.state_pousser_droit,
            'RETOUR': self.state_retour,
            'EXPLORER': self.state_explorer,
            'EXPLORER_RETURN_CENTER': self.state_explorer_return_center,
        }.get(self.state)
        if handler:
            handler()

    # ============================================================ States

    def state_chercher(self):
        """Rotation sur place pour trouver un cube.

        Accumule les rotations depuis l'entree dans l'etat. Apres 360deg
        sans confirmation de cube, passe en mode EXPLORER (ou auto-stop si
        les 2 quads ont deja ete explores).

        Zone morte: quand le cap pointe vers un mur de depot (+/-30deg autour
        de 0 ou 180), on n'accepte pas de confirmation -> evite de re-ramasser
        un cube deja trie qui serait encore visible dans la zone de depot.
        """
        self.target_color = None

        # Accumulateur de rotation (en valeur absolue) depuis l'entree CHERCHER
        if self._chercher_start_heading is None and self.has_imu:
            self._chercher_start_heading = self.imu_heading
            self._chercher_rotation_done = 0.0
            self._chercher_last_heading = self.imu_heading
        if self.has_imu and self._chercher_start_heading is not None:
            delta = self._norm(self.imu_heading - self._chercher_last_heading)
            self._chercher_rotation_done += abs(delta)
            self._chercher_last_heading = self.imu_heading

        # Diag periodique (visible dans les logs et dans le chip "rot" du dashboard)
        now = time.time()
        if now - self._chercher_last_diag_log > 3.0:
            self.get_logger().info(
                f'[CHERCHER DIAG] rotation_done={self._chercher_rotation_done:.0f}deg '
                f'imu={self.imu_heading:.1f} total_sorted={self.total_sorted} '
                f'seuil_stop={self.auto_stop_rotation_deg:.0f}'
            )
            self._chercher_last_diag_log = now

        # Escalade: 360deg sans cube -> EXPLORER si disponible, sinon TERMINE
        if self._chercher_rotation_done >= self.auto_stop_rotation_deg:
            self.stop()
            if self._left_explored and self._right_explored:
                self.active = False
                self.auto_stopped = True
                self.get_logger().info(
                    f'[CHERCHER] {self._chercher_rotation_done:.0f}deg sans cube '
                    f'(2 cotes explores) apres {self.total_sorted} trie(s) '
                    f'-> AUTO-STOP TERMINE'
                )
                return
            self.get_logger().info(
                f'[CHERCHER] {self._chercher_rotation_done:.0f}deg sans cube '
                f'au centre -> EXPLORER'
            )
            self._enter_explorer()
            return

        # Zone morte devant les murs de depot (evite le re-pickup des cubes tries)
        if self.has_imu:
            h = abs(self.imu_heading)
            if h < 30 or h > 150:
                self.cmd(angular=self.search_turn_speed)
                self.confirm_count = 0
                self.confirm_color = None
                return

        # Filtre UI: ignore les cubes dont la couleur n'est pas dans la liste
        # des couleurs a trier (configurable via dashboard).
        cube = self.get_target_cube()
        if cube and cube['color'] not in self.sort_colors:
            cube = None

        if cube:
            # Hysteresis: 3 frames consecutifs avec la meme couleur avant confirmation
            if cube['color'] == self.confirm_color:
                self.confirm_count += 1
            else:
                self.confirm_color = cube['color']
                self.confirm_count = 1
            if self.confirm_count >= self.confirm_needed:
                self.stop()
                time.sleep(0.2)
                self.target_color = cube['color']
                self.confirm_count = 0
                self.confirm_color = None
                # NB: on ne reset PAS _chercher_start_heading/rotation_done ici.
                # Si ALIGNER echoue et qu'on revient en CHERCHER (faux positif),
                # on veut que le compteur continue plutot que de repartir a 0 a
                # chaque aller-retour parasite. Le vrai reset ne se fait qu'en
                # RETOUR -> CHERCHER (cycle de tri reellement acheve).
                self.get_logger().info(
                    f'[CHERCHER] {cube["color"]} CONFIRME (area={cube["area"]}) '
                    f'rotation_done={self._chercher_rotation_done:.0f}deg'
                )
                self.change_state('ALIGNER')
                return
        else:
            self.confirm_count = 0
            self.confirm_color = None

        self.cmd(angular=self.search_turn_speed)

    def state_aligner(self):
        """Centre le cube cible horizontalement dans le frame camera (PID pixel).

        Sorties possibles:
          - Cube centre (|error| < align_tolerance) -> APPROCHER.
          - Cube perdu (aucune detection depuis lost_timeout) -> retour arriere
            si on avait deja avance, sinon CHERCHER.
          - blind_spot (cube disparu mais etait tres proche juste avant) -> POUSSER.
          - Timeout (> state_timeout) -> CHERCHER.
        """
        cube = self.get_target_cube()
        if not cube:
            if self.in_blind_spot():
                # Cube tombe sous la camera, on fonce a l'aveugle pour le grab
                self.push_start = time.time()
                self.push_color = self.target_color
                self.change_state('POUSSER')
                return
            if self.target_lost():
                self.stop()
                if self.distance_forward > 0.05:
                    # On avait deja avance -> revenir au point de recherche
                    self.push_color = None
                    self.change_state('RETOUR_CENTRE')
                else:
                    self._abandon_target()
                return
            return
        if self.state_timed_out():
            self.stop()
            self._abandon_target()
            return
        center_x = self.frame_width // 2
        error = cube['x'] - center_x
        if abs(error) < self.align_tolerance:
            self.stop()
            time.sleep(0.2)
            # distance_forward commence a 0 ici: la phase APPROCHER partira
            # de "zero parcouru" pour que RETOUR_CENTRE puisse reculer pile
            # ce qui a ete avance depuis ALIGNER.
            self.distance_forward = 0.0
            self.get_logger().info(f'[ALIGNER] {self.target_color} aligne error={error}')
            self.change_state('APPROCHER')
            return
        # Correction angulaire proportionnelle a l'erreur pixel (signe inverse:
        # cube a droite -> error > 0 -> angular < 0 pour tourner vers la droite).
        self.cmd(angular=-self.pid_angular(error))

    def state_approcher(self):
        """Avance vers le cube en gardant le centrage (PID). Ralentit quand le
        cube grossit dans le frame (proche). Declenche POUSSER quand l'aire
        depasse close_area_for(color).

        Accumule distance_forward pour que RETOUR_CENTRE puisse reculer la
        meme distance si la capture echoue.
        """
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        cube = self.get_target_cube()

        if not cube:
            # Cube perdu mais tres proche -> push en aveugle
            if self.in_blind_spot():
                self.push_start = time.time()
                self.push_color = self.target_color
                self.get_logger().info('[APPROCHER] Blind spot -> POUSSER')
                self.change_state('POUSSER')
                return
            if self.target_lost():
                self.stop()
                if self.distance_forward > 0.05:
                    self.push_color = None
                    self.change_state('RETOUR_CENTRE')
                else:
                    self._abandon_target()
                return
            # Cube brievement hors frame (ghost memoire): on continue tout droit
            # a vitesse lente pour le retrouver (cube_detector rendra un ghost
            # pendant 300ms puis le vrai cube si encore visible).
            self.distance_forward += self.min_forward_speed * dt
            self.cmd(linear=self.min_forward_speed)
            return

        if self.state_timed_out():
            self.stop()
            if self.distance_forward > 0.05:
                self.push_color = None
                self.change_state('RETOUR_CENTRE')
            else:
                self._abandon_target()
            return

        center_x = self.frame_width // 2
        error = cube['x'] - center_x
        angular = -self.pid_angular(error)

        # Seuil de proximite "POUSSER": aire pixel du bounding box du cube
        close_th = self.close_area_for(self.target_color)
        if cube['area'] > close_th:
            self.push_color = self.target_color
            self.push_start = time.time()
            self.get_logger().info(
                f'[APPROCHER] {self.target_color} proche (area={cube["area"]}/{close_th})!'
            )
            self.change_state('POUSSER')
            return

        # Modulation de vitesse: plein gaz quand le cube est petit/loin, ralenti
        # progressivement en approche. Empeche de depasser le cube a pleine vitesse.
        area_ratio = min(cube['area'] / close_th, 1.0)
        speed = max(self.min_forward_speed, self.max_forward_speed * (1.0 - area_ratio * 0.6))

        # Si le cube est tres decentre (>120 px), on arrete d'avancer le temps
        # de le recentrer -> evite de passer a cote.
        if abs(error) > 120:
            self.cmd(angular=angular)
            return

        self.distance_forward += speed * dt
        # Reduction de l'angular en approche pour eviter de zigzaguer pres du cube
        self.cmd(linear=speed, angular=angular * (1.0 - area_ratio * 0.5))

    def state_pousser(self):
        """Capture: fonce tout droit pour embarquer le cube dans le grabber.

        Timeline:
          t=0      : memorise le cap actuel (servira a reculer droit)
          0-3s     : push en ligne droite a push_speed
          t=2.7s   : ferme le servo (le cube est presume dans le grabber)
          t=3.0s   : fin -> RETOUR_CENTRE
        """
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.push_start

        # Capture du cap au tout debut de POUSSER (avant que le robot avance)
        if elapsed < 0.2 and self.has_imu:
            self.push_heading = self.imu_heading

        # Comptabilise la distance parcourue en push pour que RETOUR_CENTRE
        # puisse reculer exactement autant.
        self.distance_forward += self.push_speed * dt

        if elapsed > 2.7:
            self.servo_close()

        if elapsed > 3.0:
            self.stop()
            time.sleep(0.3)
            self.get_logger().info(
                f'[CONTACT] {self.push_color} capture! heading={self.push_heading:.0f}'
            )
            self.change_state('RETOUR_CENTRE')
            return

        self.cmd(linear=self.push_speed)

    def state_retour_centre(self):
        """Recule a la meme vitesse jusqu'a consommer tout distance_forward.

        A la fin:
          - push_color defini + on etait en EXPLORER -> EXPLORER_RETURN_CENTER
            (on est au quad, il faut encore rentrer au centre arene).
          - push_color defini + cas normal -> ORIENTER_ZONE (deposit).
          - push_color vide (le cube avait ete perdu pendant ALIGNER/APPROCHER)
            -> on retourne juste en CHERCHER (pas de deposit a faire).

        Timeout 15s en secu. Pas de stall detection: les encodeurs mentent
        en patinage donc on ne peut pas detecter un "blocage" lineaire.
        """
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.state_start_time

        self.distance_forward -= self.push_speed * dt
        done = self.distance_forward <= 0 or elapsed > 15.0

        if done:
            self.stop()
            self.distance_forward = 0.0
            time.sleep(0.3)
            if self.push_color:
                if self._explorer_was_grabbing:
                    self.get_logger().info(
                        '[RETOUR_CENTRE] (explorer) -> EXPLORER_RETURN_CENTER'
                    )
                    self.change_state('EXPLORER_RETURN_CENTER')
                else:
                    self.get_logger().info('[RETOUR_CENTRE] -> ORIENTER_ZONE')
                    self.change_state('ORIENTER_ZONE')
            else:
                self.get_logger().info('[RETOUR_CENTRE] fausse detect')
                self._abandon_target()
            return

        # Recul ligne droite sans asservissement: avec angular=0 le driver diff
        # envoie la meme vitesse aux 2 roues -> trajectoire rectiligne mecanique.
        self.cmd(linear=-self.push_speed)

    def state_orienter_zone(self):
        """Rotation IMU vers le cap du mur de depot correspondant a la couleur
        du cube capture (lu dans self.depot_heading, configurable via UI).

        Sortie normale: 3 frames consecutifs dans la tolerance -> POUSSER_DROIT.
        Abandon (cube drop, retour CHERCHER): pas d'IMU disponible.
        """
        color = self.push_color or 'unknown'
        target = self.depot_heading.get(color, 0.0)

        if not self.has_imu:
            self.stop()
            self.push_color = None
            self.servo_open()
            self.change_state('CHERCHER')
            return

        # Hysteresis: on exige 3 frames au target avant de valider la rotation.
        # Evite de transitionner sur un seul frame de bruit IMU.
        if self.imu_at_target(target, tolerance=5):
            self._at_target_count += 1
            if self._at_target_count >= 3:
                self.stop()
                time.sleep(0.2)
                self._at_target_count = 0
                self.distance_push = 0.0
                self.push_start = time.time()
                self.push_heading = self.imu_heading
                self.get_logger().info(
                    f'[ORIENTER] cap={self.imu_heading:.0f} target={target:.0f} -> POUSSER_DROIT'
                )
                self.change_state('POUSSER_DROIT')
                return
            self.stop()
            return
        else:
            self._at_target_count = 0

        self.imu_turn_toward(target)

    def state_pousser_droit(self):
        """Pousse le cube jusqu'au mur de depot pendant push_duration secondes.

        A la fin: ouvre le servo (depot), incremente les compteurs, transitionne
        vers RETOUR pour que le robot recule du mur.

        Asservissement IMU volontairement ABSENT: si push_heading est legerement
        mal memorise ou si l'IMU drift, la correction peut tirer le robot
        lateralement. Avec les 2 roues a la meme vitesse (angular=0), la
        trajectoire est rectiligne par construction du driver differentiel.
        """
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.push_start
        color = self.push_color or 'unknown'

        self.distance_push += self.push_speed * dt

        if elapsed >= self.push_duration:
            self.stop()
            self.servo_open()  # lache le cube dans la zone de depot
            self.cubes_sorted[color] = self.cubes_sorted.get(color, 0) + 1
            self.total_sorted += 1
            self.get_logger().info(
                f'[POUSSER_DROIT] {color} livre! dist={self.distance_push:.2f} '
                f'Total: {self.total_sorted}'
            )
            self.change_state('RETOUR')
            return

        self.cmd(linear=self.push_speed)

    def state_retour(self):
        """Apres deposit, recule de la meme distance qu'on a pousse en
        POUSSER_DROIT. Sortie:
          - Si on etait en EXPLORER -> re-enter EXPLORER (flags L/R resetes
            car la scene a change avec le cube deplace).
          - Sinon -> CHERCHER (cycle normal).
        Timeout 15s en secu.
        """
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.state_start_time

        self.distance_push -= self.push_speed * dt
        done = self.distance_push <= 0 or elapsed > 15.0

        if done:
            self.stop()
            self.distance_push = 0.0
            time.sleep(0.2)
            self.last_cube_area = 0
            self.last_cube_color = None
            self.target_color = None
            self.push_color = None
            if self._explorer_was_grabbing:
                # Un cube vient d'etre deplace (capture a un quad -> depose au
                # mur). La configuration visible des cubes a change, on re-verifie
                # les 2 cotes proprement depuis le debut.
                self._explorer_was_grabbing = False
                self._left_explored = False
                self._right_explored = False
                self.get_logger().info(
                    '[RETOUR] Termine -> EXPLORER (resume, flags L/R reset)'
                )
                self._enter_explorer()
            else:
                self.get_logger().info('[RETOUR] Termine -> CHERCHER')
                self.change_state('CHERCHER')
            return

        self.cmd(linear=-self.push_speed)


    # ============================================================ EXPLORER
    #
    # Machine a sous-etats quand il n'y a plus de cube au centre. Visite LEFT_QUAD
    # puis (si necessaire) RIGHT_QUAD, scanne un demi-cercle a chaque quad, et
    # si rien n'est trouve termine par un retour au centre + auto-stop.
    # Les phases sont stockees dans self._explorer_phase (string), dispatch
    # dans state_explorer().

    def _enter_explorer(self):
        """Entre (ou re-entre) dans EXPLORER. Choisit la phase de depart en
        fonction des drapeaux _left_explored / _right_explored.

        Appele depuis:
          - state_chercher (apres 360deg sans cube)
          - state_retour (apres un depot reussi, pour reprendre l'exploration)
        Dans les deux cas le robot est suppose au CENTRE arene apres le dernier
        retour -> _explorer_start_from_center = True (ADV vers un quad dure
        explorer_adv_left_s, pas explorer_adv_right_s).
        """
        self._explorer_start_from_center = True
        if not self._left_explored:
            self._explorer_phase = 'GOTO_LEFT'
            self._explorer_last_side = 'LEFT'
        elif not self._right_explored:
            self._explorer_phase = 'GOTO_RIGHT'
            self._explorer_last_side = 'RIGHT'
        else:
            # Les 2 cotes deja explores -> final: retour au centre + face mur A
            self._explorer_phase = 'GOTO_CENTER'
        self._explorer_adv_start = None
        self.change_state('EXPLORER')

    def _ex_rotate_to(self, target_deg, next_phase):
        """Helper: rotation IMU vers target, transition vers next_phase quand
        atteint (tolerance 5 deg)."""
        if self.imu_at_target(target_deg, tolerance=5):
            self.stop()
            time.sleep(0.2)
            self._explorer_phase = next_phase
            self._explorer_adv_start = None
            return
        self.imu_turn_toward(target_deg)

    def _ex_advance(self, duration_s, next_phase):
        """Helper: avance tout droit pendant duration_s secondes (timer pur).

        Pas de stall detection lineaire: les encodeurs roues comptent meme
        pendant le patinage, impossible de detecter un blocage. La duree
        max limite naturellement la phase.
        """
        if self._explorer_adv_start is None:
            self._explorer_adv_start = time.time()
        elapsed = time.time() - self._explorer_adv_start
        if elapsed >= duration_s:
            self.stop()
            self._explorer_adv_start = None
            self._explorer_phase = next_phase
            return
        self.cmd(linear=self.explorer_forward_speed)

    def _ex_scan(self, target_deg, next_phase):
        """Helper: rotation lente vers target en surveillant les cubes.
        Si cube confirme (5 frames + aire >= 600 + couleur dans sort_colors)
        -> ALIGNER, le flag _explorer_was_grabbing permet au cycle de
        grab/depot de revenir ensuite en EXPLORER.

        Filtres plus stricts que CHERCHER (5 frames, aire >= 600) pour ne
        pas se faire piéger par un reflet ou une tache de couleur sur un
        mur: une fausse détection en EXPLORER coûte cher.

        En cas de fausse détection (cube perdu en ALIGNER/APPROCHER/RETOUR),
        _abandon_target restaure _explorer_phase depuis _explorer_resume_phase
        memorise ci-dessous, donc on reprend le scan sur la sous-phase courante
        au lieu de relancer EXPLORER from scratch.

        IMPORTANT: PAS de zone morte ici. Depuis LEFT_QUAD ou RIGHT_QUAD
        (decales de l'axe central), les zones de depot (60x60cm centrees
        sur les murs) ne sont plus alignees avec le cap 0 ou 180. Le
        demi-cercle de scan couvre donc des cubes legitimement posables.
        """
        EX_SCAN_MIN_AREA = 600
        EX_SCAN_CONFIRM = 5

        cube = self.get_target_cube()
        if cube and cube['color'] not in self.sort_colors:
            cube = None
        if cube and cube.get('area', 0) < EX_SCAN_MIN_AREA:
            cube = None

        if cube:
            if cube['color'] == self.confirm_color:
                self.confirm_count += 1
            else:
                self.confirm_color = cube['color']
                self.confirm_count = 1
            if self.confirm_count >= EX_SCAN_CONFIRM:
                self.stop()
                time.sleep(0.2)
                self.target_color = cube['color']
                self.confirm_count = 0
                self.confirm_color = None
                # Flag pour que RETOUR_CENTRE et RETOUR sachent qu'on etait
                # en EXPLORER et rebranchent correctement a la fin du cycle.
                self._explorer_was_grabbing = True
                # Memorise la sous-phase du scan pour reprendre ici si fausse detect
                self._explorer_resume_phase = self._explorer_phase
                self.get_logger().info(
                    f'[EXPLORER] {cube["color"]} CONFIRME (area={cube["area"]}) '
                    f'phase={self._explorer_phase} -> ALIGNER'
                )
                self.change_state('ALIGNER')
                return
        else:
            self.confirm_count = 0
            self.confirm_color = None

        if self.imu_at_target(target_deg, tolerance=5):
            # Grace period: cube en bord de scan (detection pas encore confirmee
            # car rotation trop rapide / frame loupee) a le temps de s'accumuler
            # pendant EX_SCAN_PAUSE_S camera immobile avant de quitter la phase.
            EX_SCAN_PAUSE_S = 3.0
            if self._ex_scan_pause_start is None:
                self._ex_scan_pause_start = time.time()
                self.get_logger().info(
                    f'[EXPLORER SCAN] pause {EX_SCAN_PAUSE_S:.1f}s @ target={target_deg:.0f} '
                    f'(next={next_phase})'
                )
            self.stop()
            elapsed = time.time() - self._ex_scan_pause_start
            if elapsed >= EX_SCAN_PAUSE_S:
                self.get_logger().info(
                    f'[EXPLORER SCAN] fin pause ({elapsed:.2f}s) -> {next_phase}'
                )
                self._ex_scan_pause_start = None
                self._explorer_phase = next_phase
            return

        # Rotation lente pour laisser le temps de detecter les cubes
        error = self.imu_error_to(target_deg)
        speed = self.search_turn_speed
        # Convention v6 (signe du cablage): erreur positive -> angular negatif
        self.cmd(angular=-speed if error > 0 else speed)

    def state_explorer(self):
        """Dispatcher des sous-phases d'exploration. Chaque phase est identifiee
        par un label string dans self._explorer_phase:

          GOTO_LEFT    - rotation vers cap 90
          ADV_LEFT     - avance explorer_adv_left_s (centre -> LEFT_QUAD)
          SCAN_L1      - scan vers 0 (en surveillant les cubes)
          SCAN_L2      - scan vers 180 (demi-cercle 180deg total)
          DONE_LEFT    - marque left_explored = True, enchaine la suite

          GOTO_RIGHT   - rotation vers cap -90
          ADV_RIGHT    - avance explorer_adv_left_s (depuis centre) ou
                         explorer_adv_right_s (depuis LEFT_QUAD, ~2x plus loin)
          SCAN_R1      - scan vers -180
          SCAN_R2      - scan vers 0
          DONE_RIGHT   - marque right_explored = True

          GOTO_CENTER  - rotation vers cap 90 (pour retour depuis RIGHT_QUAD)
          ADV_CENTER   - avance explorer_adv_center_s (RIGHT_QUAD -> centre)
          FINAL_ORIENT - rotation vers cap 0 (face mur A pour fin propre)
          DONE_ALL     - auto-stop avec badge TERMINE
        """
        phase = self._explorer_phase

        if phase == 'GOTO_LEFT':
            self._explorer_last_side = 'LEFT'
            self._ex_rotate_to(90.0, 'ADV_LEFT')
        elif phase == 'ADV_LEFT':
            self._ex_advance(self.explorer_adv_left_s, 'SCAN_L1')
        elif phase == 'SCAN_L1':
            self._ex_scan(0.0, 'SCAN_L2')
        elif phase == 'SCAN_L2':
            self._ex_scan(180.0, 'DONE_LEFT')
        elif phase == 'DONE_LEFT':
            self._left_explored = True
            self.get_logger().info('[EXPLORER] cote GAUCHE termine sans nouveau cube')
            if not self._right_explored:
                self._explorer_phase = 'GOTO_RIGHT'
                self._explorer_last_side = 'RIGHT'
                # On part de LEFT_QUAD donc il faut explorer_adv_right_s
                # (~2x la demi-arene) pour atteindre RIGHT_QUAD en traversant.
                self._explorer_start_from_center = False
            else:
                self._explorer_phase = 'GOTO_CENTER'
            self._explorer_adv_start = None

        elif phase == 'GOTO_RIGHT':
            self._explorer_last_side = 'RIGHT'
            self._ex_rotate_to(-90.0, 'ADV_RIGHT')
        elif phase == 'ADV_RIGHT':
            # Depuis le centre: explorer_adv_left_s (meme duree que GOTO_LEFT).
            # Depuis LEFT_QUAD (apres DONE_LEFT): explorer_adv_right_s pour traverser tout.
            dur = (self.explorer_adv_left_s
                   if self._explorer_start_from_center
                   else self.explorer_adv_right_s)
            self._ex_advance(dur, 'SCAN_R1')
        elif phase == 'SCAN_R1':
            self._ex_scan(-180.0, 'SCAN_R2')
        elif phase == 'SCAN_R2':
            self._ex_scan(0.0, 'DONE_RIGHT')
        elif phase == 'DONE_RIGHT':
            self._right_explored = True
            self.get_logger().info('[EXPLORER] cote DROIT termine sans nouveau cube')
            if not self._left_explored:
                self._explorer_phase = 'GOTO_LEFT'
                self._explorer_last_side = 'LEFT'
            else:
                self._explorer_phase = 'GOTO_CENTER'
            self._explorer_adv_start = None

        elif phase == 'GOTO_CENTER':
            self._ex_rotate_to(90.0, 'ADV_CENTER')
        elif phase == 'ADV_CENTER':
            self._ex_advance(self.explorer_adv_center_s, 'FINAL_ORIENT')
        elif phase == 'FINAL_ORIENT':
            self._ex_rotate_to(0.0, 'DONE_ALL')
        elif phase == 'DONE_ALL':
            self.stop()
            self.active = False
            self.auto_stopped = True
            self.get_logger().info(
                f'[EXPLORER] Exploration complete (2 cotes vides) apres '
                f'{self.total_sorted} cube(s) trie(s) -> AUTO-STOP TERMINE'
            )

        else:
            # Phase inconnue, fallback
            self.get_logger().warn(f'[EXPLORER] Phase inconnue: {phase}')
            self.active = False
            self.auto_stopped = True

    def state_explorer_return_center(self):
        """Apres un grab declenche depuis un quad (LEFT ou RIGHT), RETOUR_CENTRE
        nous a ramene au quad (pas au centre arene). Cet etat fait le pont:

          Phase 1: rotation vers le cap oppose au quad
                   LEFT (etait arrive par cap 90)  -> rotation vers -90
                   RIGHT (etait arrive par cap -90) -> rotation vers 90
          Phase 2: avance explorer_return_adv_s secondes -> centre arene

        Puis transition vers ORIENTER_ZONE (deposit normal).
        """
        if self._explorer_last_side == 'LEFT':
            target_heading = -90.0
        else:  # RIGHT (ou None, fallback sur 90)
            target_heading = 90.0

        # Phase 1: rotation vers le cap oppose
        if not self._explorer_sub_rotated:
            if self.imu_at_target(target_heading, tolerance=5):
                self.stop()
                time.sleep(0.2)
                self._explorer_sub_rotated = True
                self._explorer_adv_start = time.time()
                self.get_logger().info(
                    f'[EXPLORER_RETURN] rotation OK cap={target_heading:.0f}, avance'
                )
                return
            self.imu_turn_toward(target_heading)
            return

        # Phase 2: avance explorer_return_adv_s secondes pour rejoindre le centre
        elapsed = time.time() - (self._explorer_adv_start or time.time())
        if elapsed >= self.explorer_return_adv_s:
            self.stop()
            self.get_logger().info('[EXPLORER_RETURN] centre atteint -> ORIENTER_ZONE')
            self.change_state('ORIENTER_ZONE')
            return
        # Pas de stall detection (voir _ex_advance).
        self.cmd(linear=self.explorer_forward_speed)


def main(args=None):
    rclpy.init(args=args)
    node = CubeStrategy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
