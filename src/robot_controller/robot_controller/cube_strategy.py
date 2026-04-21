import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import time
import math
from std_msgs.msg import Float32, String


class CubeStrategy(Node):
    """
    Strategie v6.1 - v6 IMU-only + verification ArUco au depot uniquement.

    Base v6: CHERCHER -> ALIGNER -> APPROCHER -> POUSSER(contact)
             -> RETOUR_CENTRE -> (RETOUR_ARENE si offset) -> ORIENTER_ZONE
             -> POUSSER_DROIT -> RETOUR -> CHERCHER

    Exploration: si rien trouve apres 360 IMU:
    CHERCHER -> EXPLORER(90) -> CHERCHER -> EXPLORER(-90) -> CHERCHER

    Hypothese de demarrage: robot face mur A, heading=0 au launch.

    Ajouts ArUco (minimaux, uniquement pour le depot):
      - ORIENTER_ZONE: apres avoir atteint le cap IMU (0 ou 180), centre le
        marker du mur de depot (M0 pour vert/bleu, M2 pour rouge/jaune) via
        l'offset pixel. Si marker invisible apres quelques secondes, on passe
        quand meme.
      - POUSSER_DROIT: arret anticipe quand distance(marker) < 0.20 m. Sinon
        fallback timer (push_duration).
    """

    def __init__(self):
        super().__init__('cube_strategy')

        self.state = 'CHERCHER'
        self.push_start = None
        self.push_color = None
        self.target_color = None
        self.confirm_count = 0
        self.confirm_color = None
        self.confirm_needed = 3
        self.last_detection_time = 0
        self.state_start_time = time.time()
        self.detections = []
        self.real_detections = []
        self.frame_width = 640
        self.distance_push = 0.0
        self.distance_forward = 0.0
        self.last_loop_time = time.time()

        # Exploration
        self.search_heading = None
        self.search_time = 3.0
        self.search_offset = False
        self.full_rotations = 0
        self._explore_phase = 'TOURNER'
        self._retour_phase = 'TOURNER'

        # Rotation tracking CHERCHER (IMU 360)
        self._chercher_start_heading = None
        self._chercher_rotation_done = 0.0
        self._chercher_last_heading = 0.0

        # Push heading pour correction recul
        self.push_heading = 0.0

        # Servo
        import easygopigo3
        self.gpg = easygopigo3.EasyGoPiGo3()
        self.servo = self.gpg.init_servo('SERVO1')
        self.servo.rotate_servo(130)
        self.servo_closed = False

        # Blind spot
        self.last_cube_area = 0
        self.last_cube_color = None
        self.last_cube_time = 0

        # IMU
        self.imu_heading = 0.0
        self.has_imu = False

        # ArUco (juste pour verifier le depot)
        self.aruco_marker_id = -1
        self.aruco_distance = 0.0
        self.aruco_confidence = 0.0
        self.aruco_age = 99.0
        self.aruco_marker_offset_px = 0.0

        # Odometrie roue (pour remplacer le timer-based dead reckoning qui ment
        # quand le robot est coince sur des carreaux inegaux)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.has_odom = False
        # Reset a chaque change_state: permet de mesurer la distance / rotation
        # reelle accomplie depuis le debut de l'etat courant
        self._state_start_x = 0.0
        self._state_start_y = 0.0
        self._state_start_imu = 0.0
        # Hysteresis pour imu_at_target: evite de transitionner sur un seul
        # frame "au target" (bruit IMU) - on demande plusieurs frames consecutifs
        self._at_target_count = 0

        # === PARAMETRES ===
        self.search_turn_speed = 0.6
        self.max_forward_speed = 0.5
        self.min_forward_speed = 0.3
        self.push_speed = 0.8

        # PID
        self.kp = 0.003
        self.ki = 0.0001
        self.kd = 0.002
        self.integral = 0.0
        self.prev_error = 0.0
        self.max_angular = 0.4

        # Seuils
        self.align_tolerance = 40
        self.close_area = 4000
        self.blind_spot_area = 3000
        # Overrides par couleur: le masque HSV du jaune est plus etroit que la
        # taille reelle du cube (luminance/saturation qui mangent les bords),
        # donc l'area observee plafonne vers 2500 au lieu de 4000+ comme les
        # autres couleurs. On abaisse les seuils pour que POUSSER declenche
        # bien quand le cube est reellement proche.
        self.close_area_per_color = {
            'yellow': 2200,
        }
        self.blind_spot_area_per_color = {
            'yellow': 1800,
        }
        self.push_duration = 6.0
        self.lost_timeout = 4.0
        self.state_timeout = 30.0

        # Depot ArUco
        self.depot_stop_dist_m = 0.20
        self.depot_marker_center_tol_px = 30
        self.depot_marker_id = {
            'green': 0, 'blue': 0,
            'red': 2, 'yellow': 2,
        }
        self._orienter_phase = 'ROTATE'

        self.cubes_sorted = {}
        self.total_sorted = 0

        # ROS
        self.det_sub = self.create_subscription(String, '/cube_detections', self.detection_cb, 10)
        self.imu_sub = self.create_subscription(Float32, '/imu/heading', self.imu_cb, 10)
        self.aruco_sub = self.create_subscription(String, '/robot_position', self.aruco_cb, 10)
        self.odom_sub = self.create_subscription(String, '/odom_simple', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info('=== CUBE SORTER v6.2 (IMU + ArUco depot + odom stall) ===')

    # === CALLBACKS ===

    def imu_cb(self, msg):
        self.imu_heading = msg.data
        self.has_imu = True

    def detection_cb(self, msg):
        data = json.loads(msg.data)
        self.detections = data.get('detections', [])
        self.frame_width = data.get('frame_width', 640)
        self.real_detections = [d for d in self.detections if not d.get('ghost', False)]
        if self.real_detections:
            self.last_detection_time = time.time()
            if self.target_color:
                target_dets = [d for d in self.real_detections if d['color'] == self.target_color]
                if target_dets:
                    best = target_dets[0]
                    self.last_cube_area = best['area']
                    self.last_cube_color = best['color']
                    self.last_cube_time = time.time()
            else:
                best = self.real_detections[0]
                self.last_cube_area = best['area']
                self.last_cube_color = best['color']
                self.last_cube_time = time.time()

    def aruco_cb(self, msg):
        data = json.loads(msg.data)
        self.aruco_marker_id = data.get('marker_id', -1)
        self.aruco_distance = data.get('distance', 0.0)
        self.aruco_confidence = data.get('confidence', 0.0)
        self.aruco_age = data.get('age', 99.0)
        self.aruco_marker_offset_px = data.get('marker_offset_px', 0.0)

    def odom_cb(self, msg):
        data = json.loads(msg.data)
        self.odom_x = data.get('x', 0.0)
        self.odom_y = data.get('y', 0.0)
        self.has_odom = True

    # === UTILITAIRES ===

    def cmd(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.pub.publish(t)

    def stop(self):
        self.cmd(0.0, 0.0)

    def servo_close(self):
        if not self.servo_closed:
            self.servo.rotate_servo(0)
            self.servo_closed = True

    def servo_open(self):
        if self.servo_closed:
            self.servo.rotate_servo(130)
            self.servo_closed = False

    def has_marker(self, marker_id):
        """True si on voit actuellement le marker demande, recent et fiable."""
        return (
            self.aruco_confidence > 0.2
            and self.aruco_age < 1.0
            and self.aruco_marker_id == marker_id
        )

    def close_area_for(self, color):
        """Seuil d'area pour declencher POUSSER, ajuste par couleur (jaune detecte
        plus petit que les autres)."""
        return self.close_area_per_color.get(color, self.close_area)

    def blind_spot_area_for(self, color):
        """Seuil d'area pour considerer qu'un cube vient de passer en dessous
        de la camera (blind spot), ajuste par couleur."""
        return self.blind_spot_area_per_color.get(color, self.blind_spot_area)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'target': self.target_color,
            'sorted': self.cubes_sorted,
            'total': self.total_sorted,
            'last_area': self.last_cube_area,
            'imu': round(self.imu_heading, 1),
            'offset': self.search_offset,
            'aruco_m': self.aruco_marker_id,
            'aruco_d': round(self.aruco_distance, 2),
            'aruco_conf': round(self.aruco_confidence, 2),
        })
        self.status_pub.publish(msg)

    def get_target_cube(self):
        if not self.real_detections:
            return None
        if self.target_color:
            targets = [d for d in self.real_detections if d['color'] == self.target_color]
            return targets[0] if targets else None
        else:
            center = self.frame_width // 2
            return min(self.real_detections, key=lambda d: abs(d['x'] - center))

    def target_lost(self):
        if not self.target_color:
            return True
        if any(d['color'] == self.target_color for d in self.real_detections):
            return False
        return time.time() - self.last_cube_time > self.lost_timeout

    def in_blind_spot(self):
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

    def change_state(self, new_state):
        old = self.state
        self.state = new_state
        self.state_start_time = time.time()
        self.last_loop_time = time.time()
        self.integral = 0.0
        self.prev_error = 0.0
        # Snapshot odom/imu pour calculer distance/rotation reelle dans cet etat
        self._state_start_x = self.odom_x
        self._state_start_y = self.odom_y
        self._state_start_imu = self.imu_heading
        self._at_target_count = 0
        if new_state == 'CHERCHER':
            self._chercher_start_heading = None
            self._chercher_rotation_done = 0.0
        if new_state == 'ORIENTER_ZONE':
            self._orienter_phase = 'ROTATE'
        self.get_logger().info(f'[{old}] --> [{new_state}] target={self.target_color}')
        self.publish_status()

    # Odometrie / detection de stall - base sur capteurs reels, pas sur timer

    def odom_distance_traveled(self):
        """Distance reelle parcourue depuis le debut de l'etat (metres)."""
        if not self.has_odom:
            return 0.0
        return math.hypot(
            self.odom_x - self._state_start_x,
            self.odom_y - self._state_start_y,
        )

    def imu_rotation_since_state(self):
        """Rotation IMU totale depuis le debut de l'etat (degres absolus)."""
        if not self.has_imu:
            return 0.0
        d = self.imu_heading - self._state_start_imu
        while d > 180: d -= 360
        while d < -180: d += 360
        return abs(d)

    def is_stalled_linear(self, after_s=3.0, min_m=0.05):
        """True si le robot est cense avancer/reculer mais n'a pas bouge."""
        elapsed = time.time() - self.state_start_time
        if elapsed < after_s or not self.has_odom:
            return False
        return self.odom_distance_traveled() < min_m

    def is_stalled_angular(self, after_s=3.0, min_deg=5.0):
        """True si le robot est cense tourner mais n'a pas tourne."""
        elapsed = time.time() - self.state_start_time
        if elapsed < after_s or not self.has_imu:
            return False
        return self.imu_rotation_since_state() < min_deg

    def pid_angular(self, error):
        self.integral = max(-1000, min(1000, self.integral + error))
        derivative = error - self.prev_error
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.max_angular, min(self.max_angular, output))

    def imu_error_to(self, target_deg):
        error = target_deg - self.imu_heading
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error

    def imu_at_target(self, target_deg, tolerance=5):
        if target_deg == 180.0 and abs(self.imu_heading) > (180 - tolerance):
            return True
        return abs(self.imu_error_to(target_deg)) < tolerance

    def imu_turn_toward(self, target_deg):
        error = self.imu_error_to(target_deg)
        turn_speed = 2.0 if abs(error) > 45 else 0.7
        if error > 0:
            self.cmd(angular=-turn_speed)
        else:
            self.cmd(angular=turn_speed)

    # === BOUCLE ===

    def loop(self):
        self.publish_status()
        states = {
            'CHERCHER': self.state_chercher,
            'ALIGNER': self.state_aligner,
            'APPROCHER': self.state_approcher,
            'POUSSER': self.state_pousser,
            'RETOUR_CENTRE': self.state_retour_centre,
            'RETOUR_ARENE': self.state_retour_arene,
            'ORIENTER_ZONE': self.state_orienter_zone,
            'POUSSER_DROIT': self.state_pousser_droit,
            'RETOUR': self.state_retour,
            'EXPLORER': self.state_explorer,
        }
        handler = states.get(self.state)
        if handler:
            handler()

    # === ETATS ===

    def state_chercher(self):
        self.target_color = None

        # Init tracking rotation
        if self._chercher_start_heading is None and self.has_imu:
            self._chercher_start_heading = self.imu_heading
            self._chercher_rotation_done = 0.0
            self._chercher_last_heading = self.imu_heading

        # Calcul rotation accumulee
        if self.has_imu and self._chercher_start_heading is not None:
            delta = self.imu_heading - self._chercher_last_heading
            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360
            self._chercher_rotation_done += abs(delta)
            self._chercher_last_heading = self.imu_heading

        # Zone morte devant depots (cubes deja deposes possibles)
        if self.has_imu:
            h = abs(self.imu_heading)
            if h < 25 or h > 155:
                self.cmd(angular=self.search_turn_speed)
                self.confirm_count = 0
                self.confirm_color = None
                return

        # Tour complet -> Explorer
        if self._chercher_rotation_done >= 360 and not self.search_offset:
            self.stop()
            self._chercher_start_heading = None
            if self.full_rotations == 0:
                self.search_heading = 90.0
            elif self.full_rotations == 1:
                self.search_heading = -90.0
            else:
                self.full_rotations = 0
                self.search_heading = 90.0
            self.full_rotations += 1
            self.get_logger().info(f'[CHERCHER] Tour complet! -> EXPLORER {self.search_heading} deg')
            self.change_state('EXPLORER')
            return

        cube = self.get_target_cube()
        if cube:
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
                self.full_rotations = 0
                self._chercher_start_heading = None
                self.get_logger().info(f'[CHERCHER] {cube["color"]} CONFIRME (area={cube["area"]})')
                self.change_state('ALIGNER')
                return
        else:
            self.confirm_count = 0
            self.confirm_color = None

        self.cmd(angular=self.search_turn_speed)

    def state_explorer(self):
        elapsed = time.time() - self.state_start_time
        if self._explore_phase == 'TOURNER':
            if self.imu_at_target(self.search_heading, tolerance=5):
                self.stop()
                time.sleep(0.2)
                self._explore_phase = 'AVANCER'
                self.state_start_time = time.time()
                self.get_logger().info(f'[EXPLORER] Oriente {self.search_heading} deg -> avance')
                return
            if elapsed > 10.0:
                self._explore_phase = 'AVANCER'
                self.state_start_time = time.time()
                return
            self.imu_turn_toward(self.search_heading)
        elif self._explore_phase == 'AVANCER':
            elapsed2 = time.time() - self.state_start_time
            if elapsed2 >= self.search_time:
                self.stop()
                self.search_offset = True
                self._explore_phase = 'TOURNER'
                self.get_logger().info('[EXPLORER] Arrive! -> CHERCHER')
                self.change_state('CHERCHER')
                return
            self.cmd(linear=0.5)

    def state_aligner(self):
        cube = self.get_target_cube()
        if not cube:
            if self.in_blind_spot():
                self.push_start = time.time()
                self.push_color = self.target_color
                self.change_state('POUSSER')
                return
            if self.target_lost():
                self.stop()
                if self.distance_forward > 0.05:
                    self.push_color = None
                    self.change_state('RETOUR_CENTRE')
                else:
                    self.change_state('CHERCHER')
                return
            return
        if self.state_timed_out():
            self.stop()
            self.change_state('CHERCHER')
            return
        center_x = self.frame_width // 2
        error = cube['x'] - center_x
        if abs(error) < self.align_tolerance:
            self.stop()
            time.sleep(0.2)
            self.distance_forward = 0.0
            self.get_logger().info(f'[ALIGNER] {self.target_color} aligne! error={error}')
            self.change_state('APPROCHER')
            return
        angular = -self.pid_angular(error)
        self.cmd(angular=angular)

    def state_approcher(self):
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        cube = self.get_target_cube()
        if not cube:
            if self.in_blind_spot():
                self.push_start = time.time()
                self.push_color = self.target_color
                self.get_logger().info('[APPROCHER] Blind spot! -> POUSSER')
                self.change_state('POUSSER')
                return
            if self.target_lost():
                self.stop()
                if self.distance_forward > 0.05:
                    self.push_color = None
                    self.change_state('RETOUR_CENTRE')
                else:
                    self.change_state('CHERCHER')
                return
            self.distance_forward += self.min_forward_speed * dt
            self.cmd(linear=self.min_forward_speed)
            return
        if self.state_timed_out():
            self.stop()
            if self.distance_forward > 0.05:
                self.push_color = None
                self.change_state('RETOUR_CENTRE')
            else:
                self.change_state('CHERCHER')
            return
        center_x = self.frame_width // 2
        error = cube['x'] - center_x
        angular = -self.pid_angular(error)
        close_th = self.close_area_for(self.target_color)
        if cube['area'] > close_th:
            self.push_color = self.target_color
            self.push_start = time.time()
            self.get_logger().info(
                f'[APPROCHER] {self.target_color} proche (area={cube["area"]}/{close_th})! -> POUSSER'
            )
            self.change_state('POUSSER')
            return
        area_ratio = min(cube['area'] / close_th, 1.0)
        speed = self.max_forward_speed * (1.0 - area_ratio * 0.6)
        speed = max(self.min_forward_speed, speed)
        if abs(error) > 120:
            self.cmd(angular=angular)
            return
        angular_factor = 1.0 - (area_ratio * 0.5)
        self.distance_forward += speed * dt
        self.cmd(linear=speed, angular=angular * angular_factor)

    def state_pousser(self):
        """CONTACT: push + fermer servo sur le cube.

        Timing:
          [0 - 2.7s]  push en ligne droite vers le cube
          [2.7s]      servo ferme sur le cube
          [2.7 - 3.0s] un peu de push en plus pour que le servo se ferme
                       correctement pendant que le cube est maintenu
          [3.0s]      stop et transition RETOUR_CENTRE
        """
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.push_start

        # Memoriser heading au debut
        if elapsed < 0.2 and self.has_imu:
            self.push_heading = self.imu_heading

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
        """Reculer de la distance avancee.

        Utilise l'odometrie roue pour distance reelle (+ stall detection si
        carreau inegal bloque la roue). Plus de timer-based dead reckoning
        qui mentait quand le robot etait coince.
        """
        elapsed = time.time() - self.state_start_time
        target = max(0.10, self.distance_forward)  # au moins 10cm de recul

        # Progression reelle via odometrie
        traveled = self.odom_distance_traveled()
        done = (self.has_odom and traveled >= target) or elapsed > 15.0

        # Stall: robot commande reverse mais ne bouge pas (carreau bloque)
        stalled = self.is_stalled_linear(after_s=3.5, min_m=0.05)
        if stalled:
            self.get_logger().warn(
                f'[RETOUR_CENTRE] STALL detecte (traveled={traveled:.2f}m en '
                f'{elapsed:.1f}s) -> transition forcee'
            )
            done = True

        if done:
            self.stop()
            self.distance_forward = 0.0
            time.sleep(0.3)
            if self.push_color:
                if self.search_offset:
                    self.get_logger().info(
                        f'[RETOUR_CENTRE] traveled={traveled:.2f}m -> RETOUR_ARENE'
                    )
                    self.change_state('RETOUR_ARENE')
                else:
                    self.get_logger().info(
                        f'[RETOUR_CENTRE] traveled={traveled:.2f}m -> ORIENTER_ZONE'
                    )
                    self.change_state('ORIENTER_ZONE')
            else:
                self.get_logger().info('[RETOUR_CENTRE] Fausse detect -> CHERCHER')
                self.change_state('CHERCHER')
            return

        # Correction heading en reculant
        angular = 0.0
        if self.has_imu:
            error = self.imu_error_to(self.push_heading)
            angular = max(-0.3, min(0.3, error * 0.02))
        self.cmd(linear=-self.push_speed, angular=angular)

    def state_retour_arene(self):
        """Revenir au centre avant de deposer"""
        elapsed = time.time() - self.state_start_time

        if self._retour_phase == 'TOURNER':
            target = self.search_heading + 180
            if target > 180:
                target -= 360
            if self.imu_at_target(target, tolerance=5):
                self.stop()
                time.sleep(0.2)
                self._retour_phase = 'AVANCER'
                self.state_start_time = time.time()
                self.get_logger().info(f'[RETOUR_ARENE] Oriente {target:.0f} deg -> avance')
                return
            if elapsed > 10.0:
                self._retour_phase = 'AVANCER'
                self.state_start_time = time.time()
                return
            self.imu_turn_toward(target)

        elif self._retour_phase == 'AVANCER':
            elapsed2 = time.time() - self.state_start_time
            if elapsed2 >= self.search_time:
                self.stop()
                self.search_offset = False
                self._retour_phase = 'TOURNER'
                self.get_logger().info('[RETOUR_ARENE] Centre! -> ORIENTER_ZONE')
                self.change_state('ORIENTER_ZONE')
                return
            self.cmd(linear=0.5)

    def state_orienter_zone(self):
        """Pivoter vers la zone de depot (IMU) puis centrer le marker (ArUco).

        Ajouts robustesse:
          - hysteresis imu_at_target (3 frames consecutifs requis) pour eviter
            qu'un bruit IMU fasse transitionner trop tot
          - stall detection rotation: si roues tournent mais robot ne pivote
            pas (IMU delta < 8 deg en 4s), abandon propre (drop servo)
        """
        color = self.push_color or 'unknown'
        target = 0.0 if color in ['green', 'blue'] else 180.0
        target_marker = self.depot_marker_id.get(color, -1)
        elapsed = time.time() - self.state_start_time

        if not self.has_imu:
            self.stop()
            self.push_color = None
            self.servo_open()
            self.change_state('CHERCHER')
            return

        # Phase 1: rotation IMU vers le cap du mur de depot
        if self._orienter_phase == 'ROTATE':
            # Hysteresis: 3 frames consecutifs au target avant de transitionner
            if self.imu_at_target(target, tolerance=5):
                self._at_target_count += 1
                if self._at_target_count >= 3:
                    self.stop()
                    time.sleep(0.2)
                    self._orienter_phase = 'CENTER_MARKER'
                    self.state_start_time = time.time()
                    self._state_start_imu = self.imu_heading
                    self._at_target_count = 0
                    self.get_logger().info(
                        f'[ORIENTER] IMU cap atteint ({self.imu_heading:.1f}, '
                        f'target={target:.0f}) -> CENTER M{target_marker}'
                    )
                    return
                # Pas encore confirme: on arrete brievement pour laisser
                # l'IMU se stabiliser
                self.stop()
                return
            else:
                self._at_target_count = 0

            # Stall: roues tournent mais IMU ne progresse pas (coince)
            if self.is_stalled_angular(after_s=4.0, min_deg=8.0):
                self.stop()
                self.get_logger().warn(
                    f'[ORIENTER] STALL rotation (rot={self.imu_rotation_since_state():.1f}) '
                    f'-> ABANDON cube drop'
                )
                self.push_color = None
                self.servo_open()
                self.change_state('CHERCHER')
                return

            if elapsed > 15.0:
                self.stop()
                self.get_logger().warn(
                    f'[ORIENTER] TIMEOUT rotation IMU={self.imu_heading:.0f} ABANDON'
                )
                self.push_color = None
                self.servo_open()
                self.change_state('CHERCHER')
                return
            self.imu_turn_toward(target)
            return

        # Phase 2: centrer le marker du mur de depot
        if self._orienter_phase == 'CENTER_MARKER':
            if not self.has_marker(target_marker):
                # Marker invisible: apres 4s, on n'ose PAS pousser au culot
                # (trop risque: si l'IMU mentait, on pousse dans le mauvais mur).
                # On abandonne proprement - drop le cube.
                if elapsed > 4.0:
                    self.stop()
                    self.get_logger().warn(
                        f'[ORIENTER] M{target_marker} invisible 4s - drop cube'
                    )
                    self.push_color = None
                    self.servo_open()
                    self.change_state('CHERCHER')
                    return
                # Oscillation lente pour tomber sur le marker
                direction = 1 if int(elapsed * 1.5) % 2 == 0 else -1
                self.cmd(angular=0.25 * direction)
                return

            offset = self.aruco_marker_offset_px
            if abs(offset) < self.depot_marker_center_tol_px:
                self.stop()
                time.sleep(0.2)
                self.distance_push = 0.0
                self.push_start = time.time()
                self.push_heading = self.imu_heading
                self.get_logger().info(
                    f'[ORIENTER] {color} marker centre (off={offset:.0f}px) -> POUSSER_DROIT'
                )
                self.change_state('POUSSER_DROIT')
                return
            # Rotation douce pour ramener le marker au centre
            self.cmd(angular=-0.35 if offset > 0 else 0.35)
            return

    def state_pousser_droit(self):
        """Pousser droit vers la zone de depot, arret ArUco ou fallback timer."""
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.push_start
        color = self.push_color or 'unknown'
        target_marker = self.depot_marker_id.get(color, -1)

        self.distance_push += self.push_speed * dt

        # Arret ArUco: marker proche = mur proche = cube en zone
        if self.has_marker(target_marker) and self.aruco_distance < self.depot_stop_dist_m:
            self.stop()
            self.servo_open()
            self.cubes_sorted[color] = self.cubes_sorted.get(color, 0) + 1
            self.total_sorted += 1
            self.get_logger().info(
                f'[POUSSER_DROIT] {color} livre (ARUCO d={self.aruco_distance:.2f}m) '
                f'Total: {self.total_sorted}'
            )
            self.change_state('RETOUR')
            return

        # Fallback timer
        if elapsed >= self.push_duration:
            self.stop()
            self.servo_open()
            self.cubes_sorted[color] = self.cubes_sorted.get(color, 0) + 1
            self.total_sorted += 1
            self.get_logger().info(
                f'[POUSSER_DROIT] {color} livre (TIMER) dist={self.distance_push:.2f}m '
                f'Total: {self.total_sorted}'
            )
            self.change_state('RETOUR')
            return

        # Correction trajectoire: heading IMU + ajustement marker si visible
        angular = 0.0
        if self.has_imu:
            h_err = self.imu_error_to(self.push_heading)
            angular = max(-0.2, min(0.2, h_err * 0.02))
        if self.has_marker(target_marker):
            marker_corr = max(-0.2, min(0.2, -self.aruco_marker_offset_px * 0.002))
            angular = 0.6 * angular + 0.4 * marker_corr

        self.cmd(linear=self.push_speed, angular=angular)

    def state_retour(self):
        """Reculer de la distance poussee - odom + stall detection."""
        elapsed = time.time() - self.state_start_time
        target = max(0.10, self.distance_push)
        traveled = self.odom_distance_traveled()
        done = (self.has_odom and traveled >= target) or elapsed > 15.0

        if self.is_stalled_linear(after_s=3.5, min_m=0.05):
            self.get_logger().warn(
                f'[RETOUR] STALL (traveled={traveled:.2f}m) -> transition'
            )
            done = True

        if done:
            self.stop()
            self.distance_push = 0.0
            time.sleep(0.2)
            self.last_cube_area = 0
            self.last_cube_color = None
            self.target_color = None
            self.push_color = None
            self.search_offset = False
            self.get_logger().info(f'[RETOUR] Termine (traveled={traveled:.2f}m) -> CHERCHER')
            self.change_state('CHERCHER')
            return

        # Corriger heading en reculant
        angular = 0.0
        if self.has_imu:
            error = self.imu_error_to(self.push_heading)
            angular = max(-0.3, min(0.3, error * 0.02))
        self.cmd(linear=-self.push_speed, angular=angular)


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
