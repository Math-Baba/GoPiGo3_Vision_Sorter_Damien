import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import time
import math
from std_msgs.msg import Float32, String


class CubeStrategy(Node):
    """
    Strategie v6.4 - base v6 pure (sans EXPLORER) + stall detection.

    Flux:
      CHERCHER -> ALIGNER -> APPROCHER -> POUSSER (contact/capture)
      -> RETOUR_CENTRE -> ORIENTER_ZONE -> POUSSER_DROIT -> RETOUR -> CHERCHER

    Principe cle v6 (distance = vitesse * temps accumulee):
      - distance_forward += speed * dt pendant APPROCHER et POUSSER
      - RETOUR_CENTRE: distance_forward -= push_speed * dt, fin quand <= 0
      - Meme principe pour distance_push / POUSSER_DROIT -> RETOUR
      Comme on accumule et decompte avec les memes unites vitesse, le robot
      recule exactement ce qu'il a avance (meme si la vitesse forward etait
      differente de la vitesse reverse, en temps).

      Filet de securite: stall detection via odom roue. Si pendant 2.5s le
      robot est cense avancer/reculer mais n'a pas bouge (roues patinent sur
      carreau inegal), on force la transition.

    Hypothese demarrage: robot face mur A, heading IMU = 0.
      - heading = 0   -> face mur A (depot vert/bleu)
      - heading = 180 -> face mur B (depot rouge/jaune)

    ArUco: la souscription reste pour l'aruco_localizer (dashboard) mais n'est
    PAS utilise pour la navigation/depot dans cette version. On rajoutera plus
    tard.
    """

    def __init__(self):
        super().__init__('cube_strategy')

        # Etat + controle depuis UI (demarrage en pause, l'utilisateur clique
        # sur Start dans le dashboard pour autoriser les mouvements).
        self.state = 'CHERCHER'
        self.active = False
        self.state_start_time = time.time()
        self.last_loop_time = time.time()

        # Couleurs a trier + mapping mur de depot (configurable depuis l'UI).
        # Par defaut: toutes les couleurs, vert/bleu -> mur A (0), rouge/jaune -> mur B (180).
        self.sort_colors = ['green', 'blue', 'red', 'yellow']
        self.depot_heading = {
            'green': 0.0,
            'blue': 0.0,
            'red': 180.0,
            'yellow': 180.0,
        }

        # Cube cible
        self.target_color = None
        self.confirm_count = 0
        self.confirm_color = None
        self.confirm_needed = 3
        self.detections = []
        self.real_detections = []
        self.frame_width = 640
        self.last_cube_area = 0
        self.last_cube_color = None
        self.last_cube_time = 0
        self.last_detection_time = 0

        # PID pixel
        self.integral = 0.0
        self.prev_error = 0.0

        # Push timing / heading
        self.push_start = None
        self.push_color = None
        self.push_heading = 0.0

        # Compteurs distance v6 (en unites vitesse * temps)
        self.distance_forward = 0.0  # accumule APPROCHER + POUSSER, consomme RETOUR_CENTRE
        self.distance_push = 0.0     # accumule POUSSER_DROIT, consomme RETOUR

        # CHERCHER rotation tracking (info)
        self._chercher_start_heading = None
        self._chercher_rotation_done = 0.0
        self._chercher_last_heading = 0.0

        # Servo
        import easygopigo3
        self.gpg = easygopigo3.EasyGoPiGo3()
        self.servo = self.gpg.init_servo('SERVO1')
        self.servo.rotate_servo(130)
        self.servo_closed = False

        # IMU
        self.imu_heading = 0.0
        self.has_imu = False

        # Odom (stall detection safety net)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.has_odom = False
        self._state_start_x = 0.0
        self._state_start_y = 0.0
        self._state_start_imu = 0.0

        # Hysteresis imu_at_target
        self._at_target_count = 0

        # === PARAMETRES (v6) ===
        self.search_turn_speed = 0.6
        self.max_forward_speed = 0.5
        self.min_forward_speed = 0.3
        self.push_speed = 0.8

        # PID
        self.kp = 0.003
        self.ki = 0.0001
        self.kd = 0.002
        self.max_angular = 0.4

        # Seuils cube
        self.align_tolerance = 40
        self.close_area = 4000
        self.blind_spot_area = 3000
        self.close_area_per_color = {'yellow': 2200}
        self.blind_spot_area_per_color = {'yellow': 1800}

        # Timings
        self.push_duration = 6.0
        self.lost_timeout = 4.0
        self.state_timeout = 30.0

        # Stats
        self.cubes_sorted = {}
        self.total_sorted = 0

        # ROS
        self.create_subscription(String, '/cube_detections', self.detection_cb, 10)
        self.create_subscription(Float32, '/imu/heading', self.imu_cb, 10)
        self.create_subscription(String, '/odom_simple', self.odom_cb, 10)
        self.create_subscription(String, '/robot_control', self.control_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.create_timer(0.1, self.loop)

        self.get_logger().info(
            '=== CUBE SORTER v6.5 (base v6 + stall + UI control) - EN PAUSE (Start via UI) ==='
        )

    # ============================================================ Callbacks

    def imu_cb(self, msg):
        self.imu_heading = float(msg.data)
        self.has_imu = True

    def detection_cb(self, msg):
        data = json.loads(msg.data)
        self.detections = data.get('detections', [])
        self.frame_width = data.get('frame_width', 640)
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
            self.get_logger().info('[CTRL] START')
        elif action == 'pause':
            self.active = False
            self.stop()
            self.get_logger().info('[CTRL] PAUSE')
        elif action == 'reset':
            self.active = False
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

    def close_area_for(self, color):
        return self.close_area_per_color.get(color, self.close_area)

    def blind_spot_area_for(self, color):
        return self.blind_spot_area_per_color.get(color, self.blind_spot_area)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'active': self.active,
            'target': self.target_color,
            'sorted': self.cubes_sorted,
            'total': self.total_sorted,
            'last_area': self.last_cube_area,
            'imu': round(self.imu_heading, 1),
            'odom_x': round(self.odom_x, 2),
            'odom_y': round(self.odom_y, 2),
            'dist_fwd': round(self.distance_forward, 2),
            'dist_push': round(self.distance_push, 2),
            'sort_colors': self.sort_colors,
            'depot_heading': self.depot_heading,
        })
        self.status_pub.publish(msg)

    def get_target_cube(self):
        if not self.real_detections:
            return None
        if self.target_color:
            targs = [d for d in self.real_detections if d['color'] == self.target_color]
            return targs[0] if targs else None
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

    @staticmethod
    def _norm(a):
        while a > 180: a -= 360
        while a < -180: a += 360
        return a

    def change_state(self, new_state):
        old = self.state
        self.state = new_state
        self.state_start_time = time.time()
        self.last_loop_time = time.time()
        self.integral = 0.0
        self.prev_error = 0.0
        self._state_start_x = self.odom_x
        self._state_start_y = self.odom_y
        self._state_start_imu = self.imu_heading
        self._at_target_count = 0
        if new_state == 'CHERCHER':
            self._chercher_start_heading = None
            self._chercher_rotation_done = 0.0
        self.get_logger().info(
            f'[{old}] --> [{new_state}] target={self.target_color} '
            f'dist_fwd={self.distance_forward:.2f} dist_push={self.distance_push:.2f}'
        )
        self.publish_status()

    def pid_angular(self, error):
        self.integral = max(-1000, min(1000, self.integral + error))
        derivative = error - self.prev_error
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.max_angular, min(self.max_angular, out))

    def imu_error_to(self, target_deg):
        return self._norm(target_deg - self.imu_heading)

    def imu_at_target(self, target_deg, tolerance=5):
        if target_deg == 180.0 and abs(self.imu_heading) > (180 - tolerance):
            return True
        return abs(self.imu_error_to(target_deg)) < tolerance

    def imu_turn_toward(self, target_deg):
        error = self.imu_error_to(target_deg)
        turn_speed = 2.0 if abs(error) > 45 else 0.7
        # v6: erreur positive -> angular negatif
        self.cmd(angular=-turn_speed if error > 0 else turn_speed)

    # ---- Odom-based stall detection (safety only) ----

    def odom_distance_traveled(self):
        if not self.has_odom:
            return 0.0
        return math.hypot(
            self.odom_x - self._state_start_x,
            self.odom_y - self._state_start_y,
        )

    def imu_rotation_since_state(self):
        if not self.has_imu:
            return 0.0
        return abs(self._norm(self.imu_heading - self._state_start_imu))

    def is_stalled_linear(self, after_s=2.5, min_m=0.04):
        elapsed = time.time() - self.state_start_time
        if elapsed < after_s or not self.has_odom:
            return False
        return self.odom_distance_traveled() < min_m

    def is_stalled_angular(self, after_s=4.0, min_deg=8.0):
        elapsed = time.time() - self.state_start_time
        if elapsed < after_s or not self.has_imu:
            return False
        return self.imu_rotation_since_state() < min_deg

    # ============================================================ Main loop

    def loop(self):
        self.publish_status()
        # Gate principal: si on n'est pas actif (start UI pas cliquee, ou pause),
        # on ne publie AUCUNE commande moteur. Le driver gopigo3 coupe les
        # moteurs apres 1s sans cmd_vel (timeout existant dans cmd_vel_cb).
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
        }.get(self.state)
        if handler:
            handler()

    # ============================================================ States

    def state_chercher(self):
        self.target_color = None

        # Tracking rotation (info)
        if self._chercher_start_heading is None and self.has_imu:
            self._chercher_start_heading = self.imu_heading
            self._chercher_rotation_done = 0.0
            self._chercher_last_heading = self.imu_heading
        if self.has_imu and self._chercher_start_heading is not None:
            delta = self._norm(self.imu_heading - self._chercher_last_heading)
            self._chercher_rotation_done += abs(delta)
            self._chercher_last_heading = self.imu_heading

        # Zone morte devant depots
        if self.has_imu:
            h = abs(self.imu_heading)
            if h < 25 or h > 155:
                self.cmd(angular=self.search_turn_speed)
                self.confirm_count = 0
                self.confirm_color = None
                return

        cube = self.get_target_cube()
        # Filtre UI: on ignore les cubes dont la couleur n'est pas dans la
        # liste des couleurs a trier (configurable depuis le dashboard).
        if cube and cube['color'] not in self.sort_colors:
            cube = None

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
                self._chercher_start_heading = None
                self.get_logger().info(
                    f'[CHERCHER] {cube["color"]} CONFIRME (area={cube["area"]})'
                )
                self.change_state('ALIGNER')
                return
        else:
            self.confirm_count = 0
            self.confirm_color = None

        self.cmd(angular=self.search_turn_speed)

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
                    self.target_color = None
                    self.change_state('CHERCHER')
                return
            return
        if self.state_timed_out():
            self.stop()
            self.target_color = None
            self.change_state('CHERCHER')
            return
        center_x = self.frame_width // 2
        error = cube['x'] - center_x
        if abs(error) < self.align_tolerance:
            self.stop()
            time.sleep(0.2)
            self.distance_forward = 0.0
            self.get_logger().info(f'[ALIGNER] {self.target_color} aligne error={error}')
            self.change_state('APPROCHER')
            return
        self.cmd(angular=-self.pid_angular(error))

    def state_approcher(self):
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        cube = self.get_target_cube()
        if not cube:
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
                    self.target_color = None
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
                self.target_color = None
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
                f'[APPROCHER] {self.target_color} proche (area={cube["area"]}/{close_th})!'
            )
            self.change_state('POUSSER')
            return
        area_ratio = min(cube['area'] / close_th, 1.0)
        speed = max(self.min_forward_speed, self.max_forward_speed * (1.0 - area_ratio * 0.6))
        if abs(error) > 120:
            self.cmd(angular=angular)
            return
        self.distance_forward += speed * dt
        self.cmd(linear=speed, angular=angular * (1.0 - area_ratio * 0.5))

    def state_pousser(self):
        """Contact: push 3s avec servo a 2.7s."""
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.push_start

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
        """Recule exactement ce qu'on a avance (distance_forward decrement)."""
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.state_start_time

        self.distance_forward -= self.push_speed * dt

        # Stall: roues patinent, le counter baisse mais robot immobile
        stalled = self.is_stalled_linear(after_s=2.5, min_m=0.04)
        done = self.distance_forward <= 0 or elapsed > 15.0 or stalled

        if stalled:
            self.get_logger().warn(
                f'[RETOUR_CENTRE] STALL (odom={self.odom_distance_traveled():.2f}m)'
            )

        if done:
            self.stop()
            self.distance_forward = 0.0
            time.sleep(0.3)
            if self.push_color:
                self.get_logger().info('[RETOUR_CENTRE] -> ORIENTER_ZONE')
                self.change_state('ORIENTER_ZONE')
            else:
                self.get_logger().info('[RETOUR_CENTRE] fausse detect -> CHERCHER')
                self.target_color = None
                self.change_state('CHERCHER')
            return

        # Recul ligne droite sans asservissement (le driver pilote les 2 roues
        # a la meme vitesse, donc trajectoire droite mecaniquement).
        self.cmd(linear=-self.push_speed)

    def state_orienter_zone(self):
        """Rotation IMU vers le cap du mur de depot pour la couleur capturee.
        Le mapping couleur -> cap (0 ou 180) est configurable depuis le dashboard.
        """
        color = self.push_color or 'unknown'
        target = self.depot_heading.get(color, 0.0)
        elapsed = time.time() - self.state_start_time

        if not self.has_imu:
            self.stop()
            self.push_color = None
            self.servo_open()
            self.change_state('CHERCHER')
            return

        # Hysteresis 3 frames au target
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

        if self.is_stalled_angular(after_s=4.0, min_deg=8.0):
            self.stop()
            self.get_logger().warn(
                f'[ORIENTER] STALL rot={self.imu_rotation_since_state():.1f} -> drop cube'
            )
            self.push_color = None
            self.servo_open()
            self.change_state('CHERCHER')
            return

        if elapsed > 15.0:
            self.stop()
            self.get_logger().warn(
                f'[ORIENTER] TIMEOUT cap={self.imu_heading:.0f} -> drop cube'
            )
            self.push_color = None
            self.servo_open()
            self.change_state('CHERCHER')
            return

        self.imu_turn_toward(target)

    def state_pousser_droit(self):
        """Push droit vers mur depot. Timer v6 (push_duration=6s)."""
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.push_start
        color = self.push_color or 'unknown'

        self.distance_push += self.push_speed * dt

        if elapsed >= self.push_duration:
            self.stop()
            self.servo_open()
            self.cubes_sorted[color] = self.cubes_sorted.get(color, 0) + 1
            self.total_sorted += 1
            self.get_logger().info(
                f'[POUSSER_DROIT] {color} livre! dist={self.distance_push:.2f} Total: {self.total_sorted}'
            )
            self.change_state('RETOUR')
            return

        # Push droit sans asservissement IMU (l'asservissement peut tirer le
        # robot de cote si push_heading etait mal memorise ou si l'IMU drift).
        self.cmd(linear=self.push_speed)

    def state_retour(self):
        """Recule la distance poussee (distance_push decrement)."""
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.state_start_time

        self.distance_push -= self.push_speed * dt

        stalled = self.is_stalled_linear(after_s=2.5, min_m=0.04)
        done = self.distance_push <= 0 or elapsed > 15.0 or stalled

        if stalled:
            self.get_logger().warn(
                f'[RETOUR] STALL (odom={self.odom_distance_traveled():.2f}m)'
            )

        if done:
            self.stop()
            self.distance_push = 0.0
            time.sleep(0.2)
            self.last_cube_area = 0
            self.last_cube_color = None
            self.target_color = None
            self.push_color = None
            self.get_logger().info('[RETOUR] Termine -> CHERCHER')
            self.change_state('CHERCHER')
            return

        # Recul ligne droite sans asservissement
        self.cmd(linear=-self.push_speed)


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
