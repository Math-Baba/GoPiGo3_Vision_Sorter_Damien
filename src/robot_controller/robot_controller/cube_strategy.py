import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import time
import math
from std_msgs.msg import Float32, String


class CubeStrategy(Node):
    """
    Strategie v6 - Navigation avec IMU + tracking de distance.
    
    Flux:
    CHERCHER -> ALIGNER -> APPROCHER -> POUSSER(contact) -> RETOUR_CENTRE
    -> ORIENTER_ZONE -> POUSSER_DROIT -> RETOUR -> REORIENTER -> AVANCER_CENTRE -> CHERCHER
    
    Anti-decentrage: recule si fausse detection.
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

        # Blind spot
        self.last_cube_area = 0
        self.last_cube_color = None
        self.last_cube_time = 0

        # Robot state
        self.robot_heading = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0

        # IMU
        self.imu_heading = 0.0
        self.has_imu = False

        # === PARAMETRES ===
        self.search_turn_speed = 0.4
        self.max_forward_speed = 0.3
        self.min_forward_speed = 0.15
        self.push_speed = 0.5

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
        self.push_duration = 7.0
        self.lost_timeout = 4.0
        self.state_timeout = 30.0

        self.cubes_sorted = {}
        self.total_sorted = 0

        # ROS
        self.det_sub = self.create_subscription(String, '/cube_detections', self.detection_cb, 10)
        self.odom_sub = self.create_subscription(String, '/odom_simple', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Float32, '/imu/heading', self.imu_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info('=== CUBE SORTER v6 ===')
        self.get_logger().info('IMU + distance tracking + anti-decentrage')

    # === CALLBACKS ===

    def imu_cb(self, msg):
        self.imu_heading = msg.data
        self.has_imu = True

    def odom_cb(self, msg):
        data = json.loads(msg.data)
        self.robot_heading = math.radians(data['heading_deg'])
        self.robot_x = data['x']
        self.robot_y = data['y']

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

    # === UTILITAIRES ===

    def cmd(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.pub.publish(t)

    def stop(self):
        self.cmd(0.0, 0.0)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'target': self.target_color,
            'sorted': self.cubes_sorted,
            'total': self.total_sorted,
            'last_area': self.last_cube_area,
            'imu': round(self.imu_heading, 1)
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
        was_close = self.last_cube_area > self.blind_spot_area
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
        self.get_logger().info(f'[{old}] --> [{new_state}] target={self.target_color}')
        self.publish_status()

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
        turn_speed = 1.5 if abs(error) > 45 else 0.5
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
            'ORIENTER_ZONE': self.state_orienter_zone,
            'POUSSER_DROIT': self.state_pousser_droit,
            'RETOUR': self.state_retour,
            'REORIENTER': self.state_reorienter,
            'AVANCER_CENTRE': self.state_avancer_centre,
        }
        handler = states.get(self.state)
        if handler:
            handler()

    # === ETATS ===

    def state_chercher(self):
        self.target_color = None
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
                self.get_logger().info(f'[CHERCHER] {cube["color"]} CONFIRME (area={cube["area"]}) -> LOCK')
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
                    self.get_logger().info(f'[ALIGNER] Perdu, recule {self.distance_forward:.2f}m')
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
                self.get_logger().info(f'[APPROCHER] Blind spot! dist={self.distance_forward:.2f}m -> POUSSER')
                self.change_state('POUSSER')
                return
            if self.target_lost():
                self.stop()
                if self.distance_forward > 0.05:
                    self.push_color = None
                    self.get_logger().info(f'[APPROCHER] Perdu, recule {self.distance_forward:.2f}m')
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

        if cube['area'] > self.close_area:
            self.push_color = self.target_color
            self.push_start = time.time()
            self.get_logger().info(f'[APPROCHER] {self.target_color} proche! dist={self.distance_forward:.2f}m -> POUSSER')
            self.change_state('POUSSER')
            return

        area_ratio = min(cube['area'] / self.close_area, 1.0)
        speed = self.max_forward_speed * (1.0 - area_ratio * 0.6)
        speed = max(self.min_forward_speed, speed)

        if abs(error) > 120:
            self.cmd(angular=angular)
            return

        angular_factor = 1.0 - (area_ratio * 0.5)
        self.distance_forward += speed * dt
        self.cmd(linear=speed, angular=angular * angular_factor)
        self.get_logger().info(
            f'[APPROCHER] {self.target_color} area={cube["area"]} error={error} speed={speed:.2f} dist={self.distance_forward:.2f}m'
        )

    def state_pousser(self):
        """CONTACT: Foncer 4s pour capturer le cube"""
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.push_start

        self.distance_forward += self.push_speed * dt

        if elapsed > 4.0:
            self.stop()
            time.sleep(0.3)
            self.get_logger().info(f'[CONTACT] {self.push_color} capture! dist={self.distance_forward:.2f}m')
            self.change_state('RETOUR_CENTRE')
            return

        self.cmd(linear=self.push_speed)

    def state_retour_centre(self):
        """Reculer de la distance avancee"""
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.state_start_time

        self.distance_forward -= self.push_speed * dt

        if self.distance_forward <= 0 or elapsed > 15.0:
            self.stop()
            self.distance_forward = 0.0
            time.sleep(0.3)
            if self.push_color:
                self.get_logger().info(f'[RETOUR_CENTRE] Revenu! -> ORIENTER_ZONE')
                self.change_state('ORIENTER_ZONE')
            else:
                self.get_logger().info(f'[RETOUR_CENTRE] Revenu (fausse detect) -> REORIENTER')
                self.change_state('REORIENTER')
            return

        self.cmd(linear=-self.push_speed)

    def state_orienter_zone(self):
        """Pivoter vers la zone de depot avec IMU"""
        color = self.push_color or 'unknown'
        elapsed = time.time() - self.state_start_time
        target = 0.0 if color in ['green', 'blue'] else 180.0

        if self.has_imu:
            if self.imu_at_target(target, tolerance=5):
                self.stop()
                time.sleep(0.3)
                self.distance_push = 0.0
                self.push_start = time.time()
                self.get_logger().info(f'[ORIENTER] {color} IMU={self.imu_heading:.0f}° -> POUSSER_DROIT')
                self.change_state('POUSSER_DROIT')
                return

            if elapsed > 15.0:
                self.stop()
                time.sleep(0.3)
                self.distance_push = 0.0
                self.push_start = time.time()
                self.get_logger().info(f'[ORIENTER] {color} TIMEOUT -> POUSSER_DROIT')
                self.change_state('POUSSER_DROIT')
                return

            self.imu_turn_toward(target)
            self.get_logger().info(f'[ORIENTER] {color} IMU={self.imu_heading:.0f}° target={target:.0f}°')
        else:
            turn_time = 0.5 if color in ['green', 'blue'] else 2.5
            if elapsed > turn_time:
                self.stop()
                time.sleep(0.3)
                self.distance_push = 0.0
                self.push_start = time.time()
                self.change_state('POUSSER_DROIT')
                return
            self.cmd(angular=0.5)

    def state_pousser_droit(self):
        """Pousser droit vers la zone de depot"""
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.push_start
        color = self.push_color or 'unknown'

        self.distance_push += self.push_speed * dt

        if elapsed >= self.push_duration:
            self.stop()
            self.cubes_sorted[color] = self.cubes_sorted.get(color, 0) + 1
            self.total_sorted += 1
            self.get_logger().info(
                f'[POUSSER_DROIT] {color} livre! dist={self.distance_push:.2f}m Total: {self.total_sorted}'
            )
            self.change_state('RETOUR')
            return

        self.cmd(linear=self.push_speed)

    def state_retour(self):
        """Reculer de la distance poussee"""
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        elapsed = now - self.state_start_time

        self.distance_push -= self.push_speed * dt

        if self.distance_push <= 0 or elapsed > 15.0:
            self.stop()
            self.distance_push = 0.0
            time.sleep(0.2)
            self.get_logger().info(f'[RETOUR] Termine -> REORIENTER')
            self.last_cube_area = 0
            self.last_cube_color = None
            self.target_color = None
            self.push_color = None
            self.change_state('CHERCHER')
            return

        self.cmd(linear=-self.push_speed)

    def state_reorienter(self):
        """Retourner face a 0° avec IMU"""
        elapsed = time.time() - self.state_start_time

        if not self.has_imu or elapsed > 10.0:
            self.stop()
            self.last_cube_area = 0
            self.last_cube_color = None
            self.target_color = None
            self.change_state('AVANCER_CENTRE')
            return

        if self.imu_at_target(0.0, tolerance=5):
            self.stop()
            time.sleep(0.2)
            self.last_cube_area = 0
            self.last_cube_color = None
            self.target_color = None
            self.get_logger().info(f'[REORIENTER] Face a 0° -> AVANCER_CENTRE')
            self.change_state('AVANCER_CENTRE')
            return

        self.imu_turn_toward(0.0)

    def state_avancer_centre(self):
        """Avancer vers le centre si on vient du cote B"""
        elapsed = time.time() - self.state_start_time
        last_color = self.push_color

        if last_color in ['green', 'blue']:
            # Depot cote A: on est face au mur A -> pas avancer
            self.stop()
            self.push_color = None
            self.get_logger().info(f'[AVANCER_CENTRE] Cote A, pas d avance -> CHERCHER')
            self.change_state('CHERCHER')
            return

        if last_color in ['yellow', 'red']:
            # Depot cote B: on est face a 0° (vers centre) -> avancer
            if elapsed > 1.5:
                self.stop()
                self.push_color = None
                self.get_logger().info(f'[AVANCER_CENTRE] Avance termine -> CHERCHER')
                self.change_state('CHERCHER')
                return
            self.cmd(linear=0.3)
            return

        # Fausse detection ou pas de couleur: juste chercher
        self.stop()
        self.push_color = None
        self.get_logger().info(f'[AVANCER_CENTRE] Pas de couleur -> CHERCHER')
        self.change_state('CHERCHER')

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