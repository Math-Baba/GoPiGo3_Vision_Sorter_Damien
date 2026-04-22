import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math


class ArenaMap(Node):
    """
    Noeud de cartographie de l'arene.
    
    Setup: Le robot est place face au COTE A (vert+bleu) au demarrage.
    - COTE A (devant au demarrage) = zones vert + bleu
    - COTE B (derriere au demarrage) = zones jaune + rouge
    
    L'arene fait 2.2m x 2.2m.
    Le robot part d'une position aleatoire mais FACE AU COTE A.
    
    Publie /arena_state avec:
    - Position du robot
    - Cubes connus (position + couleur)
    - Direction de poussee recommandee pour chaque couleur
    - Prochain cube recommande
    """

    def __init__(self):
        super().__init__('arena_map')

        # === CONFIG ARENE ===
        self.arena_width = 2.2   # metres
        self.arena_height = 2.2  # metres

        # Direction de poussee par couleur
        # angle 0 = direction initiale du robot (vers cote A)
        # angle pi = direction opposee (vers cote B)
        self.push_directions = {
            'green': 0.0,          # pousser vers cote A (devant)
            'blue':  0.0,          # pousser vers cote A (devant)
            'yellow': math.pi,     # pousser vers cote B (derriere)
            'red':   math.pi,      # pousser vers cote B (derriere)
        }

        # === ETAT ROBOT ===
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0  # radians, 0 = face au cote A

        # === MEMOIRE CUBES ===
        # {id: {color, x, y, last_seen, delivered}}
        self.known_cubes = {}
        self.cube_id_counter = 0
        self.merge_distance = 0.3  # metres, distance pour fusionner 2 detections

        # Cubes livres
        self.delivered = {'green': 0, 'blue': 0, 'yellow': 0, 'red': 0}

        # Camera params (approximatif)
        self.camera_fov = math.radians(60)  # champ de vision horizontal
        self.frame_width = 640

        # === ROS ===
        self.odom_sub = self.create_subscription(String, '/odom_simple', self.odom_cb, 10)
        self.detection_sub = self.create_subscription(String, '/cube_detections', self.detection_cb, 10)
        self.state_pub = self.create_publisher(String, '/arena_state', 10)
        self.timer = self.create_timer(0.2, self.publish_state)

        self.get_logger().info('=== ARENA MAP ===')
        self.get_logger().info(f'Arene: {self.arena_width}x{self.arena_height}m')
        self.get_logger().info('Cote A (devant): vert + bleu')
        self.get_logger().info('Cote B (derriere): jaune + rouge')

    def odom_cb(self, msg):
        data = json.loads(msg.data)
        self.robot_x = data['x']
        self.robot_y = data['y']
        self.robot_heading = math.radians(data['heading_deg'])

    def detection_cb(self, msg):
        data = json.loads(msg.data)
        detections = data.get('detections', [])
        self.frame_width = data.get('frame_width', 640)

        for det in detections:
            if det.get('ghost', False):
                continue

            # Estimer la distance au cube a partir de l'aire
            # Calibration approximative: aire 1000px ~ 80cm, aire 8000px ~ 20cm
            area = det['area']
            if area < 200:
                continue

            # Distance approximative (relation inverse racine de l'aire)
            distance = 800.0 / math.sqrt(area)  # en cm
            distance = distance / 100.0  # en metres
            distance = max(0.1, min(2.0, distance))

            # Angle du cube par rapport au robot
            pixel_offset = det['x'] - self.frame_width / 2
            angle_offset = (pixel_offset / self.frame_width) * self.camera_fov

            # Position absolue du cube
            cube_angle = self.robot_heading + angle_offset
            cube_x = self.robot_x + distance * math.cos(cube_angle)
            cube_y = self.robot_y + distance * math.sin(cube_angle)

            # Fusionner avec un cube connu ou en creer un nouveau
            self._update_or_create_cube(det['color'], cube_x, cube_y)

    def _update_or_create_cube(self, color, x, y):
        now = time.time()
        best_id = None
        best_dist = self.merge_distance

        for cid, cube in self.known_cubes.items():
            if cube['color'] != color or cube['delivered']:
                continue
            dist = math.sqrt((cube['x'] - x) ** 2 + (cube['y'] - y) ** 2)
            if dist < best_dist:
                best_dist = dist
                best_id = cid

        if best_id is not None:
            # Mettre a jour position (moyenne mobile)
            cube = self.known_cubes[best_id]
            alpha = 0.3
            cube['x'] = cube['x'] * (1 - alpha) + x * alpha
            cube['y'] = cube['y'] * (1 - alpha) + y * alpha
            cube['last_seen'] = now
            cube['confidence'] = min(cube['confidence'] + 1, 20)
        else:
            # Nouveau cube
            self.cube_id_counter += 1
            self.known_cubes[str(self.cube_id_counter)] = {
                'color': color,
                'x': x,
                'y': y,
                'last_seen': now,
                'delivered': False,
                'confidence': 1
            }

    def mark_delivered(self, color):
        """Marquer le cube le plus recent de cette couleur comme livre"""
        now = time.time()
        best_id = None
        best_time = 0
        for cid, cube in self.known_cubes.items():
            if cube['color'] == color and not cube['delivered']:
                if cube['last_seen'] > best_time:
                    best_time = cube['last_seen']
                    best_id = cid
        if best_id:
            self.known_cubes[best_id]['delivered'] = True
            self.delivered[color] = self.delivered.get(color, 0) + 1

    def get_nearest_cube(self):
        """Retourne le cube non-livre le plus proche du robot"""
        best = None
        best_dist = float('inf')
        now = time.time()

        for cid, cube in self.known_cubes.items():
            if cube['delivered']:
                continue
            # Ignorer les cubes pas vus depuis longtemps
            if now - cube['last_seen'] > 30.0:
                continue
            # Ignorer les cubes avec faible confiance
            if cube['confidence'] < 2:
                continue

            dist = math.sqrt(
                (cube['x'] - self.robot_x) ** 2 +
                (cube['y'] - self.robot_y) ** 2
            )
            if dist < best_dist:
                best_dist = dist
                best = cube
                best['distance'] = dist

        return best

    def get_push_angle(self, color):
        """Retourne l'angle absolu vers lequel pousser ce cube"""
        return self.push_directions.get(color, 0.0)

    def get_angle_to_center(self):
        """Angle pour revenir au centre de l'arene"""
        # Le centre est a (0, 0) en coordonnees robot (point de depart)
        # Mais comme le robot part aleatoirement, le centre est approximatif
        dx = -self.robot_x
        dy = -self.robot_y
        return math.atan2(dy, dx)

    def publish_state(self):
        # Cubes actifs (non livres, vus recemment)
        now = time.time()
        active_cubes = []
        for cid, cube in self.known_cubes.items():
            if not cube['delivered'] and now - cube['last_seen'] < 30.0:
                active_cubes.append({
                    'id': cid,
                    'color': cube['color'],
                    'x': round(cube['x'], 2),
                    'y': round(cube['y'], 2),
                    'confidence': cube['confidence'],
                    'age': round(now - cube['last_seen'], 1)
                })

        nearest = self.get_nearest_cube()

        msg = String()
        msg.data = json.dumps({
            'robot': {
                'x': round(self.robot_x, 3),
                'y': round(self.robot_y, 3),
                'heading_deg': round(math.degrees(self.robot_heading), 1)
            },
            'cubes': active_cubes,
            'cubes_total': len(active_cubes),
            'delivered': self.delivered,
            'nearest': {
                'color': nearest['color'],
                'distance': round(nearest['distance'], 2),
                'x': round(nearest['x'], 2),
                'y': round(nearest['y'], 2)
            } if nearest else None,
            'push_directions': {
                'green_blue': 'COTE_A (devant initial)',
                'yellow_red': 'COTE_B (derriere initial)'
            }
        })
        self.state_pub.publish(msg)

        # Log periodique
        if int(now) % 5 == 0:
            self.get_logger().info(
                f'Robot: ({self.robot_x:.2f}, {self.robot_y:.2f}) '
                f'heading={math.degrees(self.robot_heading):.0f}° | '
                f'Cubes connus: {len(active_cubes)} | '
                f'Livres: {self.delivered}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArenaMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()