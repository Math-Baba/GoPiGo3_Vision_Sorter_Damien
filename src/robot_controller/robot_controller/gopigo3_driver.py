"""Pont ROS2 <-> GoPiGo3 (moteurs, encodeurs, batterie).

Ce noeud:
  - Ecoute /cmd_vel (Twist) et commande les moteurs via easygopigo3
  - Lit les encodeurs pour publier /odom_simple (position estimee)
  - Publie /battery (tension)

Specifications cinematique:
  - wheel_base = 0.117 m (distance entre les 2 roues)
  - wheel_circumference = 0.066 * pi m (~20.7 cm)
  - scale = 300 dps par unite Twist linear (donc linear=1 -> 300 dps,
    soit ~17 cm/s de vitesse sol)

Notes d'implementation:
  - Rampe logicielle (50 dps par tick de 50ms) pour eviter les a-coups qui
    font patiner les roues sur les carreaux inegaux.
  - Negation + swap MOTOR_LEFT/MOTOR_RIGHT: compense le cablage physique du
    robot pour que linear>0 = avance et angular>0 = rotation CCW cote logiciel.
  - Timeout 1s sans /cmd_vel -> stop moteurs (securite en cas de crash strategy).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
import easygopigo3 as easy
import time
import json
import math


class GoPiGo3Driver(Node):
    # --- Parametres moteurs ---
    MAX_DPS = 300          # saturation dps (borne hardware)
    TWIST_SCALE = 300      # conversion Twist.linear.x (m/s adim) -> dps
    RAMP_RATE = 50         # dps par tick motor_loop (20Hz -> 1000 dps/s)
    CMD_VEL_TIMEOUT_S = 1.0

    # --- Parametres geometriques robot ---
    WHEEL_BASE_M = 0.117
    WHEEL_CIRCUMFERENCE_M = 0.066 * math.pi  # ~0.207 m

    def __init__(self):
        super().__init__('gopigo3_driver')
        self.gpg = easy.EasyGoPiGo3()
        self.gpg.set_speed(0)
        self.gpg.reset_encoders()  # odom part toujours de (0, 0)

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.battery_pub = self.create_publisher(Float32, '/battery', 10)
        self.odom_pub = self.create_publisher(String, '/odom_simple', 10)

        # Etat commande moteurs (current = valeur courante apres rampe,
        # target = valeur cible demandee par le dernier /cmd_vel)
        self.current_left = 0
        self.current_right = 0
        self.target_left = 0
        self.target_right = 0

        # Odometrie: integree a partir des encodeurs, publiee sur /odom_simple.
        # Attention: cette odometrie MENT en cas de patinage (les encodeurs
        # comptent les tours meme quand les roues glissent sans avancer).
        self.left_encoder_prev = self.gpg.read_encoders()[0]
        self.right_encoder_prev = self.gpg.read_encoders()[1]
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0

        self.last_cmd_time = time.time()

        # Timers: rampe a 20Hz, odom a 5Hz, batterie toutes les 5s
        self.motor_timer = self.create_timer(0.05, self.motor_loop)
        self.battery_timer = self.create_timer(5.0, self.publish_battery)
        self.odom_timer = self.create_timer(0.2, self.publish_odom)

        self.get_logger().info('GoPiGo3 driver pret - rampe + odom + batterie')
        self.publish_battery()

    def cmd_vel_cb(self, msg):
        """Convertit Twist -> consignes dps par roue (modele differentiel).
        Les targets sont appliquees progressivement par motor_loop via la rampe.
        """
        linear = msg.linear.x
        angular = msg.angular.z

        # Modele differentiel: v_gauche = v - omega*L/2, v_droite = v + omega*L/2
        left = linear - (angular * self.WHEEL_BASE_M / 2)
        right = linear + (angular * self.WHEEL_BASE_M / 2)

        # Conversion adim -> dps avec negation (cablage du robot: le signe
        # logiciel est inverse par rapport au sens physique des moteurs).
        self.target_left = int(-left * self.TWIST_SCALE)
        self.target_right = int(-right * self.TWIST_SCALE)

        self.target_left = max(-self.MAX_DPS, min(self.MAX_DPS, self.target_left))
        self.target_right = max(-self.MAX_DPS, min(self.MAX_DPS, self.target_right))

        self.last_cmd_time = time.time()

    def motor_loop(self):
        # Securite: si la strategie ne publie plus de /cmd_vel pendant 1s,
        # on coupe les moteurs (evite un emballement apres crash).
        if time.time() - self.last_cmd_time > self.CMD_VEL_TIMEOUT_S:
            self.target_left = 0
            self.target_right = 0

        self.current_left = self._ramp(self.current_left, self.target_left)
        self.current_right = self._ramp(self.current_right, self.target_right)

        # IMPORTANT: swap MOTOR_LEFT/MOTOR_RIGHT -> les moteurs sont cables
        # "croises" sur ce robot. Combine avec la negation dans cmd_vel_cb,
        # l'effet net est une cinematique standard cote logiciel.
        self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, self.current_right)
        self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, self.current_left)

    def _ramp(self, current, target):
        """Approche progressivement `target` depuis `current` par increments
        de RAMP_RATE. Evite les sauts de vitesse brutaux qui font patiner
        les roues au demarrage."""
        diff = target - current
        if abs(diff) <= self.RAMP_RATE:
            return target
        return current + self.RAMP_RATE if diff > 0 else current - self.RAMP_RATE

    def publish_battery(self):
        msg = Float32()
        msg.data = float(self.gpg.volt())
        self.battery_pub.publish(msg)
        self.get_logger().info(f'Batterie: {msg.data:.1f}V')

    def publish_odom(self):
        """Integre les deltas des encodeurs en (x, y, heading). Publie 5 Hz.

        Modele: deux roues differentielles. Chaque roue parcourt dl/dr
        metres entre 2 tics, le centre du robot parcourt (dl+dr)/2 en ligne
        droite et tourne de (dr-dl)/wheel_base radians.
        """
        try:
            left_enc, right_enc = self.gpg.read_encoders()

            dl = (left_enc - self.left_encoder_prev) / 360.0 * self.WHEEL_CIRCUMFERENCE_M
            dr = (right_enc - self.right_encoder_prev) / 360.0 * self.WHEEL_CIRCUMFERENCE_M
            self.left_encoder_prev = left_enc
            self.right_encoder_prev = right_enc

            d_center = (dl + dr) / 2.0
            d_theta = (dr - dl) / self.WHEEL_BASE_M

            self.heading += d_theta
            while self.heading > math.pi:
                self.heading -= 2 * math.pi
            while self.heading < -math.pi:
                self.heading += 2 * math.pi

            self.pos_x += d_center * math.cos(self.heading)
            self.pos_y += d_center * math.sin(self.heading)

            msg = String()
            msg.data = json.dumps({
                'x': round(self.pos_x, 3),
                'y': round(self.pos_y, 3),
                'heading_deg': round(math.degrees(self.heading), 1),
                'left_enc': left_enc,
                'right_enc': right_enc,
            })
            self.odom_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Odom erreur: {e}')

    def destroy_node(self):
        # Arret propre des moteurs a la fermeture du noeud (Ctrl+C)
        self.gpg.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoPiGo3Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
