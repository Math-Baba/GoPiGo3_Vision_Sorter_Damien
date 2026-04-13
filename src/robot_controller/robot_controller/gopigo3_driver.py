import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
import easygopigo3 as easy
import time
import json
import math


class GoPiGo3Driver(Node):
    def __init__(self):
        super().__init__('gopigo3_driver')
        self.gpg = easy.EasyGoPiGo3()
        self.gpg.set_speed(0)

        # Reset encodeurs a chaque lancement
        self.gpg.reset_encoders()

        # Subscribers
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # Publishers
        self.battery_pub = self.create_publisher(Float32, '/battery', 10)
        self.odom_pub = self.create_publisher(String, '/odom_simple', 10)

        # Etat moteurs
        self.current_left = 0
        self.current_right = 0
        self.target_left = 0
        self.target_right = 0
        self.ramp_rate = 50

        # Odometrie simple
        self.left_encoder_prev = self.gpg.read_encoders()[0]
        self.right_encoder_prev = self.gpg.read_encoders()[1]
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0
        self.wheel_base = 0.117
        self.wheel_circumference = 0.066 * math.pi

        # Timers
        self.motor_timer = self.create_timer(0.05, self.motor_loop)
        self.battery_timer = self.create_timer(5.0, self.publish_battery)
        self.odom_timer = self.create_timer(0.2, self.publish_odom)

        self.last_cmd_time = time.time()

        self.get_logger().info('GoPiGo3 driver v3 pret - rampe + odom + batterie')
        self.publish_battery()

    def cmd_vel_cb(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        max_speed = 300
        scale = 300

        left = linear - (angular * self.wheel_base / 2)
        right = linear + (angular * self.wheel_base / 2)

        self.target_left = int(-left * scale)
        self.target_right = int(-right * scale)

        self.target_left = max(-max_speed, min(max_speed, self.target_left))
        self.target_right = max(-max_speed, min(max_speed, self.target_right))

        self.last_cmd_time = time.time()

    def motor_loop(self):
        if time.time() - self.last_cmd_time > 1.0:
            self.target_left = 0
            self.target_right = 0

        self.current_left = self._ramp(self.current_left, self.target_left)
        self.current_right = self._ramp(self.current_right, self.target_right)

        self.gpg.set_motor_dps(self.gpg.MOTOR_LEFT, self.current_right)
        self.gpg.set_motor_dps(self.gpg.MOTOR_RIGHT, self.current_left)

    def _ramp(self, current, target):
        diff = target - current
        if abs(diff) <= self.ramp_rate:
            return target
        elif diff > 0:
            return current + self.ramp_rate
        else:
            return current - self.ramp_rate

    def publish_battery(self):
        msg = Float32()
        msg.data = float(self.gpg.volt())
        self.battery_pub.publish(msg)
        self.get_logger().info(f'Batterie: {msg.data:.1f}V')

    def publish_odom(self):
        try:
            left_enc, right_enc = self.gpg.read_encoders()

            dl = (left_enc - self.left_encoder_prev) / 360.0 * self.wheel_circumference
            dr = (right_enc - self.right_encoder_prev) / 360.0 * self.wheel_circumference

            self.left_encoder_prev = left_enc
            self.right_encoder_prev = right_enc

            d_center = (dl + dr) / 2.0
            d_theta = (dr - dl) / self.wheel_base

            self.heading += d_theta
            # Normaliser entre -pi et pi
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
                'right_enc': right_enc
            })
            self.odom_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Odom erreur: {e}')

    def destroy_node(self):
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