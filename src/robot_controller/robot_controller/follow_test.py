import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time


class FollowTest(Node):
    def __init__(self):
        super().__init__('follow_test')

        # === PARAMETRES A TUNER ===
        self.kp = 0.003
        self.ki = 0.0001
        self.kd = 0.002
        self.integral = 0.0
        self.prev_error = 0.0
        self.max_angular = 0.4

        self.max_forward = 0.3
        self.min_forward = 0.1
        self.align_tolerance = 35

        self.stop_area = 7200
        self.min_area = 800

        # Etat
        self.detections = []
        self.frame_width = 320
        self.last_seen = 0
        self.following_color = None

        # ROS
        self.sub = self.create_subscription(String, '/cube_detections', self.detection_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info('=== FOLLOW TEST ===')
        self.get_logger().info('Le robot suit le cube le plus gros')
        self.get_logger().info('Ctrl+C pour arreter')

    def detection_cb(self, msg):
        data = json.loads(msg.data)
        self.detections = [d for d in data.get('detections', [])
                          if not d.get('ghost', False) and d['area'] > self.min_area]
        self.frame_width = data.get('frame_width', 320)
        if self.detections:
            self.last_seen = time.time()

    def cmd(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.pub.publish(t)

    def pid(self, error):
        self.integral += error
        self.integral = max(-1000, min(1000, self.integral))
        derivative = error - self.prev_error
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.max_angular, min(self.max_angular, output))

    def loop(self):
        # Pas de cube depuis 4s -> stop
        if time.time() - self.last_seen > 4.0:
            if self.following_color:
                self.get_logger().info('Cube perdu, en attente...')
                self.following_color = None
            self.cmd()
            return

        if not self.detections:
            # Pas de detection mais vu recemment -> continuer doucement
            self.cmd(linear=self.min_forward)
            return

        cube = self.detections[0]
        color = cube['color']
        area = cube['area']
        cx = cube['x']

        if self.following_color != color:
            self.following_color = color
            self.get_logger().info(f'Suivi: {color}')

        center = self.frame_width // 2
        error = cx - center

        if area > self.stop_area:
            self.cmd()
            self.get_logger().info(f'[STOP] {color} area={area} (trop proche)')
            return

        angular = -self.pid(error)

        if abs(error) > 80:
            # Tres decentre -> tourner seulement
            self.cmd(angular=angular)
            self.get_logger().info(f'[ALIGNER] {color} error={error:+d} angular={angular:.2f}')
            return

        area_ratio = min(area / self.stop_area, 1.0)
        speed = self.max_forward * (1.0 - area_ratio * 0.6)
        speed = max(self.min_forward, speed)

        self.cmd(linear=speed, angular=angular * 0.7)
        self.get_logger().info(
            f'[AVANCE] {color} area={area} error={error:+d} '
            f'speed={speed:.2f} angular={angular:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()