import rclpy
from rclpy.node import Node
from robot_interfaces.msg import CubeDetection
import easygopigo3
import time

# ================= CONFIG =================

DEAD_ZONE     = 40
SEARCH_SPEED  = 50
ALIGN_SPEED   = 100
FORWARD_SPEED = 200

COOLDOWN = 0.1  # évite spam commandes

# ================= NODE =================

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')

        self.gpg = easygopigo3.EasyGoPiGo3()

        self.subscription = self.create_subscription(
            CubeDetection,
            'cube_detection',
            self.on_detection,
            10
        )

        self.aligned = False
        self.last_time = 0

        self.get_logger().info('Motion node démarré')

    def on_detection(self, msg):
        # ===== COOLDOWN =====
        if time.time() - self.last_time < COOLDOWN:
            return
        self.last_time = time.time()

        # ===== PAS DE CUBE =====
        if not msg.detected or not msg.color:
            self.get_logger().info('Recherche du cube...')
            self.aligned = False

            self.gpg.set_speed(SEARCH_SPEED)
            self.gpg.left()  # rotation lente continue
            return

        frame_cx = msg.frame_width // 2
        offset = msg.cx - frame_cx

        # ===== ALIGNEMENT =====
        if abs(offset) > DEAD_ZONE:
            self.aligned = False

            if offset < 0:
                self.get_logger().info(f'Alignement gauche (offset {offset})')
                self.gpg.set_speed(ALIGN_SPEED)
                self.gpg.left()
            else:
                self.get_logger().info(f'Alignement droite (offset {offset})')
                self.gpg.set_speed(ALIGN_SPEED)
                self.gpg.right()

        # ===== AVANCE =====
        else:
            if not self.aligned:
                self.get_logger().info('Cube centré → avance')
                self.aligned = True

            self.gpg.set_speed(FORWARD_SPEED)
            self.gpg.backward()

# ================= MAIN =================

def main():
    rclpy.init()
    node = MotionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()