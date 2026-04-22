import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Empty
import serial
import struct
import time
import json


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.heading = 0.0
        self.heading_offset = None
        self.ser = None

        self.init_serial()

        self.heading_pub = self.create_publisher(Float32, '/imu/heading', 10)
        self.imu_data_pub = self.create_publisher(String, '/imu/data', 10)
        self.reset_sub = self.create_subscription(Empty, '/imu/reset', self.reset_cb, 10)
        self.timer = self.create_timer(0.05, self.read_imu)  # 20Hz
        self.buf = b''

        self.get_logger().info('=== IMU BNO085 UART-RVC ===')

    def reset_cb(self, _msg):
        """Re-zerote le heading sur la prochaine lecture (offset = yaw courant)."""
        self.heading_offset = None
        self.get_logger().info('[IMU] Reset demande: offset recalcule au prochain tick')

    def init_serial(self):
        try:
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.1)
            self.get_logger().info('BNO085 UART connecte!')
        except Exception as e:
            self.get_logger().error(f'Serial erreur: {e}')
            self.ser = None

    def read_imu(self):
        if self.ser is None:
            return

        try:
            self.buf += self.ser.read(200)

            # Parser tous les paquets disponibles
            while len(self.buf) >= 19:
                idx = self.buf.find(b'\xaa\xaa')
                if idx < 0:
                    self.buf = b''
                    break
                if idx > 0:
                    self.buf = self.buf[idx:]
                if len(self.buf) < 19:
                    break
                pkt = self.buf[:19]
                self.buf = self.buf[19:]

                yaw = struct.unpack_from('<h', pkt, 3)[0] / 100.0

                if self.heading_offset is None:
                    self.heading_offset = yaw
                    self.get_logger().info(f'IMU zero calibre a {yaw:.1f}')

                self.heading = yaw - self.heading_offset

                # Normaliser -180 a 180
                while self.heading > 180:
                    self.heading -= 360
                while self.heading < -180:
                    self.heading += 360

            # Publier le dernier heading
            msg = Float32()
            msg.data = float(self.heading)
            self.heading_pub.publish(msg)

            data = String()
            data.data = json.dumps({
                'heading_deg': round(self.heading, 1)
            })
            self.imu_data_pub.publish(data)

        except Exception as e:
            self.get_logger().warn(f'IMU erreur: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()