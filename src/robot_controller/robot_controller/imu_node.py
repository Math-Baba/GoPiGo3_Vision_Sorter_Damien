"""Lecture de l'IMU BNO085 en mode UART-RVC et publication du cap (yaw).

Le mode UART-RVC envoie un paquet de 19 octets a 100Hz sur /dev/ttyAMA0:
  [0xAA, 0xAA, index, yaw_lo, yaw_hi, pitch_lo, pitch_hi, roll_lo, roll_hi,
   acc_x_lo, acc_x_hi, acc_y_lo, acc_y_hi, acc_z_lo, acc_z_hi, MI, checksum]

On utilise uniquement le yaw (offset 3-4, int16 signe, unite = centiemes
de degres). Le cap est zerote au premier paquet recu de sorte que le
heading publie vaut 0 au moment du launch -> pratique pour mettre le robot
face au mur A avant de demarrer.

Topics:
  pub  /imu/heading   (Float32)  - cap normalise dans [-180, +180]
  pub  /imu/data      (String JSON)
  sub  /imu/reset     (Empty)    - remet l'offset a None, re-zerote au prochain tick
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Empty
import serial
import struct
import time
import json


class IMUNode(Node):
    # Parametres UART BNO085 en mode RVC
    UART_DEVICE = '/dev/ttyAMA0'
    UART_BAUDRATE = 115200
    PACKET_SIZE = 19
    PACKET_HEADER = b'\xaa\xaa'

    def __init__(self):
        super().__init__('imu_node')

        self.heading = 0.0
        # heading_offset: yaw brut capture au 1er paquet; tous les caps publies
        # sont relatifs a cette origine. None tant qu'on n'a pas encore recu de
        # paquet (ou apres un /imu/reset) -> sera initialise au prochain tick.
        self.heading_offset = None
        self.ser = None
        self.buf = b''

        self.init_serial()

        self.heading_pub = self.create_publisher(Float32, '/imu/heading', 10)
        self.imu_data_pub = self.create_publisher(String, '/imu/data', 10)
        self.reset_sub = self.create_subscription(Empty, '/imu/reset', self.reset_cb, 10)
        # 20Hz (l'IMU en RVC envoie a 100Hz, on parse tout ce qui a ete bufferise)
        self.timer = self.create_timer(0.05, self.read_imu)

        self.get_logger().info('=== IMU BNO085 UART-RVC ===')

    def reset_cb(self, _msg):
        """Commande de re-zerotage (bouton 'Reset IMU' du dashboard).
        On ne recalcule pas maintenant: on laisse None, read_imu() posera le
        nouvel offset sur la prochaine trame reçue -> evite une race si un
        paquet etait deja en cours de parsing."""
        self.heading_offset = None
        self.get_logger().info('[IMU] Reset demande: offset recalcule au prochain tick')

    def init_serial(self):
        try:
            self.ser = serial.Serial(self.UART_DEVICE, self.UART_BAUDRATE, timeout=0.1)
            self.get_logger().info('BNO085 UART connecte!')
        except Exception as e:
            self.get_logger().error(f'Serial erreur: {e}')
            self.ser = None

    def read_imu(self):
        if self.ser is None:
            return

        try:
            # On lit tout ce qu'il y a de disponible (jusqu'a 200 bytes = ~10 paquets)
            # pour rattraper le buffer UART meme si le timer a du retard.
            self.buf += self.ser.read(200)

            # Parsing de tous les paquets complets dans le buffer
            while len(self.buf) >= self.PACKET_SIZE:
                # Cherche le preambule 0xAA 0xAA; si pas trouve on jette le buffer.
                idx = self.buf.find(self.PACKET_HEADER)
                if idx < 0:
                    self.buf = b''
                    break
                # Aligne le buffer sur le debut du prochain paquet
                if idx > 0:
                    self.buf = self.buf[idx:]
                if len(self.buf) < self.PACKET_SIZE:
                    break
                pkt = self.buf[:self.PACKET_SIZE]
                self.buf = self.buf[self.PACKET_SIZE:]

                # Yaw: int16 little-endian, unite = 1/100 degre
                yaw = struct.unpack_from('<h', pkt, 3)[0] / 100.0

                # Premier paquet (ou apres /imu/reset): pose l'origine
                if self.heading_offset is None:
                    self.heading_offset = yaw
                    self.get_logger().info(f'IMU zero calibre a {yaw:.1f}')

                # Cap relatif a l'origine, normalise [-180, +180]
                self.heading = yaw - self.heading_offset
                while self.heading > 180:
                    self.heading -= 360
                while self.heading < -180:
                    self.heading += 360

            # Publie le dernier heading calcule (meme si aucun paquet lu ce tick)
            msg = Float32()
            msg.data = float(self.heading)
            self.heading_pub.publish(msg)

            data = String()
            data.data = json.dumps({'heading_deg': round(self.heading, 1)})
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
