import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Empty
from sensor_msgs.msg import Image
import json
import time
import threading
import os
import base64
import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.latest_data = json.dumps({})
        self.latest_frame_b64 = None
        self.latest_state = {}
        self.latest_detections = {}
        self.latest_imu = 0.0

        self.create_subscription(String, '/robot_status', self.status_cb, 10)
        self.create_subscription(String, '/cube_detections', self.detections_cb, 10)
        self.create_subscription(Float32, '/imu/heading', self.imu_cb, 10)
        self.create_subscription(Image, '/image_raw', self.image_cb, 10)

        # Publishers pour les commandes depuis l'UI (endpoints HTTP POST)
        self.control_pub = self.create_publisher(String, '/robot_control', 10)
        self.imu_reset_pub = self.create_publisher(Empty, '/imu/reset', 10)

        self.create_timer(0.1, self.update_data)

        self.get_logger().info('=== DASHBOARD NODE ===')
        self.get_logger().info('Web UI: http://0.0.0.0:8080')

    def publish_control(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self.control_pub.publish(msg)

    def publish_imu_reset(self):
        self.imu_reset_pub.publish(Empty())

    def status_cb(self, msg):
        self.latest_state = json.loads(msg.data)

    def detections_cb(self, msg):
        self.latest_detections = json.loads(msg.data)

    def imu_cb(self, msg):
        self.latest_imu = msg.data

    def image_cb(self, msg):
        try:
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding in ('yuyv', 'yuv422_yuy2'):
                yuv = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 2)
                img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)
            else:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

            dets = self.latest_detections.get('detections', [])
            for d in dets:
                x, y, w, h = d.get('x',0), d.get('y',0), d.get('w',0), d.get('h',0)
                cn = d.get('color','')
                area = d.get('area',0)
                ghost = d.get('ghost', False)
                cmap = {'red':(0,0,255),'green':(0,255,0),'blue':(255,0,0),'yellow':(0,255,255)}
                bgr = cmap.get(cn, (255,255,255))
                t = 1 if ghost else 2
                cv2.rectangle(img, (int(x-w/2),int(y-h/2)), (int(x+w/2),int(y+h/2)), bgr, t)
                cv2.putText(img, f'{cn} {area}', (int(x-w/2), int(y-h/2)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 1)

            ih, iw = img.shape[:2]
            cv2.line(img, (iw//2, 0), (iw//2, ih), (128,128,128), 1)
            st = self.latest_state.get('state','')
            cv2.putText(img, st, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,136), 2)
            cv2.putText(img, f'IMU:{self.latest_imu:.1f}', (10, ih-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)

            _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 65])
            self.latest_frame_b64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')
        except Exception as e:
            self.get_logger().warn(f'Image error: {e}')

    def update_data(self):
        self.latest_data = json.dumps({
            'state': self.latest_state,
            'detections': self.latest_detections,
            'imu': round(self.latest_imu, 1),
            'time': round(time.time(), 1),
        })


_node = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self.serve_file('index.html', 'text/html')
        elif self.path == '/api/state':
            self.send_json(_node.latest_data)
        elif self.path == '/api/frame':
            self.send_text(_node.latest_frame_b64 or '')
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/api/control':
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length).decode('utf-8') if length > 0 else '{}'
            try:
                payload = json.loads(body)
            except Exception:
                self.send_error(400, 'bad json')
                return
            _node.publish_control(payload)
            self.send_json('{"ok":true}')
        elif self.path == '/api/imu_reset':
            _node.publish_imu_reset()
            self.send_json('{"ok":true}')
        else:
            self.send_error(404)

    def do_OPTIONS(self):
        # CORS preflight
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def serve_file(self, name, ctype):
        web_dir = os.path.expanduser('~/ROS2_WS/src/robot_controller/web2')
        path = os.path.join(web_dir, name)
        if os.path.exists(path):
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.end_headers()
            with open(path, 'rb') as f:
                self.wfile.write(f.read())
        else:
            self.send_error(404, f'{name} not found in {web_dir}')

    def send_json(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
        self.wfile.write(data.encode())

    def send_text(self, data):
        self.send_response(200)
        self.send_header('Content-Type', 'text/plain')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(data.encode())


def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = DashboardNode()

    t = threading.Thread(target=lambda: HTTPServer(('0.0.0.0', 8080), Handler).serve_forever(), daemon=True)
    t.start()

    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        pass
    _node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()