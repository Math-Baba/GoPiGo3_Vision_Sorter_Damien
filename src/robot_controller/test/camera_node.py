import cv2
import numpy as np
import json
import base64
import threading
import time
import os
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from robot_interfaces.msg import CubeDetection

# ================= CONFIG =================

COLOR_RANGES = {
    'green':  {'low': [48, 70, 60],  'high': [85, 255, 255],  'bgr': (0, 255, 0)},
    'blue':   {'low': [90, 70, 60],  'high': [130, 255, 255], 'bgr': (255, 100, 0)},
    'yellow': {'low': [20, 80, 80],  'high': [48, 255, 255],  'bgr': (0, 255, 255)},
    'red':    {'low': [0, 80, 80],   'high': [8, 255, 255],   'bgr': (0, 0, 255)},
    'red2':   {'low': [170, 80, 80], 'high': [179, 255, 255], 'bgr': (0, 0, 255)},
}

MIN_AREA = 400
MAX_AREA = 20000 

kernel_open = np.ones((5, 5), np.uint8)
kernel_close = np.ones((15, 15), np.uint8)

state = {
    'selected_color': None,
    'frame': None,
    'detected': False,
    'cx': 0,
    'frame_width': 320,
}

lock = threading.Lock()

WEB_DIR = os.path.join(
    get_package_share_directory('robot_controller'),
    'web'
)

# ================= DETECTION =================

def detect_color(frame, color_name):
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)

    # CLAHE
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    enhanced = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)

    hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)

    # Rouge (2 plages)
    if color_name == 'red':
        mask1 = cv2.inRange(hsv,
                            np.array(COLOR_RANGES['red']['low']),
                            np.array(COLOR_RANGES['red']['high']))
        mask2 = cv2.inRange(hsv,
                            np.array(COLOR_RANGES['red2']['low']),
                            np.array(COLOR_RANGES['red2']['high']))
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        cfg = COLOR_RANGES[color_name]
        mask = cv2.inRange(hsv,
                           np.array(cfg['low']),
                           np.array(cfg['high']))

    # Nettoyage
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
    mask = cv2.dilate(mask, kernel_open, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_area = 0

    for c in contours:
        area = cv2.contourArea(c)

        if MIN_AREA < area < MAX_AREA:
            x, y, w, h = cv2.boundingRect(c)
            ratio = w / float(h)

            if 0.4 < ratio < 2.5:
                if area > best_area:
                    best = c
                    best_area = area

    if best is not None:
        x, y, w, h = cv2.boundingRect(best)
        cx = x + w // 2

        color_draw = COLOR_RANGES[color_name]['bgr']

        cv2.rectangle(frame, (x, y), (x+w, y+h), color_draw, 2)
        cv2.circle(frame, (cx, y + h // 2), 6, (0, 0, 255), -1)

        cv2.putText(frame, color_name.upper(), (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_draw, 2)

        # Direction
        fw = frame.shape[1]
        frame_cx = fw // 2

        if cx < frame_cx - 40:
            direction = "< GAUCHE"
        elif cx > frame_cx + 40:
            direction = "DROITE >"
        else:
            direction = "CENTRE"

        cv2.putText(frame, direction, (cx - 30, y + h + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        return True, cx

    return False, 0

# ================= CAMERA =================

def camera_loop(node):
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        node.get_logger().error("Impossible d'ouvrir la caméra")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while rclpy.ok():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue

        with lock:
            color = state['selected_color']
            fw = frame.shape[1]
            state['frame_width'] = fw

        # Ligne centrale
        cv2.line(frame, (fw // 2, 0), (fw // 2, frame.shape[0]), (100, 100, 100), 1)

        detected, cx = False, 0
        if color and color in COLOR_RANGES:
            detected, cx = detect_color(frame, color)

        with lock:
            state['detected'] = detected
            state['cx'] = cx
            _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            state['frame'] = base64.b64encode(jpg.tobytes()).decode('utf-8')

        msg = CubeDetection()
        msg.color = color or ''
        msg.detected = detected
        msg.cx = cx
        msg.frame_width = fw
        node.publisher.publish(msg)

        time.sleep(0.05)

    cap.release()

# ================= WEB =================

class WebHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path == '/':
            self._serve_file('index.html', 'text/html')
        elif path == '/style.css':
            self._serve_file('style.css', 'text/css')
        elif path == '/app.js':
            self._serve_file('app.js', 'application/javascript')
        elif path == '/frame':
            self._serve_frame()
        elif path == '/set_color':
            params = urllib.parse.parse_qs(parsed.query)
            if 'color' in params:
                with lock:
                    state['selected_color'] = params['color'][0]
            self._json({'ok': True})
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_file(self, filename, content_type):
        filepath = os.path.join(WEB_DIR, filename)
        try:
            with open(filepath, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-type', content_type)
            self.end_headers()
            self.wfile.write(content)
        except FileNotFoundError:
            self.send_response(404)
            self.end_headers()

    def _serve_frame(self):
        with lock:
            data = {
                'img': state['frame'] or '',
                'detected': state['detected'],
                'cx': state['cx'],
                'frame_width': state['frame_width'],
                'color': state['selected_color'] or '',
            }
        self._json(data)

    def _json(self, data):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def log_message(self, format, *args):
        pass

# ================= ROS2 =================

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(CubeDetection, 'cube_detection', 10)
        self.get_logger().info('Camera node démarré — http://11.255.255.201:8080')

def main():
    rclpy.init()
    node = CameraNode()

    threading.Thread(target=camera_loop, args=(node,), daemon=True).start()

    server = HTTPServer(('0.0.0.0', 8080), WebHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    print("WEB_DIR:", WEB_DIR)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()