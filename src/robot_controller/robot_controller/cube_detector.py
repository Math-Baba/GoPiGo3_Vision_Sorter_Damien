import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import json
import time


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        # HSV calibre - 4 couleurs
        self.colors = {
            'green':  {'low': [48, 70, 60],  'high': [85, 255, 255]},
            'blue':   {'low': [90, 100, 60],  'high': [130, 255, 255]},
            'yellow': {'low': [20, 80, 80],  'high': [48, 255, 255]},
            'red1':   {'low': [0, 80, 80],   'high': [8, 255, 255]},
            'red2':   {'low': [170, 80, 80], 'high': [179, 255, 255]},
        }
        self.min_area = 400
        self.max_area = 20000
        self.min_ratio = 0.4
        self.max_ratio = 2.5

        # Morphologie
        self.kernel_open = np.ones((5, 5), np.uint8)
        self.kernel_close = np.ones((15, 15), np.uint8)

        # Tracking / lissage
        self.tracked = {}
        self.smoothing = 0.5
        self.last_seen = {}
        self.memory_duration = 0.3

        # FPS
        self.frame_count = 0
        self.fps_start = time.time()
        self.current_fps = 0.0

        # ROS
        self.sub = self.create_subscription(Image, '/image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(String, '/cube_detections', 10)
        self.fps_timer = self.create_timer(3.0, self.log_fps)

        self.get_logger().info('Cube detector v3 - 4 couleurs (vert, bleu, jaune, rouge)')

    def log_fps(self):
        elapsed = time.time() - self.fps_start
        if elapsed > 0:
            self.current_fps = self.frame_count / elapsed
        self.get_logger().info(f'Detection FPS: {self.current_fps:.1f}')
        self.frame_count = 0
        self.fps_start = time.time()

    def detect_color(self, enhanced, color_key):
        cfg = self.colors[color_key]
        hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(cfg['low']), np.array(cfg['high']))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)
        return mask

    def find_cubes_in_mask(self, mask, color_name):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area or area > self.max_area:
                continue

            x, y, w, h = cv2.boundingRect(c)
            ratio = w / h if h > 0 else 0
            if ratio < self.min_ratio or ratio > self.max_ratio:
                continue

            cx, cy = x + w // 2, y + h // 2
            detections.append({
                'color': color_name,
                'x': cx, 'y': cy,
                'w': w, 'h': h,
                'area': int(area),
                'ratio': round(ratio, 2)
            })
        return detections

    def smooth_detections(self, raw_detections):
        now = time.time()
        smoothed = []

        by_color = {}
        for d in raw_detections:
            c = d['color']
            if c not in by_color or d['area'] > by_color[c]['area']:
                by_color[c] = d

        for color, det in by_color.items():
            self.last_seen[color] = now
            if color in self.tracked:
                prev = self.tracked[color]
                s = self.smoothing
                det['x'] = int(prev['x'] * s + det['x'] * (1 - s))
                det['y'] = int(prev['y'] * s + det['y'] * (1 - s))
                det['area'] = int(prev['area'] * s + det['area'] * (1 - s))
            self.tracked[color] = det.copy()
            smoothed.append(det)

        for color in list(self.tracked.keys()):
            if color not in by_color:
                if now - self.last_seen.get(color, 0) < self.memory_duration:
                    ghost = self.tracked[color].copy()
                    ghost['ghost'] = True
                    smoothed.append(ghost)
                else:
                    del self.tracked[color]

        return smoothed

    def image_cb(self, msg):
        self.frame_count += 1

        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            # v4l2_camera publie en RGB, OpenCV utilise BGR
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except ValueError:
            return

        # Ne garder que les 2/3 du bas (cubes au sol, ignore le fond)
        crop_top = msg.height // 2
        frame = frame[crop_top:, :, :]

        # Pretraitement
        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)

        raw_detections = []

        # Vert
        mask_green = self.detect_color(enhanced, 'green')
        raw_detections += self.find_cubes_in_mask(mask_green, 'green')

        # Bleu
        mask_blue = self.detect_color(enhanced, 'blue')
        raw_detections += self.find_cubes_in_mask(mask_blue, 'blue')

        # Jaune
        mask_yellow = self.detect_color(enhanced, 'yellow')
        raw_detections += self.find_cubes_in_mask(mask_yellow, 'yellow')

        # Rouge (2 plages HSV)
        mask_red1 = self.detect_color(enhanced, 'red1')
        mask_red2 = self.detect_color(enhanced, 'red2')
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        raw_detections += self.find_cubes_in_mask(mask_red, 'red')

        # Corriger y pour l'image complete
        for d in raw_detections:
            d['y'] += crop_top

        # Lissage + tracking
        smoothed = self.smooth_detections(raw_detections)
        smoothed.sort(key=lambda d: d['area'], reverse=True)

        # Publier
        out = String()
        out.data = json.dumps({
            'detections': smoothed,
            'frame_width': msg.width,
            'frame_height': msg.height,
            'fps': round(self.current_fps, 1)
        })
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()