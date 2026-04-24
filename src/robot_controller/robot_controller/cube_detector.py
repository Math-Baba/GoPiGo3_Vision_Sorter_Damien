"""Detection HSV des cubes colores dans le flux camera.

Pipeline par frame:
  1. Lecture /image_raw (RGB8) -> conversion BGR OpenCV.
  2. Crop vertical: on ne garde que la moitie basse de l'image (les cubes
     sont au sol, le haut de la frame = murs/plafond = bruit).
  3. Pretraitement: flou gaussien 11x11 + CLAHE sur le canal L de LAB
     pour normaliser l'eclairage -> robustesse aux ombres et aux
     variations de luminosite.
  4. Pour chaque couleur (vert, bleu, jaune, rouge):
       - Seuillage HSV via cv2.inRange (rouge en 2 plages car H enjambe
         la discontinuite 0/180).
       - Nettoyage morphologique: erode 3x3 (x2) tue le bruit pixel,
         puis dilate 7x7 (x2) gonfle les petits cubes lointains pour
         qu'ils depassent MIN_AREA.
       - Contours + filtres aire [MIN_AREA, MAX_AREA], ratio w/h
         [MIN_RATIO, MAX_RATIO] et dimension mini (MIN_DIM).
  5. Lissage temporel + tracking: EMA sur (x, y, area), memoire 300ms si
     un cube disparait brievement (on publie un "ghost" pour eviter que
     la strategy perde sa cible au moindre flicker).
  6. Publication JSON sur /cube_detections.

Format de sortie /cube_detections:
  {
    "detections": [{"color": "green", "x": 320, "y": 380, "w": 40, "h": 38,
                    "area": 1520, "ratio": 1.05, "ghost": false}, ...],
    "frame_width": 640, "frame_height": 480, "fps": 17.3
  }
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import json
import time


class CubeDetector(Node):
    # Plages HSV calibrees pour les 4 couleurs de cubes.
    # Rouge est en 2 plages ('red1' bas + 'red2' haut) parce que la teinte
    # rouge enjambe la discontinuite H=0/180 en representation HSV.
    COLORS = {
        'green':  {'low': [45, 80, 50],   'high': [85, 255, 255]},
        'blue':   {'low': [95, 110, 50],  'high': [130, 255, 255]},
        'yellow': {'low': [18, 60, 60],   'high': [50, 255, 255]},
        'red1':   {'low': [0, 100, 90],   'high': [10, 255, 255]},
        'red2':   {'low': [165, 100, 90], 'high': [179, 255, 255]},
    }

    # Filtres geometriques pour ecarter le bruit HSV
    MIN_AREA = 400           # pixels - rejette les petits blobs parasites
    MAX_AREA = 22000         # pixels - rejette les grosses zones (murs colores)
    MIN_RATIO = 0.25         # largeur / hauteur - tolerant (cubes de loin/de biais)
    MAX_RATIO = 4.0
    MIN_DIM = 8              # pixels - rejette les tres petits rectangles

    # Tracking temporel
    SMOOTHING_ALPHA = 0.5    # EMA (0.5 = moyenne equilibree entre nouveau et ancien)
    GHOST_MEMORY_S = 0.3     # garde un cube fantome 300ms apres sa disparition

    def __init__(self):
        super().__init__('cube_detector')

        # Kernels morphologie style PyImageSearch: erode agressif pour tuer
        # le bruit, puis dilate gros pour gonfler les petits cubes lointains
        # au-dessus de MIN_AREA.
        self.kernel_erode = np.ones((3, 3), np.uint8)
        self.kernel_dilate = np.ones((7, 7), np.uint8)

        # Etat tracking par couleur
        self.tracked = {}     # {color: last detection dict}
        self.last_seen = {}   # {color: timestamp}

        # Instrumentation FPS (log toutes les 3s)
        self.frame_count = 0
        self.fps_start = time.time()
        self.current_fps = 0.0

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

    def detect_color(self, enhanced_bgr, color_key):
        """Retourne un masque binaire des pixels dans la plage HSV de color_key.

        Morpho: 2 passes erode 3x3 (tue le bruit pixel) puis 2 passes dilate 7x7
        (gonfle les petits cubes lointains pour qu'ils depassent MIN_AREA).
        """
        cfg = self.COLORS[color_key]
        hsv = cv2.cvtColor(enhanced_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(cfg['low']), np.array(cfg['high']))
        mask = cv2.erode(mask, self.kernel_erode, iterations=2)
        mask = cv2.dilate(mask, self.kernel_dilate, iterations=2)
        return mask

    def find_cubes_in_mask(self, mask, color_name):
        """Extrait les contours du masque et garde ceux qui respectent les
        filtres geometriques:
          - aire dans [MIN_AREA, MAX_AREA]  (ecarte bruit et grosses zones)
          - dimensions w, h >= MIN_DIM      (ecarte les micro-rectangles)
          - ratio w/h dans [MIN_RATIO, MAX_RATIO] (ecarte les bandes)
        Retourne une liste de dicts {color, x, y, w, h, area, ratio}."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.MIN_AREA or area > self.MAX_AREA:
                continue

            x, y, w, h = cv2.boundingRect(c)
            if w < self.MIN_DIM or h < self.MIN_DIM:
                continue
            ratio = w / h if h > 0 else 0
            if ratio < self.MIN_RATIO or ratio > self.MAX_RATIO:
                continue

            detections.append({
                'color': color_name,
                'x': x + w // 2, 'y': y + h // 2,   # centre du bounding box
                'w': w, 'h': h,
                'area': int(area),
                'ratio': round(ratio, 2),
            })
        return detections

    def smooth_detections(self, raw_detections):
        """Lisse les detections entre frames et gere la memoire "ghost".

        Pour chaque couleur:
          - On garde uniquement la detection la plus grosse (evite les doublons
            quand 2 blobs d'une meme couleur apparaissent).
          - EMA sur (x, y, area) pour lisser les micro-fluctuations.
          - Si une couleur disparait, on la conserve en "ghost" pendant
            GHOST_MEMORY_S secondes -> la strategy ne perd pas sa cible
            sur un simple blink du detecteur.
        """
        now = time.time()
        smoothed = []

        # 1 detection max par couleur (la plus grosse)
        by_color = {}
        for d in raw_detections:
            c = d['color']
            if c not in by_color or d['area'] > by_color[c]['area']:
                by_color[c] = d

        # EMA + mise a jour du tracking
        for color, det in by_color.items():
            self.last_seen[color] = now
            if color in self.tracked:
                prev = self.tracked[color]
                a = self.SMOOTHING_ALPHA
                det['x'] = int(prev['x'] * a + det['x'] * (1 - a))
                det['y'] = int(prev['y'] * a + det['y'] * (1 - a))
                det['area'] = int(prev['area'] * a + det['area'] * (1 - a))
            self.tracked[color] = det.copy()
            smoothed.append(det)

        # Ghosts: couleurs disparues mais recemment vues
        for color in list(self.tracked.keys()):
            if color not in by_color:
                if now - self.last_seen.get(color, 0) < self.GHOST_MEMORY_S:
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
            # v4l2_camera publie en RGB, OpenCV attend BGR
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except ValueError:
            return

        # Crop: on ignore la moitie haute (horizon / murs) pour se concentrer
        # sur le sol ou sont les cubes. Gain de perf + moins de faux positifs.
        crop_top = msg.height // 2
        frame = frame[crop_top:, :, :]

        # Pretraitement: blur 11x11 pour lisser le bruit (et les artefacts de
        # CLAHE en zone uniforme), puis CLAHE sur le canal L (luminance) de
        # LAB. Cela egalise l'eclairage localement et rend la detection HSV
        # beaucoup plus robuste aux ombres.
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)

        # Detection par couleur
        raw_detections = []
        raw_detections += self.find_cubes_in_mask(self.detect_color(enhanced, 'green'),  'green')
        raw_detections += self.find_cubes_in_mask(self.detect_color(enhanced, 'blue'),   'blue')
        raw_detections += self.find_cubes_in_mask(self.detect_color(enhanced, 'yellow'), 'yellow')
        # Rouge: union des 2 plages HSV (H bas + H haut)
        mask_red = cv2.bitwise_or(
            self.detect_color(enhanced, 'red1'),
            self.detect_color(enhanced, 'red2'),
        )
        raw_detections += self.find_cubes_in_mask(mask_red, 'red')

        # On a crope la frame, donc y des detections est relatif au crop.
        # On le remet en coordonnees image complete pour que la strategy et
        # le dashboard puissent utiliser les memes reperes.
        for d in raw_detections:
            d['y'] += crop_top

        smoothed = self.smooth_detections(raw_detections)
        smoothed.sort(key=lambda d: d['area'], reverse=True)

        out = String()
        out.data = json.dumps({
            'detections': smoothed,
            'frame_width': msg.width,
            'frame_height': msg.height,
            'fps': round(self.current_fps, 1),
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
