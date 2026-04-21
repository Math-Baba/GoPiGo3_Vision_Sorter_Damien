import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
import cv2
import numpy as np
import json
import math
import time


class ArucoLocalizer(Node):
    """
    Localization ArUco 6DOF avec desambiguisation IPPE par IMU.

    Pipeline:
      1. detectMarkers -> on ne garde que les IDs connus (0..3), le plus gros.
      2. solvePnPGeneric + SOLVEPNP_IPPE_SQUARE -> 1 ou 2 solutions miroir.
      3. Desambiguation:
           retval=1              -> unique, on prend.
           2 solutions + ancre   -> on predit delta_heading attendu via d_imu
                                    et on choisit le candidat dont d_world matche.
           pas d'ancre / IMU     -> fallback sur l'erreur de reprojection.
      4. Auto-calibration imu_sign (correlation rotation IMU / rotation ArUco
         depuis une reference stable) et imu_offset (EMA).
      5. Publication sur /robot_position: x, y, heading, imu_sign, imu_offset,
         confidence, age.

    Convention heading: 0 = +Y (mur A), CCW+, plage [-180, +180].
    Convention marker frame OpenCV: X right, Y up, Z out-of-plane.
    """

    # --- parametres geometriques ---
    ARENA_HALF = 1.2
    MARKER_SIZE = 0.06
    FOCAL_PX = 730.0

    # --- parametres disambiguation / calibration ---
    ANCHOR_TIMEOUT_S = 1.0
    IMU_ROT_THRESHOLD_DEG = 3.0
    IMU_SIGN_ROT_DEG = 15.0
    OFFSET_EMA_ALPHA = 0.1

    # Pose monde de chaque marker: (position xy, angle normale CCW+ depuis +X)
    MARKER_POSES = {
        0: (np.array([0.0,  ARENA_HALF]), -math.pi / 2),   # mur A
        1: (np.array([ ARENA_HALF, 0.0]),  math.pi),       # mur droit
        2: (np.array([0.0, -ARENA_HALF]),  math.pi / 2),   # mur B
        3: (np.array([-ARENA_HALF, 0.0]),  0.0),           # mur gauche
    }

    def __init__(self):
        super().__init__('aruco_localizer')

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

        s = self.MARKER_SIZE / 2.0
        self.obj_points = np.array([
            [-s,  s, 0], [ s,  s, 0], [ s, -s, 0], [-s, -s, 0],
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)

        # Etat pose (en frame arene)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.last_marker_id = -1
        self.last_distance = 0.0
        self.last_marker_time = 0.0
        self.confidence = 0.0
        # Offset pixel du marker choisi par rapport au centre image (pour
        # asservissement lateral pendant le depot - independant de la pose arene)
        self.marker_offset_px = 0.0

        # Etat IMU
        self.imu_heading = 0.0
        self.has_imu = False
        self.imu_sign = None        # +1, -1 ou None
        self.imu_offset = None      # degres

        # Ancres
        self._anchor_heading = None  # pose acceptee frame precedente (disambig temporel)
        self._anchor_imu = None
        self._anchor_time = 0.0
        self._calib_ref_heading = None  # reference stable pour calibrer imu_sign
        self._calib_ref_imu = None

        # Diag
        self._frames = 0
        self._detections = 0
        self._last_ids_seen = []
        self._last_diag = time.time()

        # ROS
        self.create_subscription(Image, '/image_raw', self.image_cb, 10)
        self.create_subscription(Float32, '/imu/heading', self.imu_cb, 10)
        self.pos_pub = self.create_publisher(String, '/robot_position', 10)
        self.create_timer(0.2, self.publish_position)

        self.get_logger().info('=== ARUCO LOCALIZER (IPPE + IMU disambig) ===')

    # ------------------------------------------------------------- callbacks

    def imu_cb(self, msg):
        self.imu_heading = float(msg.data)
        self.has_imu = True

    def image_cb(self, msg):
        try:
            self._frames += 1
            self._maybe_log_diag()

            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            corners, ids, _ = self.detector.detectMarkers(frame)
            if ids is None or len(ids) == 0:
                return

            # Selection: plus gros marker parmi les IDs connus
            best_idx = None
            best_size = 0
            for i, cs in enumerate(corners):
                mid = int(ids[i][0])
                if mid not in self.MARKER_POSES:
                    continue
                sz = cv2.contourArea(cs[0])
                if sz > best_size:
                    best_size = sz
                    best_idx = i

            self._last_ids_seen = [int(i[0]) for i in ids]
            if best_idx is None:
                return
            self._detections += 1

            marker_id = int(ids[best_idx][0])
            img_pts = corners[best_idx][0].astype(np.float32)

            # Offset pixel du centre du marker vs centre image (positif = droite)
            self.marker_offset_px = float(img_pts.mean(axis=0)[0] - msg.width / 2.0)

            K = np.array([
                [self.FOCAL_PX, 0, msg.width / 2.0],
                [0, self.FOCAL_PX, msg.height / 2.0],
                [0, 0, 1.0],
            ], dtype=np.float32)

            retval, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                self.obj_points, img_pts, K, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if retval < 1:
                return

            marker_pos, facing = self.MARKER_POSES[marker_id]
            sin_f, cos_f = math.sin(facing), math.cos(facing)
            U_x = np.array([-sin_f,  cos_f])   # axe X_marker en monde 2D
            U_z = np.array([ cos_f,  sin_f])   # axe Z_marker en monde 2D

            # Calcule pose monde pour chaque solution IPPE
            candidates = []
            for rv, tv in zip(rvecs, tvecs):
                t = tv.flatten()
                R_mc, _ = cv2.Rodrigues(rv)
                R_cm = R_mc.T
                cam_in_marker = -R_cm @ t
                cx_m, cz_m = float(cam_in_marker[0]), float(cam_in_marker[2])
                cam_world = marker_pos + cx_m * U_x + cz_m * U_z

                cam_fwd = R_cm[:, 2]
                fwd_world = float(cam_fwd[0]) * U_x + float(cam_fwd[2]) * U_z
                h_deg = self._norm(math.degrees(math.atan2(fwd_world[1], fwd_world[0])) - 90.0)

                candidates.append({'pos': cam_world, 'heading': h_deg, 'tvec': t})

            chosen = self._pick(retval, candidates)

            # Commit
            self.robot_x = float(np.clip(chosen['pos'][0], -self.ARENA_HALF, self.ARENA_HALF))
            self.robot_y = float(np.clip(chosen['pos'][1], -self.ARENA_HALF, self.ARENA_HALF))
            self.robot_heading = chosen['heading']
            self.last_marker_id = marker_id
            self.last_distance = float(np.linalg.norm(chosen['tvec']))
            self.last_marker_time = time.time()
            self.confidence = min(1.0, best_size / 5000.0)

            # MAJ ancre temporelle (toujours sur la solution acceptee)
            self._anchor_heading = self.robot_heading
            self._anchor_imu = self.imu_heading if self.has_imu else None
            self._anchor_time = self.last_marker_time

            # Tentative de calibration imu_sign / imu_offset
            self._try_calibrate_imu()

            self.get_logger().info(
                f'[ARUCO] M{marker_id} d={self.last_distance:.2f} '
                f'pos=({self.robot_x:+.2f},{self.robot_y:+.2f}) '
                f'hdg={self.robot_heading:+.1f} sign={self.imu_sign} '
                f'off={self._fmt_offset()} conf={self.confidence:.2f}'
            )

        except Exception as e:
            self.get_logger().warn(f'ArUco error: {e}')

    # --------------------------------------------------------- disambiguation

    def _pick(self, retval, candidates):
        if retval == 1:
            return candidates[0]

        now = time.time()
        have_anchor = (
            self._anchor_heading is not None
            and (now - self._anchor_time) < self.ANCHOR_TIMEOUT_S
        )
        if not have_anchor:
            # Bootstrap: meilleure erreur de reprojection (candidates[0])
            return candidates[0]

        d_worlds = [self._norm(c['heading'] - self._anchor_heading) for c in candidates]

        if self.has_imu and self._anchor_imu is not None:
            d_imu = self._norm(self.imu_heading - self._anchor_imu)

            if abs(d_imu) < self.IMU_ROT_THRESHOLD_DEG:
                # Rotation negligeable: pur temporel
                idx = min(range(len(d_worlds)), key=lambda i: abs(d_worlds[i]))
                return candidates[idx]

            if self.imu_sign is not None:
                predicted = self.imu_sign * d_imu
                idx = min(range(len(d_worlds)),
                          key=lambda i: abs(self._norm(d_worlds[i] - predicted)))
                return candidates[idx]

            # Sign inconnu: match par magnitude (|d_world| == |d_imu|)
            idx = min(range(len(d_worlds)),
                      key=lambda i: abs(abs(d_worlds[i]) - abs(d_imu)))
            return candidates[idx]

        # Pas d'IMU: pur temporel
        idx = min(range(len(d_worlds)), key=lambda i: abs(d_worlds[i]))
        return candidates[idx]

    def _try_calibrate_imu(self):
        if not self.has_imu:
            return

        # Premier accept: on pose la reference de calibration
        if self._calib_ref_heading is None:
            self._calib_ref_heading = self.robot_heading
            self._calib_ref_imu = self.imu_heading
            return

        # Sign: correlation rotation IMU vs rotation ArUco depuis la ref stable.
        # Garde-fou: le ratio |d_world|/|d_imu| doit etre proche de 1 (robot
        # monte sur une base rigide, pas de slip). Si c'est absurde (>1.5 ou
        # <0.5), on suit probablement la mauvaise branche IPPE -> reset la ref
        # pour retenter a partir de la position courante.
        if self.imu_sign is None:
            d_imu = self._norm(self.imu_heading - self._calib_ref_imu)
            d_world = self._norm(self.robot_heading - self._calib_ref_heading)
            if abs(d_imu) > self.IMU_SIGN_ROT_DEG and abs(d_world) > 5.0:
                ratio = abs(d_world) / abs(d_imu)
                if 0.5 < ratio < 1.5:
                    self.imu_sign = 1 if (d_imu * d_world) > 0 else -1
                    self.get_logger().info(
                        f'[CALIB] imu_sign={self.imu_sign:+d} '
                        f'(d_imu={d_imu:+.1f}, d_world={d_world:+.1f}, ratio={ratio:.2f})'
                    )
                else:
                    self.get_logger().warn(
                        f'[CALIB] ratio={ratio:.2f} absurde (d_imu={d_imu:+.1f} '
                        f'd_world={d_world:+.1f}) -> reset ref + anchor'
                    )
                    self._calib_ref_heading = self.robot_heading
                    self._calib_ref_imu = self.imu_heading
                    # Force le prochain frame a reprendre par reproj error
                    self._anchor_heading = None

        # Offset: arena = sign*imu + offset  =>  offset = arena - sign*imu (EMA)
        if self.imu_sign is not None:
            candidate = self._norm(self.robot_heading - self.imu_sign * self.imu_heading)
            if self.imu_offset is None:
                self.imu_offset = candidate
                self.get_logger().info(f'[CALIB] imu_offset={candidate:+.1f}')
            else:
                diff = self._norm(candidate - self.imu_offset)
                self.imu_offset = self._norm(self.imu_offset + self.OFFSET_EMA_ALPHA * diff)

    # --------------------------------------------------------- publishing

    def publish_position(self):
        age = time.time() - self.last_marker_time
        if age > 5.0:
            self.confidence = 0.0

        data = {
            'x': round(self.robot_x, 3),
            'y': round(self.robot_y, 3),
            'heading': round(self.robot_heading, 1),
            'marker_id': int(self.last_marker_id),
            'distance': round(self.last_distance, 3),
            'marker_offset_px': round(self.marker_offset_px, 1),
            'confidence': round(self.confidence, 2),
            'age': round(age, 1),
            'imu_sign': self.imu_sign,
            'imu_offset': round(self.imu_offset, 1) if self.imu_offset is not None else None,
            'timestamp': round(time.time(), 1),
        }
        msg = String()
        msg.data = json.dumps(data)
        self.pos_pub.publish(msg)

    # --------------------------------------------------------- utils

    @staticmethod
    def _norm(a):
        while a > 180.0:
            a -= 360.0
        while a < -180.0:
            a += 360.0
        return a

    def _fmt_offset(self):
        return f'{self.imu_offset:+.1f}' if self.imu_offset is not None else 'None'

    def _maybe_log_diag(self):
        now = time.time()
        if now - self._last_diag > 3.0:
            self.get_logger().info(
                f'[DIAG] frames={self._frames} dets={self._detections} '
                f'ids={self._last_ids_seen} sign={self.imu_sign} off={self._fmt_offset()}'
            )
            self._frames = 0
            self._detections = 0
            self._last_ids_seen = []
            self._last_diag = now


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
