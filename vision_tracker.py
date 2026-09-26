import cv2
import numpy as np
import json
import os
import math

class VisionTracker:
    def __init__(self, camera_idx=0, config_file="hsv_config.json"):
        self.cap = cv2.VideoCapture(camera_idx)
        self.config_file = config_file
        self.hsv_ranges = {}
        self.load_config()
        
        # Aruco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Tuned for robustness while the robot is moving (motion blur), viewed at an
        # angle, or under uneven lighting — defaults are tuned for sharp, head-on shots.
        # Wider adaptive-threshold sweep: catches the marker under uneven lighting
        # across the field instead of only the default narrow window range.
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 53
        self.aruco_params.adaptiveThreshWinSizeStep = 4
        # More tolerant polygon-edge fitting: a motion-blurred marker's edges are
        # slightly wavy/rounded rather than crisp straight lines; the default
        # accuracy rate (0.03) can reject these as "not square enough".
        self.aruco_params.polygonalApproxAccuracyRate = 0.06
        # Detect a smaller minimum marker size, so the marker is still found even
        # if it's a bit farther from the camera or partially blurred at the edges.
        self.aruco_params.minMarkerPerimeterRate = 0.02
        # Sub-pixel corner refinement: improves heading-angle stability (used for
        # atan2 in get_state) even when the marker is slightly blurred.
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.1

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Field dimensions in mm
        self.FIELD_W_MM = 2100
        self.FIELD_H_MM = 1200
        
        # Homography Matrix (Mocked default - overwritten below if calibration.json exists)
        scale_x = self.FIELD_W_MM / 640.0
        scale_y = self.FIELD_H_MM / 480.0
        self.H_matrix = np.array([
            [scale_x, 0, 0],
            [0, scale_y, 0],
            [0, 0, 1]
        ], dtype=np.float32)

        # Auto-load the latest calibration saved by calibrate_camera.py, if present,
        # so every script uses the exact same, current homography.
        self.field_mask = None  # None = no boundary restriction (full frame)
        self.load_calibration()

        # Drop zones are painted on the field and never move, so their positions
        # are loaded once from the manual calibration file (setup_dropzones.py)
        # instead of being re-detected every frame from noisy color blobs.
        self.manual_drop_zones = {}
        self.load_manual_dropzones()

        # --- Persistent stone tracking ---
        # Previously each stone got a fresh id every frame (id = scan order of
        # contours), so "id" was meaningless across frames and the robot had no
        # way to keep aiming at "the same stone". Instead we now match each
        # frame's detections against the previous frame's tracked stones by
        # position (nearest neighbor of the same color, within STONE_MATCH_RADIUS_MM)
        # and keep the same id as long as a stone keeps being seen near where it
        # was. A stone that isn't matched for STONE_MAX_MISSES frames in a row is
        # dropped (assumed collected / moved / occluded).
        self._next_stone_id = 0
        self._tracked_stones = {}  # id -> {'pos': (x,y), 'color': str, 'misses': int}
        self.STONE_MATCH_RADIUS_MM = 60
        self.STONE_MAX_MISSES = 5

    def load_manual_dropzones(self, dropzones_file="manual_dropzones.json"):
        if not os.path.exists(dropzones_file):
            print(f"[WARNING] {dropzones_file} not found. Drop zones will be empty. "
                  f"Run setup_dropzones.py first.")
            return False

        with open(dropzones_file, 'r') as f:
            data = json.load(f)
        self.manual_drop_zones = {color: tuple(pos) for color, pos in data.items()}
        print(f"[INFO] Loaded {len(self.manual_drop_zones)} fixed drop zones from {dropzones_file}")
        return True

    def load_calibration(self, calibration_file="calibration.json"):
        if not os.path.exists(calibration_file):
            print(f"[WARNING] {calibration_file} not found. Using mocked (uncalibrated) H_matrix. "
                  f"Run calibrate_camera.py first for accurate mm coordinates.")
            return False

        with open(calibration_file, 'r') as f:
            data = json.load(f)
        src_pts = data["src_pts"]

        dst_pts = [
            [0, 0],
            [self.FIELD_W_MM, 0],
            [0, self.FIELD_H_MM],
            [self.FIELD_W_MM, self.FIELD_H_MM]
        ]
        self.calibrate_homography(src_pts, dst_pts)

        # Build a pixel-space mask of the field interior from the same 4 corners,
        # so color detection (stones etc.) never picks up things outside the field
        # (e.g. the red foam border tiles around the playing area).
        # src_pts order is [top-left, top-right, bottom-left, bottom-right];
        # reorder to perimeter order [TL, TR, BR, BL] for a valid (non self-intersecting) polygon.
        tl, tr, bl, br = src_pts
        polygon = np.array([tl, tr, br, bl], dtype=np.int32)
        self.field_mask = np.zeros((480, 640), dtype=np.uint8)
        cv2.fillConvexPoly(self.field_mask, polygon, 255)

        print(f"[INFO] Loaded calibration from {calibration_file}: src_pts={src_pts}")
        return True

    def calibrate_homography(self, src_points, dst_points):
        """src_points: 4 corners in pixel, dst_points: 4 corners in mm"""
        self.H_matrix, _ = cv2.findHomography(
            np.array(src_points, dtype=np.float32),
            np.array(dst_points, dtype=np.float32))

    def _update_stone_tracks(self, detections):
        """
        detections: list of {'color': str, 'pos': (x_mm, y_mm)} for THIS frame,
        with no id yet.

        Returns: list of {'id', 'color', 'pos'} for stones visible this frame,
        where 'id' is stable across frames for the same physical stone (matched
        by nearest position of the same color, within STONE_MATCH_RADIUS_MM).
        """
        unmatched = list(detections)

        # 1. Try to match each existing track to the closest unmatched detection
        #    of the same color, within the match radius (greedy nearest-neighbor).
        for tid, track in self._tracked_stones.items():
            best_idx, best_dist = None, self.STONE_MATCH_RADIUS_MM
            for i, d in enumerate(unmatched):
                if d['color'] != track['color']:
                    continue
                dist = math.hypot(d['pos'][0] - track['pos'][0], d['pos'][1] - track['pos'][1])
                if dist < best_dist:
                    best_dist, best_idx = dist, i

            if best_idx is not None:
                d = unmatched.pop(best_idx)
                track['pos'] = d['pos']
                track['misses'] = 0
            else:
                track['misses'] += 1

        # 2. Drop tracks that have been missing too long (stone likely collected/gone).
        for tid in [tid for tid, t in self._tracked_stones.items() if t['misses'] > self.STONE_MAX_MISSES]:
            del self._tracked_stones[tid]

        # 3. Any leftover detections are new stones — assign fresh ids.
        for d in unmatched:
            tid = self._next_stone_id
            self._next_stone_id += 1
            self._tracked_stones[tid] = {'pos': d['pos'], 'color': d['color'], 'misses': 0}

        # 4. Output only stones actually seen this frame (misses == 0) — used for
        # picking NEW targets (we don't want to pick a stale/ghost position).
        return [{'id': tid, 'color': t['color'], 'pos': t['pos']}
                for tid, t in self._tracked_stones.items() if t['misses'] == 0]

    def get_stone_by_id(self, stone_id):
        """
        Look up a specific tracked stone by its persistent id — used by the robot
        to keep aiming at "the same stone" it already locked onto. Unlike the
        list returned by get_state()['stones'] (fresh-detections only), this
        still returns the stone's last known position even if it was missed for
        up to STONE_MAX_MISSES frames (e.g. briefly hidden behind the robot's
        own body while approaching) — it only returns None once the track has
        actually expired and been dropped, meaning the stone is gone for good
        (collected, or lost track for too long).
        """
        t = self._tracked_stones.get(stone_id)
        if t is None:
            return None
        return {'id': stone_id, 'color': t['color'], 'pos': t['pos'], 'misses': t['misses']}

    def load_config(self):
        if os.path.exists(self.config_file):
            with open(self.config_file, 'r') as f:
                self.hsv_ranges = json.load(f)
        else:
            print(f"[WARNING] {self.config_file} not found. Please run calibrate_hsv.py first.")

    def px_to_mm(self, px_pos):
        """Convert pixel coordinates to millimeter coordinates using Homography"""
        pt = np.array([[[px_pos[0], px_pos[1]]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(pt, self.H_matrix)
        return (int(dst[0][0][0]), int(dst[0][0][1]))

    def _mm_to_px_local(self, mm_pos):
        """Inverse of px_to_mm — only used for drawing debug overlays (e.g. stone id labels)."""
        inv_H = np.linalg.inv(self.H_matrix)
        pt = np.array([[[mm_pos[0], mm_pos[1]]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(pt, inv_H)
        return (int(dst[0][0][0]), int(dst[0][0][1]))

    def calculate_dynamic_zones(self, stones):
        if not stones:
            return None, None
            
        sum_x = sum(s['pos'][0] for s in stones)
        sum_y = sum(s['pos'][1] for s in stones)
        count = len(stones)
        
        center_x = sum_x / count
        center_y = sum_y / count
        
        # Danger zone is centered around the mean stone position
        danger_zone = {
            'x_min': center_x - 350,
            'x_max': center_x + 350,
            'y_min': center_y - 200,
            'y_max': center_y + 200
        }
        return (center_x, center_y), danger_zone

    def get_state(self):
        """Returns the current state of the field: robot pos and stones."""
        # Opt-1 FIX: Flush old frames from buffer to reduce latency
        for _ in range(3):
            self.cap.grab()
        ret, frame = self.cap.read()
        if not ret:
            return None, None
            
        frame = cv2.resize(frame, (640, 480))
        state = {
            "robot": {"pos": None, "heading": 0, "mouth_pos": None},
            "stones": [],
            "drop_zones": dict(self.manual_drop_zones)  # fixed, calibrated once — not re-detected per frame
        }
        
        # 1. Detect Robot (ArUco ID:0)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        if ids is not None and 0 in ids:
            idx = np.where(ids == 0)[0][0]
            marker_corners = corners[idx][0]
            
            # Center of marker in pixels
            cX_px = int(np.mean(marker_corners[:, 0]))
            cY_px = int(np.mean(marker_corners[:, 1]))
            
            # Center of marker in MM
            state["robot"]["pos"] = self.px_to_mm((cX_px, cY_px))
            center_mm = state["robot"]["pos"]
            
            # CRIT-2 FIX: Convert front_mid to MM before calculating atan2
            # Assuming corners[0] and corners[1] represent the front edge.
            front_mid_X_px = (marker_corners[0][0] + marker_corners[1][0]) / 2
            front_mid_Y_px = (marker_corners[0][1] + marker_corners[1][1]) / 2
            front_mid_mm = self.px_to_mm((front_mid_X_px, front_mid_Y_px))
            
            # Calculate heading in MM-space
            angle_rad = math.atan2(front_mid_mm[1] - center_mm[1], front_mid_mm[0] - center_mm[0])
            state["robot"]["heading"] = angle_rad
            
            # Calculate mouth pos (110mm offset in front of marker)
            mX = center_mm[0] + 110 * math.cos(angle_rad)
            mY = center_mm[1] + 110 * math.sin(angle_rad)
            state["robot"]["mouth_pos"] = (int(mX), int(mY))
            
            cv2.polylines(frame, [marker_corners.astype(int)], True, (0, 255, 0), 2)
            
        # 2. Detect Stones (HSV Masking)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        raw_stone_detections = []  # collected first, then matched to persistent ids below
        for color_name, bounds in self.hsv_ranges.items():
            if color_name == "yellow": continue 
            
            lower = np.array(bounds['lower'])
            upper = np.array(bounds['upper'])
            
            # LOGIC-5 FIX: Red Hue Wrap-around
            if color_name == 'red':
                mask1 = cv2.inRange(hsv, lower, upper)
                mask2 = cv2.inRange(hsv, np.array([170, 100, 100]), np.array([175, 255, 255]))
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, lower, upper)
            
            # Noise reduction
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # Restrict detection to inside the field boundary only, so the red/brown
            # foam border tiles (or anything else outside the field) can never be
            # picked up as a stone or zone, regardless of color.
            if self.field_mask is not None:
                mask = cv2.bitwise_and(mask, self.field_mask)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if 100 < area < 2000: 
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        pos_mm = self.px_to_mm((cX, cY))

                        raw_stone_detections.append({"color": color_name, "pos": pos_mm})
                        cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
                # NOTE: drop zones are no longer detected here — they're fixed positions
                # loaded once from manual_dropzones.json (see load_manual_dropzones).
                # Re-detecting them from a per-frame color blob was unreliable: any other
                # object in view with a matching hue and area > 3000px (glare, cables,
                # skin, etc.) could hijack the position away from the real painted zone.

        # Match this frame's raw detections to previous frames' tracks so each
        # physical stone keeps the same 'id' for as long as it's tracked.
        state["stones"] = self._update_stone_tracks(raw_stone_detections)

        # Label each stone with its persistent id on screen, for debugging —
        # confirms the same stone keeps the same number across frames instead
        # of the numbers reshuffling every frame.
        for stone in state["stones"]:
            stone_px = self._mm_to_px_local(stone["pos"])
            cv2.putText(frame, f"#{stone['id']}", (stone_px[0] + 6, stone_px[1] + 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        return state, frame

    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()