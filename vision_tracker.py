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
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Field dimensions in mm
        self.FIELD_W_MM = 2100
        self.FIELD_H_MM = 1200
        
        # Homography Matrix (Mocked for now - should be calibrated using 4 corners of the field)
        scale_x = self.FIELD_W_MM / 640.0
        scale_y = self.FIELD_H_MM / 480.0
        self.H_matrix = np.array([
            [scale_x, 0, 0],
            [0, scale_y, 0],
            [0, 0, 1]
        ], dtype=np.float32)

    def calibrate_homography(self, src_points, dst_points):
        """src_points: 4 corners in pixel, dst_points: 4 corners in mm"""
        self.H_matrix, _ = cv2.findHomography(
            np.array(src_points, dtype=np.float32),
            np.array(dst_points, dtype=np.float32))

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
            "drop_zones": {}
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
        stone_id = 0
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
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if 100 < area < 2000: 
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        pos_mm = self.px_to_mm((cX, cY))
                        
                        state["stones"].append({
                            "id": stone_id,
                            "color": color_name,
                            "pos": pos_mm
                        })
                        stone_id += 1
                        cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
                elif area > 3000:
                    # Detect drop zone (large colored area)
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        pos_mm = self.px_to_mm((cX, cY))
                        
                        if color_name not in state["drop_zones"]:
                            state["drop_zones"][color_name] = pos_mm
                            cv2.drawContours(frame, [cnt], -1, (255, 0, 0), 2)
        
        return state, frame

    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()
