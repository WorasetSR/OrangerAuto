import math
import time
import json
import websocket
import cv2
import numpy as np

# Import custom modules
from vision_tracker import VisionTracker
from strategy_engine import StrategyEngine
from path_planner import PathPlanner


def mm_to_px(H_matrix, mm_pos):
    """แปลงพิกัด mm กลับเป็นพิกเซลบนจอ (เหมือนใน debug_vision.py) ใช้วาดลูกศรบอกทิศทาง"""
    inv_H = np.linalg.inv(H_matrix)
    pt = np.array([[[mm_pos[0], mm_pos[1]]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(pt, inv_H)
    return (int(dst[0][0][0]), int(dst[0][0][1]))


def describe_motor_command(vL, vR, tolerance=20):
    """แปลงค่า vL, vR เป็นคำอธิบายทิศทางที่มนุษย์อ่านเข้าใจง่าย"""
    if vL == 0 and vR == 0:
        return "STOP"
    if abs(vL - vR) < tolerance:
        return "FORWARD" if vL > 0 else "BACKWARD"
    # ตามธรรมเนียมในโค้ด calculate_steering: vL<vR -> เลี้ยวขวา, vL>vR -> เลี้ยวซ้าย
    return "TURN RIGHT" if vL < vR else "TURN LEFT"

class GemstoneRobotController:
    """
    Finite State Machine for CPE 102 Gemstone Color Sorting Robot
    (Code Review V8 Final Polish)
    """
    def __init__(self, host="ws://192.168.4.1:81"):
        # FSM State
        self.state = "INIT"
        self.target_color = None
        self._last_valid_color = None 
        self.target_stone = None
        self.target_stone_id = None  # persistent id (from vision_tracker's stone tracker) we're locked onto
        self.stone_count = 0
        self._loaded_color = None # NEW-2 FIX: Track color inside the robot
        self.max_capacity = 4
        
        # Modules
        # VisionTracker automatically loads calibration.json (homography + field
        # boundary mask, so detection ignores the border tiles) and
        # manual_dropzones.json (fixed drop-zone positions) in its own __init__ —
        # the same calibrated files debug_vision.py uses. No need to load them again here.
        self.vision = VisionTracker()
        self.strategy = StrategyEngine()
        self.planner = PathPlanner()
        
        # Navigation
        self.waypoints = []
        self.current_waypoint_idx = 0
        
        # Timers
        self._state_timer = 0
        self._finished_printed = False # NEW-3 FIX: Prevent print spam

        # Robot-marker tracking: used to (a) log how often the ArUco marker drops
        # out, and (b) stop state timeouts (ALIGN_APPROACH etc.) from counting
        # time when we simply can't see the robot.
        self._robot_lost_since = None
        self._robot_lost_frame_count = 0

        # ค่าคำสั่งมอเตอร์ล่าสุด ใช้แสดงผลบนจอว่ากำลังสั่งให้รถไปทางไหน
        self._last_vL = 0
        self._last_vR = 0
        
        self.FALLBACK_DROP_ZONES = {
            'red': (2000, 1000), 'blue': (100, 1000), 'green': (100, 600),
            'purple': (100, 200), 'cyan': (1050, 200), 'orange': (2000, 200)
        }
        # Drop zones come from self.vision (calibrated manual_dropzones.json),
        # picked up in the INIT state below via state_data["drop_zones"].
        self.drop_zones = {}

        # WebSocket Setup
        self.ws_host = host
        self.ws = None
        self.connect_websocket()
        
        # Start Time
        self.start_time = time.time()
        self.MATCH_LIMIT_SEC = 300 # 5 minutes

    def connect_websocket(self):
        try:
            print(f"[NETWORK] Connecting to {self.ws_host}...")
            self.ws = websocket.WebSocket()
            self.ws.connect(self.ws_host, timeout=3)
            print("[NETWORK] Connected to ESP32 successfully!")
        except Exception as e:
            print(f"[NETWORK] ERROR: Could not connect to ESP32: {e}")
            self.ws = None

    def send_command(self, vL=0, vR=0, door_cmd=None):
        # เก็บค่าล่าสุดไว้แสดงผลบนจอ ไม่ว่าจะส่งสำเร็จหรือไม่
        self._last_vL = int(vL)
        self._last_vR = int(vR)

        if not self.ws:
            return 
            
        payload = {"vL": int(vL), "vR": int(vR)}
        if door_cmd:
            payload["door_cmd"] = door_cmd
            
        try:
            self.ws.send(json.dumps(payload))
        except Exception:
            self.connect_websocket()

    def update(self):
        # 1. Update World State from Camera
        state_data, frame = self.vision.get_state()
        if not state_data:
            return
            
        robot_pos = state_data["robot"]["pos"]
        robot_heading = state_data["robot"]["heading"]
        mouth_pos = state_data["robot"]["mouth_pos"]
        stones = state_data["stones"]
        
        # Timer check
        if time.time() - self.start_time > self.MATCH_LIMIT_SEC and self.state != "FINISH":
            self.state = "FINISH"

        # Display State
        cv2.putText(frame, f"STATE: {self.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Stones: {self.stone_count}/{self.max_capacity}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if self.target_color:
            cv2.putText(frame, f"Target: {self.target_color}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # แสดงคำสั่งมอเตอร์ล่าสุดที่กำลังสั่งอยู่ (ค่านี้มาจากรอบก่อนหน้า 1 เฟรม
        # เพราะ FSM ของเฟรมนี้ยังไม่คำนวณ vL/vR ใหม่ ณ จุดที่วาดภาพนี้)
        direction_label = describe_motor_command(self._last_vL, self._last_vR)
        cv2.putText(frame, f"CMD: {direction_label}  (vL={self._last_vL}, vR={self._last_vR})",
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if robot_pos is not None:
            robot_px = mm_to_px(self.vision.H_matrix, robot_pos)
            if direction_label in ("FORWARD", "BACKWARD"):
                # วาดลูกศรตามทิศทางที่รถกำลังมุ่งหน้า (หรือย้อนกลับถ้าถอยหลัง)
                travel_angle = robot_heading if direction_label == "FORWARD" else robot_heading + math.pi
                tip_mm = (robot_pos[0] + 220 * math.cos(travel_angle),
                          robot_pos[1] + 220 * math.sin(travel_angle))
                tip_px = mm_to_px(self.vision.H_matrix, tip_mm)
                arrow_color = (0, 255, 0) if direction_label == "FORWARD" else (0, 165, 255)
                cv2.arrowedLine(frame, robot_px, tip_px, arrow_color, 3, tipLength=0.35)
            elif direction_label in ("TURN LEFT", "TURN RIGHT"):
                # วาดวงกลมโค้งพร้อมหัวลูกศรบอกทิศการหมุน
                spin_color = (255, 0, 255)
                clockwise = (direction_label == "TURN RIGHT")
                start_angle, end_angle = (0, 300) if clockwise else (300, 0)
                cv2.ellipse(frame, robot_px, (35, 35), 0, start_angle, end_angle, spin_color, 3)
                cv2.putText(frame, direction_label, (robot_px[0] - 40, robot_px[1] - 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, spin_color, 2)

        cv2.imshow("Robot Command Center", frame)
        cv2.waitKey(1) # MIN-3: Moved waitKey here to avoid blocking vision pipeline

        # Skip logic if robot not found
        if not robot_pos:
            self._robot_lost_frame_count += 1
            if self._robot_lost_since is None:
                self._robot_lost_since = time.time()
            # Print every ~30 frames (roughly once a second) instead of every frame, to avoid log spam
            if self._robot_lost_frame_count % 30 == 1:
                lost_secs = time.time() - self._robot_lost_since
                print(f"[VISION] Robot ArUco marker not detected "
                      f"({self._robot_lost_frame_count} frames, {lost_secs:.1f}s so far)")
            self.send_command(0, 0)
            return

        if self._robot_lost_since is not None:
            # Marker just came back — extend the current state's timeout deadline
            # by however long it was gone, so lost-tracking time isn't charged
            # against ALIGN_APPROACH / DRIVE_INGEST / NAV_WAYPOINTS timeouts.
            lost_duration = time.time() - self._robot_lost_since
            print(f"[VISION] Robot ArUco marker reacquired after {lost_duration:.2f}s "
                  f"({self._robot_lost_frame_count} frames lost)")
            self._state_timer += lost_duration
            self._robot_lost_since = None
            self._robot_lost_frame_count = 0

        # 2. FSM Logic
        if self.state == "INIT":
            # Wait a few frames to collect data and setup zones
            if self._state_timer == 0:
                print("[STATE] INIT: Scanning field to calibrate zones...")
                self._state_timer = time.time()
                self.send_command(0, 0)
                
            # ถ้าระบบกล้องตรวจเจอโซนสีใหญ่ๆ ก็อัปเดตทับพิกัดให้แม่นยำขึ้น
            if state_data["drop_zones"]:
                self.drop_zones.update(state_data["drop_zones"])
                
            if time.time() - self._state_timer > 3.0:
                # 3 seconds scan complete
                center, danger_zone = self.vision.calculate_dynamic_zones(stones)
                if center and danger_zone:
                    self.strategy.update_center(center[0], center[1])
                    self.planner.update_danger_zone(
                        danger_zone['x_min'], danger_zone['x_max'],
                        danger_zone['y_min'], danger_zone['y_max']
                    )
                    print(f"[INIT] Dynamic Center set to: {center}")
                    print(f"[INIT] Dynamic Danger Zone set to: {danger_zone}")
                
                # CRIT-3: Fallback drop zones if not detected
                if len(self.drop_zones) < 6:
                    print(f"[WARNING] Only {len(self.drop_zones)}/6 drop zones detected, using fallbacks")
                    for color, pos in self.FALLBACK_DROP_ZONES.items():
                        if color not in self.drop_zones:
                            self.drop_zones[color] = pos
                            
                print(f"[INIT] Detected Drop Zones: {self.drop_zones}")
                # Reset timer for next state
                self._state_timer = 0
                self.state = "SCAN_OUTER_RING"
            
            return
            
        elif self.state == "SCAN_OUTER_RING":
            self.send_command(0, 0) # Stop
            
            # NEW-2 FIX: Only scan for new color if robot is empty
            if self.stone_count == 0 or self._loaded_color is None:
                self.target_color = self.strategy.analyze_outer_ring_color(stones, self.drop_zones)
            else:
                self.target_color = self._loaded_color
                
            if not self.target_color:
                if self.stone_count > 0 and self._last_valid_color:
                    self.planner.reset_pid()
                    drop_pos = self.drop_zones.get(self._last_valid_color, self.FALLBACK_DROP_ZONES.get(self._last_valid_color))
                    self.waypoints = self.planner.generate_perimeter_waypoints(robot_pos, drop_pos)
                    self.current_waypoint_idx = 0
                    self._state_timer = time.time() # RES-1 prep
                    self.state = "NAV_WAYPOINTS"
                else:
                    self.state = "FINISH"
                return 
                
            self._last_valid_color = self.target_color
                
            self.target_stone = self.strategy.find_best_stone(stones, self.target_color, robot_pos, self.drop_zones)
            if self.target_stone:
                self.target_stone_id = self.target_stone['id']  # lock onto this physical stone by persistent id
                self.planner.reset_pid()
                self._state_timer = time.time() # RES-2 prep
                self.state = "ALIGN_APPROACH"
            else:
                if self.stone_count > 0:
                    self.planner.reset_pid()
                    drop_pos = self.drop_zones.get(self.target_color, self.FALLBACK_DROP_ZONES.get(self.target_color))
                    self.waypoints = self.planner.generate_perimeter_waypoints(robot_pos, drop_pos)
                    self.current_waypoint_idx = 0
                    self._state_timer = time.time() # RES-1 prep
                    self.state = "NAV_WAYPOINTS"
                else:
                    self.target_color = None 
                    
        elif self.state == "ALIGN_APPROACH":
            # RES-2 FIX: Timeout to prevent soft-lock if stone is unreachable
            if time.time() - self._state_timer > 8.0:
                print("[WARNING] ALIGN Timeout! Target stone unreachable. Rescanning.")
                self.state = "SCAN_OUTER_RING"
                return

            # ล็อกเป้าด้วย persistent id แทนการคำนวณ "หินที่ดีที่สุด" ใหม่ทุกเฟรม —
            # กันไม่ให้เป้าหมายสลับไปมาระหว่างหินก้อนอื่นที่อยู่ใกล้ๆ กัน
            locked_stone = self.vision.get_stone_by_id(self.target_stone_id)
            if locked_stone is None:
                print(f"[TARGET] Stone id={self.target_stone_id} lost/collected. Rescanning.")
                self.state = "SCAN_OUTER_RING"
                return
            self.target_stone = locked_stone

            vL, vR, dist, aligned = self.planner.calculate_steering(robot_pos, robot_heading, self.target_stone['pos'])
            self.send_command(vL, vR)

            # DEBUG: log steering state a few times a second so we can see whether
            # the error is converging (heading bug) or the target keeps jumping
            # between different stone ids (target flapping in find_best_stone).
            self._align_debug_count = getattr(self, "_align_debug_count", 0) + 1
            if self._align_debug_count % 10 == 1:
                heading_deg = math.degrees(robot_heading)
                target_dx = self.target_stone['pos'][0] - robot_pos[0]
                target_dy = self.target_stone['pos'][1] - robot_pos[1]
                target_angle_deg = math.degrees(math.atan2(target_dy, target_dx))
                print(f"[DEBUG ALIGN] stone_id={self.target_stone_id} (misses={locked_stone.get('misses')}) "
                      f"robot_pos={robot_pos} heading={heading_deg:.1f}deg "
                      f"target_pos={self.target_stone['pos']} target_angle={target_angle_deg:.1f}deg "
                      f"dist={dist:.0f}mm vL={vL} vR={vR} aligned={aligned}")

            if aligned and dist < 200: # 20cm away, go straight
                self.send_command(0, 0, door_cmd="collect") # MAJ-5: Open door early
                self.planner.reset_pid()
                self._state_timer = time.time() 
                self.state = "DRIVE_INGEST"
                
        # =======================================================================================
        # 🟢 ZONE: ระบบเก็บหิน (STONE COLLECTION LOGIC)
        # ⚠️ เพื่อนร่วมทีมที่ทำระบบเก็บหิน ให้นำโค้ดมาเสียบที่ช่วงสเตทด้านล่างนี้ได้เลยครับ!
        # (สเตท DRIVE_INGEST -> LOCK_DOORS -> BACKOUT_CLEAR -> CHECK_CAPACITY)
        # =======================================================================================
        
        elif self.state == "DRIVE_INGEST":
            if time.time() - self._state_timer > 5.0:
                print("[WARNING] Ingest Timeout! Aborting stone.")
                self._state_timer = time.time()
                self.state = "BACKOUT_CLEAR"
                return

            locked_stone = self.vision.get_stone_by_id(self.target_stone_id)
            if locked_stone is not None:
                self.target_stone = locked_stone
            # else: track expired (likely just scooped up / occluded by the mouth
            # mechanism) — keep driving to the last known position since we're
            # already this close, rather than aborting the ingest.
                
            # CRIT-4 & NEW-1 FIX: Split mouth_pos for dist, robot_pos for steering
            vL, vR, _, _ = self.planner.calculate_steering(robot_pos, robot_heading, self.target_stone['pos'])
            self.send_command(vL, vR, door_cmd="collect") 
            
            # Distance from mouth to stone
            dist_to_stone = math.hypot(mouth_pos[0] - self.target_stone['pos'][0],
                                       mouth_pos[1] - self.target_stone['pos'][1])
            
            if dist_to_stone < 30: # 30mm from mouth_pos to stone
                self.send_command(0, 0)
                self.stone_count += 1
                self._loaded_color = self.target_color # NEW-2 FIX: Track loaded color
                self._state_timer = time.time() 
                self.state = "LOCK_DOORS"
            
        elif self.state == "LOCK_DOORS":
            self.send_command(0, 0, door_cmd="close") 
            
            if time.time() - self._state_timer > 0.3:
                self._state_timer = time.time()
                self.state = "BACKOUT_CLEAR"
            
        elif self.state == "BACKOUT_CLEAR":
            self.send_command(-150, -150) 
            
            if time.time() - self._state_timer > 0.5:
                self.send_command(0, 0) 
                self.state = "CHECK_CAPACITY"
            
        elif self.state == "CHECK_CAPACITY":
            if self.stone_count < self.max_capacity:
                self.state = "SCAN_OUTER_RING"
            else:
                self.planner.reset_pid()
                drop_pos = self.drop_zones.get(self.target_color, self.FALLBACK_DROP_ZONES.get(self.target_color))
                self.waypoints = self.planner.generate_perimeter_waypoints(robot_pos, drop_pos)
                self.current_waypoint_idx = 0
                self._state_timer = time.time() # RES-1 prep
                self.state = "NAV_WAYPOINTS"
                
        # =======================================================================================
        # 🔴 สิ้นสุด ZONE: ระบบเก็บหิน
        # =======================================================================================
                
        elif self.state == "NAV_WAYPOINTS":
            if not self.waypoints:
                self._state_timer = time.time()
                self.state = "UNLOAD_REVERSE"
                return

            # RES-1 FIX: Timeout for stuck waypoints
            if time.time() - self._state_timer > 8.0:
                print(f"[WARNING] Waypoint {self.current_waypoint_idx} Timeout! Skipping to next.")
                self.current_waypoint_idx += 1
                self.planner.reset_pid()
                self._state_timer = time.time()
                
            # Check completion after potential skip
            if self.current_waypoint_idx >= len(self.waypoints):
                self.send_command(0, 0)
                self._state_timer = time.time()
                self.state = "UNLOAD_REVERSE"
                return

            target_wp = self.waypoints[self.current_waypoint_idx]
            vL, vR, dist, _ = self.planner.calculate_steering(robot_pos, robot_heading, target_wp)
            
            # OPT-2 FIX: Scale up speed for navigation
            vL = int(vL * 1.3)
            vR = int(vR * 1.3)
            self.send_command(vL, vR)
            
            if dist < 100: # Reached waypoint
                self.current_waypoint_idx += 1
                self.planner.reset_pid()
                self._state_timer = time.time() # Reset timer for next WP
                if self.current_waypoint_idx >= len(self.waypoints):
                    self.send_command(0, 0)
                    self._state_timer = time.time()
                    self.state = "UNLOAD_REVERSE"
                
        elif self.state == "UNLOAD_REVERSE":
            elapsed = time.time() - self._state_timer
            
            # NEW-1 FIX: Properly ordered command phases
            if elapsed > 2.5:
                # Phase 3: Done - close doors and reset
                self.send_command(0, 0, door_cmd="close")
                self.stone_count = 0  
                self.target_color = None 
                self._loaded_color = None # NEW-2 FIX: Reset loaded color
                self.state = "SCAN_OUTER_RING"
            elif elapsed > 0.5:
                # Phase 2: Reverse with doors open (MAJ-2: 2.0 seconds)
                self.send_command(-250, -250, door_cmd="release")
            else:
                # Phase 1: Stop and open doors (wait for servo)
                self.send_command(0, 0, door_cmd="release")

        elif self.state == "FINISH":
            self.send_command(0, 0)
            
            # NEW-3 FIX: Prevent print spam, removed blocking sleep
            if not self._finished_printed:
                print("[STATE] FINISH: Mission Complete or Time Up!")
                self._finished_printed = True

if __name__ == "__main__":
    robot = GemstoneRobotController()
    try:
        while True:
            robot.update()
    except KeyboardInterrupt:
        robot.vision.close()
        print("System shutting down...")