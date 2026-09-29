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
        self.vision = VisionTracker(camera_idx=0)
        self.strategy = StrategyEngine()
        self.planner = PathPlanner()
        
        # Navigation
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.stone_waypoints = []
        self.current_stone_wp_idx = 0
        
        # Timers
        self._state_timer = 0
        self._finished_printed = False # NEW-3 FIX: Prevent print spam

        # BUGFIX: get_state()['stones'] only contains *fresh* detections
        # (misses == 0 in vision_tracker), so a single noisy frame (motion
        # blur, glare, robot's own body occluding the pile right after a
        # rescan) can legitimately report zero stones even though stones are
        # still on the field. Require this many *real* seconds of continuous
        # "no target color found" before concluding the match is truly over,
        # instead of finishing on the very first empty frame.
        self._empty_scan_since = None
        self.EMPTY_SCAN_FINISH_SEC = 1.5

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
        self.MATCH_LIMIT_SEC = 600 # 5 minutes

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
            
            # POINT 2 FIX: ให้โอกาสกล้องเบลอ 1.0 วินาที ให้รถวิ่งด้วยความเร็วเดิมไปก่อน
            if time.time() - self._robot_lost_since < 1.0:
                self.send_command(self._last_vL, self._last_vR)
            else:
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
            # Manual Mode: Load everything from JSON and skip the 3 second wait
            if self._state_timer == 0:
                print("[STATE] INIT: Loading Manual Zones...")
                self._state_timer = time.time()
                self.send_command(0, 0)
                
            # ใช้ Drop zones ที่ตั้งค่ามาด้วยมือทั้งหมด
            self.drop_zones = self.vision.manual_drop_zones
            
            if self.vision.manual_danger_zone and self.vision.manual_center:
                dz = self.vision.manual_danger_zone
                self.planner.update_danger_zone(
                    dz['x_min'], dz['x_max'],
                    dz['y_min'], dz['y_max']
                )
                cx, cy = self.vision.manual_center
                self.strategy.update_center(cx, cy)
                print(f"[INIT] Manual Center set to: ({cx}, {cy})")
                print(f"[INIT] Manual Danger Zone set to: {dz}")
            else:
                print("\n[CRITICAL ERROR] Danger Zone not found in manual_dropzones.json!")
                print("Please run 'python setup_dropzones.py' to draw the Danger Zone.\n")
                self.state = "FINISH"
                return

            print(f"[INIT] Loaded Drop Zones: {self.drop_zones}")
            # ข้ามไปทำงานทันที ไม่ต้องรอ
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
                    self._empty_scan_since = None
                    self.planner.reset_pid()
                    drop_pos = self.drop_zones.get(self._last_valid_color, self.FALLBACK_DROP_ZONES.get(self._last_valid_color))
                    self.waypoints = self.planner.generate_perimeter_waypoints(robot_pos, drop_pos)
                    self.current_waypoint_idx = 0
                    self._state_timer = time.time() # RES-1 prep
                    self.state = "NAV_WAYPOINTS"
                else:
                    # BUGFIX: don't finish on one empty frame — wait to see if
                    # stones reappear once the camera gets a clean look again.
                    if self._empty_scan_since is None:
                        self._empty_scan_since = time.time()
                        print("[SCAN] No stones detected this frame, waiting to confirm field is actually empty...")
                    elif time.time() - self._empty_scan_since > self.EMPTY_SCAN_FINISH_SEC:
                        self.state = "FINISH"
                return 
                
            self._empty_scan_since = None
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
            
            # เช็คว่าเส้นทางขับชน Danger Zone หรือไม่
            needs_reroute = self.planner.is_inside_danger(robot_pos) or self.planner.line_crosses_danger(robot_pos, self.target_stone['pos'])
            if needs_reroute:
                self.stone_waypoints = self.planner.generate_perimeter_waypoints(robot_pos, self.target_stone['pos'])
                if len(self.stone_waypoints) > 1:
                    print(f"[NAV] Danger Zone detected in path to stone. Rerouting via {self.stone_waypoints[:-1]}")
                    self.current_stone_wp_idx = 0
                    self.planner.reset_pid()
                    self._state_timer = time.time()
                    self.state = "NAV_TO_STONE_WP"
                    return

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

            if aligned:
                # คำนวณระยะห่างจาก "ปาก" ถึง "หิน"
                mouth_pos = state_data["robot"]["mouth_pos"]
                dist_from_mouth = math.hypot(mouth_pos[0] - self.target_stone['pos'][0],
                                             mouth_pos[1] - self.target_stone['pos'][1])
                
                # ถ้าหินอยู่ห่างจากปากน้อยกว่า 120mm (อ้าประตูล่วงหน้าแต่เนิ่นๆ)
                if dist_from_mouth < 120:
                    raw_width = self.target_stone.get('width', 40)
                    
                    # ล็อกขนาดเปิด 2 ระดับ: ใหญ่=50mm, เล็ก=40mm (ปรับเกณฑ์ที่ 40mm)
                    if raw_width > 40: 
                        target_width = 50
                    else:
                        target_width = 40
                        
                    print(f"[ACTION] Aligned & Close! Stone Width: {raw_width}mm -> Opening door to {target_width}mm.")
                    self.send_command(0, 0, door_cmd=f"open:{target_width}") # สั่งเปิดประตูตามโหมดที่ล็อกไว้
                    self.planner.reset_pid()
                    self._state_timer = time.time() 
                    self.state = "DOOR_OPENING"
                
        # =======================================================================================
        # 🟢 ZONE: ระบบเก็บหิน (STONE COLLECTION LOGIC)
        # ⚠️ เพื่อนร่วมทีมที่ทำระบบเก็บหิน ให้นำโค้ดมาเสียบที่ช่วงสเตทด้านล่างนี้ได้เลยครับ!
        # (สเตท NAV_TO_STONE_WP -> DOOR_OPENING -> DRIVE_INGEST -> LOCK_DOORS -> BACKOUT_CLEAR -> CHECK_CAPACITY)
        # =======================================================================================
        
        elif self.state == "NAV_TO_STONE_WP":
            if not self.target_stone:
                self.state = "SCAN_OUTER_RING"
                return
            
            # ถ้าถึง waypoint รองสุดท้าย (พ้นอันตรายแล้ว) ให้กลับไป ALIGN_APPROACH เพื่อใช้กล้องเล็งเป้าจริง
            if self.current_stone_wp_idx >= len(self.stone_waypoints) - 1:
                self.planner.reset_pid()
                self._state_timer = time.time()
                self.state = "ALIGN_APPROACH"
                return
                
            # Timeout สำหรับ waypoint ที่ไปไม่ถึงสักที
            if time.time() - self._state_timer > 6.0:
                print(f"[WARNING] Stone Waypoint {self.current_stone_wp_idx} Timeout! Skipping.")
                self.current_stone_wp_idx += 1
                self.planner.reset_pid()
                self._state_timer = time.time()
                
            target_wp = self.stone_waypoints[self.current_stone_wp_idx]
            vL, vR, dist, _ = self.planner.calculate_steering(robot_pos, robot_heading, target_wp)
            self.send_command(vL, vR)
            
            if dist < 100: # ถึง waypoint ที่ใช้อ้อมแล้ว
                self.current_stone_wp_idx += 1
                self.planner.reset_pid()
                self._state_timer = time.time()

        elif self.state == "DOOR_OPENING":
            self.send_command(0, 0)  # หยุดรถรอประตูเปิด
            if time.time() - self._state_timer > 0.4:  # รอ 400ms ให้ servo กางจนสุด
                self._state_timer = time.time()
                self.state = "DRIVE_INGEST"
        
        elif self.state == "DRIVE_INGEST":
            # Condition A: จับเวลา 5 วินาที เป็นระบบฉุกเฉินถ้ารถติดกำแพง
            if time.time() - self._state_timer > 5.0:
                print("[WARNING] Ingest Timeout! Aborting stone.")
                # แอบจำพิกัดหินไว้ก่อนหุบประตู เพื่อเอาไปให้ VERIFY_COLLECTION ตรวจสอบ
                self._last_target_pos = self.target_stone['pos']
                self._state_timer = time.time()
                self.state = "LOCK_DOORS"
                return

            locked_stone = self.vision.get_stone_by_id(self.target_stone_id)
            if locked_stone is not None:
                self.target_stone = locked_stone
                
            # พุ่งตรงอย่างเดียว (Blind Ram) 
            vL, vR = 140, 140
            self.send_command(vL, vR)
            
            # Distance from mouth to stone
            dist_to_stone = math.hypot(mouth_pos[0] - self.target_stone['pos'][0],
                                       mouth_pos[1] - self.target_stone['pos'][1])
            
            # Condition B: ตัวสั่งเบรกและหุบประตู (ระยะใกล้พอ หรือ พุ่งนานพอ 1.5 วิ)
            if dist_to_stone < 30 or (time.time() - self._state_timer > 1.5):
                self.send_command(0, 0)
                # !! เอาคำสั่งบวกคะแนนออกไปแล้ว !!
                # แอบจำพิกัดหินสุดท้ายเอาไว้ เพื่อให้สเตทต่อไปมาตรวจผลงาน
                self._last_target_pos = self.target_stone['pos']
                self._state_timer = time.time() 
                self.state = "LOCK_DOORS"
                return
            
        elif self.state == "LOCK_DOORS":
            self.send_command(0, 0, door_cmd="close") 
            
            # ให้เวลาเซอร์โวปิดประตูนานขึ้น (จาก 0.3s เป็น 0.8s) เพื่อให้ประตูหุบสนิทจริงๆ ก่อนถอยตั้งหลัก
            if time.time() - self._state_timer > 0.8:
                self._state_timer = time.time()
                self.state = "BACKOUT_CLEAR"
            
        elif self.state == "BACKOUT_CLEAR":
            # ถอยหลังช้าๆ (-120) เป็นเวลา 0.8 วินาที เพื่อถอยตั้งหลักและหลบทางให้กล้องมองเห็นพื้น
            self.send_command(-120, -120) 
            
            if time.time() - self._state_timer > 0.8:
                self.send_command(0, 0) 
                self._state_timer = time.time()
                self.state = "VERIFY_COLLECTION" # ไปตรวจผลงานก่อนเช็คความจุ
                
        elif self.state == "VERIFY_COLLECTION":
            # ให้รถหยุดนิ่ง 0.5 วินาที เพื่อรอกล้องจับภาพพื้นให้ชัดเจน (กันภาพเบลอตอนรถเบรก)
            self.send_command(0, 0)
            if time.time() - self._state_timer > 0.5:
                # กวาดสายตาดูว่าพิกัดเดิมที่เราเพิ่งงับไป ยังมีหินสีเป้าหมายหลงเหลืออยู่ไหม
                missed = False
                for s in stones:
                    if s['color'] == self.target_color:
                        # วัดระยะห่างจากหินที่เห็น กับ พิกัดสุดท้ายที่เราจำไว้
                        dist_from_last_pos = math.hypot(s['pos'][0] - self._last_target_pos[0],
                                                        s['pos'][1] - self._last_target_pos[1])
                        # ถ้าระยะห่างน้อยกว่า 80mm แปลว่ามันคือก้อนเดิมที่ยังไม่ขยับไปไหน!
                        if dist_from_last_pos < 80:
                            missed = True
                            break
                
                if missed:
                    print(f"[VERIFY] งับพลาด! หินยังอยู่ที่เดิม (ไม่บวกคะแนน)")
                    # กลับไปเริ่มหาหินใหม่
                    self.state = "SCAN_OUTER_RING"
                else:
                    print(f"[VERIFY] สำเร็จ! หินหายไปจากพื้นแล้ว (บวก 1 คะแนน)")
                    self.stone_count += 1
                    self._loaded_color = self.target_color
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
            
            # คำนวณระยะห่างปัจจุบันระหว่างรถกับจุดกึ่งกลาง Dropzone
            drop_pos = self.drop_zones.get(self.target_color, self.FALLBACK_DROP_ZONES.get(self.target_color))
            dist_from_drop = 0
            if robot_pos and drop_pos:
                dist_from_drop = math.hypot(robot_pos[0] - drop_pos[0], robot_pos[1] - drop_pos[1])
            
            # ถอยหลังจนกว่าระยะห่างจะพ้นรัศมีดรอปโซน (120mm) + ครึ่งคันรถ (ประมาณ 150mm) = 270mm
            # หรือถ้าเกิน 2.5 วินาทีแล้วก็ตัดจบเลย (รอประตู 1.5วิ + ถอย 1.0วิ) เผื่อกล้องค้างหรือรถติด
            is_clear_of_zone = (dist_from_drop > 270)
            
            if elapsed > 1.5 and (is_clear_of_zone or elapsed > 2.5):
                # Phase 3: พ้นโซนแล้ว หุบประตูและไปต่อ
                self.send_command(0, 0, door_cmd="close")
                self.stone_count = 0  
                self.target_color = None 
                self._loaded_color = None
                self.state = "SCAN_OUTER_RING"
            elif elapsed > 1.5:
                # Phase 2: เข้าเกียร์ถอยหลัง
                self.send_command(-140, -140, door_cmd="release")
            else:
                # Phase 1: Stop and open doors (รอ 1.5 วินาที เพื่อให้ Servo กางออกจนสุดจริงๆ)
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
        robot.send_command(0, 0, door_cmd="close")
        robot.vision.close()
        if robot.ws:
            robot.ws.close()
        print("System shutting down...")