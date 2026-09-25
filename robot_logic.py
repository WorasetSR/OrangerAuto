import math
import time
import json
import os
import websocket
import cv2

# Import custom modules
from vision_tracker import VisionTracker
from strategy_engine import StrategyEngine
from path_planner import PathPlanner

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
        self.stone_count = 0
        self._loaded_color = None # NEW-2 FIX: Track color inside the robot
        self.max_capacity = 4
        
        # Modules
        self.vision = VisionTracker()
        self.strategy = StrategyEngine()
        self.planner = PathPlanner()
        
        # Navigation
        self.waypoints = []
        self.current_waypoint_idx = 0
        
        # Timers
        self._state_timer = 0
        self._finished_printed = False # NEW-3 FIX: Prevent print spam
        
        self.FALLBACK_DROP_ZONES = {
            'red': (2000, 1000), 'blue': (100, 1000), 'green': (100, 600),
            'purple': (100, 200), 'cyan': (1050, 200), 'orange': (2000, 200)
        }
        # Dynamically discovered or Manually set drop zones
        self.drop_zones = {}
        if os.path.exists("manual_dropzones.json"):
            try:
                with open("manual_dropzones.json", "r") as f:
                    raw = json.load(f)
                    self.drop_zones = {k: tuple(v) for k, v in raw.items()}
                print(f"[INIT] Loaded MANUAL Drop Zones from file: {self.drop_zones}")
            except Exception as e:
                print(f"[INIT] Error loading manual dropzones: {e}")

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
        cv2.imshow("Robot Command Center", frame)
        cv2.waitKey(1) # MIN-3: Moved waitKey here to avoid blocking vision pipeline

        # Skip logic if robot not found
        if not robot_pos:
            self.send_command(0, 0)
            return

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

            updated_stone = self.strategy.find_best_stone(stones, self.target_color, robot_pos, self.drop_zones)
            if updated_stone:
                self.target_stone = updated_stone
            else:
                self.state = "SCAN_OUTER_RING"
                return

            vL, vR, dist, aligned = self.planner.calculate_steering(robot_pos, robot_heading, self.target_stone['pos'])
            self.send_command(vL, vR)
            
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

            updated_stone = self.strategy.find_best_stone(stones, self.target_color, robot_pos, self.drop_zones)
            if updated_stone:
                self.target_stone = updated_stone
                
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
