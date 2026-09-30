import math

class PathPlanner:
    def __init__(self):
        # PID constants for heading correction
        self.Kp_heading = 2.0
        self.Kd_heading = 0.5
        self.last_heading_error = 0
        
        # Base speed for navigation
        self.base_speed = 170
        self.max_speed = 255
        self.FIELD_W = 2100
        self.FIELD_H = 1200
        self.DANGER_ZONE = None

    def update_danger_zone(self, x_min, x_max, y_min, y_max):
        self.DANGER_ZONE = {
            'x_min': x_min, 'x_max': x_max,
            'y_min': y_min, 'y_max': y_max
        }

    def reset_pid(self):
        """LOGIC-6 FIX: Reset D-term accumulation when changing targets"""
        self.last_heading_error = 0

    def is_inside_danger(self, pos):
        """Check if a point is inside the danger zone."""
        if not self.DANGER_ZONE: return False
        return (self.DANGER_ZONE['x_min'] <= pos[0] <= self.DANGER_ZONE['x_max'] and
                self.DANGER_ZONE['y_min'] <= pos[1] <= self.DANGER_ZONE['y_max'])

    def line_crosses_danger(self, p1, p2):
        """Check if line segment p1-p2 intersects the danger zone (AABB)."""
        if not self.DANGER_ZONE: return False
        # Quick bounding box overlap check for the line segment
        min_x, max_x = min(p1[0], p2[0]), max(p1[0], p2[0])
        min_y, max_y = min(p1[1], p2[1]), max(p1[1], p2[1])
        
        # If the bounding box of the line doesn't even intersect the danger zone, it's safe
        if (max_x < self.DANGER_ZONE['x_min'] or min_x > self.DANGER_ZONE['x_max'] or
            max_y < self.DANGER_ZONE['y_min'] or min_y > self.DANGER_ZONE['y_max']):
            return False
            
        # Simplified for grid-aligned movement check:
        # Since we use simple waypoints, if the line spans across the zone, we consider it crossing.
        # This is a safe over-estimation.
        return True

    def generate_perimeter_waypoints(self, current_pos, target_pos):
        """
        Creates a list of waypoints that routes around the central danger zone.
        """
        waypoints = []
        
        # Check if we are inside or crossing the danger zone
        needs_reroute = self.is_inside_danger(current_pos) or self.line_crosses_danger(current_pos, target_pos)

        if needs_reroute:
            # 4 safe highways around the danger zone
            safe_top_y = 300
            safe_bot_y = 900
            safe_left_x = 400
            safe_right_x = 1700
            
            if self.DANGER_ZONE:
                safe_top_y = min(self.DANGER_ZONE['y_min'] - 200, 300)
                safe_bot_y = max(self.DANGER_ZONE['y_max'] + 200, 900)
                safe_left_x = min(self.DANGER_ZONE['x_min'] - 200, 400)
                safe_right_x = max(self.DANGER_ZONE['x_max'] + 200, 1700)
            
            # Decide which horizontal highway to use based on current position
            safe_y = safe_top_y if current_pos[1] < self.FIELD_H / 2 else safe_bot_y
            target_safe_y = safe_top_y if target_pos[1] < self.FIELD_H / 2 else safe_bot_y
            
            # 1. Get out to the nearest safe Y highway
            waypoints.append((current_pos[0], safe_y))
            
            # 2. Check if we need to cross the danger zone vertically
            if safe_y != target_safe_y:
                # Cross vertically via a safe X highway (choose the one closer to target)
                safe_x = safe_left_x if target_pos[0] < self.FIELD_W / 2 else safe_right_x
                waypoints.append((safe_x, safe_y))          # Move horizontally to vertical highway
                waypoints.append((safe_x, target_safe_y))   # Move vertically across field
                waypoints.append((target_pos[0], target_safe_y)) # Move horizontally to target's column
            else:
                # Same side, just move horizontally to target's column
                waypoints.append((target_pos[0], safe_y))
            
        # Final waypoint: The target itself
        waypoints.append(target_pos)
        
        # Clean up duplicate consecutive waypoints
        clean_wps = []
        for wp in waypoints:
            if not clean_wps or (abs(clean_wps[-1][0] - wp[0]) > 10 or abs(clean_wps[-1][1] - wp[1]) > 10):
                clean_wps.append(wp)
                
        return clean_wps

    def calculate_steering(self, robot_pos, robot_heading, target_pos):
        """
        Calculates left and right motor speeds using PID on heading error.
        Returns: (vL, vR, distance, is_aligned)
        """
        if not robot_pos or not target_pos:
            return 0, 0, 0, False
            
        # Calculate angle to target
        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]
        target_angle = math.atan2(dy, dx)
        
        # Calculate error in heading (-PI to PI)
        error = target_angle - robot_heading
        # Normalize error
        while error > math.pi: error -= 2 * math.pi
        while error < -math.pi: error += 2 * math.pi
        
        # PID (PD actually)
        p_term = self.Kp_heading * error
        d_term = self.Kd_heading * (error - self.last_heading_error)
        correction = int(p_term + d_term)
        self.last_heading_error = error
        
        # Distance to target
        distance = math.hypot(dx, dy)
        
        # If the angle is severely wrong (e.g. > 45 degrees), just spin in place
        if abs(error) > math.radians(45):
            # Spin (หมุนอยู่กับที่)
            # แก้บั๊ก: error > 0 คือเป้าหมายอยู่ทางขวา ต้องหมุนขวา (ล้อซ้ายเดินหน้า, ล้อขวาถอยหลัง)
            spin_speed = 190
            if error > 0:
                vL, vR = spin_speed, -spin_speed  # หมุนขวา
            else:
                vL, vR = -spin_speed, spin_speed  # หมุนซ้าย
            return vL, vR, distance, False
            
        # ถ้ามุมเริ่มตรงแล้ว ให้วิ่งเดินหน้าพร้อมปรับแต่งทิศทาง
        # แก้บั๊ก: error > 0 ต้องให้ล้อซ้ายเร็วกว่าล้อขวา เพื่อเลี้ยวขวา
        vL = self.base_speed + correction
        vR = self.base_speed - correction
        
        # Constrain speeds
        vL = max(-self.max_speed, min(self.max_speed, vL))
        vR = max(-self.max_speed, min(self.max_speed, vR))
        
        is_aligned = abs(error) < math.radians(10)
        
        return vL, vR, distance, is_aligned
