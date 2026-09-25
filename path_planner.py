import math

class PathPlanner:
    def __init__(self):
        # PID constants for heading correction
        self.Kp_heading = 2.0
        self.Kd_heading = 0.5
        self.last_heading_error = 0
        
        # Base speed for navigation
        self.base_speed = 150
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

    def generate_perimeter_waypoints(self, current_pos, drop_zone):
        """
        Creates a list of waypoints that routes around the central danger zone.
        """
        waypoints = []
        
        # Check if we are inside or crossing the danger zone
        needs_reroute = self.is_inside_danger(current_pos) or self.line_crosses_danger(current_pos, drop_zone)

        if needs_reroute:
            # Determine if top route or bottom route is closer
            dist_to_top = current_pos[1]
            dist_to_bottom = self.FIELD_H - current_pos[1]
            
            # Use 300 / 900 to avoid edges where Drop Zones might be
            safe_y = 300 if dist_to_top < dist_to_bottom else 900
            
            # NEW-2 FIX: Avoid duplicate waypoints
            # 1st waypoint: Move to the safe highway (align horizontally)
            waypoints.append((current_pos[0], safe_y))
                
            # 2nd waypoint: Move horizontally across the safe highway
            waypoints.append((drop_zone[0], safe_y))
            
        # Final waypoint: The drop zone
        waypoints.append(drop_zone)
        
        return waypoints

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
            # Spin
            spin_speed = 100
            if error > 0:
                vL, vR = -spin_speed, spin_speed # Turn right
            else:
                vL, vR = spin_speed, -spin_speed # Turn left
            return vL, vR, distance, False
            
        # If mostly aligned, drive forward with correction
        vL = self.base_speed - correction
        vR = self.base_speed + correction
        
        # Constrain speeds
        vL = max(-self.max_speed, min(self.max_speed, vL))
        vR = max(-self.max_speed, min(self.max_speed, vR))
        
        is_aligned = abs(error) < math.radians(10)
        
        return vL, vR, distance, is_aligned
