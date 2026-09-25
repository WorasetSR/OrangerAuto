import math

class StrategyEngine:
    def __init__(self):
        # Field center where the pile of 54 stones is located (in mm)
        self.CENTER_X = 1050
        self.CENTER_Y = 600

    def update_center(self, x, y):
        self.CENTER_X = x
        self.CENTER_Y = y

    def get_distance_from_center(self, pos):
        """Calculate radial distance from the center pile."""
        return math.hypot(pos[0] - self.CENTER_X, pos[1] - self.CENTER_Y)

    def get_valid_stones(self, stones, drop_zones):
        """Filter out stones that are already in their respective drop zones."""
        valid = []
        # รัศมี Drop zone จริงคือ 8.5 cm (85 mm) + เผื่อระยะหินกระเด็นขอบๆ อีกนิดหน่อย เป็น 120 mm
        DROP_RADIUS = 120 
        for s in stones:
            in_drop = False
            # MAJ-2 FIX: ตรวจสอบเฉพาะ Drop zone ที่สีตรงกับหิน
            if s['color'] in drop_zones:
                dz_pos = drop_zones[s['color']]
                if math.hypot(s['pos'][0] - dz_pos[0], s['pos'][1] - dz_pos[1]) < DROP_RADIUS:
                    in_drop = True
                    
            if not in_drop:
                valid.append(s)
        return valid

    def analyze_outer_ring_color(self, stones, drop_zones):
        """
        Determines the best color to collect based on outer-ring abundance.
        The top 15 stones furthest from center are considered 'outer ring'.
        """
        stones = self.get_valid_stones(stones, drop_zones)
        if not stones:
            return None
            
        # Sort all stones by distance from center (descending)
        sorted_stones = sorted(stones, key=lambda s: self.get_distance_from_center(s['pos']), reverse=True)
        
        # Take the top 15 furthest stones
        outer_ring = sorted_stones[:15]
        
        # Count color frequencies in the outer ring
        color_counts = {}
        for s in outer_ring:
            color = s['color']
            color_counts[color] = color_counts.get(color, 0) + 1
            
        if not color_counts:
            return None
            
        # Pick the color with the highest count
        best_color = max(color_counts, key=color_counts.get)
        return best_color

    def find_best_stone(self, stones, target_color, robot_pos, drop_zones):
        """
        Finds the most accessible stone of the target color.
        Prioritizes stones furthest from center (to peel from outside in),
        then by proximity to the robot.
        """
        stones = self.get_valid_stones(stones, drop_zones)
        if not stones or not robot_pos:
            return None
            
        valid_stones = [s for s in stones if s['color'] == target_color]
        if not valid_stones:
            return None
            
        # Sort by furthest from center pile first, then closest to robot
        # We use a combined score: (Distance from center * weight) - (Distance to robot * weight)
        best_stone = None
        best_score = -float('inf')
        
        for s in valid_stones:
            dist_center = self.get_distance_from_center(s['pos'])
            dist_robot = math.hypot(s['pos'][0] - robot_pos[0], s['pos'][1] - robot_pos[1])
            
            # Score logic: we want HIGH dist_center and LOW dist_robot
            score = dist_center - (dist_robot * 0.5) 
            
            if score > best_score:
                best_score = score
                best_stone = s
                
        return best_stone
