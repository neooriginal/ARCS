
import cv2
import numpy as np
import math
import logging
from collections import deque

logger = logging.getLogger(__name__)

class SimpleSLAM:
    def __init__(self, width=640, height=480, map_size_pixels=800, map_resolution=0.05):
        self.width = width
        self.height = height
        
        # Map parameters
        self.map_resolution = map_resolution # meters per pixel
        self.map_size = map_size_pixels
        self.map_center = map_size_pixels // 2
        
        # 0 = Occupied, 127 = Unknown, 255 = Free
        # We use a float map for accumulation to allow "Log-Odds" style soft updates
        # But for performance and simplicity with OpenCV, we'll use uint8 with saturation.
        # Start at 127 (50% probability).
        # Free space adds value (towards 255).
        # Obstacles subtract value (towards 0).
        self.grid_map = np.full((self.map_size, self.map_size), 127, dtype=np.uint8)
        
        # Robot State
        # x, y in meters (global frame), theta in radians
        self.x = 0.0
        self.y = 0.0
        self.theta = -math.pi / 2 # Pointing UP (North) initially

        # Path trace for visualization
        self.path = deque(maxlen=1000)
        
        # Visual Odometry State
        self.last_frame = None
        self.last_keypoints = None
        
        # Feature Detection Parameters
        self.feature_params = dict(maxCorners=100,
                                   qualityLevel=0.3,
                                   minDistance=7,
                                   blockSize=7)
        
        # LK Optical Flow Parameters
        self.lk_params = dict(winSize=(15, 15),
                              maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
                              
        # Camera Calibration
        # y=480 is bottom (close), y=0 is top (far/horizon).
        self.cam_height = 0.2 # meters
        self.cam_tilt = -0.0  # radians
        
        # Reusing basic obstruction logic for map
        from obstacle_detection import ObstacleDetector
        self.detector = ObstacleDetector(width, height)


    def process(self, frame, movement_cmd=None):
        """
        Main SLAM Loop:
        1. Visual Odometry -> Update Pose
        2. Obstacle Detection -> Update Map
        """
        if frame is None:
            return
            
        # 1. Preprocess
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 2. Visual Odometry
        dx, dy, dtheta = 0, 0, 0
        vo_dist = 0.0
        
        if self.last_frame is not None:
             if self.last_keypoints is None or len(self.last_keypoints) < 8:
                 # Detect new features
                 self.last_keypoints = cv2.goodFeaturesToTrack(self.last_frame, mask=None, **self.feature_params)
             
             if self.last_keypoints is not None and len(self.last_keypoints) > 0:
                 p1, st, err = cv2.calcOpticalFlowPyrLK(self.last_frame, gray, self.last_keypoints, None, **self.lk_params)
                 
                 # Select good points
                 if p1 is not None:
                    good_new = p1[st == 1]
                    good_old = self.last_keypoints[st == 1]
                    
                    if len(good_new) > 4:
                        # Estimate transformation
                        m, _ = cv2.estimateAffinePartial2D(good_old, good_new)
                        
                        if m is not None:
                            img_tx = m[0, 2]
                            img_ty = m[1, 2]
                            
                            # Heuristic conversion to Robot motion
                            # Pixel shift X -> Rotation (Yaw)
                            # Image moves LEFT (+tx) -> Robot turned LEFT (-yaw)?
                            # Wait: If image moves LEFT, object moves LEFT. Robot turned RIGHT.
                            # Standard: Rotation = -tx * scale
                            yaw_scale = 0.0015 
                            dtheta = -img_tx * yaw_scale 
                            
                            # Translation Update
                            # Image moves DOWN (+ty) -> Robot moved FORWARD (+dist)
                            trans_scale = 0.001 
                            vo_dist = img_ty * trans_scale
                        
                    # Update keypoints for next step
                    self.last_keypoints = good_new.reshape(-1, 1, 2)
                 else:
                    self.last_keypoints = None
             else:
                 self.last_keypoints = None
                    
        # Motion Model
        cmd_v = 0.0
        cmd_w = 0.0
        
        if movement_cmd:
            if movement_cmd.get('forward'): cmd_v = 0.1
            if movement_cmd.get('backward'): cmd_v = -0.1
            if movement_cmd.get('left'): cmd_w = -0.15 # Left turn = +rotation? Usually. 
            # In standard ROS: Counter-Clockwise is Positive.
            # If dtheta reduces angle, it turns Right.
            # Initial theta = -pi/2 (-90).
            # Turn Left -> -pi/2 + 0.1 = -1.47 (towards 0/East).
            # Turn Right -> -pi/2 - 0.1 = -1.67 (towards -pi/West).
            # Visual check needed. Let's stick to standard CCW (+).
            if movement_cmd.get('left'): cmd_w = -0.15 
            if movement_cmd.get('right'): cmd_w = 0.15

        # Update Heading
        if abs(dtheta) > 0.001:
             self.theta += dtheta
        else:
             self.theta += cmd_w 
             
        # Normalize theta to -pi..pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Update Position
        if abs(vo_dist) > 0.001:
             distance = vo_dist
             if cmd_v != 0:
                 distance = (distance + cmd_v) / 2.0
        else:
             distance = cmd_v

        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        
        # Update State
        self.last_frame = gray.copy()
        self.path.append((self.x, self.y))
        
        # 3. Mapping Update
        self.update_map(frame)


    def update_map(self, frame):
        """Update occupancy grid using robust features from ObstacleDetector."""
        
        # 1. Use the robust edge detection from ObstacleDetector (Bilateral + Canny)
        # This gives us "Walls" that match the obstacle detection system.
        edges, _ = self.detector._detect_edges(frame)
        h, w = edges.shape
        horizon_y = h // 2
        
        # 2. Scan Columns to find the "Floor Boundary"
        # We step every 4 pixels for performance
        edge_points = self.detector._scan_columns(edges, w, h, step=4)
        
        # Robot position in grid
        rx = int(self.x / self.map_resolution) + self.map_center
        ry = int(self.y / self.map_resolution) + self.map_center
        
        if not (0 <= rx < self.map_size and 0 <= ry < self.map_size):
            return

        # Polygon points for Free Space
        poly_points = [(rx, ry)]
        obstacle_points = []
        
        # Process edges to find World Coordinates
        curr_wall_segment = []
        walls = [] # List of segments
        
        last_world_pos = None

        for (c, obs_y) in edge_points:
            # Ray angle
            ray_angle_cam = (c - (w/2)) * (math.radians(60) / w)
            ray_angle_global = self.theta + ray_angle_cam
            
            dist = 3.0 # Default max range
            is_obstacle = False
            
            # Distance Calculation matching SimpleSLAM logic
            if obs_y > horizon_y + 5:
                offset_y = obs_y - horizon_y
                K = 15.0 # Calibration constant
                dist = K / offset_y
                is_obstacle = True
            
            if dist > 3.0: 
                dist = 3.0
                is_obstacle = False
            
            # End point in World Frame
            ex = self.x + dist * math.cos(ray_angle_global)
            ey = self.y + dist * math.sin(ray_angle_global)
            
            # Grid Coords
            gex = int(ex / self.map_resolution) + self.map_center
            gey = int(ey / self.map_resolution) + self.map_center
            
            poly_points.append((gex, gey))
            
            if is_obstacle and dist < 2.5:
                # Add to obstacle points for dots
                obstacle_points.append((gex, gey))
                
                # Wall Connection Logic
                # If this point is close to the last point, add to segment
                if last_world_pos:
                    # dist between (ex, ey) and last_world_pos
                    d = math.hypot(ex - last_world_pos[0], ey - last_world_pos[1])
                    if d < 0.3: # 30cm discontinuity threshold
                         curr_wall_segment.append((gex, gey))
                    else:
                         # Break segment
                         if len(curr_wall_segment) > 1:
                             walls.append(curr_wall_segment)
                         curr_wall_segment = [(gex, gey)]
                else:
                    curr_wall_segment.append((gex, gey))
                
                last_world_pos = (ex, ey)
            else:
                # Free space break
                if len(curr_wall_segment) > 1:
                    walls.append(curr_wall_segment)
                curr_wall_segment = []
                last_world_pos = None

        # Flush last segment
        if len(curr_wall_segment) > 1:
            walls.append(curr_wall_segment)
        
        # 1. Create Masks for Update
        # Free Space Mask (Polygon)
        free_mask = np.zeros_like(self.grid_map)
        cv2.fillPoly(free_mask, [np.array(poly_points)], 255)
        
        # Obstacle Mask (Lines/Walls)
        obs_mask = np.zeros_like(self.grid_map)
        
        # Draw Dots (Strong)
        for (ox, oy) in obstacle_points:
            if 0 <= ox < self.map_size and 0 <= oy < self.map_size:
                 cv2.circle(obs_mask, (ox, oy), 2, 255, -1)
        
        # Draw Walls (Stronger Connection)
        for segment in walls:
            if len(segment) > 1:
                cv2.polylines(obs_mask, [np.array(segment)], False, 255, 2)

        
        # 2. Probabilistic Update
        # Increase "Free-ness"
        increment = 3 
        cv2.add(self.grid_map, increment, dst=self.grid_map, mask=free_mask)

        # Decrease "Free-ness" (Increase Obstacle probability)
        # Walls are very likely real
        decrement = 50 
        cv2.subtract(self.grid_map, decrement, dst=self.grid_map, mask=obs_mask)

    def get_map_overlay(self):
        """Return the map as a BGR image with robot pose drawn."""
        # Color Map Visualization
        # 0 (Occupied) -> Black/Blue
        # 127 (Unknown) -> Gray
        # 255 (Free) -> White
        
        # Apply nice color map
        # cv2.applyColorMap expects 0..255.
        # Let's customize:
        # Create 3-channel image
        vis_map = cv2.cvtColor(self.grid_map, cv2.COLOR_GRAY2BGR)
        
        # Draw Grid (every 1 meter)
        # 1 meter = 1 / 0.05 = 20 pixels
        grid_step = 20
        # Draw faint grid lines
        # Only draw where it is UNKNOWN (127) or FREE? 
        # Just draw everywhere with alpha blending? Too slow manually.
        # Draw lines directly
        
        # Vertical lines
        for x in range(0, self.map_size, grid_step):
            cv2.line(vis_map, (x, 0), (x, self.map_size), (40, 40, 40), 1)
        
        # Horizontal lines
        for y in range(0, self.map_size, grid_step):
            cv2.line(vis_map, (0, y), (self.map_size, y), (40, 40, 40), 1)
            
        # Draw Path
        path_points = []
        for (px, py) in self.path:
             gx = int(px / self.map_resolution) + self.map_center
             gy = int(py / self.map_resolution) + self.map_center
             path_points.append((gx, gy))
        
        if len(path_points) > 1:
            cv2.polylines(vis_map, [np.array(path_points)], False, (0, 255, 255), 1) # Yellow path
            
        # Draw Robot
        rx = int(self.x / self.map_resolution) + self.map_center
        ry = int(self.y / self.map_resolution) + self.map_center
        
        # Robot Body
        cv2.circle(vis_map, (rx, ry), 5, (0, 0, 255), -1) # Red dot
        
        # Heading Vector
        hx = int(rx + 15 * math.cos(self.theta))
        hy = int(ry + 15 * math.sin(self.theta))
        cv2.line(vis_map, (rx, ry), (hx, hy), (0, 0, 255), 2)
        
        # Draw Tracked Features (Projected onto Map)
        if self.last_keypoints is not None:
            # These are in Image Space. We need to project them to Map Space?
            # That's hard without depth for every point.
            # But we can just draw them in the CORNER of the image as a "Camera View" PIP (Picture in Picture)?
            # Or just ignore them on the map.
            # User wants to know "indexed" map.
            # Let's draw the "Field of View" wedge outline clearly
            fov_len = 30 # pixels on map (1.5m)
            fov_angle = math.radians(60)
            
            # Left edge
            lx = int(rx + fov_len * math.cos(self.theta - fov_angle/2))
            ly = int(ry + fov_len * math.sin(self.theta - fov_angle/2))
            cv2.line(vis_map, (rx, ry), (lx, ly), (0, 255, 0), 1)
            
            # Right edge
            rrx = int(rx + fov_len * math.cos(self.theta + fov_angle/2))
            rry = int(ry + fov_len * math.sin(self.theta + fov_angle/2))
            cv2.line(vis_map, (rx, ry), (rrx, rry), (0, 255, 0), 1)
            
        return vis_map
