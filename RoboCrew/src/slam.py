
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
        
        # 0 = Unknown/Free, 1 = Free, 255 = Occupied (Visually: 127=Gray, 255=White, 0=Black)
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
        """Update occupancy grid using polygon filling for FOV."""
        small = cv2.resize(frame, (160, 120))
        edges = cv2.Canny(small, 50, 150)
        h, w = edges.shape
        horizon_y = h // 2
        
        # Robot position in grid
        rx = int(self.x / self.map_resolution) + self.map_center
        ry = int(self.y / self.map_resolution) + self.map_center
        
        if not (0 <= rx < self.map_size and 0 <= ry < self.map_size):
            return

        # Polygon points for Free Space
        poly_points = [(rx, ry)]
        obstacle_points = []
        
        # Reduce sampling stepping for smoother polygon, but keep performance
        step = 2 
        
        for c in range(0, w, step):
            # Find bottom-most edge in column
            obs_y = -1
            for r in range(h-1, -1, -1):
                if edges[r, c] > 0:
                    obs_y = r
                    break
            
            # Ray angle
            ray_angle_cam = (c - (w/2)) * (math.radians(60) / w)
            ray_angle_global = self.theta + ray_angle_cam # + or - depends on camera mount
            
            dist = 3.0 # Default max range
            is_obstacle = False
            
            if obs_y > horizon_y + 5:
                # Valid ground pixel
                offset_y = obs_y - horizon_y
                K = 15.0 # Calibration constant
                dist = K / offset_y
                is_obstacle = True
            
            if dist > 3.0: 
                dist = 3.0
                is_obstacle = False
            
            # End point
            ex = self.x + dist * math.cos(ray_angle_global)
            ey = self.y + dist * math.sin(ray_angle_global)
            
            gex = int(ex / self.map_resolution) + self.map_center
            gey = int(ey / self.map_resolution) + self.map_center
            
            poly_points.append((gex, gey))
            
            if is_obstacle and dist < 2.5:
                obstacle_points.append((gex, gey))
        
        # 1. Clear Free Space (White Polygon)
        # We draw a filled polygon representing the current FOV as "Free"
        # This overwrites any previous "Unknown" (Gray) or "transient" obstacles
        # However, to avoid erasing REAL static obstacles we saw before, 
        # usually we use log-odds. But for "SimpleSLAM", Overwriting is fine 
        # as long as the sensor is trusted.
        cv2.fillPoly(self.grid_map, [np.array(poly_points)], 255)
        
        # 2. Draw Obstacles (Black Dots)
        for (ox, oy) in obstacle_points:
            if 0 <= ox < self.map_size and 0 <= oy < self.map_size:
                 cv2.circle(self.grid_map, (ox, oy), 2, 0, -1)
    
    def draw_line(self, x0, y0, x1, y1, color):
        cv2.line(self.grid_map, (x0, y0), (x1, y1), int(color), 1)

    def get_map_overlay(self):
        """Return the map as a BGR image with robot pose drawn."""
        # Convert grid to color
        vis_map = cv2.cvtColor(self.grid_map, cv2.COLOR_GRAY2BGR)
        
        # Draw Path
        path_points = []
        for (px, py) in self.path:
             gx = int(px / self.map_resolution) + self.map_center
             gy = int(py / self.map_resolution) + self.map_center
             path_points.append((gx, gy))
        
        if len(path_points) > 1:
            cv2.polylines(vis_map, [np.array(path_points)], False, (255, 0, 0), 1)
            
        # Draw Robot
        rx = int(self.x / self.map_resolution) + self.map_center
        ry = int(self.y / self.map_resolution) + self.map_center
        
        cv2.circle(vis_map, (rx, ry), 3, (0, 0, 255), -1)
        
        # Heading Vector
        hx = int(rx + 10 * math.cos(self.theta))
        hy = int(ry + 10 * math.sin(self.theta))
        cv2.line(vis_map, (rx, ry), (hx, hy), (0, 0, 255), 2)
        
        return vis_map
