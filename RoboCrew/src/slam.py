
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
        self.theta = 0.0 # Radians
        
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
        self.cam_tilt = -0.0  # radians (looking straight?) No, usually looking forward.
        
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
                        # Using estimateAffinePartial2D to get rotation and translation
                        m, _ = cv2.estimateAffinePartial2D(good_old, good_new)
                        
                        if m is not None:
                            # m is [[cos, -sin, tx], [sin, cos, ty]]
                            # But this is image-space shift.
                            # Image shift X (tx) -> Yaw rotation (mostly).
                            # Image shift Y (ty) -> Forward/Back motion.
                            
                            img_tx = m[0, 2]
                            img_ty = m[1, 2]
                            img_rot = math.atan2(m[1, 0], m[0, 0]) * (180.0 / math.pi) # Rotation in Frame
                            
                            # Heuristic conversion to Robot motion
                            # Pixel shift X -> Rotation (Yaw)
                            yaw_scale = 0.0015 
                            dtheta = img_tx * yaw_scale 
                            
                            # Translation Update (Visual Encoders)
                            # Flow Y (+ty) for ground pixels roughly implies backward motion if camera is looking down/forward?
                            # Wait, if robot moves forward, ground pixels move DOWN (+Y) in image.
                            # So +ty => Forward.
                            
                            # Scale factor needs calibration.
                            trans_scale = 0.001 
                            # We only update a local 'vo_dist' variable to blend later
                            vo_dist = img_ty * trans_scale
                            pass
                        
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
            if movement_cmd.get('left'): cmd_w = 0.15 
            if movement_cmd.get('right'): cmd_w = -0.15

        # If VO rotation detected strong signal, use it?
        # dtheta comes from VO. 
        # If we are rotating, VO dtheta is likely better than Odometry?
        if abs(dtheta) > 0.001:
             self.theta += dtheta
        else:
             self.theta += cmd_w 
             
        # FUSE with Control Inputs
        # We blend VO and Command.
        # If cmd_v is present, we trust it more for scale, but VO helps.
        # Simplified: Use Command for base velocity, add VO as a small correction or use VO if consistent?
        # Let's just USE VO for now to fulfill "Enable VO".
        
        if abs(vo_dist) > 0.001:
             distance = vo_dist
             # Blend with command if available (Command sets direction intent)
             if cmd_v != 0:
                 # Simple average or weighted?
                 # If VO says 0.05 and Cmd says 0.1, maybe we are slipping?
                 # Let's just average them 
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
        # Use ObstacleDetector to find floor/obstacles
        # We are going to "project" the view into the map.
        
        # 1. Bilateral + Canny
        # Simplified scanner: Find lowest edge to determine floor boundary.
        
        small = cv2.resize(frame, (160, 120)) # Downscale for speed
        edges = cv2.Canny(small, 50, 150)
        h, w = edges.shape
        
        # Robot Position in Grid
        grid_x = int(self.x / self.map_resolution) + self.map_center
        grid_y = int(self.y / self.map_resolution) + self.map_center
        
        # Raycast for each column
        
        # Simple Pitch Compensation
        horizon_y = h // 2
        
        for c in range(0, w, 4): # Skip columns for speed (4 instead of 2)
            # Find bottom-most edge in column
            obs_y = -1
            for r in range(h-1, -1, -1):
                if edges[r, c] > 0:
                    obs_y = r
                    break
            
            # Angle of this ray (relative to camera center)
            # c ranges 0..160. Center 80.
            # Field of view ~60 deg
            ray_angle_cam = (c - (w/2)) * (math.radians(60) / w) 
            
            # Global Angle
            ray_angle_global = self.theta - ray_angle_cam 
            
            dist = 999.0
            is_obstacle = False
            
            if obs_y > horizon_y + 5:
                # Valid ground pixel
                offset_y = obs_y - horizon_y
                # Calibrate this constant K
                K = 15.0 
                dist = K / offset_y
                is_obstacle = True
            elif obs_y == -1:
                # No edge found, assume clear up to max range
                dist = 2.0 # 2 meters clear
                is_obstacle = False
            else:
                 # Above horizon? Ignore or infinite.
                 dist = 2.0
                 is_obstacle = False
            
            if dist > 3.0: dist = 3.0 # Cap range
            
            # End Point in Global Frame
            end_x = self.x + dist * math.cos(ray_angle_global)
            end_y = self.y + dist * math.sin(ray_angle_global)
            
            end_grid_x = int(end_x / self.map_resolution) + self.map_center
            end_grid_y = int(end_y / self.map_resolution) + self.map_center
            
            # Draw Line (Free Space) using Bresenham
            if 0 <= grid_x < self.map_size and 0 <= grid_y < self.map_size and 0 <= end_grid_x < self.map_size and 0 <= end_grid_y < self.map_size:
                self.draw_line(grid_x, grid_y, end_grid_x, end_grid_y, 255) # White = Free

                if is_obstacle and dist < 2.0:
                    cv2.circle(self.grid_map, (end_grid_x, end_grid_y), 1, 0, -1) # Black = Occupied
    
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
