import cv2
import numpy as np
import logging

logger = logging.getLogger(__name__)

class ObstacleDetector:
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        # Thresholds tuned for "Stop at the very last moment"
        # Y=480 is bottom. Y=420 is very close.
        self.obstacle_threshold_y = 420
        self.center_x_threshold = 310
        
    def process(self, frame):
        """
        Process the frame to detect obstacles and determine navigation command.
        Returns:
            safe_actions (list): List of allowed actions ['FORWARD', 'LEFT', 'RIGHT', 'BACKWARD']
            overlay (np.ndarray): Frame with debug drawing
            metrics (dict): Internal metrics
        """
        if frame is None:
            return ["STOP"], None, {}

        # 1. Noise Reduction using Bilateral Filter
        filtered = cv2.bilateralFilter(frame, 9, 75, 75)
        
        # 2. Canny Edge Detection
        edges = cv2.Canny(filtered, 50, 150)
        
        h, w = edges.shape
        edge_points = []
        
        # Visualization setup
        overlay = frame.copy()
        
        # Create a separate layer for semi-transparent shapes
        shapes = frame.copy()
        
        # 3. Column Scan
        for x in range(0, w, 5):
            detected_y = 0 
            found = False
            for y in range(h - 1, -1, -1):
                if edges[y, x] == 255:
                    detected_y = y
                    found = True
                    break
            edge_points.append((x, detected_y))
            if found:
                cv2.circle(overlay, (x, detected_y), 2, (0, 0, 255), -1)

        # 4. Chunking & Metics
        num_points = len(edge_points)
        chunk_size = num_points // 3
        
        left_chunk = edge_points[:chunk_size]
        center_chunk = edge_points[chunk_size:2*chunk_size]
        right_chunk = edge_points[2*chunk_size:]
        
        def get_avg_y(chunk):
            if not chunk: return 0
            return sum(p[1] for p in chunk) / len(chunk)
            
        c_left = get_avg_y(left_chunk)
        c_fwd = get_avg_y(center_chunk)
        c_right = get_avg_y(right_chunk)

        # 5. Safety Logic (The "Prohibit" Logic)
        safe_actions = ["BACKWARD"] # Backward is mostly always safe in this context (blind luck)
        
        threshold = self.obstacle_threshold_y
        
        # --- BLINDNESS CHECK ---
        # If robot is face-to-face with a smooth wall, Canny sees NO edges.
        # We check the total number of edge pixels.
        # A normal scene with floor/furniture should have thousands of edge pixels.
        # A blank wall might have < 100.
        
        total_edge_pixels = np.count_nonzero(edges)
        # Threshold: Experimentally, 500 pixels is very low for 640x480 resolution (which has 300k pixels)
        # Even a simple horizon line is ~640 pixels.
        min_edge_pixels = 200 
        
        is_blind = total_edge_pixels < min_edge_pixels
        
        if is_blind:
            # We are likely staring at a blank wall or in darkness.
            # Block Forward.
            cv2.rectangle(shapes, (int(w*0.2), int(h*0.2)), (int(w*0.8), int(h*0.8)), (0, 0, 255), -1)
            cv2.putText(overlay, "BLOCKED (NO VISUALS)", (int(w*0.3), int(h*0.5)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            # Safe actions only backward (maybe turn? but we don't know what's there)
            # Let's allow turning just in case we need to unstick.
            safe_actions.append("LEFT")
            safe_actions.append("RIGHT")
            
        else:
            # Standard Logic
            
            # Check Center for Forward
            # If Center is blocked -> No Forward
            if c_fwd <= threshold:
                safe_actions.append("FORWARD")
            else:
                 # Draw Red Box on Center
                 cv2.rectangle(shapes, (int(w*0.33), int(h*0.5)), (int(w*0.66), h), (0, 0, 255), -1)
                 
            # Check Sides
            side_threshold = threshold + 50 
            
            if c_left <= side_threshold:
                safe_actions.append("LEFT")
                pts = np.array([[0, h], [0, h//2], [w//3, h//2], [0, h]], np.int32)
            else:
                cv2.rectangle(shapes, (0, int(h*0.5)), (int(w*0.33), h), (0, 0, 255), -1)
                
            if c_right <= side_threshold:
                safe_actions.append("RIGHT")
            else:
                cv2.rectangle(shapes, (int(w*0.66), int(h*0.5)), (w, h), (0, 0, 255), -1)
                
            # Draw "Safe Paths" (Green Zones)
            if "FORWARD" in safe_actions:
                 pts = np.array([[int(w*0.3), h], [int(w*0.7), h], [int(w*0.6), int(h*0.4)], [int(w*0.4), int(h*0.4)]], np.int32)
                 cv2.fillPoly(shapes, [pts], (0, 255, 0))
                 cv2.putText(overlay, "FWD OK", (int(w*0.45), int(h*0.8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
             
        # Blend overlay
        alpha = 0.4
        cv2.addWeighted(shapes, alpha, overlay, 1 - alpha, 0, overlay)
        
        # Text Info
        status_text = "SAFE: " + " ".join([a[0] for a in safe_actions if a != "BACKWARD"])
        cv2.putText(overlay, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(overlay, f"L:{int(c_left)} C:{int(c_fwd)} R:{int(c_right)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        return safe_actions, overlay, {
            'c_left': c_left, 'c_fwd': c_fwd, 'c_right': c_right, 'edges': total_edge_pixels
        }

