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
        
        # Hysteresis / Flicker Safety
        # Store recent "Blocked" states.
        # If an action was blocked recently, we keep it blocked for a bit.
        from collections import deque
        import threading
        self.history_len = 12 # Approx 0.5-1.0s depending on FPS
        self.block_history = deque(maxlen=self.history_len) # Stores set of BLOCKED actions
        self.lock = threading.Lock()
        
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
        
        def get_top_average(chunk, top_n=2):
            """
            Get the average of the Top N closest points.
            Robustly detects thin obstacles (like cables with 2 edges)
            while filtering single-line noise.
            """
            if not chunk: return 0
            ys = sorted([p[1] for p in chunk], reverse=True) # Descending (Closest first)
            top_values = ys[:top_n]
            if not top_values: return 0
            return sum(top_values) / len(top_values)
            
        c_left = get_top_average(left_chunk)
        c_fwd = get_top_average(center_chunk)
        c_right = get_top_average(right_chunk)

        # 5. Safety Logic (The "Prohibit" Logic)
        
        # Determine INSTANT blocked actions
        instant_blocked = set()
        threshold = self.obstacle_threshold_y
        
        # --- BLINDNESS CHECK ---
        total_edge_pixels = np.count_nonzero(edges)
        min_edge_pixels = 200 
        is_blind = total_edge_pixels < min_edge_pixels
        
        if is_blind:
            instant_blocked.add("FORWARD")
            cv2.rectangle(shapes, (int(w*0.2), int(h*0.2)), (int(w*0.8), int(h*0.8)), (0, 0, 255), -1)
            cv2.putText(overlay, "BLOCKED (NO VISUALS)", (int(w*0.3), int(h*0.5)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            if c_fwd > threshold:
                instant_blocked.add("FORWARD")
                cv2.rectangle(shapes, (int(w*0.33), int(h*0.5)), (int(w*0.66), h), (0, 0, 255), -1)
            
            side_threshold = threshold + 50
            if c_left > side_threshold:
                instant_blocked.add("LEFT")
                cv2.rectangle(shapes, (0, int(h*0.5)), (int(w*0.33), h), (0, 0, 255), -1)
            
            if c_right > side_threshold:
                instant_blocked.add("RIGHT")
                cv2.rectangle(shapes, (int(w*0.66), int(h*0.5)), (w, h), (0, 0, 255), -1)

        # Update History (Thread Safe)
        with self.lock:
            self.block_history.append(instant_blocked)
            
            # Calculate PERSISTENT blocked actions
            persistent_blocked = set()
            for b_set in self.block_history:
                persistent_blocked.update(b_set)
            
        # Determine Safe Actions based on persistent_blocked
        safe_actions = ["BACKWARD"]
        
        if "FORWARD" not in persistent_blocked:
             safe_actions.append("FORWARD")
             if not is_blind:
                  pts = np.array([[int(w*0.3), h], [int(w*0.7), h], [int(w*0.6), int(h*0.4)], [int(w*0.4), int(h*0.4)]], np.int32)
                  cv2.fillPoly(shapes, [pts], (0, 255, 0))
                  cv2.putText(overlay, "FWD OK", (int(w*0.45), int(h*0.8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                 
        if "LEFT" not in persistent_blocked:
            safe_actions.append("LEFT")
            if not is_blind:
                 pts = np.array([[0, h], [0, h//2], [w//3, h//2], [0, h]], np.int32)
                 # cv2.fillPoly for left zone if desired
                 
        if "RIGHT" not in persistent_blocked:
            safe_actions.append("RIGHT")
            
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

