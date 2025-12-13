import cv2
import time
import logging

logger = logging.getLogger(__name__)

class QRScanner:
    def __init__(self):
        self.detector = cv2.QRCodeDetector()
        self.seen_codes = set()
        self.last_seen_time = {}
        # Cooldown in seconds before reporting the same code again (if we want to allow re-reporting)
        # Setting strictly once per session for now.
        
    def scan(self, frame, pose=None):
        """
        Scan frame for QR codes.
        Args:
            frame: OpenCV image
            pose: dict with 'x', 'y' (optional)
        Returns:
            tuple: (
                visual_data (str|None),   # Text for display (Title)
                visual_points (list|None), # Bounding box points
                new_context_data (str|None) # Full text if new, else None
            )
        """
        if frame is None:
            return None, None, None
            
        try:
            # detectAndDecode returns: retval (str), points (array), straight_qrcode (array)
            data, points, _ = self.detector.detectAndDecode(frame)
            
            if data and points is not None:
                title = data.split(':', 1)[0].strip()
                
                new_context = None
                if data not in self.seen_codes:
                    self.seen_codes.add(data)
                    
                    loc_str = "Unknown"
                    if pose:
                        loc_str = f"x={pose.get('x', 0):.2f}, y={pose.get('y', 0):.2f}"
                        
                    logger.info(f"QR CODE DETECTED: '{data}' at {loc_str}")
                    new_context = data
                
                return title, points, new_context
                    
        except Exception as e:
            logger.warning(f"QR Scan error: {e}")
            
        return None, None, None
