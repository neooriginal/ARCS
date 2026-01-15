
import numpy as np
import logging
from typing import List, Tuple, Optional

logger = logging.getLogger(__name__)

class LidarScanner:
    """
    Analyzes LIDAR depth profiles from rotational scans.
    """
    
    def __init__(self):
        # Buffer stores (angle_deg, distance_cm)
        self.buffer: List[Tuple[float, float]] = []
        
    def clear(self):
        self.buffer = []
        
    def add_reading(self, angle: float, distance: Optional[float]):
        if distance is not None:
            self.buffer.append((angle, distance))
            
    def analyze_gap(self) -> dict:
        """
        Analyze the buffer to find the best gap to pass through.
        Expected pattern: Close (Wall) -> Far (Gap) -> Close (Wall)
        
        Returns:
            dict containing:
            - 'found': bool
            - 'center_angle': float (angle to aim for)
            - 'width_deg': float (angular width of gap)
            - 'left_edge_angle': float (angle)
            - 'right_edge_angle': float (angle)
            - 'wall_dists': Tuple[float, float]
            - 'raw_profile': Tuple[np.ndarray, np.ndarray]
        """
        if len(self.buffer) < 10:
            return {'found': False, 'reason': 'insufficient_data'}
            
        data = sorted(self.buffer, key=lambda x: x[0])
        angles = np.array([x[0] for x in data])
        dists = np.array([x[1] for x in data])
        
        # 3-point moving average
        dists_smooth = np.convolve(dists, np.ones(3)/3, mode='same')
        
        # Dynamic thresholding for gap detection
        p20 = np.percentile(dists_smooth, 20)
        p80 = np.percentile(dists_smooth, 80)
        
        if (p80 - p20) < 50:
             return {'found': False, 'reason': 'no_depth_contrast'}
             
        threshold = (p20 + p80) / 2
        is_gap = dists_smooth > threshold
        
        # Find gap segments
        diffs = np.diff(is_gap.astype(int))
        starts = np.where(diffs == 1)[0] + 1
        ends = np.where(diffs == -1)[0] + 1
        
        if is_gap[0]:
            starts = np.insert(starts, 0, 0)
        if is_gap[-1]:
            ends = np.append(ends, len(is_gap))
            
        if len(starts) == 0:
            return {'found': False, 'reason': 'no_gap_segments'}
            
        # Find widest segment
        best_segment = None
        max_len = 0
        
        for s, e in zip(starts, ends):
            width_deg = angles[e-1] - angles[s]
            if width_deg > max_len:
                max_len = width_deg
                best_segment = (s, e)
                
        if not best_segment:
             return {'found': False, 'reason': 'unknown_error'}
             
        s, e = best_segment
        
        left_angle = angles[s]
        right_angle = angles[e-1] # Use last index of segment
        
        center_angle = (left_angle + right_angle) / 2
        width_deg = right_angle - left_angle
        
        # Validation (min 10 degrees)
        if width_deg < 10:
             return {'found': False, 'reason': 'gap_too_narrow', 'width_deg': width_deg}
             
        wall_left_dist = dists_smooth[max(0, s-1)]
        wall_right_dist = dists_smooth[min(len(dists)-1, e)]
        
        return {
            'found': True,
            'center_angle': center_angle,
            'width_deg': width_deg,
            'left_edge_angle': left_angle,
            'right_edge_angle': right_angle,
            'wall_dists': (wall_left_dist, wall_right_dist),
            'raw_profile': (angles, dists_smooth)
        }
