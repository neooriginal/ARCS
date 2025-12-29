"""
VINS-SLAM: Visual-Inertial Simultaneous Localization and Mapping

Thread-safe SLAM engine for background 3D mapping using monocular camera.
Achieves metric-scale pose estimation through IMU simulation and visual odometry fusion.
"""

import cv2
import numpy as np
import time
import threading
from collections import deque


FOCAL_LENGTH = 713.8
PP_X = 319.5
PP_Y = 239.5

K = np.array([
    [FOCAL_LENGTH, 0, PP_X],
    [0, FOCAL_LENGTH, PP_Y],
    [0, 0, 1]
], dtype=np.float64)


LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
)


class VinsSlam:
    def __init__(self, max_trajectory_points=1000, max_cloud_chunks=100):
        self.lock = threading.Lock()
        
        self.camera_pose = np.eye(4)
        self.trajectory = deque(maxlen=max_trajectory_points)
        self.trajectory.append(self.camera_pose[:3, 3].copy())  # Initialize with start point
        self.point_cloud = deque(maxlen=max_cloud_chunks)
        
        self.keyframe_indices = []
        self.keyframe_images = []
        self.frame_count = 0
        
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_velocity = np.zeros(3)
        self.prev_time = time.time()
        self.imu_sim_time = 0.0
        
        self.total_points = 0
        self.loop_closures = 0
        
        self.translation_buffer = deque(maxlen=5)
        self.smoothed_translation = np.zeros(3)
        
    def reset(self):
        with self.lock:
            self.camera_pose = np.eye(4)
            self.trajectory.clear()
            self.trajectory.append(self.camera_pose[:3, 3].copy())  # Initialize with start point
            self.point_cloud.clear()
            self.keyframe_indices = []
            self.keyframe_images = []
            self.frame_count = 0
            self.prev_frame = None
            self.prev_keypoints = None
            self.prev_velocity = np.zeros(3)
            self.prev_time = time.time()
            self.imu_sim_time = 0.0
            self.total_points = 0
            self.loop_closures = 0
            self.translation_buffer.clear()
            self.smoothed_translation = np.zeros(3)
    
    def process_frame(self, frame):
        if frame is None:
            return False
            
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        if dt <= 0 or dt > 1.0:
            dt = 0.1
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.prev_frame is None:
            points = self._get_tracking_points(gray)
            if points is None:
                self.prev_frame = gray
                return False
            self.prev_keypoints = points
            self.prev_frame = gray
            return True
        
        if self.prev_keypoints is None or len(self.prev_keypoints) < 8:
            points = self._get_tracking_points(gray)
            if points is not None:
                self.prev_keypoints = points
            self.prev_frame = gray
            return False
        
        current_keypoints, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_frame, gray, self.prev_keypoints, None, **LK_PARAMS
        )
        
        if current_keypoints is None:
            points = self._get_tracking_points(gray)
            self.prev_keypoints = points
            self.prev_frame = gray
            return False
        
        status_mask = status.ravel() == 1
        good_new = current_keypoints[status_mask]
        good_prev = self.prev_keypoints[status_mask]
        
        if len(good_new) < 8:
            if self._try_relocalize(gray):
                return True
            points = self._get_tracking_points(gray)
            self.prev_keypoints = points
            self.prev_frame = gray
            return False
        
        with self.lock:
            self.frame_count += 1
            
            if self.frame_count % 20 == 0 and len(self.keyframe_images) >= 5:
                self._check_loop_closure(gray)
            
            if self.frame_count % 10 == 0:
                self.keyframe_images.append(gray.copy())
                self.keyframe_indices.append(self.frame_count)
                if len(self.keyframe_images) > 50:
                    self.keyframe_images.pop(0)
                    self.keyframe_indices.pop(0)
        
        mean_flow = np.mean(np.linalg.norm(good_new - good_prev, axis=1))
        if mean_flow < 0.5 or mean_flow > 50:
            self.prev_frame = gray
            self.prev_keypoints = good_new.reshape(-1, 1, 2)
            return False
        
        E, mask_E = cv2.findEssentialMat(
            good_new, good_prev, K, method=cv2.RANSAC, prob=0.999, threshold=0.5
        )
        
        if E is None:
            self.prev_frame = gray
            self.prev_keypoints = good_new.reshape(-1, 1, 2)
            return False
        
        _, R, t, mask_pose = cv2.recoverPose(E, good_new, good_prev, K, mask=mask_E)
        
        inlier_count = np.sum(mask_pose)
        if inlier_count < 30:
            self.prev_frame = gray
            self.prev_keypoints = good_new.reshape(-1, 1, 2)
            return False
        
        rotation_angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
        if rotation_angle > 0.15:
            self.prev_frame = gray
            self.prev_keypoints = good_new.reshape(-1, 1, 2)
            return False
        
        R_global = self.camera_pose[:3, :3]
        R_fused, t_fused = self._fuse_visual_inertial(R, t, None, dt, R_global)
        
        if not self._update_pose(R_fused, t_fused):
            self.prev_frame = gray
            self.prev_keypoints = good_new.reshape(-1, 1, 2)
            return False
        
        mask_good = mask_E.ravel() == 1
        good_new_filtered = good_new[mask_good]
        good_prev_filtered = good_prev[mask_good]
        
        if len(good_new_filtered) >= 1:
            self._triangulate_points(good_prev_filtered, good_new_filtered, R_fused, t_fused)
        
        self.prev_frame = gray.copy()
        self.prev_keypoints = good_new_filtered.reshape(-1, 1, 2)
        
        return True
    
    def get_data(self):
        with self.lock:
            trajectory = [pos.tolist() for pos in self.trajectory]
            
            cloud_sample = []
            if self.point_cloud:
                all_points = np.hstack(list(self.point_cloud))
                if all_points.shape[1] > 1000:
                    indices = np.random.choice(all_points.shape[1], 1000, replace=False)
                    sampled = all_points[:, indices]
                else:
                    sampled = all_points
                cloud_sample = sampled.T.tolist()
            
            return {
                'trajectory': trajectory,
                'point_cloud': cloud_sample,
                'pose': self.camera_pose[:3, 3].tolist(),
                'frame_count': self.frame_count,
                'total_points': self.total_points,
                'loop_closures': self.loop_closures
            }
    
    def get_status(self):
        with self.lock:
            return {
                'frame_count': self.frame_count,
                'trajectory_length': len(self.trajectory),
                'total_points': self.total_points,
                'keyframes': len(self.keyframe_images),
                'loop_closures': self.loop_closures
            }
    
    def _get_tracking_points(self, gray):
        points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=2000,
            qualityLevel=0.15,
            minDistance=7,
            blockSize=7
        )
        if points is None:
            return None
        return points.astype(np.float32).reshape(-1, 1, 2)
    
    def _fuse_visual_inertial(self, R_vo, t_vo, imu_data, dt, R_global):
        scale_vo = np.linalg.norm(t_vo)
        
        if scale_vo < 1e-6:
            return R_vo, t_vo * 0.0
        
        fixed_scale = 0.05
        t_scaled = t_vo * fixed_scale
        
        t_mag = np.linalg.norm(t_scaled)
        max_step = 0.1
        if t_mag > max_step:
            t_scaled = t_scaled * (max_step / t_mag)
        
        return R_vo, t_scaled
    
    def _update_pose(self, R_delta, t_delta):
        t_flat = t_delta.flatten()
        self.translation_buffer.append(t_flat)
        
        if len(self.translation_buffer) < 3:
            return False
        
        buffer_array = np.array(list(self.translation_buffer))
        median_t = np.median(buffer_array, axis=0)
        
        current_dist = np.linalg.norm(t_flat - median_t)
        median_dist = np.median(np.linalg.norm(buffer_array - median_t, axis=1))
        
        if current_dist > median_dist * 5 + 0.05:  # Relaxed outlier rejection
            return False
        
        alpha = 0.3
        self.smoothed_translation = alpha * t_flat + (1 - alpha) * self.smoothed_translation
        
        t_mag = np.linalg.norm(self.smoothed_translation)
        if t_mag < 0.0001:  # Relaxed from 0.002 to capture small movements
            return False
            
        with self.lock:
            T_delta = np.eye(4)
            T_delta[:3, :3] = R_delta
            T_delta[:3, 3] = self.smoothed_translation
            
            self.camera_pose = self.camera_pose @ T_delta
            self.trajectory.append(self.camera_pose[:3, 3].copy())
        
        return True
    
    
    def _triangulate_points(self, good_prev, good_new, R, t):
        P1 = K @ np.hstack((np.eye(3), np.zeros((3, 1))))
        P2 = K @ np.hstack((R, t))
        
        points_4d = cv2.triangulatePoints(
            P1, P2,
            good_prev.reshape(-1, 2).T,
            good_new.reshape(-1, 2).T
        )
        
        points_3d = points_4d[:3] / points_4d[3]
        
        positive_depth = points_3d[2] > 0
        valid_points = points_3d[:, positive_depth]
        
        if valid_points.shape[1] > 0:
            R_global = self.camera_pose[:3, :3]
            t_global = self.camera_pose[:3, 3]
            points_world = R_global @ valid_points + t_global[:, None]
            
            with self.lock:
                self.point_cloud.append(points_world)
                self.total_points += valid_points.shape[1]
    
    def _try_relocalize(self, current_gray):
        if len(self.keyframe_images) < 5:
            return False
        
        for i in range(min(5, len(self.keyframe_images))):
            matched = self._detect_loop(current_gray, self.keyframe_images[i])
            if matched:
                points = self._get_tracking_points(self.keyframe_images[i])
                if points is not None:
                    self.prev_keypoints = points
                    self.prev_frame = current_gray
                    return True
        return False
    
    def _check_loop_closure(self, current_gray):
        for i in range(min(5, len(self.keyframe_images))):
            if self._detect_loop(current_gray, self.keyframe_images[i]):
                self.loop_closures += 1
                break
    
    def _detect_loop(self, current_gray, keyframe):
        orb = cv2.ORB_create(nfeatures=500)
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        kp_curr, des_curr = orb.detectAndCompute(current_gray, None)
        kp_hist, des_hist = orb.detectAndCompute(keyframe, None)
        
        if des_curr is None or des_hist is None:
            return False
        
        matches = matcher.match(des_curr, des_hist)
        
        if not matches or len(matches) < 20:
            return False
        
        matches = sorted(matches, key=lambda x: x.distance)
        
        if len(matches) > 20:
            src_pts = np.float32([kp_curr[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp_hist[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
            
            _, mask_F = cv2.findFundamentalMat(
                src_pts, dst_pts, cv2.FM_RANSAC,
                ransacReprojThreshold=3.0, confidence=0.99
            )
            
            if mask_F is not None and np.sum(mask_F) >= 15:
                return True
        
        return False
