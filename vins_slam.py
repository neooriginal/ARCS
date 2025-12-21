
import cv2
import numpy as np
import time

# --- 1. SLAM CONSTANTS ---
# Placeholder Camera Intrinsics Matrix (K)
# Assuming a standard 640x480 resolution.
FOCAL_LENGTH = 713.8 # Generic focal length in pixels
PP_X = 319.5 # Principal point X
PP_Y = 239.5 # Principal point Y

K = np.array([
    [FOCAL_LENGTH, 0, PP_X],
    [0, FOCAL_LENGTH, PP_Y],
    [0, 0, 1]
], dtype=np.float64)

LK_PARAMS = dict(winSize=(21, 21),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

class SLAMMap:
    def __init__(self):
        # 4x4 Transformation Matrix storing the camera's current global pose (Rotation and Translation)
        self.camera_pose = np.eye(4)
        # List to store 3D coordinats (X,Y,Z) of all map points
        self.point_cloud = []
        # List to store the camera's path (just the 3D translation vector [X, Y, Z])
        self.trajectory = []
        # Stores the frame indices (ID) of frames selected as keyframes
        self.keyframe_indices = []
        # Stores the actual grayscale images for the keyframes
        self.keyframe_images = []
        # Frame counter for tracking current index
        self.frame_count = 0

    def update_pose(self, R_delta, t_delta):
        # Create a 4x4 transformation matrix for the new movement
        T_delta = np.eye(4)
        T_delta[:3,:3] = R_delta
        T_delta[:3,3] = t_delta.flatten()

        # Integrate (multiply) the new movement into the current global pose
        self.camera_pose = self.camera_pose @ T_delta

        # Store the new global translation (trajectory)
        self.trajectory.append(self.camera_pose[:3,3].copy())

    def add_points(self, new_points_3d):
        # Convert the new points (3xN) into world coordinates (relative to the global pose)
        R = self.camera_pose[:3,:3]
        t = self.camera_pose[:3,3]
        points_w = R @ new_points_3d + t[:,None]
        self.point_cloud.append(points_w)

class IMUData:
    def __init__(self):
        # Linear acceleration in X,Y,Z (m/s^2) - includes gravity
        self.acceleration = np.zeros(3)
        # Angular velocity (rate of rotation) in X,Y,Z (rad/s)
        self.angular_velocity = np.zeros(3)

class VinsSlamSystem:
    def __init__(self):
        self.slam_map = SLAMMap()
        self.prev_keypoints = None
        self.prev_frame = None
        self.prev_velocity = np.zeros(3)
        self.prev_time = time.time()
        self.imu_sim_time = 0.0
        self.orb = cv2.ORB_create(nfeatures=2000)
        self.initialized = False

    def generate_imu_reading(self, dt):
        self.imu_sim_time += dt
        imu_data = IMUData()
        # Simulate constant gravity
        imu_data.acceleration[1] = -9.81
        # Simulate forward / backward movement
        imu_data.acceleration[2] += 0.25 * np.sin(self.imu_sim_time * 0.5)
        # Simulate Noise / Jitter
        imu_data.angular_velocity[0] = 0.001 * np.random.randn()
        imu_data.angular_velocity[1] = 0.001 * np.random.randn()
        return imu_data

    def fuse_visual_and_intertial(self, R_vo, t_vo, imu_data, dt, R_global):
        # Correct for gravity bias
        a_motion = imu_data.acceleration
        a_motion[1] += 9.81 

        a_world = R_global @ a_motion
        delta_v = a_world * dt
        
        # Integrate velocity (simplified model from reference)
        local_prev_vel = np.zeros(3)
        current_velocity = local_prev_vel + delta_v
        
        # Calculate distance traveled
        delta_t_imu = current_velocity * dt

        scale_imu = np.linalg.norm(delta_t_imu)
        scale_vo = np.linalg.norm(t_vo)

        if scale_vo < 1e-6 or scale_imu < 1e-6:
            t_fused = t_vo * 0.001
        else:
            scale_factor = scale_imu / scale_vo
            t_fused = t_vo * scale_factor

        return R_vo, t_fused, current_velocity

    def get_tracking_points(self, image_gray):
        points = cv2.goodFeaturesToTrack(
            image_gray,
            maxCorners=2000,
            qualityLevel=0.15,
            minDistance=7,
            blockSize=7
        )
        if points is None:
            return None
        return points.astype(np.float32).reshape(-1,1,2)

    def detect_loop(self, current_frame_gray, keyframe, keyframe_index):
        orb_check = cv2.ORB_create(nfeatures=500)
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        kp_curr, des_curr = orb_check.detectAndCompute(current_frame_gray, None)
        kp_hist, des_hist = orb_check.detectAndCompute(keyframe, None)

        if des_curr is None or des_hist is None:
            return None

        matches = matcher.match(des_curr, des_hist)
        if not isinstance(matches, list) or not matches:
            return None

        matches.sort(key=lambda x: x.distance)
        MIN_GOOD_MATCHES = 20

        if len(matches) > MIN_GOOD_MATCHES:
            src_pts = np.float32([kp_curr[m.queryIdx].pt for m in matches]).reshape(-1,1,2)
            dst_pts = np.float32([kp_hist[m.trainIdx].pt for m in matches]).reshape(-1,1,2)

            _, mask_F = cv2.findFundamentalMat(src_pts, dst_pts, cv2.FM_RANSAC, ransacReprojThreshold=3.0, confidence=0.99)
            if mask_F is None: return None
            
            inlier_count = np.sum(mask_F)
            if inlier_count >= 15:
                return keyframe_index
        return None

    def process_frame(self, frame):
        if frame is None:
            return

        current_time = time.time()
        # Initialize prev_time on first run
        if not self.initialized:
            self.prev_time = current_time
            self.initialized = True
            
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        # Avoid zero dt
        if dt == 0: dt = 0.001

        imu_reading = self.generate_imu_reading(dt)

        current_frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_frame is None:
            initial_points = self.get_tracking_points(current_frame_gray)
            if initial_points is not None:
                self.prev_keypoints = initial_points
                self.prev_frame = current_frame_gray
            return

        # Feature Tracking
        if self.prev_keypoints is None or self.prev_keypoints.size < 8:
            new_points = self.get_tracking_points(current_frame_gray)
            if new_points is not None:
                self.prev_keypoints = new_points
                self.prev_frame = current_frame_gray
            return

        current_keypoints, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_frame, current_frame_gray, self.prev_keypoints, None, **LK_PARAMS
        )

        if current_keypoints is None:
            new_points = self.get_tracking_points(current_frame_gray)
            if new_points is not None:
                self.prev_keypoints = new_points
                self.prev_frame = current_frame_gray
            return
            
        status_mask = (status.ravel() == 1)
        good_new = current_keypoints[status_mask]
        good_prev = self.prev_keypoints[status_mask]

        if len(good_new) < 8:
            # Try relocalization logic or just reset
            # For simplicity in this non-interactive version, we just reset features
            new_points = self.get_tracking_points(current_frame_gray)
            if new_points is not None:
                self.prev_keypoints = new_points
                self.prev_frame = current_frame_gray
            return

        # Updates
        self.slam_map.frame_count += 1
        current_frame_index = self.slam_map.frame_count
        loop_closed = False

        # Loop Closure Check
        if current_frame_index % 20 == 0 and len(self.slam_map.keyframe_images) >= 5:
            for i in range(min(5, len(self.slam_map.keyframe_images))):
                matched_index = self.detect_loop(
                    current_frame_gray,
                    self.slam_map.keyframe_images[i],
                    self.slam_map.keyframe_indices[i]
                )
                if matched_index is not None:
                    loop_closed = True
                    break

        # Keyframe creation
        if current_frame_index % 10 == 0:
            self.slam_map.keyframe_images.append(current_frame_gray.copy())
            self.slam_map.keyframe_indices.append(current_frame_index)

        # Essential Matrix & Pose
        E, mask_E = cv2.findEssentialMat(
            good_new, good_prev, K, method=cv2.RANSAC, prob=0.999, threshold=1.0
        )
        
        if E is None:
             self.prev_frame = current_frame_gray.copy()
             self.prev_keypoints = good_new.reshape(-1,1,2)
             return

        _, R, t, mask_pose = cv2.recoverPose(E, good_new, good_prev, K, mask=mask_E)

        R_global = self.slam_map.camera_pose[:3,:3]
        
        R_fused, t_fused, current_velocity = self.fuse_visual_and_intertial(R, t, imu_reading, dt, R_global)

        if loop_closed:
            t_fused = t_fused * 0.90

        self.slam_map.update_pose(R_fused, t_fused)
        self.prev_velocity = current_velocity

        # Triangulation
        mask_good = mask_E.ravel() == 1
        good_new_filtered = good_new[mask_good]
        good_prev_filtered = good_prev[mask_good]
        
        # Only add points if we have moved enough to get a baseline
        # Avoid degenerate triangulation when stationary
        # Reference code didn't check t_fused threshold, just point count.
        if len(good_new_filtered) >= 1:
            P1 = K @ np.hstack((np.eye(3), np.zeros((3,1))))
            P2 = K @ np.hstack((R_fused, t_fused))
            
            points_4d = cv2.triangulatePoints(P1, P2, good_prev_filtered.reshape(-1,2).T, good_new_filtered.reshape(-1,2).T)
            points_3d = points_4d[:3] / points_4d[3]
            
            # Filter points behind camera (Z <= 0) or too far
            positive_depth_mask = (points_3d[2] > 0) & (points_3d[2] < 50)
            valid_points = points_3d[:, positive_depth_mask]
            
            if valid_points.shape[1] > 0:
                self.slam_map.add_points(valid_points)

        # Prepare for next iteration
        self.prev_frame = current_frame_gray.copy()
        self.prev_keypoints = good_new_filtered.reshape(-1,1,2)

