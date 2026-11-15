"""
SimpleVO — Pure-Python Visual Odometry for drone-vio-bridge.

Pipeline:
  1. FAST feature detection + LK optical flow tracking
  2. 5-point RANSAC essential matrix → relative R, t (unit scale)
  3. Scale recovery from rangefinder altitude OR IMU velocity
  4. Mahony complementary filter for attitude from IMU
  5. 9-state EKF (pos + vel + attitude) fusing visual position + IMU

Works on Raspberry Pi 5 without ROS.
Requires: opencv-python, numpy, scipy
"""

import logging
import math
import time
from typing import Optional, List, Tuple

import cv2
import numpy as np

from .base import VIOEstimator, Pose, IMUSample, CameraFrame

log = logging.getLogger(__name__)

# ─── Mahony attitude filter ────────────────────────────────────────────────────

class MahonyFilter:
    """Lightweight Mahony complementary filter for attitude estimation."""

    def __init__(self, kp: float = 2.0, ki: float = 0.005):
        self.kp = kp
        self.ki = ki
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self._int_fb = np.zeros(3)

    def update(self, gx, gy, gz, ax, ay, az, dt: float):
        q = self.q
        a_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if a_norm < 1e-6:
            return
        ax /= a_norm; ay /= a_norm; az /= a_norm

        # Estimated direction of gravity in body frame
        vx = 2*(q[1]*q[3] - q[0]*q[2])
        vy = 2*(q[0]*q[1] + q[2]*q[3])
        vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]

        # Error: cross product between measured and estimated gravity
        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx

        # Integral feedback
        self._int_fb += self.ki * np.array([ex, ey, ez]) * dt
        gx += self.kp*ex + self._int_fb[0]
        gy += self.kp*ey + self._int_fb[1]
        gz += self.kp*ez + self._int_fb[2]

        # Quaternion derivative
        qDot = 0.5 * np.array([
            -q[1]*gx - q[2]*gy - q[3]*gz,
             q[0]*gx + q[2]*gz - q[3]*gy,
             q[0]*gy - q[1]*gz + q[3]*gx,
             q[0]*gz + q[1]*gy - q[2]*gx,
        ])
        q += qDot * dt
        self.q = q / np.linalg.norm(q)

    def euler(self) -> Tuple[float, float, float]:
        """Return (roll, pitch, yaw) in radians."""
        q = self.q
        roll  = math.atan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2+q[2]**2))
        pitch = math.asin(max(-1.0, min(1.0, 2*(q[0]*q[2]-q[3]*q[1]))))
        yaw   = math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
        return roll, pitch, yaw

    def rotation_matrix(self) -> np.ndarray:
        q = self.q
        w, x, y, z = q
        return np.array([
            [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
            [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
            [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
        ])


# ─── 9-state EKF ───────────────────────────────────────────────────────────────

class NavEKF:
    """
    Simple 9-state navigation EKF.
    State: [px, py, pz, vx, vy, vz, roll, pitch, yaw]
    """

    def __init__(self, pos_noise: float = 0.1, vel_noise: float = 0.05):
        self.x = np.zeros(9)
        self.P = np.eye(9) * 1.0
        # Process noise
        self._Q = np.diag([
            pos_noise**2, pos_noise**2, pos_noise**2,
            vel_noise**2, vel_noise**2, vel_noise**2,
            0.01, 0.01, 0.1,  # attitude noise
        ])
        # Measurement noise for visual position
        self._R_vis = np.eye(3) * pos_noise**2 * 4

    def predict(self, ax, ay, az, dt: float, R_body2world: np.ndarray):
        """Propagate state using IMU acceleration in body frame."""
        g = np.array([0.0, 0.0, 9.81])  # NED gravity (down = positive)
        a_body = np.array([ax, ay, az])
        a_world = R_body2world @ a_body - g

        px, py, pz, vx, vy, vz = self.x[:6]
        self.x[0] += vx*dt + 0.5*a_world[0]*dt*dt
        self.x[1] += vy*dt + 0.5*a_world[1]*dt*dt
        self.x[2] += vz*dt + 0.5*a_world[2]*dt*dt
        self.x[3] += a_world[0]*dt
        self.x[4] += a_world[1]*dt
        self.x[5] += a_world[2]*dt

        # Simple F matrix (linearised around current state)
        F = np.eye(9)
        F[0,3] = dt; F[1,4] = dt; F[2,5] = dt

        self.P = F @ self.P @ F.T + self._Q * dt

    def update_vision(self, px_vis: float, py_vis: float, pz_vis: float,
                      quality: float = 1.0):
        """Fuse visual position measurement."""
        H = np.zeros((3, 9))
        H[0,0] = 1.0; H[1,1] = 1.0; H[2,2] = 1.0

        R = self._R_vis / max(0.01, quality)
        y = np.array([px_vis, py_vis, pz_vis]) - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(9) - K @ H) @ self.P

    @property
    def position(self) -> np.ndarray:
        return self.x[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:6].copy()


# ─── Feature tracker ───────────────────────────────────────────────────────────

class FeatureTracker:
    LK_PARAMS = dict(
        winSize=(21, 21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    FAST_THRESHOLD = 20

    def __init__(self, max_features: int = 400, min_features: int = 50):
        self.max_features = max_features
        self.min_features = min_features
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts: Optional[np.ndarray] = None

    def _detect(self, gray: np.ndarray) -> np.ndarray:
        fast = cv2.FastFeatureDetector_create(threshold=self.FAST_THRESHOLD)
        kps = fast.detect(gray, None)
        if not kps:
            # Fallback to Shi-Tomasi
            pts = cv2.goodFeaturesToTrack(gray, self.max_features, 0.01, 10)
            return pts if pts is not None else np.empty((0, 1, 2), dtype=np.float32)
        kps = sorted(kps, key=lambda k: -k.response)[:self.max_features]
        pts = np.array([[k.pt] for k in kps], dtype=np.float32)
        return pts

    def process(self, frame: np.ndarray) -> Tuple[Optional[np.ndarray],
                                                   Optional[np.ndarray]]:
        """
        Track features from previous frame to current.
        Returns (prev_pts, curr_pts) matched pairs, or (None, None) on first frame.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame

        if self._prev_gray is None or self._prev_pts is None or len(self._prev_pts) == 0:
            self._prev_gray = gray
            self._prev_pts = self._detect(gray)
            return None, None

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_pts, None, **self.LK_PARAMS
        )

        if curr_pts is None or status is None:
            self._prev_gray = gray
            self._prev_pts = self._detect(gray)
            return None, None

        good_prev = self._prev_pts[status.ravel() == 1]
        good_curr = curr_pts[status.ravel() == 1]

        # Refresh if too few features
        self._prev_gray = gray
        if len(good_curr) < self.min_features:
            self._prev_pts = self._detect(gray)
            log.debug("Feature redetection: %d → %d", len(good_curr), len(self._prev_pts))
        else:
            self._prev_pts = good_curr.reshape(-1, 1, 2)

        return good_prev, good_curr


# ─── Main SimpleVO estimator ───────────────────────────────────────────────────

class SimpleVO(VIOEstimator):
    """
    Pure-Python Visual-Inertial Odometry.

    Use `scale_source` to tell how to recover metric scale:
      - "rangefinder": best for indoor hover, needs rangefinder altitude
      - "imu": estimate from IMU velocity magnitude
      - "fixed": use a fixed scale (testing only)
    """

    def __init__(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray,
                 scale_source: str = "rangefinder",
                 fixed_scale: float = 1.0,
                 min_features: int = 50,
                 max_features: int = 400,
                 ransac_threshold: float = 1.0,
                 pos_noise: float = 0.1,
                 vel_noise: float = 0.05):
        self.K = camera_matrix
        self.D = dist_coeffs
        self.scale_source = scale_source
        self.fixed_scale = fixed_scale

        self._tracker = FeatureTracker(max_features=max_features,
                                       min_features=min_features)
        self._mahony = MahonyFilter(kp=2.0, ki=0.005)
        self._ekf = NavEKF(pos_noise=pos_noise, vel_noise=vel_noise)

        self._ransac_thr = ransac_threshold
        self._last_imu_t: Optional[float] = None
        self._last_frame_t: Optional[float] = None
        self._ready = False
        self._frame_count = 0

        # Visual pose in camera frame (accumulated, unit scale)
        self._R_cum = np.eye(3)
        self._t_cum = np.zeros((3, 1))

        # Last known altitude from rangefinder
        self._last_altitude: Optional[float] = None
        self._prev_altitude: Optional[float] = None

    # ── VIOEstimator interface ──────────────────────────────────────────────

    def initialize(self) -> bool:
        log.info("SimpleVO initialized (scale_source=%s)", self.scale_source)
        return True

    def reset(self):
        self._tracker = FeatureTracker()
        self._mahony = MahonyFilter()
        self._ekf = NavEKF()
        self._R_cum = np.eye(3)
        self._t_cum = np.zeros((3, 1))
        self._ready = False
        self._frame_count = 0
        log.info("SimpleVO reset")

    @property
    def is_ready(self) -> bool:
        return self._ready

    def update(self, frame: CameraFrame, imu_samples: List[IMUSample]) -> Optional[Pose]:
        now = frame.timestamp

        # 1. Integrate IMU
        R_body2world = self._integrate_imu(imu_samples)

        # 2. Track visual features
        if frame.image is None:
            return self._build_pose(now) if self._ready else None

        prev_pts, curr_pts = self._tracker.process(frame.image)
        self._frame_count += 1

        if frame.altitude is not None:
            self._last_altitude = frame.altitude

        if prev_pts is None or len(prev_pts) < 8:
            if self._frame_count > 3:
                log.debug("Waiting for enough features (%d so far)", self._frame_count)
            return self._build_pose(now) if self._ready else None

        # 3. Estimate relative pose (Essential matrix)
        rel_R, rel_t, quality = self._recover_relative_pose(prev_pts, curr_pts)
        if rel_R is None:
            return self._build_pose(now) if self._ready else None

        # 4. Determine metric scale
        dt = now - (self._last_frame_t or now)
        self._last_frame_t = now
        scale = self._estimate_scale(dt)

        # 5. Accumulate pose in camera frame → convert to NED
        self._t_cum = self._R_cum @ rel_t * scale + self._t_cum
        self._R_cum = self._R_cum @ rel_R

        # Camera frame is usually X-right, Y-down, Z-forward
        # Convert to NED: N=Z_cam, E=X_cam, D=-Y_cam
        t = self._t_cum.flatten()
        pos_ned = np.array([t[2], t[0], -t[1]])

        # 6. EKF update with visual position
        if imu_samples:
            last_imu = imu_samples[-1]
            dt_imu = (now - self._last_imu_t) if self._last_imu_t else 0.01
            self._ekf.predict(last_imu.ax, last_imu.ay, last_imu.az,
                              dt_imu, R_body2world)

        self._ekf.update_vision(pos_ned[0], pos_ned[1], pos_ned[2], quality)
        self._ready = True

        return self._build_pose(now, quality)

    # ── Internal helpers ────────────────────────────────────────────────────

    def _integrate_imu(self, samples: List[IMUSample]) -> np.ndarray:
        for s in samples:
            dt = s.timestamp - (self._last_imu_t or s.timestamp)
            self._last_imu_t = s.timestamp
            if dt <= 0 or dt > 0.5:
                continue
            self._mahony.update(s.gx, s.gy, s.gz, s.ax, s.ay, s.az, dt)
        return self._mahony.rotation_matrix()

    def _recover_relative_pose(self, prev_pts: np.ndarray, curr_pts: np.ndarray
                                ) -> Tuple[Optional[np.ndarray],
                                           Optional[np.ndarray], float]:
        if len(prev_pts) < 8:
            return None, None, 0.0

        # Undistort points
        prev_ud = cv2.undistortPoints(prev_pts.reshape(-1, 1, 2), self.K, self.D,
                                      P=self.K).reshape(-1, 2)
        curr_ud = cv2.undistortPoints(curr_pts.reshape(-1, 1, 2), self.K, self.D,
                                      P=self.K).reshape(-1, 2)

        E, mask = cv2.findEssentialMat(
            curr_ud, prev_ud, self.K,
            method=cv2.RANSAC, prob=0.999,
            threshold=self._ransac_thr / self.K[0, 0],  # normalize threshold
        )
        if E is None:
            return None, None, 0.0

        inlier_ratio = mask.sum() / len(mask) if mask is not None else 0.0
        if inlier_ratio < 0.3:
            return None, None, 0.0

        _, R, t, pose_mask = cv2.recoverPose(
            E, curr_ud, prev_ud, self.K, mask=mask
        )
        quality = float(pose_mask.sum()) / len(pose_mask)
        return R, t, quality

    def _estimate_scale(self, dt: float) -> float:
        if self.scale_source == "fixed":
            return self.fixed_scale

        if self.scale_source == "rangefinder":
            if (self._last_altitude is not None and
                    self._prev_altitude is not None and
                    abs(self._last_altitude) > 0.05):
                # Use altitude change to scale vertical component
                # For hovering use current altitude as the scale reference
                scale = abs(self._last_altitude)  # height above ground
                self._prev_altitude = self._last_altitude
                return max(0.01, min(scale, 20.0))
            self._prev_altitude = self._last_altitude
            return self.fixed_scale  # fallback until first rangefinder reading

        if self.scale_source == "imu":
            # Very rough: assume |v_imu| ~ |v_visual| / dt
            v = self._ekf.velocity
            speed = np.linalg.norm(v)
            if speed > 0.05:
                return speed * dt
            return self.fixed_scale

        return self.fixed_scale

    def _build_pose(self, timestamp: float, quality: float = 1.0) -> Optional[Pose]:
        if not self._ready:
            return None
        pos = self._ekf.position
        vel = self._ekf.velocity
        roll, pitch, yaw = self._mahony.euler()
        return Pose(
            x=float(pos[0]), y=float(pos[1]), z=float(pos[2]),
            vx=float(vel[0]), vy=float(vel[1]), vz=float(vel[2]),
            roll=roll, pitch=pitch, yaw=yaw,
            quality=quality,
            timestamp=timestamp,
        )


# ─── Mock estimator (for testing without hardware) ─────────────────────────────

class MockVO(VIOEstimator):
    """Generates a smooth figure-8 trajectory for testing."""

    def __init__(self, radius: float = 2.0, period: float = 20.0):
        self._r = radius
        self._T = period
        self._start = time.time()
        self._ready_flag = False

    def initialize(self) -> bool:
        self._ready_flag = True
        return True

    def reset(self):
        self._start = time.time()

    @property
    def is_ready(self) -> bool:
        return self._ready_flag

    def update(self, frame: CameraFrame, imu_samples: list) -> Optional[Pose]:
        t = (time.time() - self._start) / self._T * 2 * math.pi
        return Pose(
            x=self._r * math.sin(t),
            y=self._r * math.sin(2*t) / 2,
            z=-1.0,  # 1 m altitude
            vx=self._r * 2*math.pi/self._T * math.cos(t),
            vy=self._r * 2*math.pi/self._T * math.cos(2*t),
            roll=0.0, pitch=0.0, yaw=t,
            quality=1.0,
            timestamp=time.time(),
        )


def build_vio(cfg) -> VIOEstimator:
    """Factory: create a VIO estimator from AppConfig."""
    vcfg = cfg.vio
    if vcfg.algorithm == "mock":
        log.info("Using MockVO (no camera/IMU required)")
        return MockVO()

    # Load camera calibration
    cam_matrix, dist_coeffs = _load_calibration(cfg.camera.calibration_file,
                                                  cfg.camera.width,
                                                  cfg.camera.height)

    if vcfg.algorithm in ("simple_vo", "simple"):
        log.info("Using SimpleVO")
        return SimpleVO(
            camera_matrix=cam_matrix,
            dist_coeffs=dist_coeffs,
            scale_source=vcfg.scale_source,
            fixed_scale=vcfg.fixed_scale,
            min_features=vcfg.min_features,
            max_features=vcfg.max_features,
            ransac_threshold=vcfg.ransac_threshold,
            pos_noise=vcfg.position_noise,
            vel_noise=vcfg.velocity_noise,
        )

    raise ValueError(f"Unknown VIO algorithm: {vcfg.algorithm}")


def _load_calibration(path: str, width: int, height: int):
    """Load or generate default camera calibration."""
    import os, yaml as _yaml
    if os.path.exists(path):
        with open(path) as f:
            d = _yaml.safe_load(f)
        K = np.array(d["camera_matrix"]["data"]).reshape(3, 3)
        D = np.array(d["dist_coeffs"]["data"])
        return K, D

    log.warning("Calibration file %s not found — using estimated defaults", path)
    # Estimate focal length from image size (typical 70° FOV)
    fx = fy = width / (2 * math.tan(math.radians(35)))
    K = np.array([[fx, 0, width/2],
                  [0, fy, height/2],
                  [0,  0,       1]], dtype=np.float64)
    D = np.zeros(5, dtype=np.float64)
    return K, D
