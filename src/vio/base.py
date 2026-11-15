"""Abstract base classes for VIO estimators."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional
import numpy as np
import time


@dataclass
class Pose:
    """6-DOF pose in NED (North-East-Down) world frame."""
    # Position in meters
    x: float = 0.0   # North
    y: float = 0.0   # East
    z: float = 0.0   # Down (negative = up)
    # Orientation in radians
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    # Velocity in m/s
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    # Quality [0.0 - 1.0]
    quality: float = 1.0
    # Timestamp (seconds)
    timestamp: float = field(default_factory=time.time)

    def to_covariance(self, pos_noise: float = 0.1, vel_noise: float = 0.05) -> list:
        """Return 6x6 upper-right triangular covariance (21 values) for MAVLink."""
        q = max(0.01, self.quality)
        p = (pos_noise / q) ** 2
        # diagonal: pos_x, pos_y, pos_z, roll, pitch, yaw
        cov = [0.0] * 21
        cov[0]  = p           # xx
        cov[6]  = p           # yy
        cov[11] = p           # zz
        cov[15] = 0.01        # roll-roll
        cov[18] = 0.01        # pitch-pitch
        cov[20] = 0.1         # yaw-yaw  (less accurate)
        return cov


@dataclass
class IMUSample:
    """Single IMU sample."""
    ax: float = 0.0   # m/s^2 (body frame, X forward)
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0   # rad/s
    gy: float = 0.0
    gz: float = 0.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class CameraFrame:
    """Single camera frame."""
    image: Optional[np.ndarray] = None   # grayscale uint8 numpy array
    timestamp: float = field(default_factory=time.time)
    altitude: Optional[float] = None     # meters from rangefinder (if available)


class VIOEstimator(ABC):
    """Abstract VIO estimator. Implement this to add a new backend."""

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize the estimator. Return True on success."""

    @abstractmethod
    def update(self, frame: CameraFrame, imu_samples: list) -> Optional[Pose]:
        """
        Process a new camera frame with accumulated IMU samples.
        Returns updated Pose or None if estimation not yet available.
        """

    @abstractmethod
    def reset(self):
        """Reset the estimator to initial state."""

    @property
    @abstractmethod
    def is_ready(self) -> bool:
        """True when estimator has a valid pose to publish."""
