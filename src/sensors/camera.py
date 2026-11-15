"""
Camera abstraction for drone-vio-bridge.

Supported backends:
  rpi    — Raspberry Pi camera (picamera2 / libcamera), incl. Global Shutter (IMX296, OV9281)
  usb    — Any USB/V4L2 camera via OpenCV
  oak_d  — Luxonis OAK-D series (DepthAI)
  mock   — Synthetic test frames
"""

import logging
import threading
import time
from abc import ABC, abstractmethod
from typing import Optional, Callable

import cv2
import numpy as np

from .imu import RangefinderReading

log = logging.getLogger(__name__)


class CameraBase(ABC):
    """Abstract camera interface."""

    def __init__(self, width: int, height: int, fps: int):
        self.width = width
        self.height = height
        self.fps = fps
        self._running = False
        self._callback: Optional[Callable] = None
        self._thread: Optional[threading.Thread] = None

    def on_frame(self, callback: Callable):
        """Register callback: callback(image: np.ndarray, timestamp: float)"""
        self._callback = callback

    @abstractmethod
    def start(self):
        """Begin streaming frames."""

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)

    def _fire(self, image: np.ndarray, ts: Optional[float] = None):
        if self._callback:
            self._callback(image, ts or time.time())


# ── Raspberry Pi camera (picamera2) ────────────────────────────────────────────

class RpiCamera(CameraBase):
    """
    Raspberry Pi camera via picamera2 / libcamera.
    Works with:
      - RPi Camera Module 3 (IMX708)
      - RPi Global Shutter Camera (IMX296)   ← recommended for VIO
      - Arducam OV9281 Global Shutter
    """

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        super().__init__(width, height, fps)

    def start(self):
        try:
            from picamera2 import Picamera2
        except ImportError:
            raise ImportError("picamera2 not installed. Run: pip install picamera2")

        self._cam = Picamera2()
        cfg = self._cam.create_video_configuration(
            main={"size": (self.width, self.height), "format": "YUV420"},
            controls={"FrameRate": float(self.fps)},
        )
        self._cam.configure(cfg)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("RPi camera started (%dx%d @ %d Hz)", self.width, self.height, self.fps)

    def _loop(self):
        self._cam.start()
        try:
            while self._running:
                buf = self._cam.capture_array("main")
                ts = time.time()
                # YUV420 → grayscale (just the Y plane)
                gray = buf[:self.height, :self.width]
                self._fire(gray, ts)
        finally:
            self._cam.stop()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)


# ── USB camera (OpenCV V4L2) ────────────────────────────────────────────────────

class UsbCamera(CameraBase):
    def __init__(self, device: str = "/dev/video0",
                 width: int = 640, height: int = 480, fps: int = 30):
        super().__init__(width, height, fps)
        self._device = device

    def start(self):
        # Try numeric index first (/dev/video0 → 0)
        idx = self._device
        if isinstance(self._device, str) and self._device.startswith("/dev/video"):
            try:
                idx = int(self._device.replace("/dev/video", ""))
            except ValueError:
                pass

        self._cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # minimal latency

        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera device: {self._device}")

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("USB camera %s started (%dx%d @ %d Hz)", self._device,
                 self.width, self.height, self.fps)

    def _loop(self):
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                log.warning("Camera read failed, retrying...")
                time.sleep(0.01)
                continue
            ts = time.time()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            self._fire(gray, ts)
        self._cap.release()


# ── Luxonis OAK-D ───────────────────────────────────────────────────────────────

class OakDCamera(CameraBase):
    """
    Luxonis OAK-D Lite / OAK-D Pro.
    Uses left rectified stereo camera (global shutter, 400p).
    IMU data (BNO085) should be read via OakDIMU in sensors/imu.py.
    """

    def __init__(self, width: int = 640, height: int = 400, fps: int = 30):
        super().__init__(width, height, fps)
        self._pipeline = None

    def start(self):
        try:
            import depthai as dai
        except ImportError:
            raise ImportError("depthai not installed. Run: pip install depthai")

        import depthai as dai
        pipeline = dai.Pipeline()

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setCamera("left")
        mono_left.setFps(self.fps)

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("left")
        mono_left.out.link(xout.input)

        self._pipeline = pipeline
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("OAK-D camera started (left stereo @ %d Hz)", self.fps)

    def _loop(self):
        import depthai as dai
        with dai.Device(self._pipeline) as dev:
            q = dev.getOutputQueue("left", maxSize=4, blocking=False)
            while self._running:
                pkt = q.tryGet()
                if pkt is not None:
                    frame = pkt.getCvFrame()
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
                    # Resize if needed
                    if gray.shape[1] != self.width or gray.shape[0] != self.height:
                        gray = cv2.resize(gray, (self.width, self.height))
                    self._fire(gray, pkt.getTimestamp().total_seconds() + time.time() - time.monotonic())
                else:
                    time.sleep(0.001)


# ── Mock camera ─────────────────────────────────────────────────────────────────

class MockCamera(CameraBase):
    """Synthetic frames with moving random features (for unit tests)."""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        super().__init__(width, height, fps)
        rng = np.random.default_rng(42)
        self._pts = rng.integers(10, [width-10, height-10], size=(200, 2)).astype(np.float32)

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("Mock camera started (%dx%d @ %d Hz)", self.width, self.height, self.fps)

    def _loop(self):
        interval = 1.0 / self.fps
        t0 = time.time()
        while self._running:
            t = time.time() - t0
            img = np.zeros((self.height, self.width), dtype=np.uint8)
            # Translate features with time (simulating drone movement)
            drift_x = int(10 * np.sin(t * 0.3))
            drift_y = int(5 * np.sin(t * 0.5))
            for px, py in self._pts:
                cx = int(px + drift_x) % self.width
                cy = int(py + drift_y) % self.height
                cv2.circle(img, (cx, cy), 3, 255, -1)
            self._fire(img, time.time())
            time.sleep(interval)


# ── Factory ─────────────────────────────────────────────────────────────────────

def build_camera(cfg) -> CameraBase:
    t = cfg.camera.type.lower()
    w, h, fps = cfg.camera.width, cfg.camera.height, cfg.camera.fps

    if t == "rpi":
        return RpiCamera(w, h, fps)
    if t == "usb":
        return UsbCamera(cfg.camera.device, w, h, fps)
    if t in ("oak_d", "oak-d", "oakd"):
        return OakDCamera(w, h, fps)
    if t == "mock":
        return MockCamera(w, h, fps)

    raise ValueError(f"Unknown camera type: {t}")
