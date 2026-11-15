"""
Betaflight bridge — position hold via MSP RC Override.

Betaflight has no position EKF; it only does rate/attitude control.
This bridge runs an external PID position controller on the companion
computer and sends throttle/pitch/roll corrections as fake RC input.

Betaflight setup (Configurator CLI):
  feature MSP
  set msp_override_channels_mask = 7   # channels 1,2,3 (roll, pitch, throttle)
  map TAER1234                         # your TX channel mapping

Then activate "MSP Override" mode from a switch in Modes tab.

WARNING: Test with props OFF first. Set hover_throttle carefully.
         The drone will NOT hold position if msp_override_channels_mask
         is wrong or MSP Override mode is not active.

MSP_SET_RAW_RC payload (32 channels × uint16, little-endian):
  Channels in TAER order (Throttle, Aileron, Elevator, Rudder, AUX1...)
  or AETR depending on your mapping. Values: 1000–2000 µs.
"""

import logging
import math
import struct
import threading
import time
from typing import Optional

from ..vio.base import Pose

log = logging.getLogger(__name__)

try:
    import serial
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False


_MSP_SET_RAW_RC = 200


def _msp_v1_encode(cmd: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    checksum = length ^ cmd
    for b in payload:
        checksum ^= b
    return b"$M<" + bytes([length, cmd]) + payload + bytes([checksum])


class PositionPID:
    """Simple 2D position PID controller → roll/pitch corrections."""

    def __init__(self, kp: float = 0.5, ki: float = 0.02, kd: float = 0.1,
                 max_tilt_deg: float = 20.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_tilt = math.radians(max_tilt_deg)

        self._int = [0.0, 0.0]
        self._last_err = [0.0, 0.0]
        self._setpoint = [0.0, 0.0]  # target x, y in NED

    def set_target(self, x: float, y: float):
        self._setpoint = [x, y]
        self._int = [0.0, 0.0]
        self._last_err = [0.0, 0.0]

    def update(self, x: float, y: float, yaw: float, dt: float):
        """
        Returns (roll_correction_rad, pitch_correction_rad).
        Corrections are in world frame; we rotate to drone body frame.
        """
        err_x = self._setpoint[0] - x
        err_y = self._setpoint[1] - y

        self._int[0] += err_x * dt
        self._int[1] += err_y * dt

        d_x = (err_x - self._last_err[0]) / max(dt, 0.001)
        d_y = (err_y - self._last_err[1]) / max(dt, 0.001)

        self._last_err = [err_x, err_y]

        # World-frame corrections
        ctrl_n = self.kp * err_x + self.ki * self._int[0] + self.kd * d_x
        ctrl_e = self.kp * err_y + self.ki * self._int[1] + self.kd * d_y

        # Rotate to body frame using yaw
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        pitch_body = -(ctrl_n * cos_y + ctrl_e * sin_y)
        roll_body  =   ctrl_e * cos_y - ctrl_n * sin_y

        # Clamp to max tilt
        pitch_body = max(-self.max_tilt, min(self.max_tilt, pitch_body))
        roll_body  = max(-self.max_tilt, min(self.max_tilt, roll_body))
        return roll_body, pitch_body


def _angle_to_rc(angle_rad: float, max_rad: float = math.radians(20)) -> int:
    """Map [-max, +max] rad → [1000, 2000] µs RC value."""
    frac = max(-1.0, min(1.0, angle_rad / max_rad))
    return int(1500 + frac * 500)


class BetaflightBridge:
    """
    External position hold for Betaflight via MSP RC Override.

    Usage:
      bridge = BetaflightBridge(...)
      bridge.start()
      bridge.set_target(x=0.0, y=0.0)   # hold here
      bridge.publish(pose)               # call each VIO update
    """

    def __init__(self, port: str, baudrate: int = 115200,
                 send_rate_hz: float = 50.0,
                 hover_throttle: int = 1500,
                 pos_p: float = 0.5, pos_i: float = 0.02, vel_p: float = 0.3):
        if not _SERIAL_AVAILABLE:
            raise RuntimeError("pyserial not installed. Run: pip install pyserial")

        self._port = port
        self._baud = baudrate
        self._interval = 1.0 / send_rate_hz
        self._hover_thr = hover_throttle

        self._pid = PositionPID(kp=pos_p, ki=pos_i)
        self._vel_p = vel_p

        self._ser: Optional[serial.Serial] = None
        self._last_pose: Optional[Pose] = None
        self._target_set = False
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_t = time.time()

        # 16 RC channels (default: all centred / hover throttle)
        self._rc = [1500] * 16
        self._rc[2] = self._hover_thr  # channel 3 = throttle (AETR: index 2)

    # ── Public API ──────────────────────────────────────────────────────────

    def start(self):
        self._ser = serial.Serial(self._port, self._baud, timeout=0.1)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                         name="bf-override")
        self._thread.start()
        log.info("Betaflight bridge started → %s @ %d baud", self._port, self._baud)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._ser:
            self._ser.close()

    def publish(self, pose: Pose):
        with self._lock:
            if not self._target_set:
                self._pid.set_target(pose.x, pose.y)
                self._target_set = True
            self._last_pose = pose

    def set_target(self, x: float, y: float):
        """Update position hold target (NED meters)."""
        with self._lock:
            self._pid.set_target(x, y)
            self._target_set = True

    # ── Send loop ───────────────────────────────────────────────────────────

    def _loop(self):
        while self._running:
            t0 = time.time()
            dt = t0 - self._last_t
            self._last_t = t0

            with self._lock:
                pose = self._last_pose

            if pose is not None and self._target_set:
                roll_rad, pitch_rad = self._pid.update(
                    pose.x, pose.y, pose.yaw, dt
                )
                # Altitude hold: simple P on z (NED: negative = up)
                # Positive z_error means we are too high → reduce throttle
                z_error = 0.0  # TODO: target altitude configurable
                thr_adj = int(-z_error * 100 * self._vel_p)

                # Map to RC values (AETR order: Roll=0, Pitch=1, Throttle=2, Yaw=3)
                self._rc[0] = _angle_to_rc(roll_rad)
                self._rc[1] = _angle_to_rc(-pitch_rad)   # pitch: positive = nose down
                self._rc[2] = max(1000, min(2000, self._hover_thr + thr_adj))
                # Leave yaw (self._rc[3]) at 1500 (no yaw correction)

            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(self._build_rc_packet())
                except serial.SerialException as e:
                    log.error("Betaflight serial error: %s", e)
                    time.sleep(1.0)

            elapsed = time.time() - t0
            sleep = self._interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def _build_rc_packet(self) -> bytes:
        """MSP_SET_RAW_RC: 16 channels as uint16 LE."""
        payload = struct.pack("<" + "H" * 16, *self._rc)
        return _msp_v1_encode(_MSP_SET_RAW_RC, payload)


def build_betaflight_bridge(cfg) -> Optional[BetaflightBridge]:
    ocfg = cfg.output.betaflight
    if not ocfg.enabled:
        return None
    return BetaflightBridge(
        port=ocfg.port,
        baudrate=ocfg.baudrate,
        send_rate_hz=ocfg.send_rate_hz,
        hover_throttle=ocfg.hover_throttle,
        pos_p=ocfg.pos_p,
        pos_i=ocfg.pos_i,
        vel_p=ocfg.vel_p,
    )
