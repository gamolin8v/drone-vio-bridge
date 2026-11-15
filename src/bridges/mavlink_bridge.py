"""
MAVLink bridge — sends VIO pose to ArduPilot via VISION_POSITION_ESTIMATE.

ArduPilot setup required (set via Mission Planner / QGC):
  AHRS_EKF_TYPE   = 3   (use EKF3)
  EK3_ENABLE      = 1
  EK2_ENABLE      = 0
  EK3_SRC1_POSXY  = 6   (ExternalNav)
  EK3_SRC1_VELXY  = 6   (ExternalNav)
  EK3_SRC1_POSZ   = 6   (ExternalNav) or 1 (barometer)
  EK3_SRC1_YAW    = 6   (ExternalNav) or 1 (compass)
  GPS_TYPE        = 0   (disable GPS for indoor-only)
  SERIAL5_BAUD    = 921  (or the serial port you use)
  SERIAL5_PROTOCOL= 1   (MAVLink 2)
"""

import logging
import math
import threading
import time
from typing import Optional

from ..vio.base import Pose

log = logging.getLogger(__name__)

try:
    from pymavlink import mavutil
    _MAVLINK_AVAILABLE = True
except ImportError:
    _MAVLINK_AVAILABLE = False
    log.warning("pymavlink not installed. Run: pip install pymavlink")


class MAVLinkBridge:
    """
    Sends VISION_POSITION_ESTIMATE to ArduPilot at a configurable rate.

    Also sends:
      - SET_GPS_GLOBAL_ORIGIN once on connect (required by ArduPilot to arm)
      - HEARTBEAT every second (keeps connection alive)
    """

    HEARTBEAT_INTERVAL = 1.0   # seconds
    ORIGIN_RESEND_INTERVAL = 5.0

    def __init__(self, port: str, baudrate: int = 921600,
                 send_rate_hz: float = 20.0):
        if not _MAVLINK_AVAILABLE:
            raise RuntimeError("pymavlink not installed")

        self._port = port
        self._baud = baudrate
        self._interval = 1.0 / send_rate_hz

        self._conn = None
        self._running = False
        self._last_pose: Optional[Pose] = None
        self._lock = threading.Lock()

        self._hb_thread: Optional[threading.Thread] = None
        self._send_thread: Optional[threading.Thread] = None
        self._last_hb = 0.0
        self._last_origin = 0.0
        self._connected = False

    # ── Public API ──────────────────────────────────────────────────────────

    def start(self):
        log.info("MAVLink bridge: connecting to %s @ %d baud", self._port, self._baud)
        self._conn = mavutil.mavlink_connection(
            self._port,
            baud=self._baud,
            source_system=1,
            source_component=mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,
        )
        self._running = True
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True,
                                              name="mavlink-send")
        self._send_thread.start()
        log.info("MAVLink bridge started")

    def stop(self):
        self._running = False
        if self._send_thread:
            self._send_thread.join(timeout=3.0)
        if self._conn:
            self._conn.close()
        log.info("MAVLink bridge stopped")

    def publish(self, pose: Pose):
        """Thread-safe pose update — called from VIO thread."""
        with self._lock:
            self._last_pose = pose

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ── Internal send loop ──────────────────────────────────────────────────

    def _send_loop(self):
        # Wait for initial heartbeat from AP (up to 10 s)
        log.info("MAVLink: waiting for ArduPilot heartbeat...")
        self._conn.wait_heartbeat(timeout=10)
        if self._conn.target_system:
            log.info("MAVLink: ArduPilot detected (sysid=%d)", self._conn.target_system)
            self._connected = True
        else:
            log.warning("MAVLink: no heartbeat received — continuing anyway")

        # Send origin so AP can initialise its local frame
        self._send_origin()

        while self._running:
            t0 = time.time()

            # Heartbeat
            if t0 - self._last_hb >= self.HEARTBEAT_INTERVAL:
                self._send_heartbeat()
                self._last_hb = t0

            # Re-send origin periodically
            if t0 - self._last_origin >= self.ORIGIN_RESEND_INTERVAL:
                self._send_origin()

            # Vision position estimate
            with self._lock:
                pose = self._last_pose

            if pose is not None:
                self._send_vision_position(pose)
                if pose.vx != 0 or pose.vy != 0 or pose.vz != 0:
                    self._send_vision_speed(pose)

            elapsed = time.time() - t0
            sleep = self._interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def _send_heartbeat(self):
        self._conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def _send_origin(self):
        """
        SET_GPS_GLOBAL_ORIGIN — ArduPilot requires this before it will
        accept VISION_POSITION_ESTIMATE and before pre-arm check passes.
        (0, 0, 0) is fine for indoor-only use; set real coords if you
        want GPS↔VIO hand-off to work correctly outdoors.
        """
        self._conn.mav.set_gps_global_origin_send(
            self._conn.target_system,
            int(0 * 1e7),   # lat  (1e-7 deg) — set real value for GPS↔VIO
            int(0 * 1e7),   # lon
            0,               # alt (mm)
        )
        self._last_origin = time.time()
        log.debug("Sent GPS_GLOBAL_ORIGIN")

    def _send_vision_position(self, pose: Pose):
        """
        VISION_POSITION_ESTIMATE (msg #102).
        Frame: NED (ArduPilot default for external vision).
          x = North (m), y = East (m), z = Down (m, positive down)
          roll/pitch/yaw in radians
        """
        t_us = int(pose.timestamp * 1e6)
        cov = pose.to_covariance()
        self._conn.mav.vision_position_estimate_send(
            t_us,
            float(pose.x),
            float(pose.y),
            float(pose.z),
            float(pose.roll),
            float(pose.pitch),
            float(pose.yaw),
            cov,
            0,  # reset_counter — increment only on estimator reset
        )

    def _send_vision_speed(self, pose: Pose):
        """
        VISION_SPEED_ESTIMATE (msg #103) — optional but improves velocity fusion.
        """
        t_us = int(pose.timestamp * 1e6)
        cov = [0.0] * 9
        cov[0] = cov[4] = cov[8] = (0.1 / max(0.01, pose.quality)) ** 2
        self._conn.mav.vision_speed_estimate_send(
            t_us,
            float(pose.vx),
            float(pose.vy),
            float(pose.vz),
            cov,
            0,
        )


class MAVLinkBridgeUDP(MAVLinkBridge):
    """UDP variant — useful for SITL testing (connects to 127.0.0.1:14550)."""

    def __init__(self, host: str = "127.0.0.1", port: int = 14550,
                 send_rate_hz: float = 20.0):
        super().__init__(f"udpout:{host}:{port}", baudrate=0,
                         send_rate_hz=send_rate_hz)
        log.info("MAVLink UDP bridge → %s:%d", host, port)


def build_mavlink_bridge(cfg) -> Optional[MAVLinkBridge]:
    ocfg = cfg.output.ardupilot
    if not ocfg.enabled:
        return None
    return MAVLinkBridge(
        port=ocfg.port,
        baudrate=ocfg.baudrate,
        send_rate_hz=ocfg.send_rate_hz,
    )
