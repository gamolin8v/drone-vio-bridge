"""
MSP bridge — injects VIO position into iNav as synthetic GPS.

iNav setup (Configurator CLI):
  feature GPS
  set gps_provider = MSP         ← use GPS over MSP
  set gps_sbas_mode = NONE
  set nav_use_gps = ON
  set opflow_hardware = NONE     ← if not using optical flow separately
  serial <port> MSP 115200 115200 0 115200

Then enable GPS in Configuration tab.

How it works:
  VIO gives local NED position (meters from origin).
  We convert that to lat/lon using flat-earth approximation around
  a user-defined origin (origin_lat, origin_lon).
  iNav receives this as a ~3D-fix GPS with high satellite count.

Limitations:
  - No global reference → position resets on restart
  - iNav GPS accuracy depends on how good VIO is
  - Yaw from VIO (not compass) if nav_heading_source = GPS

MSP protocol v1 framing:
  $ M <  length  command  [payload]  checksum
  checksum = XOR of (length XOR command XOR each payload byte)
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


# MSP command IDs
_MSP_STATUS        = 101
_MSP_SET_RAW_GPS   = 201
_MSP_SET_HEAD      = 211
_MSP_SET_RAW_RC    = 200
_MSP2_INAV_SET_MIXER = 0x2000  # (unused here, just for reference)


def _msp_v1_encode(cmd: int, payload: bytes = b"") -> bytes:
    """Encode an MSP v1 request frame."""
    length = len(payload)
    checksum = length ^ cmd
    for b in payload:
        checksum ^= b
    return b"$M<" + bytes([length, cmd]) + payload + bytes([checksum])


def _crc8_dvb_s2(data: bytes) -> int:
    """CRC for MSP v2."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _msp_v2_encode(func: int, payload: bytes = b"") -> bytes:
    """Encode an MSP v2 frame."""
    header = bytes([0, func & 0xFF, (func >> 8) & 0xFF,
                    len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    crc_data = header + payload
    crc = _crc8_dvb_s2(crc_data)
    return b"$X<" + header + payload + bytes([crc])


class MSPGPSBridge:
    """
    Sends MSP_SET_RAW_GPS to iNav at a configurable rate.
    Converts NED local coordinates to lat/lon using flat-earth.
    """

    # Earth radius in meters (WGS84 mean)
    _R_EARTH = 6_371_000.0

    def __init__(self, port: str, baudrate: int = 115200,
                 send_rate_hz: float = 10.0,
                 origin_lat: float = 0.0, origin_lon: float = 0.0,
                 origin_alt: int = 0):
        if not _SERIAL_AVAILABLE:
            raise RuntimeError("pyserial not installed. Run: pip install pyserial")

        self._port = port
        self._baud = baudrate
        self._interval = 1.0 / send_rate_hz
        self._origin_lat = math.radians(origin_lat)
        self._origin_lon = math.radians(origin_lon)
        self._origin_alt = origin_alt
        self._origin_lat_deg = origin_lat
        self._origin_lon_deg = origin_lon

        self._ser: Optional[serial.Serial] = None
        self._last_pose: Optional[Pose] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ── Public API ──────────────────────────────────────────────────────────

    def start(self):
        self._ser = serial.Serial(self._port, self._baud, timeout=0.1)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                         name="msp-gps")
        self._thread.start()
        log.info("MSP GPS bridge started → %s @ %d baud", self._port, self._baud)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._ser:
            self._ser.close()

    def publish(self, pose: Pose):
        with self._lock:
            self._last_pose = pose

    # ── Coordinate conversion ───────────────────────────────────────────────

    def _ned_to_latlon(self, north_m: float, east_m: float, down_m: float):
        """
        Convert NED offset (meters) from origin to lat/lon/alt.
        Flat-earth approximation, valid within ~20 km of origin.
        """
        lat_rad = self._origin_lat + north_m / self._R_EARTH
        lon_rad = self._origin_lon + east_m / (self._R_EARTH * math.cos(self._origin_lat))
        alt_m   = self._origin_alt - down_m   # down is negative altitude

        lat_deg = math.degrees(lat_rad)
        lon_deg = math.degrees(lon_rad)
        return lat_deg, lon_deg, alt_m

    # ── MSP packet builders ─────────────────────────────────────────────────

    def _build_raw_gps(self, pose: Pose) -> bytes:
        """
        MSP_SET_RAW_GPS payload (14 bytes):
          u8  fix_type   (2 = 3D fix)
          u8  num_sats   (number of "satellites")
          i32 lat        (1e-7 degrees)
          i32 lon        (1e-7 degrees)
          u16 alt        (meters above sea level)
          u16 speed      (cm/s, horizontal ground speed)
          u16 course     (0.1 degrees, 0=N, clockwise)
        """
        lat, lon, alt = self._ned_to_latlon(pose.x, pose.y, pose.z)

        # Map VIO quality [0..1] → number of satellites [4..20]
        num_sats = max(4, min(20, int(pose.quality * 20)))

        # Horizontal speed in cm/s
        speed_cms = int(math.sqrt(pose.vx**2 + pose.vy**2) * 100)

        # Ground course (0.1 deg from North, clockwise)
        course = int(math.degrees(math.atan2(pose.vy, pose.vx)) % 360 * 10)

        payload = struct.pack(
            "<BBiIHHH",
            2,                          # fix_type = 3D
            num_sats,
            int(lat * 1e7),
            int(lon * 1e7) & 0xFFFFFFFF,  # uint32 for lon
            max(0, int(alt)),
            min(65535, speed_cms),
            min(65535, course),
        )
        # Signed lat, unsigned lon — repack correctly
        payload = struct.pack("<BBii HHH",
            2, num_sats,
            int(lat * 1e7),
            int(lon * 1e7),
            max(0, int(alt)),
            min(65535, speed_cms),
            min(65535, course),
        )
        return _msp_v1_encode(_MSP_SET_RAW_GPS, payload)

    def _build_set_head(self, yaw_rad: float) -> bytes:
        """MSP_SET_HEAD: send compass heading derived from VIO yaw."""
        heading_deg = int(math.degrees(yaw_rad)) % 360
        payload = struct.pack("<h", heading_deg)
        return _msp_v1_encode(_MSP_SET_HEAD, payload)

    # ── Send loop ───────────────────────────────────────────────────────────

    def _loop(self):
        while self._running:
            t0 = time.time()
            with self._lock:
                pose = self._last_pose

            if pose is not None and self._ser and self._ser.is_open:
                try:
                    self._ser.write(self._build_raw_gps(pose))
                    # Optionally send heading
                    self._ser.write(self._build_set_head(pose.yaw))
                except serial.SerialException as e:
                    log.error("MSP serial error: %s", e)
                    time.sleep(1.0)

            elapsed = time.time() - t0
            sleep = self._interval - elapsed
            if sleep > 0:
                time.sleep(sleep)


def build_msp_bridge(cfg) -> Optional[MSPGPSBridge]:
    ocfg = cfg.output.inav
    if not ocfg.enabled:
        return None
    return MSPGPSBridge(
        port=ocfg.port,
        baudrate=ocfg.baudrate,
        send_rate_hz=ocfg.send_rate_hz,
        origin_lat=ocfg.origin_lat,
        origin_lon=ocfg.origin_lon,
        origin_alt=ocfg.origin_alt,
    )
