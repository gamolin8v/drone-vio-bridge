"""
IMU abstraction for drone-vio-bridge.

Supported backends:
  icm42688  — ICM-42688-P via SPI (best performance, recommended)
  mpu6050   — MPU-6050/9250 via I2C (most common, budget)
  bno085    — BNO085 via I2C (high-level fusion, easy)
  oak_d     — OAK-D built-in BNO085 via DepthAI
  mock      — Synthetic IMU data
"""

import logging
import math
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, Deque, Optional
from collections import deque

log = logging.getLogger(__name__)


@dataclass
class RangefinderReading:
    distance_m: float
    timestamp: float


class IMUBase(ABC):
    def __init__(self, rate_hz: int = 200):
        self.rate_hz = rate_hz
        self._running = False
        self._callback: Optional[Callable] = None
        self._thread: Optional[threading.Thread] = None

    def on_sample(self, callback: Callable):
        """Register callback: callback(ax, ay, az, gx, gy, gz, timestamp)"""
        self._callback = callback

    @abstractmethod
    def start(self):
        pass

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _fire(self, ax, ay, az, gx, gy, gz, ts=None):
        if self._callback:
            self._callback(ax, ay, az, gx, gy, gz, ts or time.time())


# ── ICM-42688-P via SPI ─────────────────────────────────────────────────────────

class ICM42688(IMUBase):
    """
    ICM-42688-P — best choice for VIO on RPi5.
    Low noise, 32kHz gyro, connects via SPI0 (CE0).
    Wiring: MOSI=GPIO10, MISO=GPIO9, SCLK=GPIO11, CS=GPIO8
    """

    # Register addresses
    _REG_DEVICE_CONFIG  = 0x11
    _REG_PWR_MGMT0      = 0x4E
    _REG_GYRO_CONFIG0   = 0x4F
    _REG_ACCEL_CONFIG0  = 0x50
    _REG_GYRO_CONFIG1   = 0x51
    _REG_ACCEL_DATA_X1  = 0x1F
    _REG_WHO_AM_I       = 0x75
    _WHO_AM_I_VAL       = 0x47

    def __init__(self, device: str = "/dev/spidev0.0", rate_hz: int = 200):
        super().__init__(rate_hz)
        self._device = device

    def start(self):
        try:
            import spidev
        except ImportError:
            raise ImportError("spidev not installed. Run: pip install spidev")

        import spidev
        self._spi = spidev.SpiDev()
        bus, dev = self._parse_device(self._device)
        self._spi.open(bus, dev)
        self._spi.max_speed_hz = 1_000_000
        self._spi.mode = 0

        self._init_device()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("ICM-42688-P started on %s @ %d Hz", self._device, self.rate_hz)

    def _parse_device(self, dev: str):
        # "/dev/spidev0.0" → (0, 0)
        parts = dev.replace("/dev/spidev", "").split(".")
        return int(parts[0]), int(parts[1])

    def _write_reg(self, reg: int, val: int):
        self._spi.xfer2([reg & 0x7F, val])

    def _read_reg(self, reg: int, length: int = 1) -> list:
        return self._spi.xfer2([reg | 0x80] + [0] * length)[1:]

    def _init_device(self):
        who = self._read_reg(self._REG_WHO_AM_I)[0]
        if who != self._WHO_AM_I_VAL:
            log.warning("ICM-42688 WHO_AM_I: got 0x%02X, expected 0x47", who)

        # Soft reset
        self._write_reg(self._REG_DEVICE_CONFIG, 0x01)
        time.sleep(0.001)

        # Enable accel + gyro in low-noise mode
        self._write_reg(self._REG_PWR_MGMT0, 0x0F)
        time.sleep(0.001)

        # Gyro: ±2000 dps, ODR = 1kHz
        self._write_reg(self._REG_GYRO_CONFIG0, 0x06)
        # Accel: ±4g, ODR = 1kHz
        self._write_reg(self._REG_ACCEL_CONFIG0, 0x26)

    def _loop(self):
        interval = 1.0 / self.rate_hz
        ACCEL_SCALE = 4.0 * 9.81 / 32768.0   # ±4g
        GYRO_SCALE  = 2000.0 * math.pi / 180.0 / 32768.0  # ±2000 dps

        while self._running:
            t0 = time.time()
            data = self._read_reg(self._REG_ACCEL_DATA_X1, 12)

            def s16(hi, lo): return (hi << 8 | lo) if (hi < 128) else (hi << 8 | lo) - 65536

            ax = s16(data[0],  data[1])  * ACCEL_SCALE
            ay = s16(data[2],  data[3])  * ACCEL_SCALE
            az = s16(data[4],  data[5])  * ACCEL_SCALE
            gx = s16(data[6],  data[7])  * GYRO_SCALE
            gy = s16(data[8],  data[9])  * GYRO_SCALE
            gz = s16(data[10], data[11]) * GYRO_SCALE

            self._fire(ax, ay, az, gx, gy, gz)
            elapsed = time.time() - t0
            sleep = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)


# ── MPU-6050 / MPU-9250 via I2C ─────────────────────────────────────────────────

class MPU6050(IMUBase):
    """
    MPU-6050 or MPU-9250 via I2C.
    Default address: 0x68 (AD0 low) or 0x69 (AD0 high).
    """

    _REG_PWR_MGMT_1  = 0x6B
    _REG_SMPLRT_DIV  = 0x19
    _REG_CONFIG      = 0x1A
    _REG_GYRO_CONFIG = 0x1B
    _REG_ACCEL_CONFIG= 0x1C
    _REG_ACCEL_XOUT_H= 0x3B
    _REG_WHO_AM_I    = 0x75

    def __init__(self, address: int = 0x68, rate_hz: int = 200):
        super().__init__(rate_hz)
        self._addr = address

    def start(self):
        try:
            import smbus2
        except ImportError:
            raise ImportError("smbus2 not installed. Run: pip install smbus2")

        import smbus2
        self._bus = smbus2.SMBus(1)
        self._init_device()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("MPU-6050 started on 0x%02X @ %d Hz", self._addr, self.rate_hz)

    def _write(self, reg, val):
        self._bus.write_byte_data(self._addr, reg, val)

    def _read(self, reg, length):
        return self._bus.read_i2c_block_data(self._addr, reg, length)

    def _init_device(self):
        self._write(self._REG_PWR_MGMT_1, 0x01)   # clock source = gyro X
        time.sleep(0.1)
        # Sample rate divider: 1kHz / (1 + div) = target rate
        div = max(0, min(255, int(1000 / self.rate_hz) - 1))
        self._write(self._REG_SMPLRT_DIV, div)
        self._write(self._REG_CONFIG, 0x03)        # DLPF 44Hz
        self._write(self._REG_GYRO_CONFIG, 0x18)   # ±2000 dps
        self._write(self._REG_ACCEL_CONFIG, 0x08)  # ±4g

    def _loop(self):
        interval = 1.0 / self.rate_hz
        ACCEL_SCALE = 4.0 * 9.81 / 32768.0
        GYRO_SCALE  = 2000.0 * math.pi / 180.0 / 32768.0

        while self._running:
            t0 = time.time()
            data = self._read(self._REG_ACCEL_XOUT_H, 14)

            def s16(i):
                v = data[i] << 8 | data[i+1]
                return v - 65536 if v >= 32768 else v

            ax = s16(0)  * ACCEL_SCALE
            ay = s16(2)  * ACCEL_SCALE
            az = s16(4)  * ACCEL_SCALE
            # data[6:8] = temperature, skip
            gx = s16(8)  * GYRO_SCALE
            gy = s16(10) * GYRO_SCALE
            gz = s16(12) * GYRO_SCALE

            self._fire(ax, ay, az, gx, gy, gz)
            elapsed = time.time() - t0
            sleep = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)


# ── OAK-D built-in IMU (BNO085) ─────────────────────────────────────────────────

class OakDIMU(IMUBase):
    def __init__(self, rate_hz: int = 200):
        super().__init__(rate_hz)

    def start(self):
        try:
            import depthai as dai
        except ImportError:
            raise ImportError("depthai not installed. Run: pip install depthai")

        import depthai as dai
        pipeline = dai.Pipeline()
        imu_node = pipeline.create(dai.node.IMU)
        imu_node.enableIMUSensor(
            [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW],
            self.rate_hz
        )
        imu_node.setBatchReportThreshold(1)
        imu_node.setMaxBatchReports(10)

        xout_imu = pipeline.create(dai.node.XLinkOut)
        xout_imu.setStreamName("imu")
        imu_node.out.link(xout_imu.input)

        self._pipeline = pipeline
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("OAK-D IMU started @ %d Hz", self.rate_hz)

    def _loop(self):
        import depthai as dai
        with dai.Device(self._pipeline) as dev:
            q = dev.getOutputQueue("imu", maxSize=50, blocking=False)
            while self._running:
                pkt = q.tryGet()
                if pkt is None:
                    time.sleep(0.001)
                    continue
                for pkt_data in pkt.packets:
                    a = pkt_data.acceleroMeter
                    g = pkt_data.gyroscope
                    ts = pkt_data.acceleroMeter.getTimestampDevice().total_seconds()
                    self._fire(a.x, a.y, a.z, g.x, g.y, g.z, ts)


# ── Mock IMU ─────────────────────────────────────────────────────────────────────

class MockIMU(IMUBase):
    """Synthetic IMU: gravity on Z, small random noise."""

    def __init__(self, rate_hz: int = 200):
        super().__init__(rate_hz)

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("Mock IMU started @ %d Hz", self.rate_hz)

    def _loop(self):
        interval = 1.0 / self.rate_hz
        rng = __import__("random").Random(0)
        t0 = time.time()
        while self._running:
            t = time.time() - t0
            ax = rng.gauss(0, 0.05)
            ay = rng.gauss(0, 0.05)
            az = rng.gauss(9.81, 0.05)
            # Slow yaw rotation for realism
            gx = rng.gauss(0, 0.002)
            gy = rng.gauss(0, 0.002)
            gz = rng.gauss(0.02, 0.002)
            self._fire(ax, ay, az, gx, gy, gz)
            time.sleep(interval)


# ── Sample buffer ─────────────────────────────────────────────────────────────────

class IMUSampleBuffer:
    """Thread-safe buffer that accumulates IMU samples between camera frames."""

    def __init__(self, maxlen: int = 500):
        self._buf: Deque = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def push(self, ax, ay, az, gx, gy, gz, ts):
        from src.vio.base import IMUSample
        with self._lock:
            self._buf.append(IMUSample(ax, ay, az, gx, gy, gz, ts))

    def drain(self):
        """Return and clear all buffered samples."""
        with self._lock:
            samples = list(self._buf)
            self._buf.clear()
        return samples


# ── Factory ─────────────────────────────────────────────────────────────────────

def build_imu(cfg) -> IMUBase:
    t = cfg.imu.type.lower()

    if t == "icm42688":
        return ICM42688(device=cfg.imu.device, rate_hz=cfg.imu.rate_hz)
    if t in ("mpu6050", "mpu9250"):
        return MPU6050(address=cfg.imu.i2c_address, rate_hz=cfg.imu.rate_hz)
    if t in ("oak_d", "oakd"):
        return OakDIMU(rate_hz=cfg.imu.rate_hz)
    if t == "mock":
        return MockIMU(rate_hz=cfg.imu.rate_hz)

    raise ValueError(f"Unknown IMU type: {t}")
