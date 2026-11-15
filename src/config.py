"""Configuration loader for drone-vio-bridge."""

import yaml
import os
import logging
from dataclasses import dataclass, field
from typing import Optional, List

log = logging.getLogger(__name__)


@dataclass
class CameraConfig:
    type: str = "rpi"
    device: str = "/dev/video0"
    width: int = 640
    height: int = 480
    fps: int = 30
    calibration_file: str = "config/camera_calibration.yaml"


@dataclass
class IMUConfig:
    type: str = "mock"
    bus: str = "i2c"
    device: str = "/dev/spidev0.0"
    i2c_address: int = 0x68
    rate_hz: int = 200
    gyro_noise: float = 0.005
    accel_noise: float = 0.01
    gyro_bias_noise: float = 0.0001
    accel_bias_noise: float = 0.0001


@dataclass
class VIOConfig:
    algorithm: str = "simple_vo"
    scale_source: str = "rangefinder"
    fixed_scale: float = 1.0
    min_features: int = 50
    max_features: int = 500
    ransac_threshold: float = 1.0
    position_noise: float = 0.1
    velocity_noise: float = 0.05
    output_frame: str = "NED"


@dataclass
class ArduPilotOutputConfig:
    enabled: bool = True
    port: str = "/dev/ttyAMA0"
    baudrate: int = 921600
    send_rate_hz: int = 20


@dataclass
class INavOutputConfig:
    enabled: bool = False
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    send_rate_hz: int = 10
    origin_lat: float = 0.0
    origin_lon: float = 0.0
    origin_alt: int = 0


@dataclass
class BetaflightOutputConfig:
    enabled: bool = False
    port: str = "/dev/ttyUSB1"
    baudrate: int = 115200
    send_rate_hz: int = 50
    override_channels: List[int] = field(default_factory=lambda: [1, 2, 3])
    hover_throttle: int = 1500
    pos_p: float = 0.5
    pos_i: float = 0.02
    vel_p: float = 0.3


@dataclass
class OutputConfig:
    ardupilot: ArduPilotOutputConfig = field(default_factory=ArduPilotOutputConfig)
    inav: INavOutputConfig = field(default_factory=INavOutputConfig)
    betaflight: BetaflightOutputConfig = field(default_factory=BetaflightOutputConfig)


@dataclass
class RangefinderConfig:
    enabled: bool = False
    type: str = "vl53l1x"
    i2c_address: int = 0x29
    port: str = "/dev/ttyS0"
    max_range_m: float = 4.0


@dataclass
class WebConfig:
    host: str = "0.0.0.0"
    port: int = 5000
    update_interval_ms: int = 100


@dataclass
class AppConfig:
    system_log_level: str = "INFO"
    system_update_rate_hz: int = 30
    system_web_ui: bool = True
    camera: CameraConfig = field(default_factory=CameraConfig)
    imu: IMUConfig = field(default_factory=IMUConfig)
    vio: VIOConfig = field(default_factory=VIOConfig)
    output: OutputConfig = field(default_factory=OutputConfig)
    rangefinder: RangefinderConfig = field(default_factory=RangefinderConfig)
    web: WebConfig = field(default_factory=WebConfig)


def _apply_dict(obj, d: dict):
    """Recursively apply a dict to a dataclass instance."""
    for k, v in d.items():
        if hasattr(obj, k):
            attr = getattr(obj, k)
            if hasattr(attr, '__dataclass_fields__') and isinstance(v, dict):
                _apply_dict(attr, v)
            else:
                setattr(obj, k, v)


def load_config(path: str = "config.yaml") -> AppConfig:
    cfg = AppConfig()

    if not os.path.exists(path):
        log.warning("Config file %s not found, using defaults", path)
        return cfg

    with open(path, "r") as f:
        raw = yaml.safe_load(f)

    if not raw:
        return cfg

    system = raw.get("system", {})
    cfg.system_log_level = system.get("log_level", cfg.system_log_level)
    cfg.system_update_rate_hz = system.get("update_rate_hz", cfg.system_update_rate_hz)
    cfg.system_web_ui = system.get("web_ui", cfg.system_web_ui)

    if "camera" in raw:
        _apply_dict(cfg.camera, raw["camera"])
    if "imu" in raw:
        _apply_dict(cfg.imu, raw["imu"])
    if "vio" in raw:
        _apply_dict(cfg.vio, raw["vio"])
    if "rangefinder" in raw:
        _apply_dict(cfg.rangefinder, raw["rangefinder"])
    if "web" in raw:
        _apply_dict(cfg.web, raw["web"])
    if "output" in raw:
        out = raw["output"]
        if "ardupilot" in out:
            _apply_dict(cfg.output.ardupilot, out["ardupilot"])
        if "inav" in out:
            _apply_dict(cfg.output.inav, out["inav"])
        if "betaflight" in out:
            _apply_dict(cfg.output.betaflight, out["betaflight"])

    return cfg
