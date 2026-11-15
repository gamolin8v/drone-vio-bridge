<div align="center">

```
██╗   ██╗██╗ ██████╗      ██████╗ ██████╗ ██╗██████╗  ██████╗ ███████╗
██║   ██║██║██╔═══██╗     ██╔══██╗██╔══██╗██║██╔══██╗██╔════╝ ██╔════╝
██║   ██║██║██║   ██║     ██████╔╝██████╔╝██║██║  ██║██║  ███╗█████╗  
╚██╗ ██╔╝██║██║   ██║     ██╔══██╗██╔══██╗██║██║  ██║██║   ██║██╔══╝  
 ╚████╔╝ ██║╚██████╔╝     ██████╔╝██║  ██║██║██████╔╝╚██████╔╝███████╗
  ╚═══╝  ╚═╝ ╚═════╝      ╚═════╝ ╚═╝  ╚═╝╚═╝╚═════╝  ╚═════╝ ╚══════╝
```

**Visual-Inertial Odometry bridge for Raspberry Pi 5**  
*GPS-free indoor flight for ArduPilot · iNav · Betaflight*

---

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)](https://python.org)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red.svg)](https://raspberrypi.com)
[![Tests](https://img.shields.io/badge/Tests-13%2F13%20passing-brightgreen.svg)](tests/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-EKF3-orange.svg)](https://ardupilot.org)
[![iNav](https://img.shields.io/badge/iNav-MSP%20GPS-blue.svg)](https://inavflight.com)
[![Betaflight](https://img.shields.io/badge/Betaflight-RC%20Override-yellow.svg)](https://betaflight.com)

---

**[English](README.md)** · [Русский](README_RU.md) · [中文](README_ZH.md)

</div>

---

## What is this?

**drone-vio-bridge** is a Python package that runs on a Raspberry Pi 5 and gives your drone full position awareness **without GPS** — inside buildings, tunnels, warehouses, under bridges, or anywhere satellite signals don't reach.

It runs visual-inertial odometry (tracks camera features + fuses IMU data), then sends the resulting position estimate to your flight controller over a standard serial port — speaking MAVLink for ArduPilot, MSP for iNav, or MSP RC Override for Betaflight.

**One Pi. One camera. Three autopilot families.**

---

## How it works

```
┌──────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5                         │
│                                                          │
│   Camera (Global Shutter)                                │
│       │                                                  │
│       ▼                                                  │
│   Feature tracking (FAST + Lucas-Kanade optical flow)    │
│       │                                                  │
│   IMU (ICM-42688-P / MPU-6050)                           │
│       │                                                  │
│       ▼                                                  │
│   Mahony attitude filter  +  9-state EKF                 │
│       │                                                  │
│       ▼                                                  │
│   ┌───────────┬────────────┬──────────────┐             │
│   │ MAVLink   │ MSP GPS    │ MSP RC OVR   │             │
│   │ ArduPilot │ iNav       │ Betaflight   │             │
│   └───────────┴────────────┴──────────────┘             │
│                                                          │
│   Web dashboard  :5000                                   │
└──────────────────────────────────────────────────────────┘
```

---

## Features

- **Pure Python** — no ROS, no cross-compilation, runs on stock Raspberry Pi OS
- **SimpleVO** — built-in visual odometry (LK optical flow + Essential matrix + EKF)
- **MAVLink bridge** — `VISION_POSITION_ESTIMATE` for ArduPilot EKF3
- **MSP GPS bridge** — flat-earth position injection for iNav
- **MSP RC bridge** — external position PID + RC channel override for Betaflight
- **Live web dashboard** — position, attitude, quality, CPU temp on port 5000
- **Multi-camera** — RPi Global Shutter (IMX296), OV9281, OAK-D, any USB camera
- **Multi-IMU** — ICM-42688-P (SPI), MPU-6050 (I2C), OAK-D built-in
- **Mock mode** — full pipeline test without any hardware
- **One-command install** — `bash install.sh` on RPi5 Bookworm 64-bit

---

## Hardware

### Recommended build

| Part | Model | Notes |
|------|-------|-------|
| SBC | Raspberry Pi 5 (4 GB) | 8 GB if running OpenVINS |
| Camera | RPi Global Shutter (IMX296) | **Global shutter is mandatory for VIO** |
| IMU | ICM-42688-P breakout (SPI) | Low noise, 32 kHz gyro |
| Rangefinder | VL53L1X (I2C) | Indoor altitude, fixes monocular scale |
| Flight controller | Pixhawk / Cube / H743 | Any ArduPilot-supported FC |

### Budget build

| Part | Model |
|------|-------|
| SBC | Raspberry Pi 4 (slower, still works) |
| Camera | Arducam OV9281 mono global shutter |
| IMU | MPU-6050 (I2C, cheap and everywhere) |
| All-in-one | Luxonis OAK-D Lite (camera + IMU + stereo) |

> **Why global shutter?**  
> Rolling shutter cameras (most phone cameras, cheap webcams) introduce a "jello" distortion during fast motion. Feature tracking breaks completely above ~1 m/s. For anything beyond slow hover, use a global shutter sensor.

---

## Quick start

```bash
# Clone
git clone https://github.com/gamolin8v/drone-vio-bridge
cd drone-vio-bridge

# Install (Raspberry Pi 5, Bookworm 64-bit)
bash install.sh
sudo reboot

# Test without any hardware attached
python main.py --mock

# Open live dashboard
# http://<your-pi-ip>:5000
```

---

## Wiring

### ICM-42688-P via SPI (recommended)

```
Raspberry Pi 5          ICM-42688-P
──────────────          ───────────
Pin 17  (3.3V)    →     VCC
Pin 20  (GND)     →     GND
Pin 19  (MOSI)    →     SDI
Pin 21  (MISO)    →     SDO
Pin 23  (SCLK)    →     SCL
Pin 24  (CE0)     →     CS
```

### MPU-6050 via I2C (budget)

```
Raspberry Pi 5          MPU-6050
──────────────          ────────
Pin 1   (3.3V)    →     VCC
Pin 6   (GND)     →     GND
Pin 3   (SDA)     →     SDA
Pin 5   (SCL)     →     SCL
```

### RPi Global Shutter Camera

Connect to **CAM0** port. Add to `/boot/firmware/config.txt`:

```ini
# IMX296 (RPi Global Shutter)
camera_auto_detect=0
dtoverlay=imx296

# OV9281 (Arducam)
# camera_auto_detect=0
# dtoverlay=ov9281,cam0
```

### Flight controller UART

```
Raspberry Pi 5          Flight Controller
──────────────          ─────────────────
Pin 8   (TX)      →     RX
Pin 10  (RX)      →     TX
Pin 6   (GND)     →     GND
```

---

## Configuration

Edit `config.yaml` to match your hardware:

```yaml
camera:
  type: rpi               # rpi | usb | oak_d | mock

imu:
  type: icm42688          # icm42688 | mpu6050 | oak_d | mock

vio:
  scale_source: rangefinder   # rangefinder | imu | fixed

output:
  ardupilot:
    enabled: true
    port: /dev/ttyAMA0
    baudrate: 921600

  inav:
    enabled: false
    port: /dev/ttyUSB0
    origin_lat: 50.4501     # your location
    origin_lon: 30.5234

  betaflight:
    enabled: false
    port: /dev/ttyUSB1
    hover_throttle: 1500    # tune this carefully
```

---

## Flight controller setup

### ArduPilot

Set in Mission Planner or QGroundControl:

```
AHRS_EKF_TYPE   = 3     ← EKF3 (required)
EK3_ENABLE      = 1
EK2_ENABLE      = 0
EK3_SRC1_POSXY  = 6     ← ExternalNav
EK3_SRC1_VELXY  = 6     ← ExternalNav
EK3_SRC1_POSZ   = 6     ← ExternalNav (or 1 for baro)
EK3_SRC1_YAW    = 6     ← ExternalNav (or 1 for compass)
GPS_TYPE        = 0     ← disable GPS for indoor
SERIAL5_BAUD    = 921   ← match your UART
SERIAL5_PROTOCOL= 1     ← MAVLink 2
```

You should see **"EKF3 IMU0 initial yaw alignment complete"** in GCS messages  
after the bridge starts sending data.

### iNav

In Configurator → CLI tab:

```
feature GPS
set gps_provider = MSP
serial <N> MSP 115200 115200 0 115200
save
```

Set `origin_lat` / `origin_lon` in `config.yaml` to your actual location.

### Betaflight

> ⚠️ **Always remove propellers before testing RC override.**

```
# CLI tab
feature MSP
set msp_override_channels_mask = 7   ← channels 1,2,3
save
```

In **Modes** tab: assign **MSP Override** to a switch.  
Tune `hover_throttle`, `pos_p`, `pos_i` in `config.yaml` before flight.

---

## Calibration

Print a 9×6 checkerboard (25 mm squares), hold it at different angles:

```bash
python calibration/calibrate.py --type rpi
# SPACE = capture frame  |  s = compute & save  |  q = quit
# Capture 20+ frames at different angles and distances
# Target: RMS reprojection error < 0.5 px
```

Result saved to `config/camera_calibration.yaml`.

---

## Running as a service

```bash
sudo systemctl start vio-bridge
sudo systemctl status vio-bridge
sudo journalctl -u vio-bridge -f      # live logs
```

---

## Project layout

```
drone-vio-bridge/
├── main.py                       — entry point, orchestrator
├── config.yaml                   — hardware configuration
├── requirements.txt
├── install.sh                    — RPi5 Bookworm installer
│
├── src/
│   ├── config.py                 — config dataclasses + YAML loader
│   ├── sensors/
│   │   ├── camera.py             — RPi / USB / OAK-D / Mock backends
│   │   └── imu.py                — ICM-42688 / MPU-6050 / OAK-D / Mock
│   ├── vio/
│   │   ├── base.py               — Pose, IMUSample, VIOEstimator ABC
│   │   └── simple_vo.py          — LK flow + Mahony + EKF + MockVO
│   ├── bridges/
│   │   ├── mavlink_bridge.py     — ArduPilot: VISION_POSITION_ESTIMATE
│   │   ├── msp_bridge.py         — iNav: MSP GPS injection
│   │   └── betaflight_bridge.py  — Betaflight: position PID + RC override
│   └── web/
│       └── server.py             — Flask dashboard on :5000
│
├── calibration/
│   └── calibrate.py              — checkerboard camera calibration
├── systemd/
│   └── vio-bridge.service        — autostart unit
└── tests/
    └── test_bridges.py           — 13 unit tests (no hardware needed)
```

---

## Performance (Raspberry Pi 5)

| Configuration | CPU usage | End-to-end latency |
|---------------|-----------|-------------------|
| Mock VIO | ~2 % | — |
| SimpleVO 320×240 @ 30 Hz | ~20–30 % | ~10 ms |
| SimpleVO 640×480 @ 30 Hz | ~35–50 % | ~20 ms |

> SimpleVO is a solid starting point. For longer flights or larger spaces,  
> consider swapping the backend for [OpenVINS](https://docs.openvins.com)  
> or [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion).  
> The `VIOEstimator` abstract class makes the swap straightforward.

---

## Known limitations

| Limitation | Workaround |
|-----------|------------|
| Monocular scale ambiguity | Use rangefinder (`scale_source: rangefinder`) or OAK-D stereo |
| Position drift over time | No loop closure in SimpleVO — stay within ~50 m radius |
| iNav GPS reset on restart | Re-arm drone after bridge restart |
| Betaflight RC override | Not a true position EKF — requires careful PID tuning |

---

## Extending the VIO backend

```python
# src/vio/my_backend.py
from src.vio.base import VIOEstimator, Pose, CameraFrame, IMUSample

class MyVIO(VIOEstimator):
    def initialize(self) -> bool: ...
    def update(self, frame: CameraFrame, imu: list) -> Pose: ...
    def reset(self): ...
    def is_ready(self) -> bool: ...
```

Register in `src/vio/simple_vo.py → build_vio()`, set `vio.algorithm: my_backend`.

---

## Contributing

Issues and pull requests are welcome. Things that would help most:

- OpenVINS / Basalt wrapper (better accuracy than SimpleVO)
- Stereo VO backend for OAK-D (solves monocular scale problem)
- Rangefinder drivers (VL53L1X I2C, TF-Mini UART)
- Hardware-synchronized camera + IMU timestamping
- Field test reports with actual flight logs

---

## License

MIT — use it, modify it, fly with it, share what you learn.
