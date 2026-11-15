<div align="center">

```
██╗   ██╗██╗ ██████╗      ██████╗ ██████╗ ██╗██████╗  ██████╗ ███████╗
██║   ██║██║██╔═══██╗     ██╔══██╗██╔══██╗██║██╔══██╗██╔════╝ ██╔════╝
██║   ██║██║██║   ██║     ██████╔╝██████╔╝██║██║  ██║██║  ███╗█████╗  
╚██╗ ██╔╝██║██║   ██║     ██╔══██╗██╔══██╗██║██║  ██║██║   ██║██╔══╝  
 ╚████╔╝ ██║╚██████╔╝     ██████╔╝██║  ██║██║██████╔╝╚██████╔╝███████╗
  ╚═══╝  ╚═╝ ╚═════╝      ╚═════╝ ╚═╝  ╚═╝╚═╝╚═════╝  ╚═════╝ ╚══════╝
```

**Raspberry Pi 5 视觉惯性里程计桥接器**  
*无GPS室内飞行：ArduPilot · iNav · Betaflight*

---

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)](https://python.org)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red.svg)](https://raspberrypi.com)
[![Tests](https://img.shields.io/badge/Tests-13%2F13%20passing-brightgreen.svg)](tests/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-EKF3-orange.svg)](https://ardupilot.org)
[![iNav](https://img.shields.io/badge/iNav-MSP%20GPS-blue.svg)](https://inavflight.com)
[![Betaflight](https://img.shields.io/badge/Betaflight-RC%20Override-yellow.svg)](https://betaflight.com)

---

[English](README.md) · [Русский](README_RU.md) · **[中文](README_ZH.md)**

</div>

---

## 这是什么

**drone-vio-bridge** 是一个运行在 Raspberry Pi 5 上的 Python 软件包，让无人机在**没有 GPS** 的情况下也能完全感知自身位置——适用于建筑物内部、隧道、仓库、桥梁下方等卫星信号无法到达的场所。

系统通过跟踪摄像头画面中的视觉特征点并融合 IMU 数据来估算位置，然后通过标准串口将结果发送到飞控——ArduPilot 使用 MAVLink 协议，iNav 使用 MSP 协议，Betaflight 使用 MSP RC Override。

**一块 Pi，一个摄像头，三大飞控家族全兼容。**

---

## 工作原理

```
┌──────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5                         │
│                                                          │
│   摄像头（全局快门）                                      │
│       │                                                  │
│       ▼                                                  │
│   特征点追踪（FAST + Lucas-Kanade 光流）                  │
│       │                                                  │
│   IMU（ICM-42688-P / MPU-6050）                          │
│       │                                                  │
│       ▼                                                  │
│   Mahony 姿态滤波器  +  9 状态 EKF                       │
│       │                                                  │
│       ▼                                                  │
│   ┌───────────┬────────────┬──────────────┐             │
│   │ MAVLink   │ MSP GPS    │ MSP RC OVR   │             │
│   │ ArduPilot │ iNav       │ Betaflight   │             │
│   └───────────┴────────────┴──────────────┘             │
│                                                          │
│   Web 仪表盘  :5000                                      │
└──────────────────────────────────────────────────────────┘
```

---

## 功能特性

- **纯 Python** — 无需 ROS，无需交叉编译，在原版 Raspberry Pi OS 上直接运行
- **SimpleVO** — 内置视觉里程计（LK 光流 + 本质矩阵 + EKF）
- **MAVLink 桥接** — 向 ArduPilot EKF3 发送 `VISION_POSITION_ESTIMATE`
- **MSP GPS 桥接** — 以模拟 GPS 方式向 iNav 注入位置数据
- **MSP RC 桥接** — 外部位置 PID + RC 通道覆盖（Betaflight）
- **实时 Web 仪表盘** — 位置、姿态、质量、CPU 温度，端口 5000
- **多摄像头支持** — RPi Global Shutter（IMX296）、OV9281、OAK-D、任意 USB 摄像头
- **多 IMU 支持** — ICM-42688-P（SPI）、MPU-6050（I2C）、OAK-D 内置 IMU
- **模拟模式** — 无需任何硬件即可完整测试整个数据流
- **一键安装** — 在 RPi5 Bookworm 64-bit 上执行 `bash install.sh`

---

## 硬件清单

### 推荐配置

| 组件 | 型号 | 说明 |
|------|------|------|
| 单板计算机 | Raspberry Pi 5（4 GB） | 运行 OpenVINS 建议 8 GB |
| 摄像头 | RPi Global Shutter（IMX296） | **VIO 必须使用全局快门** |
| IMU | ICM-42688-P（SPI） | 低噪声，32 kHz 陀螺仪 |
| 测距仪 | VL53L1X（I2C） | 室内测高，解决单目尺度问题 |
| 飞控 | Pixhawk / Cube / H743 | 任意 ArduPilot 支持的飞控 |

### 预算配置

| 组件 | 型号 |
|------|------|
| 单板计算机 | Raspberry Pi 4（速度较慢，可用） |
| 摄像头 | Arducam OV9281 单色全局快门 |
| IMU | MPU-6050（I2C，价格低廉，货源充足） |
| 一体方案 | Luxonis OAK-D Lite（摄像头 + IMU + 双目） |

> **为什么必须用全局快门？**  
> 卷帘快门摄像头（大多数手机和廉价网络摄像头）在快速运动时会产生"果冻"扭曲。
> 超过约 1 m/s 时特征点追踪会完全失效。除非只是慢速悬停，否则必须使用全局快门传感器。

---

## 快速开始

```bash
# 克隆仓库
git clone https://github.com/gamolin8v/drone-vio-bridge
cd drone-vio-bridge

# 安装（Raspberry Pi 5，Bookworm 64-bit）
bash install.sh
sudo reboot

# 无需任何硬件的测试
python main.py --mock

# 打开实时仪表盘
# http://<你的Pi的IP>:5000
```

---

## 接线说明

### ICM-42688-P — SPI 接口（推荐）

```
Raspberry Pi 5          ICM-42688-P
──────────────          ───────────
17 号脚  (3.3V)   →     VCC
20 号脚  (GND)    →     GND
19 号脚  (MOSI)   →     SDI
21 号脚  (MISO)   →     SDO
23 号脚  (SCLK)   →     SCL
24 号脚  (CE0)    →     CS
```

### MPU-6050 — I2C 接口（预算方案）

```
Raspberry Pi 5          MPU-6050
──────────────          ────────
1 号脚   (3.3V)   →     VCC
6 号脚   (GND)    →     GND
3 号脚   (SDA)    →     SDA
5 号脚   (SCL)    →     SCL
```

### RPi Global Shutter 摄像头

连接到 **CAM0** 接口，在 `/boot/firmware/config.txt` 中添加：

```ini
# IMX296（RPi Global Shutter）
camera_auto_detect=0
dtoverlay=imx296

# OV9281（Arducam）
# camera_auto_detect=0
# dtoverlay=ov9281,cam0
```

### 飞控 UART 连接

```
Raspberry Pi 5          飞行控制器
──────────────          ──────────
8 号脚   (TX)     →     RX
10 号脚  (RX)     →     TX
6 号脚   (GND)    →     GND
```

---

## 配置说明

编辑 `config.yaml` 以匹配您的硬件：

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
    origin_lat: 39.9042     # 您的位置坐标
    origin_lon: 116.4074

  betaflight:
    enabled: false
    port: /dev/ttyUSB1
    hover_throttle: 1500    # 请根据您的无人机仔细调整
```

---

## 飞控配置

### ArduPilot

在 Mission Planner 或 QGroundControl 中设置：

```
AHRS_EKF_TYPE   = 3     ← EKF3（必须）
EK3_ENABLE      = 1
EK2_ENABLE      = 0
EK3_SRC1_POSXY  = 6     ← 外部导航
EK3_SRC1_VELXY  = 6     ← 外部导航
EK3_SRC1_POSZ   = 6     ← 外部导航（或 1 使用气压计）
EK3_SRC1_YAW    = 6     ← 外部导航（或 1 使用罗盘）
GPS_TYPE        = 0     ← 室内使用时禁用 GPS
SERIAL5_BAUD    = 921   ← 与您的 UART 匹配
SERIAL5_PROTOCOL= 1     ← MAVLink 2
```

桥接器启动发送数据后，GCS 消息中应出现：  
**"EKF3 IMU0 initial yaw alignment complete"**

### iNav

在 Configurator → CLI 标签中执行：

```
feature GPS
set gps_provider = MSP
serial <N> MSP 115200 115200 0 115200
save
```

在 `config.yaml` 中设置真实的 `origin_lat` / `origin_lon`。

### Betaflight

> ⚠️ **测试 RC Override 前务必拆除螺旋桨。**

```
# CLI 标签
feature MSP
set msp_override_channels_mask = 7   ← 通道 1,2,3
save
```

在 **Modes** 标签中将 **MSP Override** 分配到一个拨杆。  
飞行前在 `config.yaml` 中调整 `hover_throttle`、`pos_p`、`pos_i`。

---

## 摄像头标定

打印一张 9×6 棋盘格（格子尺寸 25 mm），以不同角度拍摄：

```bash
python calibration/calibrate.py --type rpi
# 空格 = 采集帧  |  s = 计算并保存  |  q = 退出
# 以不同角度和距离采集 20+ 帧
# 目标：RMS 重投影误差 < 0.5 像素
```

结果保存至 `config/camera_calibration.yaml`。

---

## 作为系统服务运行

```bash
sudo systemctl start vio-bridge
sudo systemctl status vio-bridge
sudo journalctl -u vio-bridge -f      # 实时日志
```

---

## 性能指标（Raspberry Pi 5）

| 配置 | CPU 占用 | 端到端延迟 |
|------|---------|----------|
| 模拟 VIO | ~2 % | — |
| SimpleVO 320×240 @ 30 Hz | ~20–30 % | ~10 ms |
| SimpleVO 640×480 @ 30 Hz | ~35–50 % | ~20 ms |

> SimpleVO 是一个可靠的起点。对于长航时飞行或大范围空间，可考虑将后端替换为
> [OpenVINS](https://docs.openvins.com) 或
> [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)。
> 抽象类 `VIOEstimator` 使替换过程非常简单。

---

## 已知局限性

| 局限性 | 解决方案 |
|--------|---------|
| 单目尺度模糊 | 使用测距仪或 OAK-D 双目方案 |
| 位置随时间漂移 | SimpleVO 无闭环——保持在约 50 m 范围内 |
| iNav 重启后 GPS 复位 | 重启桥接器后重新解锁无人机 |
| Betaflight RC Override | 非真正的位置 EKF——需要仔细调整 PID |

---

## 扩展 VIO 后端

```python
# src/vio/my_backend.py
from src.vio.base import VIOEstimator, Pose, CameraFrame

class MyVIO(VIOEstimator):
    def initialize(self) -> bool: ...
    def update(self, frame: CameraFrame, imu: list) -> Pose: ...
    def reset(self): ...
    def is_ready(self) -> bool: ...
```

在 `src/vio/simple_vo.py → build_vio()` 中注册，  
并在 `config.yaml` 中设置 `vio.algorithm: my_backend`。

---

## 参与贡献

欢迎提交 Issue 和 Pull Request。目前最需要的贡献：

- OpenVINS / Basalt 适配器（比 SimpleVO 精度更高）
- OAK-D 双目 VO 后端（从根本上解决尺度问题）
- 测距仪驱动（VL53L1X I2C、TF-Mini UART）
- 摄像头与 IMU 的硬件时间戳同步
- 实际飞行测试日志和报告

---

## 许可证

MIT — 使用、修改、飞行、分享您的收获。
