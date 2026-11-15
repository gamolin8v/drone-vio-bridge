<div align="center">

```
██╗   ██╗██╗ ██████╗      ██████╗ ██████╗ ██╗██████╗  ██████╗ ███████╗
██║   ██║██║██╔═══██╗     ██╔══██╗██╔══██╗██║██╔══██╗██╔════╝ ██╔════╝
██║   ██║██║██║   ██║     ██████╔╝██████╔╝██║██║  ██║██║  ███╗█████╗  
╚██╗ ██╔╝██║██║   ██║     ██╔══██╗██╔══██╗██║██║  ██║██║   ██║██╔══╝  
 ╚████╔╝ ██║╚██████╔╝     ██████╔╝██║  ██║██║██████╔╝╚██████╔╝███████╗
  ╚═══╝  ╚═╝ ╚═════╝      ╚═════╝ ╚═╝  ╚═╝╚═╝╚═════╝  ╚═════╝ ╚══════╝
```

**Мост визуально-инерциальной одометрии для Raspberry Pi 5**  
*Полёт без GPS: ArduPilot · iNav · Betaflight*

---

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)](https://python.org)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red.svg)](https://raspberrypi.com)
[![Tests](https://img.shields.io/badge/Tests-13%2F13%20passing-brightgreen.svg)](tests/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-EKF3-orange.svg)](https://ardupilot.org)
[![iNav](https://img.shields.io/badge/iNav-MSP%20GPS-blue.svg)](https://inavflight.com)
[![Betaflight](https://img.shields.io/badge/Betaflight-RC%20Override-yellow.svg)](https://betaflight.com)

---

[English](README.md) · **[Русский](README_RU.md)** · [中文](README_ZH.md)

</div>

---

## Что это такое

**drone-vio-bridge** — Python-пакет для Raspberry Pi 5, который даёт дрону полное понимание своего положения в пространстве **без GPS**: внутри зданий, в тоннелях, складах, под мостами и везде, где спутниковый сигнал не проходит.

Система отслеживает признаки на кадрах камеры и совмещает их с данными IMU, затем отправляет результат на полётный контроллер через стандартный UART — MAVLink для ArduPilot, MSP для iNav или MSP RC Override для Betaflight.

**Один Pi. Одна камера. Три семейства автопилотов.**

---

## Как это работает

```
┌──────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5                         │
│                                                          │
│   Камера (глобальный затвор)                             │
│       │                                                  │
│       ▼                                                  │
│   Трекинг признаков (FAST + оптический поток LK)         │
│       │                                                  │
│   IMU (ICM-42688-P / MPU-6050)                           │
│       │                                                  │
│       ▼                                                  │
│   Фильтр ориентации Mahony  +  EKF (9 состояний)         │
│       │                                                  │
│       ▼                                                  │
│   ┌───────────┬────────────┬──────────────┐             │
│   │ MAVLink   │ MSP GPS    │ MSP RC OVR   │             │
│   │ ArduPilot │ iNav       │ Betaflight   │             │
│   └───────────┴────────────┴──────────────┘             │
│                                                          │
│   Веб-дашборд  :5000                                     │
└──────────────────────────────────────────────────────────┘
```

---

## Возможности

- **Чистый Python** — никакого ROS, никакой кросс-компиляции
- **SimpleVO** — встроенная визуальная одометрия (LK-поток + матрица существенности + EKF)
- **MAVLink-мост** — `VISION_POSITION_ESTIMATE` для ArduPilot EKF3
- **MSP GPS-мост** — инъекция позиции в iNav как синтетический GPS
- **MSP RC-мост** — внешний PID + перехват RC каналов для Betaflight
- **Веб-дашборд** — позиция, ориентация, качество, температура CPU на порту 5000
- **Поддержка камер** — RPi Global Shutter (IMX296), OV9281, OAK-D, любая USB
- **Поддержка IMU** — ICM-42688-P (SPI), MPU-6050 (I2C), OAK-D встроенный
- **Mock-режим** — полный тест без какого-либо железа
- **Установка одной командой** — `bash install.sh` на RPi5 Bookworm 64-bit

---

## Железо

### Рекомендуемая сборка

| Компонент | Модель | Примечание |
|-----------|--------|-----------|
| SBC | Raspberry Pi 5 (4 ГБ) | 8 ГБ если планируете OpenVINS |
| Камера | RPi Global Shutter (IMX296) | **Глобальный затвор обязателен для VIO** |
| IMU | ICM-42688-P (SPI) | Малый шум, 32 кГц гироскоп |
| Дальномер | VL53L1X (I2C) | Высота в помещении, решает проблему масштаба |
| Полётный контроллер | Pixhawk / Cube / H743 | Любой поддерживаемый ArduPilot |

### Бюджетная сборка

| Компонент | Модель |
|-----------|--------|
| SBC | Raspberry Pi 4 (медленнее, но работает) |
| Камера | Arducam OV9281 монохром глобальный затвор |
| IMU | MPU-6050 (I2C, дёшево и везде есть) |
| Всё-в-одном | Luxonis OAK-D Lite (камера + IMU + стерео) |

> **Почему глобальный затвор?**  
> Камеры с роллинг-затвором (большинство телефонов, дешёвые вебки) дают "желе"-искажение при быстром движении. Трекинг признаков ломается уже при ~1 м/с. Для чего-либо серьёзнее медленного зависания — только глобальный затвор.

---

## Быстрый старт

```bash
# Клонировать
git clone https://github.com/gamolin8v/drone-vio-bridge
cd drone-vio-bridge

# Установить (Raspberry Pi 5, Bookworm 64-bit)
bash install.sh
sudo reboot

# Тест без какого-либо железа
python main.py --mock

# Открыть дашборд
# http://<ip-вашего-pi>:5000
```

---

## Подключение

### ICM-42688-P через SPI (рекомендуется)

```
Raspberry Pi 5          ICM-42688-P
──────────────          ───────────
Пин 17  (3.3В)    →     VCC
Пин 20  (GND)     →     GND
Пин 19  (MOSI)    →     SDI
Пин 21  (MISO)    →     SDO
Пин 23  (SCLK)    →     SCL
Пин 24  (CE0)     →     CS
```

### MPU-6050 через I2C (бюджет)

```
Raspberry Pi 5          MPU-6050
──────────────          ────────
Пин 1   (3.3В)    →     VCC
Пин 6   (GND)     →     GND
Пин 3   (SDA)     →     SDA
Пин 5   (SCL)     →     SCL
```

### Камера RPi Global Shutter

Подключить к порту **CAM0**. Добавить в `/boot/firmware/config.txt`:

```ini
# IMX296 (RPi Global Shutter)
camera_auto_detect=0
dtoverlay=imx296

# OV9281 (Arducam)
# camera_auto_detect=0
# dtoverlay=ov9281,cam0
```

### UART на полётный контроллер

```
Raspberry Pi 5          Полётный контроллер
──────────────          ───────────────────
Пин 8   (TX)      →     RX
Пин 10  (RX)      →     TX
Пин 6   (GND)     →     GND
```

---

## Настройка

Отредактируйте `config.yaml` под своё железо:

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
    origin_lat: 55.7558     # ваши координаты
    origin_lon: 37.6173

  betaflight:
    enabled: false
    port: /dev/ttyUSB1
    hover_throttle: 1500    # подобрать под свой дрон
```

---

## Настройка полётных контроллеров

### ArduPilot

Установить в Mission Planner или QGroundControl:

```
AHRS_EKF_TYPE   = 3     ← EKF3 (обязательно)
EK3_ENABLE      = 1
EK2_ENABLE      = 0
EK3_SRC1_POSXY  = 6     ← ExternalNav
EK3_SRC1_VELXY  = 6     ← ExternalNav
EK3_SRC1_POSZ   = 6     ← ExternalNav (или 1 для барометра)
EK3_SRC1_YAW    = 6     ← ExternalNav (или 1 для компаса)
GPS_TYPE        = 0     ← отключить GPS (только помещение)
SERIAL5_BAUD    = 921   ← совпадает с вашим UART
SERIAL5_PROTOCOL= 1     ← MAVLink 2
```

После запуска моста в сообщениях GCS должно появиться:  
**"EKF3 IMU0 initial yaw alignment complete"**

### iNav

В Configurator → вкладка CLI:

```
feature GPS
set gps_provider = MSP
serial <N> MSP 115200 115200 0 115200
save
```

Укажите `origin_lat` / `origin_lon` в `config.yaml` — ваше реальное местоположение.

### Betaflight

> ⚠️ **Перед тестом RC Override — снять пропеллеры.**

```
# Вкладка CLI
feature MSP
set msp_override_channels_mask = 7   ← каналы 1,2,3
save
```

Во вкладке **Modes** назначьте **MSP Override** на переключатель.  
Настройте `hover_throttle`, `pos_p`, `pos_i` в `config.yaml` до полёта.

---

## Калибровка камеры

Распечатайте шахматную доску 9×6 (клетки 25 мм), держите под разными углами:

```bash
python calibration/calibrate.py --type rpi
# ПРОБЕЛ = захват кадра  |  s = вычислить и сохранить  |  q = выход
# Нужно 20+ кадров под разными углами
# Цель: RMS ошибка репроекции < 0.5 пикселя
```

Результат сохраняется в `config/camera_calibration.yaml`.

---

## Запуск как системный сервис

```bash
sudo systemctl start vio-bridge
sudo systemctl status vio-bridge
sudo journalctl -u vio-bridge -f      # живые логи
```

---

## Производительность (Raspberry Pi 5)

| Конфигурация | Нагрузка CPU | Задержка |
|--------------|-------------|---------|
| Mock VIO | ~2 % | — |
| SimpleVO 320×240 @ 30 Гц | ~20–30 % | ~10 мс |
| SimpleVO 640×480 @ 30 Гц | ~35–50 % | ~20 мс |

---

## Известные ограничения

| Ограничение | Как обойти |
|------------|-----------|
| Проблема масштаба (моноскоп) | Используйте дальномер или OAK-D стерео |
| Дрейф позиции со временем | SimpleVO без замыкания петли — держитесь в ~50 м |
| Сброс GPS iNav при перезапуске | Переарминг после перезапуска моста |
| Betaflight RC Override | Не настоящий EKF — требует аккуратной настройки PID |

---

## Расширение бэкенда VIO

```python
# src/vio/my_backend.py
from src.vio.base import VIOEstimator, Pose, CameraFrame

class MyVIO(VIOEstimator):
    def initialize(self) -> bool: ...
    def update(self, frame: CameraFrame, imu: list) -> Pose: ...
    def reset(self): ...
    def is_ready(self) -> bool: ...
```

Зарегистрируйте в `src/vio/simple_vo.py → build_vio()`,  
установите `vio.algorithm: my_backend` в `config.yaml`.

---

## Участие в разработке

Принимаются issues и pull requests. Что сейчас нужнее всего:

- Обёртка для OpenVINS / Basalt (точнее SimpleVO)
- Стерео-VO для OAK-D (решает проблему масштаба полностью)
- Драйверы дальномеров (VL53L1X I2C, TF-Mini UART)
- Аппаратная синхронизация camera + IMU по временным меткам
- Полётные логи и отчёты с реальных тестов

---

## Лицензия

MIT — используй, модифицируй, летай, делись тем, что узнал.
