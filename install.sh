#!/usr/bin/env bash
# drone-vio-bridge — one-shot installer for Raspberry Pi 5 (Bookworm 64-bit)
# Run as: bash install.sh
set -e

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV="$REPO_DIR/.venv"

echo "=== drone-vio-bridge installer ==="
echo "Installing into: $REPO_DIR"
echo

# ── 1. System dependencies ──────────────────────────────────────────────────
echo "[1/6] Installing system packages…"
sudo apt-get update -q
sudo apt-get install -y \
  python3 python3-venv python3-pip python3-dev \
  libopencv-dev python3-opencv \
  libatlas-base-dev libopenblas-dev \
  python3-picamera2 libcamera-apps \
  i2c-tools spi-tools python3-smbus2 \
  git curl

# ── 2. Enable RPi interfaces ────────────────────────────────────────────────
echo "[2/6] Enabling I2C, SPI, UART…"
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
# Disable serial console, keep serial port hardware
sudo raspi-config nonint do_serial_hw 0
sudo raspi-config nonint do_serial_cons 1
# Enable camera (libcamera)
if ! grep -q "camera_auto_detect" /boot/firmware/config.txt 2>/dev/null; then
  echo "camera_auto_detect=1" | sudo tee -a /boot/firmware/config.txt
fi

# ── 3. Python virtual environment ───────────────────────────────────────────
echo "[3/6] Creating Python venv…"
python3 -m venv --system-site-packages "$VENV"
source "$VENV/bin/activate"
pip install --upgrade pip wheel

# ── 4. Python packages ───────────────────────────────────────────────────────
echo "[4/6] Installing Python packages…"
pip install \
  numpy scipy \
  "opencv-python-headless>=4.8" \
  pyserial pymavlink \
  PyYAML flask psutil \
  spidev smbus2

# ── 5. User permissions (serial, SPI, I2C) ──────────────────────────────────
echo "[5/6] Adding user to serial/spi/i2c groups…"
sudo usermod -aG dialout,spi,i2c "$USER"

# ── 6. systemd service ──────────────────────────────────────────────────────
echo "[6/6] Installing systemd service…"
SERVICE_SRC="$REPO_DIR/systemd/vio-bridge.service"
SERVICE_DST="/etc/systemd/system/vio-bridge.service"

# Patch paths in service file
sed "s|__REPO_DIR__|$REPO_DIR|g; s|__VENV__|$VENV|g; s|__USER__|$USER|g" \
  "$SERVICE_SRC" | sudo tee "$SERVICE_DST" > /dev/null

sudo systemctl daemon-reload
sudo systemctl enable vio-bridge.service

echo
echo "=== Installation complete ==="
echo
echo "Next steps:"
echo "  1. Reboot to apply interface changes: sudo reboot"
echo "  2. After reboot, edit config.yaml to match your hardware"
echo "  3. Run calibration: $VENV/bin/python calibration/calibrate.py"
echo "  4. Start the bridge:"
echo "       sudo systemctl start vio-bridge"
echo "       # or run manually:"
echo "       $VENV/bin/python main.py"
echo
echo "  5. Open dashboard: http://$(hostname -I | awk '{print $1}'):5000"
echo
echo "  Test without hardware: python main.py --mock"
