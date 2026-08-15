#!/usr/bin/env bash
# install.sh
# ==========
# One-shot setup for Raspberry Pi 5 running Raspberry Pi OS (Bookworm).
# Run once as a regular user with sudo access:
#   chmod +x install.sh && ./install.sh

set -e
echo "=== Smart RC Car — Setup ==="

# ── 1. System packages ────────────────────────────────────────────────────────
echo "[1/6] Installing system packages..."
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-opencv \
    pigpio \
    python3-pigpio \
    libatlas-base-dev \
    libopenblas-dev \
    libcamera-tools \
    git

# ── 2. Enable & start pigpio daemon ───────────────────────────────────────────
echo "[2/6] Setting up pigpio daemon..."
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
echo "  pigpiod status: $(systemctl is-active pigpiod)"

# ── 3. Python virtual environment ────────────────────────────────────────────
echo "[3/6] Creating Python virtual environment..."
python3 -m venv venv --system-site-packages
source venv/bin/activate

# ── 4. Python packages ────────────────────────────────────────────────────────
echo "[4/6] Installing Python packages..."
pip install --upgrade pip -q
pip install -r requirements.txt -q

# ── 5. Download YOLOv8 nano model ─────────────────────────────────────────────
echo "[5/6] Pre-downloading YOLOv8n model..."
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" || \
    echo "  Warning: YOLO download failed — will retry on first run."

# ── 6. Systemd service (optional auto-start) ──────────────────────────────────
echo "[6/6] Installing systemd service..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cat > /tmp/rc_car.service << EOF
[Unit]
Description=Smart RC Car
After=network.target pigpiod.service
Requires=pigpiod.service

[Service]
Type=simple
User=$USER
WorkingDirectory=$SCRIPT_DIR
ExecStart=$SCRIPT_DIR/venv/bin/python smart_rc_car/main.py
Restart=on-failure
RestartSec=5
Environment=PYTHONPATH=$SCRIPT_DIR

[Install]
WantedBy=multi-user.target
EOF

sudo mv /tmp/rc_car.service /etc/systemd/system/rc_car.service
sudo systemctl daemon-reload
echo "  Service installed. To enable auto-start:"
echo "    sudo systemctl enable rc_car"
echo "  To start manually:"
echo "    sudo systemctl start rc_car"
echo "  To view logs:"
echo "    journalctl -u rc_car -f"

echo ""
echo "=== Setup complete! ==="
echo ""
echo "Next steps:"
echo "  1. Review smart_rc_car/config/config.yaml — verify GPIO pins."
echo "  2. See docs/wiring.md for connection diagrams."
echo "  3. Test each subsystem: see docs/testing.md"
echo "  4. Run: source venv/bin/activate && python smart_rc_car/main.py"
echo ""
