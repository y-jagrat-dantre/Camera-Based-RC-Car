#!/usr/bin/env bash
# setup_pc.sh
# ===========
# One-shot setup for running the Smart RC Car simulator on a Linux PC.
# Run once:
#   chmod +x setup_pc.sh && ./setup_pc.sh

set -e
echo "=== Smart RC Car — PC Setup ==="

# ── 1. System packages ────────────────────────────────────────────────────────
echo "[1/3] Installing system packages..."
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-tk \
    libgl1 \
    libglib2.0-0

# ── 2. Python virtual environment ────────────────────────────────────────────
echo "[2/3] Creating Python virtual environment..."
python3 -m venv venv
source venv/bin/activate

# ── 3. Python packages ────────────────────────────────────────────────────────
echo "[3/3] Installing Python packages..."
pip install --upgrade pip -q
pip install -r requirements_pc.txt -q

# Pre-download YOLO model
echo "Pre-downloading YOLOv8n model (~6MB)..."
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" || \
    echo "Warning: YOLO download failed — will retry on first run."

echo ""
echo "=== Setup complete! ==="
echo ""
echo "To run the simulator:"
echo "  source venv/bin/activate"
echo "  python simulate.py"
echo ""
echo "Options:"
echo "  python simulate.py --no-yolo       # Faster startup, edge-only detection"
echo "  python simulate.py --no-dashboard  # No web dashboard"
echo "  python simulate.py --camera 1      # Use a different webcam"
echo ""
echo "Controls in the preview window:"
echo "  W/S = Throttle  |  A/D = Steer  |  M = Mode toggle"
echo "  SPACE = Brake   |  ESC = Emergency stop"
