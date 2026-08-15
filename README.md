# Smart Camera-Assisted RC Car

A **Raspberry Pi 5-powered AI co-pilot** for a FlySky CT6B-controlled RC car.
The human drives at all times; the Pi watches the environment through a USB
webcam and silently prevents collisions.

> **"I'm driving the RC car, but it has an intelligent co-pilot that won't let me crash."**

---

## Quick Start

```bash
# 1. Clone / copy this project to your Raspberry Pi 5
# 2. Install everything
chmod +x install.sh && ./install.sh

# 3. Wire up hardware (see docs/wiring.md)
# 4. Review config
nano smart_rc_car/config/config.yaml

# 5. Run
source venv/bin/activate
python3 smart_rc_car/main.py

# 6. Open dashboard on any device on same Wi-Fi
http://<your-pi-ip>:5000
```

---

## Architecture

```
FlySky CT6B
      ↓ (PWM via FS-R6B receiver)
Raspberry Pi 5
  ├── FlySky Receiver Reader  (GPIO PWM callbacks via pigpio)
  ├── Camera Capture          (USB webcam, threaded)
  ├── Vision Pipeline
  │    ├── YOLOv8n Detector   (background thread)
  │    ├── Zone Analyzer      (LEFT / CENTER / RIGHT clearance)
  │    ├── Depth Estimator    (bbox-based distance)
  │    └── Ramp Detector      (Hough line analysis)
  ├── Safety Controller
  │    ├── Priority 1: Emergency Stop
  │    ├── Priority 2: Signal Lost
  │    ├── Priority 3: Manual Mode passthrough
  │    ├── Priority 4: Auto-Assist + Dodge State Machine
  │    └── Priority 5: Ramp assistance
  ├── L298N Motor Driver      (hardware PWM via pigpio)
  └── Web Dashboard           (Flask + SocketIO, MJPEG stream)
```

---

## Operating Modes

| Mode | CH3 switch | Behavior |
|------|-----------|----------|
| **MANUAL** | LOW (< 1500µs) | RC commands pass directly to motors. Camera optional. |
| **AUTO-ASSIST** | HIGH (≥ 1500µs) | Driver steers, AI prevents collisions automatically. |

---

## Project Structure

```
smart_rc_car/
├── main.py                     # Main 30Hz control loop
├── config/
│   ├── config.yaml             # All settings (GPIO, thresholds, etc.)
│   └── settings.py             # Typed config loader
├── remote/
│   ├── flysky_receiver.py      # FS-R6B PWM reader (pigpio)
│   └── channels.py             # PWM normalization + channel constants
├── camera/
│   ├── camera.py               # Threaded USB webcam capture
│   ├── preprocessing.py        # ROI crop, undistortion
│   └── calibration.py          # Chessboard calibration tool
├── vision/
│   ├── obstacle_detector.py    # Orchestrator → ObstacleReport
│   ├── yolo_detector.py        # YOLOv8n (threaded inference)
│   ├── driveable_area.py       # Zone analysis (YOLO + edges)
│   ├── ramp_detector.py        # Hough-line ramp detection
│   └── depth_estimator.py      # Bbox-height distance estimation
├── navigation/
│   ├── obstacle_avoidance.py   # Decision table (pure function)
│   ├── dodge_controller.py     # NORMAL→DODGING→RETURNING state machine
│   ├── ramp_controller.py      # Ramp steering + throttle constraints
│   └── safety_controller.py    # 5-priority motor command arbiter
├── motors/
│   ├── motor_driver.py         # L298N via pigpio (+ simulation mode)
│   ├── steering.py             # Differential mixing + AI blend
│   └── throttle.py             # Ramp-limited throttle controller
├── control/
│   ├── pid.py                  # Generic PID controller
│   └── smoothing.py            # RateLimiter, ExponentialFilter, Deadband
├── safety/
│   ├── emergency_stop.py       # Global ESTOP singleton
│   └── watchdog.py             # Software watchdog timer
├── dashboard/
│   ├── app.py                  # Flask + SocketIO dashboard server
│   └── templates/index.html    # Premium dark-theme UI
└── docs/
    ├── wiring.md               # GPIO pin table + connection diagrams
    ├── calibration.md          # Tuning all parameters
    └── testing.md              # Phase-by-phase test procedures
```

---

## Hardware

| Component | Notes |
|-----------|-------|
| Raspberry Pi 5 | 4GB or 8GB RAM |
| FlySky CT6B | 6-channel transmitter |
| FlySky FS-R6B | 6-channel PWM receiver |
| L298N | Dual H-bridge motor driver |
| USB Webcam | Any OpenCV-compatible camera |
| 2-motor chassis | 2WD differential-drive |
| 7.4V LiPo | For motors |
| 5V BEC or power bank | For Raspberry Pi 5 |

---

## Safety Features

- **Signal lost detection**: motors stop within 250ms of losing RC signal
- **Software watchdog**: stops motors if main loop hangs > 1 second
- **Emergency stop**: any module can trigger immediate motor cutoff
- **Gradual steering**: rate limiter prevents sudden jerky corrections
- **Throttle ramp**: acceleration limited to prevent tipping
- **Uncertainty policy**: if vision is uncertain → slow down, then stop
- **Manual override**: CT6B CH3 switch instantly returns full control

---

## Dashboard

Open `http://<pi-ip>:5000` on any device on your local network.

Features:
- Live annotated camera feed (MJPEG ~20 FPS)
- Zone status (LEFT / CENTER / RIGHT with color coding)
- AI action display (DODGING LEFT / FOLLOWING DRIVER / STOPPED)
- System status indicators (Remote, Camera, AI, E-Stop)
- Throttle / Steering telemetry bars
- Emergency stop button
- Event log

---

## Documentation

- **[Wiring Guide](docs/wiring.md)** — GPIO pin table, connection diagrams, safety rules
- **[Calibration Guide](docs/calibration.md)** — Tuning RC channels, motors, vision thresholds
- **[Testing Guide](docs/testing.md)** — Phase-by-phase test procedures

---

## Future Expansion Ready

The modular architecture supports adding:
- Stereo camera or LiDAR (replace `depth_estimator.py`)
- IMU (add to `ramp_controller.py`)
- Wheel encoders (extend `throttle.py`)
- MiDaS monocular depth (plug into `depth_estimator.py`)
- Lane following (new navigation module)
- Person tracking (extend YOLO pipeline)
- Voice commands (new input module)
