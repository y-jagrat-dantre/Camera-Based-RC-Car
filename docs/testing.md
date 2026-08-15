# Testing Guide — Smart RC Car

All tests are designed to be run **independently** — you don't need the
full system running to test each module.

> ⚠ Always test with the car elevated (wheels off the ground) when
> testing motor commands for the first time.

---

## Phase 1 — Configuration Test

Verify your `config.yaml` loads correctly:

```bash
cd /path/to/smart-rc-car
source venv/bin/activate
python3 -c "from smart_rc_car.config.settings import CFG; print(CFG.motors)"
```

Expected output: prints `MotorConfig(...)` with your pin assignments.

---

## Phase 2 — Receiver Test

Test that the Pi reads the FlySky receiver:

```bash
python3 -m smart_rc_car.remote.flysky_receiver
```

Move the joysticks on the CT6B and verify:
- `Steering` changes as you move the right stick left/right
- `Throttle` changes as you move the left/right trigger up/down
- `Valid: True` appears within 0.5s of powering the transmitter
- `Valid: False` appears within 0.25s of turning off the transmitter

**Troubleshooting:**
- If values don't change: check GPIO pin numbers in `config.yaml`
- If `Valid: False` always: check signal wire connection to GPIO
- If values are inverted: flip the joystick in CT6B menu or adjust `pwm_center_us`

### CH3 Mode Switch Test

Flip a 3-position switch assigned to CH3 on the CT6B.
Mode should switch between `MANUAL` and `AUTO_ASSIST`.

---

## Phase 3 — Motor Test

**Elevate the car before this test!**

```bash
python3 -m smart_rc_car.motors.motor_driver
```

Expected behavior:
1. Forward (both motors spin the same direction)
2. Spin left (right motor forward, left motor backward)
3. Spin right (left motor forward, right motor backward)
4. Reverse

**If a motor spins the wrong way:**
1. Swap the two motor wires on the L298N (OUT1 ↔ OUT2 or OUT3 ↔ OUT4)
2. OR swap IN1/IN2 in config.yaml

---

## Phase 4 — Camera Test

```bash
python3 -m smart_rc_car.camera.camera
```

A preview window opens. Verify:
- Image is right-side up (set flip flags in config if not)
- Frame rate feels smooth (~30 FPS)
- No severe distortion

Press `q` to exit.

---

## Phase 5 — Vision Test (YOLO + Zone Detection)

```bash
python3 -m smart_rc_car.vision.yolo_detector
```

Hold objects in front of the camera. Verify:
- Bounding boxes appear around objects
- Labels show class names (person, chair, etc.)

Then test the full zone display:

```python
# Run this in a Python shell
from smart_rc_car.camera.camera import CameraCapture
from smart_rc_car.vision.obstacle_detector import ObstacleDetector
import cv2, time

cam = CameraCapture()
cam.start()
det = ObstacleDetector()
det.start()
time.sleep(1)

while True:
    frame = cam.read()
    if frame is not None:
        report = det.process(frame)
        annotated = det.annotate_frame(frame, report)
        print(f"L:{report.zones.left} C:{report.zones.center} R:{report.zones.right} "
              f"Dist:{report.closest_distance_m:.2f}m")
        cv2.imshow("Vision", annotated)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.stop()
det.stop()
cv2.destroyAllWindows()
```

Expected: zone colors change (green/orange/red) as you place objects.

---

## Phase 6 — Safety Controller Test (No Motors)

Test the full decision pipeline with simulated inputs:

```python
from smart_rc_car.navigation.safety_controller import SafetyController
from smart_rc_car.remote.channels import ChannelValues
from smart_rc_car.vision.driveable_area import ZoneAnalysis, ZoneStatus
from smart_rc_car.vision.obstacle_detector import ObstacleReport
from smart_rc_car.motors.motor_driver import L298NDriver

# Use simulation mode
driver = L298NDriver(simulate=True)
ctrl = SafetyController(driver)

# Simulate: driver going forward, obstacle in center
channels = ChannelValues(steering=0.0, throttle=0.5, mode_raw_us=1700, valid=True)
zones = ZoneAnalysis(left=ZoneStatus.CLEAR, center=ZoneStatus.BLOCKED, right=ZoneStatus.CLEAR)
from smart_rc_car.vision.ramp_detector import RampInfo
report = ObstacleReport(zones=zones, closest_distance_m=0.5, ramp=RampInfo())

output = ctrl.update(channels, report)
print(f"Action: {output.ai_action}")
print(f"Steering: {output.steering:+.3f}")
print(f"L/R: {output.left:.3f} / {output.right:.3f}")
# Expected: DODGE_LEFT with negative steering
```

---

## Phase 7 — Emergency Stop Test

```python
from smart_rc_car.safety.emergency_stop import ESTOP, StopReason

ESTOP.trigger(StopReason.MANUAL_TRIGGER)
print(f"Active: {ESTOP.active}")   # Should be True
print(f"Reason: {ESTOP.reason}")  # Should be MANUAL_TRIGGER
ESTOP.clear()
print(f"Active: {ESTOP.active}")   # Should be False
```

---

## Phase 8 — Dashboard Test

Start only the dashboard without hardware:

```bash
python3 -c "
from smart_rc_car.dashboard.app import DashboardServer
import time
server = DashboardServer()
server.update({'mode':'AUTO_ASSIST','remote':'CONNECTED','camera':'CONNECTED',
               'ai':'RUNNING','zone_left':'CLEAR','zone_center':'BLOCKED',
               'zone_right':'CLEAR','ai_action':'DODGING LEFT',
               'distance_m': 0.6, 'ramp': False, 'throttle': 0.5, 'steering': -0.6})
server.start()
print('Dashboard at http://localhost:5000')
time.sleep(60)
"
```

Open `http://<Pi-IP>:5000` on a phone or laptop.

---

## Phase 9 — Full System Test

1. Elevate the car.
2. Power on CT6B transmitter.
3. Start the system:
   ```bash
   source venv/bin/activate
   python3 smart_rc_car/main.py
   ```
4. Open dashboard in browser.
5. In **MANUAL** mode, verify joystick drives motors.
6. Switch CT6B CH3 to **AUTO-ASSIST** mode.
7. Place an obstacle in front of the camera.
8. Command forward — verify auto-dodge occurs.
9. Remove obstacle — verify control returns to driver.
10. Turn off transmitter — verify motors stop within 0.25s.

---

## Calibration Checklist

- [ ] Camera calibration run (`python3 -m smart_rc_car.camera.calibration`)
- [ ] RC channel calibration (verify center/min/max in `config.yaml`)
- [ ] Motor dead-zone calibration (`min_duty_cycle` in `config.yaml`)
- [ ] Depth estimator calibration (measure real distances vs reported)
- [ ] Edge detection threshold tuning (test in various lighting)
- [ ] YOLO confidence threshold tuning (reduce false positives)
