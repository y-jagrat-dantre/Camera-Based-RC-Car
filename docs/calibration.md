# Calibration Guide — Smart RC Car

## 1. RC Channel Calibration

The CT6B may not output exactly 1000/1500/2000µs at extremes.
Measure actual values and update `config.yaml`.

```bash
python3 -m smart_rc_car.remote.flysky_receiver
# Add debug logging to print raw pulse_us values
```

Or temporarily add this to `flysky_receiver.py` to print raw µs:
```python
print(f"CH1: {ch1_cb.pulse_us}µs  CH2: {ch2_cb.pulse_us}µs")
```

Move each stick to full min and full max. Record values and update:

```yaml
receiver:
  pwm_min_us: 1008    # Your actual minimum
  pwm_max_us: 1984    # Your actual maximum
  pwm_center_us: 1496 # Your actual center
  deadband_us: 25     # Adjust until neutral sticks give 0.000
```

---

## 2. Motor Dead-Zone Calibration

Different motors start moving at different duty cycles.
Too low = motor hums but doesn't move. Too high = jerky start.

```python
# Test manually
from smart_rc_car.motors.motor_driver import L298NDriver
d = L298NDriver()
# Try values until motor just starts moving
d.set_speeds(0.12, 0.0)  # Try 0.10, 0.12, 0.15, 0.18
```

Update `config.yaml`:
```yaml
motors:
  min_duty_cycle: 0.15   # Adjust to your motor's minimum
```

---

## 3. Camera Lens Calibration

Required if the camera has significant barrel distortion.

1. Print a 9×6 chessboard from: https://calib.io/pages/camera-calibration-pattern-generator
2. Run:
   ```bash
   python3 -m smart_rc_car.camera.calibration
   ```
3. Move the board to 20+ different positions and press SPACE each time.
4. Press `q` to compute and save.
5. Enable in `config.yaml`:
   ```yaml
   camera:
     apply_undistort: true
   ```

---

## 4. Depth Estimator Calibration

The default depth estimator uses bounding-box height ratio.
Tune it for your objects and camera height.

1. Place a known obstacle (e.g. 30cm box) at exactly **1 metre** from camera.
2. Read the bounding box height fraction in vision test.
3. Update `depth_estimator.py`:
   ```python
   BBoxDepthEstimator(ref_height_fraction=0.XX, ref_distance_m=1.0)
   ```
4. Verify reported distance ≈ 1.0m.

---

## 5. Zone Threshold Tuning

Run the vision test in your typical driving environment.
Adjust these values in `config.yaml` until zones are accurate:

```yaml
vision:
  block_threshold: 0.30    # Fraction of zone width a bbox must cover to be BLOCKED
  roi_top_fraction: 0.35   # Ignore top X% of frame (sky, ceiling)
```

Also tune edge density thresholds in `driveable_area.py` (`_analyze_edges`):
```python
if d > 0.35:    return ZoneStatus.BLOCKED   # Tune this
elif d > 0.15:  return ZoneStatus.PARTIAL   # And this
```

---

## 6. Ramp Detection Tuning

```yaml
vision:
  ramp_angle_threshold_deg: 8.0  # Minimum slope angle to detect
```

Lower value = more sensitive (may false-positive on flat floor textures).
Higher value = less sensitive (may miss shallow ramps).

---

## 7. Dodge Aggressiveness Tuning

```yaml
navigation:
  dodge_steer_amount: 0.60     # How sharply to steer during dodge (0–1)
  caution_speed_factor: 0.50   # Speed when approaching obstacle
  dodge_hold_s: 0.8            # How long to keep dodging after obstacle clears
  return_blend_rate: 0.08      # How fast to return to driver control (lower = smoother)
```

Start with lower `dodge_steer_amount` (0.40) and increase if the car doesn't
clear obstacles fast enough.
