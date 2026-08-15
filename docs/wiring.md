# Wiring Guide — Smart RC Car

## Hardware Components

| Part | Specification |
|------|---------------|
| Raspberry Pi | Raspberry Pi 5 (4GB or 8GB) |
| Motor Driver | L298N Dual H-Bridge |
| Remote | FlySky CT6B transmitter |
| Receiver | FlySky FS-R6B (6-channel PWM) |
| Camera | USB Webcam (any OpenCV-compatible) |
| Battery | 7.4V 2S LiPo (motors) + 5V BEC or USB-C power bank (Pi) |

---

## ⚠ Critical Safety Rules

- **NEVER power the Pi directly from the motor battery** without a proper BEC or regulator.
- **NEVER connect motors directly to Pi GPIO** — always through the L298N.
- **Always power the Pi separately** (USB-C power bank or 5V BEC rated 3A+).
- Make all connections with motors/battery DISCONNECTED first.
- Double-check all connections before powering on.

---

## 1. Power Architecture

```
LiPo Battery (7.4V 2S)
       │
       ├─── L298N 12V input ─────────── Motor power
       │
       └─── 5V BEC / USB Power Bank ─── Raspberry Pi 5 (USB-C)
```

The L298N has an onboard 5V regulator (small jumper near power pins).
**Remove the 5V enable jumper** and power the Pi separately to avoid
ground loops and insufficient current.

---

## 2. L298N Motor Driver Wiring

### L298N Pin Reference

```
L298N Board
┌──────────────────────────────┐
│  OUT1  OUT2  │  OUT3  OUT4   │ ← Motor outputs
│    MOTOR A   │    MOTOR B    │
├──────────────────────────────┤
│  IN1  IN2  EN_A │ IN3  IN4  EN_B │ ← Control inputs
├──────────────────────────────┤
│  12V  GND  5V(jumper)        │ ← Power
└──────────────────────────────┘
```

### Motor Connections

```
LEFT MOTOR  → OUT1 (+), OUT2 (-)   [Motor A]
RIGHT MOTOR → OUT3 (+), OUT4 (-)   [Motor B]
```

> If motors spin the wrong direction after wiring, swap the two wires
> on that motor (OUT1 ↔ OUT2  or  OUT3 ↔ OUT4).

---

## 3. GPIO Pin Assignments

All pin numbers are **BCM (Broadcom)** numbering.

### Motor Driver — Pi GPIO to L298N

| Signal | Pi GPIO (BCM) | L298N Pin | Wire Color (suggestion) |
|--------|--------------|-----------|------------------------|
| Left speed (PWM) | **GPIO 12** | ENA | Yellow |
| Left direction A | **GPIO 23** | IN1 | Blue |
| Left direction B | **GPIO 24** | IN2 | Green |
| Right speed (PWM) | **GPIO 13** | ENB | Yellow |
| Right direction A | **GPIO 27** | IN3 | Blue |
| Right direction B | **GPIO 22** | IN4 | Green |
| Ground | Pi GND | GND | Black |

> GPIO 12 and GPIO 13 are **hardware PWM** pins on the Pi 5.
> Using hardware PWM produces much smoother motor control than software PWM.

### Pi Physical Pin Reference (for hardware PWM)

```
Pi 5 GPIO Header (40-pin)

 3.3V  [1]  [2]  5V
  SDA  [3]  [4]  5V
  SCL  [5]  [6]  GND
GPIO4  [7]  [8]  TX
  GND  [9]  [10] RX
GPIO17 [11] [12] GPIO18    ← Not used (use GPIO12/13 for PWM)
GPIO27 [13] [14] GND
GPIO22 [15] [16] GPIO23
 3.3V  [17] [18] GPIO24
GPIO10 [19] [20] GND
GPIO9  [21] [22] GPIO25
GPIO11 [23] [24] GPIO8
  GND  [25] [26] GPIO7
 GPIO0 [27] [28] GPIO1
 GPIO5 [29] [30] GND
 GPIO6 [31] [32] GPIO12  ← ENA (Left PWM)
GPIO13 [33] [34] GND
GPIO19 [35] [36] GPIO16
GPIO26 [37] [38] GPIO20
  GND  [39] [40] GPIO21
```

**Motor driver pins used:**

| BCM GPIO | Physical Pin |
|----------|-------------|
| GPIO 12  | Pin 32 (ENA — Left PWM) |
| GPIO 13  | Pin 33 (ENB — Right PWM) |
| GPIO 22  | Pin 15 (IN4) |
| GPIO 23  | Pin 16 (IN1) |
| GPIO 24  | Pin 18 (IN2) |
| GPIO 27  | Pin 13 (IN3) |

---

## 4. FlySky FS-R6B Receiver Wiring

The FS-R6B outputs standard RC PWM on individual channel pins.

### Receiver Channel Layout

```
FS-R6B Receiver
┌─────────────────────────────────┐
│  B  G  CH1  CH2  CH3  CH4  CH5 CH6 │
│  +  -  ─── ─── ─── ─── ─── ─── │
└─────────────────────────────────┘
  B = +5V (from Pi 5V pin)
  G = GND (to Pi GND)
  CH1–CH6 = PWM signal wires
```

Each channel plug has 3 pins: Signal (white/orange), +5V (red), GND (brown/black).

**Connect only the Signal (white) wire to Pi GPIO.**
**Power the receiver from Pi 5V (red) and GND (black).**

### Receiver to Pi GPIO Connections

| Receiver Pin | Pi GPIO (BCM) | Physical Pin | Function |
|-------------|--------------|-------------|----------|
| CH1 Signal | **GPIO 17** | Pin 11 | Steering |
| CH2 Signal | **GPIO 18** | Pin 12 | Throttle |
| CH3 Signal | **GPIO 27** | Pin 13 | Mode switch |
| VCC (5V) | 5V | Pin 2 or 4 | Power |
| GND | GND | Pin 6, 9, etc. | Ground |

> **Note:** GPIO 27 is shared between IN3 (motor) and CH3 (receiver) in this
> example. In production, update `config.yaml` to use a different pin for
> either the motor or the receiver. Suggested: move CH3 to **GPIO 26** (Pin 37).

#### Corrected Pin Assignments (no conflicts)

Update `config.yaml` with these values:

```yaml
receiver:
  channels:
    ch1_steering: 17
    ch2_throttle: 18
    ch3_mode: 26      # Changed from 27 to avoid motor pin conflict
```

```yaml
motors:
  right:
    enb_pin: 13
    in3_pin: 27       # Stays at 27
    in4_pin: 22
```

---

## 5. Camera Connection

USB webcam: plug into any USB 3.0 port on the Pi.

Verify with:
```bash
ls /dev/video*
# Should show /dev/video0
```

If multiple cameras: change `device_index` in `config.yaml`.

---

## 6. Full Wiring Diagram (ASCII)

```
                    ┌─────────────────────────────────┐
  LiPo Battery      │       Raspberry Pi 5             │
  7.4V ─────────────┤ 5V BEC → USB-C                  │
                    │                                  │
                    │  GPIO 12 ──────────────── ENA   │
                    │  GPIO 23 ──────────────── IN1   ├──── L298N ──┬── LEFT MOTOR
                    │  GPIO 24 ──────────────── IN2   │             │
                    │  GPIO 13 ──────────────── ENB   │             └── RIGHT MOTOR
                    │  GPIO 27 ──────────────── IN3   │
                    │  GPIO 22 ──────────────── IN4   │
                    │  GND ──────────────────── GND   │
                    │                                  │
  FS-R6B Receiver   │  GPIO 17 ←── CH1 (Steering)    │
                    │  GPIO 18 ←── CH2 (Throttle)     │
                    │  GPIO 26 ←── CH3 (Mode)         │
                    │  5V ──────── VCC receiver        │
                    │  GND ─────── GND receiver        │
                    │                                  │
  USB Webcam        │  USB 3.0 ────────────────────── │
                    └─────────────────────────────────┘

  LiPo Battery ──── 12V → L298N (separate battery terminal)
  LiPo Battery ──── GND → L298N GND (common ground with Pi)
```

---

## 7. Common Wiring Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Motors connected directly to Pi | Pi immediately shuts down or burns GPIO | Always use L298N |
| Receiver powered from wrong voltage | Receiver not responding | Use 5V from Pi (not 3.3V) |
| Missing common ground | Erratic behavior | Connect Pi GND to L298N GND and battery GND |
| ENA/ENB not connected | Motor has direction but no speed | Connect ENA/ENB to PWM GPIO pins |
| 5V jumper on L298N left in place | Pi gets wrong voltage | Remove the jumper |
| Wrong BCM vs physical pin numbering | Wrong motors moving | Verify with `gpio readall` |
