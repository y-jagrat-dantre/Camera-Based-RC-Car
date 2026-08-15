"""
config/settings.py
==================
Loads config.yaml into typed Python dataclasses.
All other modules import from here — never read YAML directly.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml


# ── Root of the project (smart_rc_car/ parent) ─────────────────────────────
PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_FILE = PROJECT_ROOT / "smart_rc_car" / "config" / "config.yaml"


# ── Dataclasses ─────────────────────────────────────────────────────────────

@dataclass
class MotorPair:
    ena_pin: int
    in1_pin: int
    in2_pin: int


@dataclass
class MotorPairB:
    enb_pin: int
    in3_pin: int
    in4_pin: int


@dataclass
class MotorConfig:
    driver: str
    left: MotorPair
    right: MotorPairB
    pwm_frequency: int
    max_duty_cycle: float
    min_duty_cycle: float
    ramp_rate: float


@dataclass
class ReceiverChannels:
    ch1_steering: int
    ch2_throttle: int
    ch3_mode: int
    ch4_aux1: Optional[int]
    ch5_aux2: Optional[int]
    ch6_aux3: Optional[int]


@dataclass
class ReceiverConfig:
    channels: ReceiverChannels
    pwm_min_us: int
    pwm_max_us: int
    pwm_center_us: int
    deadband_us: int
    signal_loss_timeout_s: float
    mode_switch_threshold_us: int


@dataclass
class CameraConfig:
    device_index: int
    width: int
    height: int
    fps: int
    calibration_file: str
    apply_undistort: bool
    flip_horizontal: bool
    flip_vertical: bool


@dataclass
class VisionConfig:
    yolo_model: str
    yolo_confidence: float
    yolo_iou: float
    yolo_input_size: int
    yolo_target_classes: Optional[list]
    zone_left_fraction: float
    zone_center_fraction: float
    zone_right_fraction: float
    roi_top_fraction: float
    block_threshold: float
    depth_method: str
    danger_distance_m: float
    warning_distance_m: float
    ramp_angle_threshold_deg: float


@dataclass
class NavigationConfig:
    caution_speed_factor: float
    dodge_steer_amount: float
    obstacle_confirm_frames: int
    dodge_hold_s: float
    return_blend_rate: float


@dataclass
class SafetyConfig:
    watchdog_timeout_s: float
    require_rearm: bool


@dataclass
class DashboardConfig:
    enabled: bool
    host: str
    port: int
    mjpeg_quality: int


@dataclass
class LoggingConfig:
    level: str
    log_file: str
    max_bytes: int
    backup_count: int


@dataclass
class Config:
    motors: MotorConfig
    receiver: ReceiverConfig
    camera: CameraConfig
    vision: VisionConfig
    navigation: NavigationConfig
    safety: SafetyConfig
    dashboard: DashboardConfig
    logging: LoggingConfig


# ── Loader ──────────────────────────────────────────────────────────────────

def _load_yaml(path: Path) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def load_config(path: Path = CONFIG_FILE) -> Config:
    raw = _load_yaml(path)

    m = raw["motors"]
    left = MotorPair(**m["left"])
    right = MotorPairB(**m["right"])
    motors = MotorConfig(
        driver=m["driver"],
        left=left,
        right=right,
        pwm_frequency=m["pwm_frequency"],
        max_duty_cycle=m["max_duty_cycle"],
        min_duty_cycle=m["min_duty_cycle"],
        ramp_rate=m["ramp_rate"],
    )

    r = raw["receiver"]
    channels = ReceiverChannels(**r["channels"])
    receiver = ReceiverConfig(
        channels=channels,
        pwm_min_us=r["pwm_min_us"],
        pwm_max_us=r["pwm_max_us"],
        pwm_center_us=r["pwm_center_us"],
        deadband_us=r["deadband_us"],
        signal_loss_timeout_s=r["signal_loss_timeout_s"],
        mode_switch_threshold_us=r["mode_switch_threshold_us"],
    )

    c = raw["camera"]
    camera = CameraConfig(**c)

    v = raw["vision"]
    vision = VisionConfig(**v)

    nav = raw["navigation"]
    navigation = NavigationConfig(**nav)

    s = raw["safety"]
    safety = SafetyConfig(**s)

    d = raw["dashboard"]
    dashboard = DashboardConfig(**d)

    lg = raw["logging"]
    logging_cfg = LoggingConfig(**lg)

    return Config(
        motors=motors,
        receiver=receiver,
        camera=camera,
        vision=vision,
        navigation=navigation,
        safety=safety,
        dashboard=dashboard,
        logging=logging_cfg,
    )


# Singleton — import and use `CFG` everywhere
CFG: Config = load_config()
