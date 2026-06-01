"""
Runtime settings layer.

The defaults for every tunable live in :mod:`components.config`. This module
adds a thin, persisted *override* layer on top so the values can be adjusted
from the settings UI without editing source files.

Two scopes:
  - ``run``      : behaviour/tuning read by the AutoNavigator. Overrides are
                   applied (re-bound onto the running modules) when an auto run
                   starts, via :func:`apply_run_overrides`. Editable in the UI.
  - ``readonly`` : hardware/wiring read once at process startup. Shown in the
                   UI for reference only; changing them means editing
                   ``config.py`` and restarting the backend.

Overrides persist to ``Backend/runtime_settings.json``. A missing or corrupt
file is treated as "no overrides" — defaults always win, so a bad file can
never stop the robot from starting.
"""

from __future__ import annotations

import json
import logging
import threading
from pathlib import Path

from components import config

logger = logging.getLogger(__name__)

_SETTINGS_PATH = Path(__file__).resolve().parent.parent / "runtime_settings.json"
_lock = threading.Lock()


# ---------------------------------------------------------------------------
# Schema — sections, fields, and per-field metadata. Defaults are pulled from
# config at access time, so this stays a single source of truth.
# ---------------------------------------------------------------------------

def _f(key, label, help, type_, scope="run", **extra):
    return {"key": key, "label": label, "help": help,
            "type": type_, "scope": scope, **extra}


SECTIONS = [
    {
        "id": "mission",
        "title": "Mission / Return to base",
        "desc": "How many plants to water before driving home, and how the base "
                "QR code is recognised.",
        "fields": [
            _f("PLANTS_PER_RUN", "Plants per run",
               "Number of watering events before the robot stops looking for "
               "plants and returns to base. 0 = unlimited (never return).",
               "int", min=0, max=100),
            _f("BASE_QR_PAYLOAD", "Base QR text",
               "The robot only treats a QR code as the base when its decoded "
               "text matches this exactly. Must match the QR you printed.",
               "str"),
            _f("BASE_ARRIVAL_AREA_RATIO", "Base arrival size",
               "Considered 'arrived' once the base QR fills this fraction of the "
               "frame (bigger = robot stops closer to the base).",
               "float", min=0.05, max=0.95, step=0.01),
        ],
    },
    {
        "id": "speeds",
        "title": "Driving speeds",
        "desc": "Motor speeds (0.0–1.0) used during autonomous navigation.",
        "fields": [
            _f("AUTO_SPEED_FORWARD", "Forward speed",
               "Speed when approaching a plant or the base.",
               "float", min=0.0, max=1.0, step=0.05),
            _f("AUTO_SPEED_BACKWARD", "Reverse speed",
               "Speed when retreating after watering.",
               "float", min=0.0, max=1.0, step=0.05),
            _f("AUTO_SPEED_TURN", "Turn speed",
               "Normal left/right turn speed.",
               "float", min=0.0, max=1.0, step=0.05),
            _f("AUTO_SPEED_TURN_GENTLE", "Gentle turn speed",
               "Slower turn speed used when the robot detects it is oscillating "
               "left-right past a target.",
               "float", min=0.0, max=1.0, step=0.05),
            _f("MIN_TURN_SPEED", "Minimum turn speed",
               "Floor so a turn never drops so low the motors stall.",
               "float", min=0.0, max=1.0, step=0.05),
        ],
    },
    {
        "id": "timing",
        "title": "Movement timing",
        "desc": "How long the robot moves per step, and how long it waits to "
                "re-check the camera (seconds).",
        "fields": [
            _f("MOVE_TURN_DURATION", "Turn step",
               "Seconds the motors run for one left/right correction.",
               "float", min=0.05, max=5.0, step=0.05),
            _f("MOVE_FORWARD_DURATION", "Forward step",
               "Seconds the motors run for one forward approach step.",
               "float", min=0.05, max=5.0, step=0.05),
            _f("SLEEP_WAIT_YOLO", "Look-again wait",
               "After stopping, seconds to wait for fresh detections before "
               "deciding what to do next.",
               "float", min=0.1, max=10.0, step=0.1),
            _f("SCAN_TURN_DURATION", "Scan turn",
               "Seconds of rotation per scan step when searching for a plant or "
               "the base.",
               "float", min=0.1, max=5.0, step=0.1),
        ],
    },
    {
        "id": "watering",
        "title": "Watering & approach",
        "desc": "When to water, how long, and how it retreats afterwards.",
        "fields": [
            _f("ARRIVAL_AREA_RATIO", "Plant arrival size",
               "Water once the plant box fills this fraction of the frame.",
               "float", min=0.05, max=0.95, step=0.01),
            _f("CENTER_MARGIN", "Center zone width",
               "Middle fraction of the frame treated as 'centered' (rest is "
               "left/right).",
               "float", min=0.05, max=0.49, step=0.01),
            _f("WATERING_DURATION", "Watering seconds",
               "How long the pump runs per plant.",
               "int", min=1, max=60),
            _f("RETREAT_DURATION", "Retreat seconds",
               "How long the robot reverses after watering to look for the next "
               "plant.",
               "float", min=0.0, max=20.0, step=0.5),
            _f("RETREAT_POLL_PERIOD", "Retreat check rate",
               "How often (seconds) to look for a new plant while retreating.",
               "float", min=0.02, max=2.0, step=0.02),
            _f("LOST_THRESHOLD", "Lost patience",
               "How many empty look-cycles before switching from holding still "
               "to actively scanning.",
               "int", min=1, max=20),
        ],
    },
    {
        "id": "reid",
        "title": "Plant memory (ReID)",
        "desc": "Appearance capture used so already-watered plants aren't "
                "watered twice.",
        "fields": [
            _f("REID_CAPTURE_SAMPLES", "Capture samples",
               "How many appearance snapshots to take while parked at a watered "
               "plant.",
               "int", min=1, max=30),
            _f("REID_CAPTURE_INTERVAL", "Capture interval",
               "Seconds between those snapshots.",
               "float", min=0.05, max=2.0, step=0.05),
            _f("REID_RETREAT_INTERVAL", "Retreat capture interval",
               "Seconds between extra appearance snapshots taken while "
               "retreating.",
               "float", min=0.05, max=2.0, step=0.05),
        ],
    },
    {
        "id": "obstacle",
        "title": "Obstacle avoidance",
        "desc": "Ultrasonic obstacle detection and the avoidance manoeuvre.",
        "fields": [
            _f("ULTRASONIC_OBSTACLE_CM", "Obstacle distance (cm)",
               "Treat anything closer than this as an obstacle to avoid.",
               "int", min=2, max=400),
            _f("ULTRASONIC_POLL_INTERVAL", "Sensor poll interval",
               "Seconds between distance readings.",
               "float", min=0.05, max=5.0, step=0.05),
            _f("ULTRASONIC_SCAN_COOLDOWN", "Scan cooldown",
               "Minimum seconds between servo sweep scans.",
               "float", min=0.0, max=30.0, step=0.5),
            _f("ULTRASONIC_AVOID_SPEED", "Avoid speed",
               "Motor speed (0.0–1.0) during the avoidance manoeuvre.",
               "float", min=0.0, max=1.0, step=0.05),
            _f("ULTRASONIC_AVOID_BACKWARD_SECONDS", "Avoid: back up",
               "Seconds to reverse when an obstacle is hit.",
               "float", min=0.0, max=10.0, step=0.1),
            _f("ULTRASONIC_AVOID_TURN_SECONDS", "Avoid: turn",
               "Seconds to turn away from the obstacle.",
               "float", min=0.0, max=10.0, step=0.1),
            _f("ULTRASONIC_AVOID_FORWARD_SECONDS", "Avoid: go around",
               "Seconds to drive forward past the obstacle.",
               "float", min=0.0, max=10.0, step=0.1),
        ],
    },
    {
        "id": "hardware",
        "title": "Hardware / wiring (read-only)",
        "desc": "These are read when the backend boots. To change them, edit "
                "components/config.py and restart the backend.",
        "fields": [
            _f("CAMERA_SOURCE", "Camera stream URL", "MJPEG source for the camera.", "str", scope="readonly"),
            _f("MOTOR_PWM_FREQUENCY", "Motor PWM frequency (Hz)", "PWM frequency for the motor drivers.", "int", scope="readonly"),
            _f("MOTOR_LEFT_FWD_PIN", "Left motor FWD pin", "BCM GPIO.", "int", scope="readonly"),
            _f("MOTOR_LEFT_BWD_PIN", "Left motor BWD pin", "BCM GPIO.", "int", scope="readonly"),
            _f("MOTOR_LEFT_EN_PIN", "Left motor EN pin", "BCM GPIO.", "int", scope="readonly"),
            _f("MOTOR_LEFT_EN_PIN1", "Left motor EN pin 2", "BCM GPIO.", "int", scope="readonly"),
            _f("MOTOR_RIGHT_FWD_PIN", "Right motor FWD pin", "BCM GPIO.", "int", scope="readonly"),
            _f("MOTOR_RIGHT_BWD_PIN", "Right motor BWD pin", "BCM GPIO.", "int", scope="readonly"),
            _f("MOTOR_RIGHT_EN_PIN", "Right motor EN pin", "BCM GPIO.", "int", scope="readonly"),
            _f("MOTOR_RIGHT_EN_PIN1", "Right motor EN pin 2", "BCM GPIO.", "int", scope="readonly"),
            _f("PUMP_PIN", "Pump relay pin", "BCM GPIO.", "int", scope="readonly"),
            _f("PUMP_ACTIVE_HIGH", "Pump active-high", "Relay polarity.", "bool", scope="readonly"),
            _f("BUZZER_PIN", "Passive buzzer pin", "BCM GPIO PWM output.", "int", scope="readonly"),
            _f("LED_PIN", "LED data pin", "BCM GPIO for the WS2812 strip.", "int", scope="readonly"),
            _f("LED_COUNT", "LED count", "Number of status LEDs.", "int", scope="readonly"),
            _f("LED_BRIGHTNESS", "LED brightness", "0.0–1.0.", "float", scope="readonly"),
            _f("ULTRASONIC_SERIAL_PORT", "Ultrasonic serial port", "Serial device for the ESP32 sensor head.", "str", scope="readonly"),
            _f("ULTRASONIC_SERIAL_BAUD", "Ultrasonic baud", "Serial baud rate.", "int", scope="readonly"),
            _f("ULTRASONIC_SERIAL_TIMEOUT", "Ultrasonic timeout", "Serial read timeout (seconds).", "float", scope="readonly"),
        ],
    },
]

# Flattened lookups
_FIELDS = {f["key"]: f for s in SECTIONS for f in s["fields"]}
_EDITABLE = {k for k, f in _FIELDS.items() if f["scope"] == "run"}


# ---------------------------------------------------------------------------
# Defaults / coercion
# ---------------------------------------------------------------------------

def _default(key):
    return getattr(config, key)


def defaults() -> dict:
    return {k: _default(k) for k in _FIELDS}


def _coerce(field: dict, value):
    """Coerce + validate a single value to the field's declared type."""
    t = field["type"]
    if t == "int":
        value = int(value)
    elif t == "float":
        value = float(value)
    elif t == "bool":
        value = value if isinstance(value, bool) else str(value).lower() in ("1", "true", "yes", "on")
    else:  # str
        value = str(value)
    if t in ("int", "float"):
        if "min" in field:
            value = max(field["min"], value)
        if "max" in field:
            value = min(field["max"], value)
    return value


# ---------------------------------------------------------------------------
# Persistence
# ---------------------------------------------------------------------------

def _load_overrides() -> dict:
    if not _SETTINGS_PATH.exists():
        return {}
    try:
        raw = json.loads(_SETTINGS_PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        logger.warning("Ignoring unreadable settings file %s: %s", _SETTINGS_PATH, exc)
        return {}
    clean = {}
    for k, v in (raw or {}).items():
        if k in _EDITABLE:
            try:
                clean[k] = _coerce(_FIELDS[k], v)
            except (TypeError, ValueError):
                logger.warning("Dropping invalid saved setting %s=%r", k, v)
    return clean


def _save_overrides(overrides: dict) -> None:
    try:
        _SETTINGS_PATH.write_text(
            json.dumps(overrides, indent=2, sort_keys=True), encoding="utf-8"
        )
    except OSError as exc:
        logger.error("Failed to save settings to %s: %s", _SETTINGS_PATH, exc)
        raise


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def effective() -> dict:
    """Current effective values: defaults with editable overrides applied."""
    values = defaults()
    values.update(_load_overrides())
    return values


def update(new_values: dict) -> dict:
    """Merge + persist editable overrides. Unknown / read-only keys are ignored.
    Returns the new effective values."""
    with _lock:
        overrides = _load_overrides()
        for k, v in (new_values or {}).items():
            if k not in _EDITABLE:
                continue  # silently skip read-only / unknown keys
            overrides[k] = _coerce(_FIELDS[k], v)
        # Drop overrides equal to the default so the file stays minimal.
        overrides = {k: v for k, v in overrides.items() if v != _default(k)}
        _save_overrides(overrides)
        logger.info("Settings updated: %s", ", ".join(sorted(new_values or {})) or "(none)")
    return effective()


def reset() -> dict:
    """Clear all overrides — every value returns to its config.py default."""
    with _lock:
        if _SETTINGS_PATH.exists():
            try:
                _SETTINGS_PATH.unlink()
            except OSError as exc:
                logger.error("Failed to remove settings file: %s", exc)
                _save_overrides({})
        logger.info("Settings reset to defaults")
    return effective()


def schema() -> list:
    """Sections with each field annotated with its current + default value,
    for rendering the UI."""
    eff = effective()
    out = []
    for section in SECTIONS:
        fields = []
        for f in section["fields"]:
            fields.append({**f, "value": eff[f["key"]], "default": _default(f["key"])})
        out.append({**{k: section[k] for k in ("id", "title", "desc")}, "fields": fields})
    return out


def apply_run_overrides() -> None:
    """Re-bind editable (``run`` scope) values onto the modules that read them,
    so the next auto run uses the latest settings. Called from
    AutoNavigator.start()."""
    eff = effective()
    # Imported lazily to avoid an import cycle (automatic imports this module).
    from components import automatic as _automatic
    from components import ultrasonic as _ultrasonic
    targets = (_automatic, _ultrasonic)
    applied = []
    for key in _EDITABLE:
        value = eff[key]
        for mod in targets:
            if hasattr(mod, key):
                setattr(mod, key, value)
                applied.append(key)
    logger.info("Applied %d navigation setting(s) for this run", len(applied))
