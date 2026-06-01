"""
Central log messages.

INFO-level messages are grouped here so component code stays focused on logic.
"""

from enum import Enum


class AppLog(Enum):
    INITIALISING = "Initialising AgriBot components..."
    READY = "All components ready"
    SHUTTING_DOWN = "Shutting down AgriBot components..."
    SHUTDOWN_COMPLETE = "Shutdown complete"


class MotorLog(Enum):
    INITIALIZED = "MotorController initialized with 4 enables and trim"
    BACKWARD = "Backward speed=%.2f"
    FORWARD = "Forward speed=%.2f"
    LEFT = "Left speed=%.2f"
    RIGHT = "Right speed=%.2f"
    STOP = "Stop"
    CLOSED = "MotorController closed"


class PumpLog(Enum):
    INITIALISED = "WaterPumpController initialised (GPIO%d, active_high=%s)"
    ON = "Water pump ON"
    OFF = "Water pump OFF"
    CLOSED = "WaterPumpController closed"


class BuzzerLog(Enum):
    INITIALISED = "BuzzerController initialised (GPIO%d)"
    DISABLED = "BuzzerController disabled (%s) - system continues without buzzer audio"
    PUMP_ACTIVE = "Buzzer pump cue active=%s"
    WATER_SONG = "Buzzer watering song: %s"
    MODE_CHANGED = "Buzzer mode cue: %s"
    COLLISION_ALARM = "Buzzer collision alarm: front=%.1f cm, sounding for %.1fs"
    COLLISION_SILENCED = "Buzzer collision alarm timed out; quiet until distance reset"
    WRITE_FAILED = "Buzzer GPIO write failed (%s); disabling buzzer audio"
    CLOSED = "BuzzerController closed"


class CameraLog(Enum):
    INITIALISED = "RobotCamera initialised (source=%s)"
    CLOSED = "RobotCamera closed"


class PtzLog(Enum):
    INITIALISED = "CameraPTZController initialised (host=%s)"
    REACHABLE = "CameraPTZController reachable; PTZ enabled"
    CLOSED = "CameraPTZController closed"


class AiLog(Enum):
    LOADING_MODEL = "Loading YOLO model from %s..."
    MODEL_LOADED = "YOLO model loaded successfully"
    TARGET_CLASS_FOUND = "Target class '%s' found with ID: %d"
    CLOSED = "PlantDetector closed"


class ReidLog(Enum):
    INITIALISED = "PlantReID initialised on %s (threshold=%.2f)"
    CLUSTER_STORED = "PlantReID: plant cluster stored (%d views, total plants=%d)"
    CLUSTER_EXTENDED = "PlantReID: +%d views to last cluster (size=%d, plants=%d)"


class UltrasonicLog(Enum):
    INITIALISED = "Ultrasonic ESP32 initialised (%s)"
    CLOSED = "UltrasonicSensor closed"


class LedLog(Enum):
    INITIALISED = "SAQI_LEDS initialised (GPIO%d, %d LEDs)"


class NavLog(Enum):
    INFER_LOOP_STARTED = "AutoNavigator inference loop started"
    INFER_LOOP_EXITED = "AutoNavigator inference loop exited"
    STARTING = "AutoNavigator starting..."
    STOPPING = "AutoNavigator stopping..."
    PLANT_CENTERED = "Auto: plant centered; resuming normal speed"
    ZONE_STABILISED = "Auto: zone stabilised (%s); resuming normal speed"
    OSCILLATION = "Auto: oscillation detected (%s -> %s -> %s); using gentle speed"
    REID_MATCH_TAG = "Auto: ReID matched watered plant; tagging id=%d"
    ARRIVED = "Auto: ARRIVED at plant (id=%s); watering..."
    WATERING_PROGRESS = "Auto: watering... (%d/%d s)"
    MARKED_WATERED = "Auto: plant id=%d marked as WATERED (total=%d)"
    WATERING_COMPLETE_NO_ID = "Auto: watering complete; no track ID to remember"
    REID_CAPTURED = "Auto: captured %d ReID embeddings for watered plant"
    RETREATING = "Auto: retreating for %.1fs to look for next plant..."
    RETREAT_NEW_PLANT = "Auto: new unwatered plant spotted during retreat (id=%s)"
    RETREAT_REID_VIEW = "Auto: +1 retreat ReID view (id=%s)"
    SCAN_TARGET_ACQUIRED = "Auto: target acquired after scan; stopping to re-evaluate"
    OBSTACLE_AVOIDANCE = "Auto: obstacle detected; avoiding to the %s"
    LOOP_EXITED = "AutoNavigator loop exited; cleanup complete"
    CAMERA_STALE = "Auto: no camera frames for %.1fs — holding (motors stopped) until the feed returns"
    CAMERA_BACK = "Auto: camera frames resumed — navigation continuing"
    # --- Return-to-base mission ---
    WATERING_COUNT = "Auto: watering event %d of %d complete"
    ALL_PLANTS_WATERED = "Auto: all %d plant(s) watered — returning to base (QR '%s')"
    RETURN_SCANNING = "Auto: base not in view; scanning %s for base QR '%s'"
    BASE_DETECTED = "Auto: base QR '%s' detected; zone=%s area=%.2f"
    BASE_TURN = "Auto: base %s -> turning %s (speed=%.2f, %.1fs)"
    BASE_APPROACH = "Auto: base CENTER -> approaching (area=%.2f/%.2f)"
    ARRIVED_BASE = "Auto: ARRIVED at base QR '%s' (area=%.2f) — mission complete"
