"""
Central Log Messages
====================
Single source of truth for INFO-level log text, grouped by component.

Each enum member's ``.value`` is the (printf-style) message used with the
existing logging args, e.g.:

    from components.logs import MotorLog
    logger.info(MotorLog.BACKWARD.value, speed)        # "Backward speed=%.2f"
    logger.info(MotorLog.STOP.value)                   # "Stop"

Only INFO logs are centralised here; warning/error/debug stay inline as
one-off diagnostics. Edit message text HERE.
"""

from enum import Enum


class AppLog(Enum):
    INITIALISING       = "Initialising AgriBot components …"
    READY              = "All components ready ✔"
    SHUTTING_DOWN      = "Shutting down AgriBot components …"
    SHUTDOWN_COMPLETE  = "Shutdown complete"


class MotorLog(Enum):
    INITIALIZED = "MotorController initialized with 4 Enables and Trim."
    BACKWARD    = "Backward speed=%.2f"
    FORWARD     = "Forward speed=%.2f"
    LEFT        = "Left speed=%.2f"
    RIGHT       = "Right speed=%.2f"
    STOP        = "Stop"
    CLOSED      = "MotorController closed"


class PumpLog(Enum):
    INITIALISED = "WaterPumpController initialised  (GPIO%d, active_high=%s)"
    ON          = "Water pump ON"
    OFF         = "Water pump OFF"
    CLOSED      = "WaterPumpController closed"


class CameraLog(Enum):
    INITIALISED = "RobotCamera initialised  (source=%s)"
    CLOSED      = "RobotCamera closed"


class PtzLog(Enum):
    INITIALISED = "CameraPTZController initialised  (host=%s)"
    REACHABLE   = "CameraPTZController reachable — PTZ enabled"
    CLOSED      = "CameraPTZController closed"


class AiLog(Enum):
    LOADING_MODEL      = "Loading YOLO model from %s …"
    MODEL_LOADED       = "YOLO model loaded successfully"
    TARGET_CLASS_FOUND = "Target class '%s' found with ID: %d"
    CLOSED             = "PlantDetector closed"


class ReidLog(Enum):
    INITIALISED      = "PlantReID initialised on %s (threshold=%.2f)"
    CLUSTER_STORED   = "PlantReID: plant cluster stored (%d views, total plants=%d)"
    CLUSTER_EXTENDED = "PlantReID: +%d views to last cluster (size=%d, plants=%d)"


class UltrasonicLog(Enum):
    INITIALISED = ("UltrasonicSensor initialised "
                   "(trig=GPIO%d, echo=GPIO%d, max=%dcm)")
    CLOSED      = "UltrasonicSensor closed"


class LedLog(Enum):
    INITIALISED = "SAQI_LEDS initialised (GPIO%d, %d LEDs)"


class NavLog(Enum):
    INFER_LOOP_STARTED      = "AutoNavigator inference loop started"
    INFER_LOOP_EXITED       = "AutoNavigator inference loop exited"
    STARTING                = "AutoNavigator starting..."
    STOPPING                = "AutoNavigator stopping..."
    PLANT_CENTERED          = "Auto: plant centered — resuming normal speed"
    ZONE_STABILISED         = "Auto: zone stabilised (%s) — resuming normal speed"
    OSCILLATION             = "Auto: oscillation detected (%s→%s→%s) — using gentle speed"
    REID_MATCH_TAG          = "Auto: ReID matched watered plant — tagging id=%d"
    ARRIVED                 = "Auto: ARRIVED at plant (id=%s) — watering..."
    WATERING_PROGRESS       = "Auto: watering... (%d/%d s)"
    MARKED_WATERED          = "Auto: plant id=%d marked as WATERED (total=%d)"
    WATERING_COMPLETE_NO_ID = "Auto: watering complete — no track ID to remember"
    REID_CAPTURED           = "Auto: captured %d ReID embeddings for watered plant"
    RETREATING              = "Auto: retreating for %.1fs to look for next plant..."
    RETREAT_NEW_PLANT       = "Auto: new unwatered plant spotted during retreat (id=%s)"
    RETREAT_REID_VIEW       = "Auto: +1 retreat ReID view (id=%s)"
    SCAN_TARGET_ACQUIRED    = "Auto: target acquired after scan — stopping to re-evaluate"
    LOOP_EXITED             = "AutoNavigator loop exited — cleanup complete"
