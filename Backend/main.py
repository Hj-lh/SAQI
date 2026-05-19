"""
AgriBot – FastAPI Main Application
====================================
Central server that exposes REST + streaming endpoints
for motor control, water pump, camera feed, and AI detection.

Run with:  uvicorn main:app --host 0.0.0.0 --port 8000
"""

import logging
import socket
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse

from components.motor import MotorController
from components.camera import RobotCamera
from components.camera_control import CameraPTZController
from components.waterpump import WaterPumpController
from components.ai import PlantDetector
from components.reid import PlantReID
from components.automatic import AutoNavigator
from components.ultrasonic import UltrasonicSensor
from components.leds import SAQI_LEDS, LEDState


def _wifi_ok() -> bool:
    """Best-effort LAN/internet reachability check for the WIFI LED."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1.0)
        s.connect(("8.8.8.8", 53))  # no packets sent; just resolves a route
        s.close()
        return True
    except OSError:
        return False

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(name)-24s  %(levelname)-8s  %(message)s",
)
from components.logs import AppLog
logger = logging.getLogger(__name__)

# ------------------------------------------------------------------
# System state — populated on startup, cleaned up on shutdown
# ------------------------------------------------------------------
system: dict = {}


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialise hardware on startup, release on shutdown."""
    logger.info(AppLog.INITIALISING.value)

    # Status LEDs first so the rest of init can be reflected on the panel.
    SAQI_LEDS.initialize()
    SAQI_LEDS.startup_animation()
    SAQI_LEDS.mode(LEDState.MANUAL)        # navigator starts in manual
    SAQI_LEDS.wifi(LEDState.INITIALIZE)
    SAQI_LEDS.camera(LEDState.INITIALIZE)

    try:
        system["motor"] = MotorController()
        system["camera"] = RobotCamera()
        system["pump"] = WaterPumpController()
        system["ai"] = PlantDetector()
        try:
            system["ptz"] = CameraPTZController()
        except Exception as e:  # noqa: BLE001
            logger.warning("PTZ camera control unavailable: %s", e)
            system["ptz"] = None
        system["reid"] = PlantReID()
        system["navigator"] = AutoNavigator(
            system["motor"],
            system["pump"],
            system["camera"],
            system["ai"],
            ptz=system["ptz"],
            reid=system["reid"],
        )
        system["ultrasonic"] = UltrasonicSensor()
        system["ultrasonic"].start()

        # WIFI LED — best-effort reachability
        SAQI_LEDS.wifi(LEDState.READY if _wifi_ok() else LEDState.ERROR)

        # CAMERA LED — give the capture thread a moment, then verify a frame
        cam_ok = False
        for _ in range(10):
            if system["camera"].get_frame() is not None:
                cam_ok = True
                break
            time.sleep(0.5)
        SAQI_LEDS.camera(LEDState.READY if cam_ok else LEDState.ERROR)

        logger.info(AppLog.READY.value)
    except Exception as e:  # noqa: BLE001
        logger.exception("Component initialisation failed: %s", e)
        SAQI_LEDS.error(True)
        raise

    yield  # ← app is running

    logger.info(AppLog.SHUTTING_DOWN.value)
    if system.get("navigator") is not None:
        system["navigator"].stop()
    if system.get("ultrasonic") is not None:
        system["ultrasonic"].close()
    if system.get("motor") is not None:
        system["motor"].close()
    if system.get("camera") is not None:
        system["camera"].close()
    if system.get("pump") is not None:
        system["pump"].close()
    if system.get("ai") is not None:
        system["ai"].close()
    if system.get("ptz") is not None:
        system["ptz"].close()
    SAQI_LEDS.all_off()
    SAQI_LEDS.close()
    logger.info(AppLog.SHUTDOWN_COMPLETE.value)


app = FastAPI(title="AgriBot API", version="1.0.0", lifespan=lifespan)

# Allow the frontend to connect from any origin
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ==================================================================
# Root
# ==================================================================

@app.get("/")
def index():
    return {
        "message": "AgriBot System Online",
        "endpoints": {
            "camera": ["/camera/feed", "/camera/snapshot"],
            "motor": [
                "/motor/move?direction=forward&speed=0.8",
                "/motor/stop",
            ],
            "pump": ["/pump/on", "/pump/off", "/pump/status"],
            "ai": ["/ai/detect"],
            "ptz": [
                "/camera/ptz/pan?angle=45",
                "/camera/ptz/tilt?angle=-10",
                "/camera/ptz/zoom?level=2000",
                "/camera/ptz/move?direction=left&speed=50",
                "/camera/ptz/look?direction=right&degrees=90",
                "/camera/ptz/center",
                "/camera/ptz/home",
                "/camera/ptz/stop",
                "/camera/ptz/position",
            ],
        },
    }


# ==================================================================
# Motor
# ==================================================================

@app.post("/motor/move")
def move_robot(
    direction: str = Query(..., description="forward | backward | left | right"),
    speed: float = Query(0.8, ge=0.0, le=1.0, description="Speed 0.0 – 1.0"),
):
    """Drive the robot in the given direction at the given speed."""
    motor: MotorController = system["motor"]

    actions = {
        "forward": motor.forward,
        "backward": motor.backward,
        "left": motor.left,
        "right": motor.right,
    }

    action = actions.get(direction)
    if action is None:
        return JSONResponse(
            status_code=400,
            content={"status": "error", "message": f"Invalid direction: {direction}"},
        )

    action(speed)
    return {"status": "moving", "direction": direction, "speed": speed}


@app.post("/motor/stop")
def stop_robot():
    """Stop all motors immediately."""
    system["motor"].stop()
    return {"status": "stopped"}


# ==================================================================
# Camera
# ==================================================================

def _mjpeg_generator(mode: str = "manual"):
    """Yield JPEG frames as an MJPEG stream.

    This is a pure viewer: YOLO inference + navigation run in their own
    background threads inside ``AutoNavigator`` (started by /camera/feed
    with mode=automatic) and keep running even with no client connected.
    In 'automatic' mode we stream the navigator's annotated frame; in
    'manual' mode we stream the raw camera frame.
    """
    camera: RobotCamera = system["camera"]
    navigator = system["navigator"]
    use_ai = mode == "automatic"

    while True:
        if use_ai:
            frame = navigator.get_annotated_jpeg()
            if frame is None:
                # Inference still warming up — show raw camera meanwhile
                frame = camera.get_frame()
        else:
            frame = camera.get_frame()

        if frame is None:
            time.sleep(0.03)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )
        time.sleep(0.03)


@app.get("/camera/feed")
def video_feed(
    mode: str = Query("manual", description="manual | automatic"),
):
    """Live MJPEG video stream. Use mode=automatic for AI detection overlay."""
    if mode == "automatic":
        system["navigator"].start()
    else:
        system["navigator"].stop()

    return StreamingResponse(
        _mjpeg_generator(mode),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/camera/snapshot")
def snapshot():
    """Return a single JPEG frame."""
    frame = system["camera"].get_frame()
    if frame is None:
        return JSONResponse(
            status_code=503,
            content={"status": "error", "message": "No frame available"},
        )
    return StreamingResponse(iter([frame]), media_type="image/jpeg")


# ==================================================================
# Water Pump
# ==================================================================

@app.post("/pump/on")
def pump_on():
    """Turn the water pump ON."""
    system["pump"].on()
    return {"status": "pump_on"}


@app.post("/pump/off")
def pump_off():
    """Turn the water pump OFF."""
    system["pump"].off()
    return {"status": "pump_off"}


@app.get("/pump/status")
def pump_status():
    """Check whether the pump is currently running."""
    return {"is_on": system["pump"].is_on}


# ==================================================================
# AI Detection
# ==================================================================

@app.get("/ai/detect")
def ai_detect():
    """
    Grab the latest camera frame, run YOLO detection, and
    return the list of detected objects.
    """
    detector: PlantDetector = system["ai"]

    if not detector.enabled:
        return JSONResponse(
            status_code=503,
            content={"status": "error", "message": "AI model not loaded"},
        )

    frame = system["camera"].get_raw_frame()
    if frame is None:
        return JSONResponse(
            status_code=503,
            content={"status": "error", "message": "No camera frame available"},
        )

    detections = detector.detect(frame)
    return {"status": "ok", "count": len(detections), "detections": detections}


# ==================================================================
# Camera PTZ (pan / tilt / zoom)
# ==================================================================

def _ptz_or_503():
    ptz: CameraPTZController | None = system.get("ptz")
    if ptz is None or not ptz.ensure_enabled():
        return None, JSONResponse(
            status_code=503,
            content={"status": "error", "message": "PTZ camera control not available"},
        )
    return ptz, None


@app.post("/camera/ptz/pan")
def ptz_pan(
    angle: float = Query(..., ge=-180.0, le=180.0, description="Absolute pan in degrees"),
):
    """Pan the camera to an absolute horizontal angle (-180..180 degrees).

    Frontend usage:
        POST /camera/ptz/pan?angle=45     → pan right to 45°
        POST /camera/ptz/pan?angle=-90    → pan left to -90°
        POST /camera/ptz/pan?angle=0      → center the pan axis

    NOTE: this is an ABSOLUTE position. Re-sending the same angle is a
    no-op — the camera is already there, so it won't move and won't error.
    For interactive / hold-to-move control use /camera/ptz/move instead.

    Response: {"status": "ok"|"error", "pan": <angle>}
    Returns 503 if the PTZ camera is unreachable.
    """
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ok = ptz.pan(angle)
    return {"status": "ok" if ok else "error", "pan": angle}


@app.post("/camera/ptz/tilt")
def ptz_tilt(
    angle: float = Query(..., ge=-90.0, le=90.0, description="Absolute tilt in degrees"),
):
    """Tilt the camera to an absolute vertical angle (-90..90 degrees).

    Frontend usage:
        POST /camera/ptz/tilt?angle=20    → tilt up to 20°
        POST /camera/ptz/tilt?angle=-20   → tilt down to -20°
        POST /camera/ptz/tilt?angle=0     → level the camera

    NOTE: this is an ABSOLUTE position. Re-sending the same angle is a
    no-op — the camera is already there, so it won't move and won't error.
    For interactive / hold-to-move control use /camera/ptz/move instead.

    Response: {"status": "ok"|"error", "tilt": <angle>}
    Returns 503 if the PTZ camera is unreachable.
    """
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ok = ptz.tilt(angle)
    return {"status": "ok" if ok else "error", "tilt": angle}


@app.post("/camera/ptz/zoom")
def ptz_zoom(
    level: int = Query(..., ge=1, le=9999, description="Absolute zoom 1..9999"),
):
    """Set absolute zoom level (1 = widest, 9999 = maximum telephoto).

    Frontend usage:
        POST /camera/ptz/zoom?level=1     → full wide-angle
        POST /camera/ptz/zoom?level=2000  → moderate zoom (good default)
        POST /camera/ptz/zoom?level=9999  → maximum telephoto

    NOTE: this is an ABSOLUTE level. Re-sending the same level is a no-op
    — the lens is already there, so nothing happens and no error is
    returned. Send a different level (e.g. drive it from a slider).

    Response: {"status": "ok"|"error", "zoom": <level>}
    Returns 503 if the PTZ camera is unreachable.
    """
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ok = ptz.zoom(level)
    return {"status": "ok" if ok else "error", "zoom": level}


@app.post("/camera/ptz/move")
def ptz_move(
    direction: str = Query(..., description="left | right | up | down"),
    speed: int = Query(50, ge=1, le=100, description="Move speed 1..100"),
):
    """Start continuous (hold-to-move) pan/tilt in one of four directions.

    The camera keeps moving at `speed` until you call /camera/ptz/stop.
    Bind this to an arrow button's press and /camera/ptz/stop to its
    release (mouseup / touchend / mouseleave). Because this is
    velocity-based — NOT an absolute position — pressing the same arrow
    repeatedly always moves the camera (unlike pan/tilt/look/zoom, which
    are absolute and do nothing if already at that value).

    Frontend usage:
        POST /camera/ptz/move?direction=left&speed=50   → pan left  while held
        POST /camera/ptz/move?direction=right&speed=50  → pan right while held
        POST /camera/ptz/move?direction=up&speed=40     → tilt up   while held
        POST /camera/ptz/move?direction=down&speed=40   → tilt down while held
        POST /camera/ptz/stop                           → on release

    Response: {"status": "ok"|"error", "direction": ..., "speed": ...}
    Returns 400 for an invalid direction, 503 if the PTZ camera is unreachable.
    """
    ptz, err = _ptz_or_503()
    if err is not None:
        return err

    vectors = {
        "left":  (-speed, 0),
        "right": ( speed, 0),
        "up":    (0,  speed),
        "down":  (0, -speed),
    }
    vec = vectors.get(direction)
    if vec is None:
        return JSONResponse(
            status_code=400,
            content={"status": "error",
                     "message": f"Invalid direction: {direction} "
                                f"(use left|right|up|down)"},
        )

    ok = ptz.move_continuous(vec[0], vec[1])
    return {"status": "ok" if ok else "error",
            "direction": direction, "speed": speed}


@app.post("/camera/ptz/stop")
def ptz_stop():
    """Stop any continuous PTZ motion started by /camera/ptz/move.

    Frontend usage:
        POST /camera/ptz/stop    → call this on mouseup / touchend to stop the camera

    Response: {"status": "ok"|"error"}
    Returns 503 if the PTZ camera is unreachable.
    """
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ok = ptz.stop_movement()
    return {"status": "ok" if ok else "error"}


@app.post("/camera/ptz/look")
def ptz_look(
    direction: str = Query(..., description="left | right | up | down | center"),
    degrees: float = Query(90.0, ge=0.0, le=180.0),
):
    """Snap the camera to a preset direction at a given angle offset.

    This is the easiest endpoint for UI buttons like "Look Left" or "Look Right".
    It moves to an absolute position (not relative to the current position).

    Frontend usage:
        POST /camera/ptz/look?direction=right&degrees=90  → snap to 90° right
        POST /camera/ptz/look?direction=left&degrees=45   → snap to 45° left
        POST /camera/ptz/look?direction=up&degrees=20     → tilt up 20°
        POST /camera/ptz/look?direction=down&degrees=20   → tilt down 20°
        POST /camera/ptz/look?direction=center            → return to center (degrees ignored)

    NOTE: this is an ABSOLUTE move. Re-sending the same direction+degrees
    is a no-op — the camera is already at that position, so it won't move
    and won't error (this is expected, not a bug). For repeatable /
    hold-to-move arrow control use /camera/ptz/move instead.

    Response: {"status": "ok"|"error", "direction": ..., "degrees": ...}
    Returns 400 for unknown direction, 503 if PTZ unreachable.
    """
    ptz, err = _ptz_or_503()
    if err is not None:
        return err

    actions = {
        "left":   lambda: ptz.look_left(degrees),
        "right":  lambda: ptz.look_right(degrees),
        "up":     lambda: ptz.look_up(degrees),
        "down":   lambda: ptz.look_down(degrees),
        "center": ptz.look_center,
    }
    action = actions.get(direction)
    if action is None:
        return JSONResponse(
            status_code=400,
            content={"status": "error", "message": f"Invalid direction: {direction}"},
        )
    ok = action()
    return {"status": "ok" if ok else "error", "direction": direction, "degrees": degrees}


@app.post("/camera/ptz/center")
def ptz_center():
    """Recenter the camera: stop any motion, then go to pan 0° AND tilt 0°
    (level, facing straight forward).

    Frontend usage:
        POST /camera/ptz/center    → "reset view" button

    First sends a continuous-move stop (in case an arrow is still held /
    motion is in progress), then an ABSOLUTE move to pan 0 + tilt 0. Zoom
    is left unchanged. If the camera is already centered the absolute move
    is a harmless no-op (no movement, no error) — that's expected.

    Response: {"status": "ok"|"error"}
    Returns 503 if the PTZ camera is unreachable.
    """
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ptz.stop_movement()          # halt any in-progress continuous motion first
    ok = ptz.look_center()       # pan 0 + tilt 0
    return {"status": "ok" if ok else "error"}


@app.post("/camera/ptz/home")
def ptz_home():
    """Move the camera to its factory/user-configured home preset position.

    The home position is set in the AXIS camera's own web UI, not in this API.
    Use this as a "safe starting position" button in the frontend.

    Frontend usage:
        POST /camera/ptz/home    → reset to home position

    Response: {"status": "ok"|"error"}
    Returns 503 if the PTZ camera is unreachable.
    """
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ok = ptz.home()
    return {"status": "ok" if ok else "error"}


@app.post("/camera/ptz/preset")
def ptz_preset(
    name: str | None = Query(None, description="Preset name configured in the camera"),
    number: int | None = Query(None, ge=1, description="Preset index (1-based)"),
):
    """Recall a saved preset position from the camera's internal preset list.

    Presets are configured in the AXIS camera web UI, not in this API.
    Provide either 'name' or 'number' — not both.

    Frontend usage:
        POST /camera/ptz/preset?name=field_overview   → go to the "field_overview" preset
        POST /camera/ptz/preset?number=1              → go to preset #1

    Response: {"status": "ok"|"error", "name": ..., "number": ...}
    Returns 400 if neither name nor number is supplied.
    Returns 503 if the PTZ camera is unreachable.
    """
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    if name is None and number is None:
        return JSONResponse(
            status_code=400,
            content={"status": "error", "message": "Provide either 'name' or 'number'"},
        )
    ok = ptz.goto_preset(name) if name is not None else ptz.goto_preset_number(number)
    return {"status": "ok" if ok else "error", "name": name, "number": number}


@app.get("/camera/ptz/position")
def ptz_position():
    """Return the current pan/tilt/zoom values as reported by the camera hardware.

    Frontend usage:
        GET /camera/ptz/position    → poll this to update a position indicator or slider UI

    Response: {"status": "ok", "position": {"pan": 45.0, "tilt": 0.0, "zoom": 2000.0, ...}}
    Returns 502 if the camera responded but returned no data.
    Returns 503 if the PTZ camera is unreachable.
    """
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    data = ptz.get_position()
    if data is None:
        return JSONResponse(
            status_code=502,
            content={"status": "error", "message": "Could not query PTZ position"},
        )
    return {"status": "ok", "position": data}
