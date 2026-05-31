"""FastAPI control panel for the SAQI robot backend."""

import logging
import socket
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app_state import app_state
from components.ai import PlantDetector
from components.automatic import AutoNavigator
from components.camera import RobotCamera
from components.camera_control import CameraPTZController
from components.leds import LEDState, SAQI_LEDS
from components.logs import AppLog
from components.motor import MotorController
from components.reid import PlantReID
from components.ultrasonic import UltrasonicSensor
from components.waterpump import WaterPumpController
from routes import ai, camera, motor, ptz, pump, root, settings, ultrasonic

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(name)-24s  %(levelname)-8s  %(message)s",
)
logger = logging.getLogger(__name__)


def _wifi_ok() -> bool:
    """Best-effort network reachability check for the WIFI LED."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0)
        sock.connect(("8.8.8.8", 53))
        sock.close()
        return True
    except OSError:
        return False


def _camera_has_frame(timeout_seconds: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if app_state.camera is not None and app_state.camera.get_frame() is not None:
            return True
        time.sleep(0.5)
    return False


def _initialize_components() -> None:
    app_state.motor = MotorController()
    app_state.camera = RobotCamera()
    app_state.pump = WaterPumpController()
    app_state.ai = PlantDetector()

    try:
        app_state.ptz = CameraPTZController()
    except Exception as exc:  # noqa: BLE001
        logger.warning("PTZ camera control unavailable: %s", exc)
        app_state.ptz = None

    app_state.reid = PlantReID()
    app_state.ultrasonic = UltrasonicSensor()
    app_state.ultrasonic.start()
    app_state.navigator = AutoNavigator(
        app_state.motor,
        app_state.pump,
        app_state.camera,
        app_state.ai,
        ptz=app_state.ptz,
        reid=app_state.reid,
        ultrasonic=app_state.ultrasonic,
    )


def _shutdown_components() -> None:
    if app_state.navigator is not None:
        app_state.navigator.stop()
    if app_state.ultrasonic is not None:
        app_state.ultrasonic.close()
    if app_state.motor is not None:
        app_state.motor.close()
    if app_state.camera is not None:
        app_state.camera.close()
    if app_state.pump is not None:
        app_state.pump.close()
    if app_state.ai is not None:
        app_state.ai.close()
    if app_state.ptz is not None:
        app_state.ptz.close()


@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info(AppLog.INITIALISING.value)

    SAQI_LEDS.initialize()
    SAQI_LEDS.startup_animation()
    SAQI_LEDS.mode(LEDState.MANUAL)
    SAQI_LEDS.wifi(LEDState.INITIALIZE)
    SAQI_LEDS.camera(LEDState.INITIALIZE)

    try:
        _initialize_components()
        SAQI_LEDS.wifi(LEDState.READY if _wifi_ok() else LEDState.ERROR)
        SAQI_LEDS.camera(LEDState.READY if _camera_has_frame() else LEDState.ERROR)
        logger.info(AppLog.READY.value)
    except Exception as exc:  # noqa: BLE001
        logger.exception("Component initialisation failed: %s", exc)
        try:
            _shutdown_components()
        except Exception:  # noqa: BLE001
            logger.exception("Cleanup after failed initialisation also failed")
        SAQI_LEDS.error(True)
        raise

    yield

    logger.info(AppLog.SHUTTING_DOWN.value)
    _shutdown_components()
    SAQI_LEDS.all_off()
    SAQI_LEDS.close()
    logger.info(AppLog.SHUTDOWN_COMPLETE.value)


app = FastAPI(title="SAQI", version="1.0.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(root.router)
app.include_router(motor.router)
app.include_router(camera.router)
app.include_router(ptz.router)
app.include_router(pump.router)
app.include_router(ai.router)
app.include_router(settings.router)
app.include_router(ultrasonic.router)
