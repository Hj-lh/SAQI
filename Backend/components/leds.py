"""
========================================================
SAQI LED Controller
========================================================
WS2812 / NeoPixel 8-LED status panel.

Adapted to project conventions:
  - No hardware work at import. Call SAQI_LEDS.initialize() once
    (done by main.py lifespan). If the NeoPixel libs / hardware /
    privileges are unavailable, the panel disables itself and every
    method becomes a safe no-op — the rest of the robot is unaffected
    (same graceful pattern as camera_control.py / reid.py).
  - Pin / count / brightness come from components.config (SSOT).

Hardware: Raspberry Pi + WS2812 on GPIO18 (PWM).
Deps:     pip install rpi_ws281x adafruit-circuitpython-neopixel
Note:     rpi_ws281x on GPIO18 typically requires running as root.

LED ORDER
  1 Manual/Auto   2 Wifi   3 Camera   4 Ultrasonic
  5 Moving        6 Detect 7 Watering 8 Error

USAGE
  from components.leds import SAQI_LEDS, LEDState
  SAQI_LEDS.initialize()
  SAQI_LEDS.mode(LEDState.AUTO)
  SAQI_LEDS.wifi(LEDState.READY)
  SAQI_LEDS.ultrasonic(True)
  SAQI_LEDS.error(True)
========================================================
"""

import time
import logging
import threading
from enum import Enum

from components.config import LED_PIN, LED_COUNT, LED_BRIGHTNESS

from components.logs import LedLog
logger = logging.getLogger(__name__)


# ========================================================
# LED STATES
# ========================================================

class LEDState(Enum):

    # General
    OFF = "off"
    ON = "on"

    # Initialize
    INITIALIZE = "initialize"

    # Mode
    MANUAL = "manual"
    AUTO = "auto"

    # Systems
    READY = "ready"
    ERROR = "error"


# ========================================================
# SAQI LED CONTROLLER
# ========================================================

class SAQI_LEDS:

    # ====================================================
    # CONFIGURATION (from components.config — SSOT)
    # ====================================================

    LED_COUNT = LED_COUNT
    BRIGHTNESS = LED_BRIGHTNESS
    AUTO_WRITE = True

    BLINK_INTERVAL = 0.5
    ERROR_BLINK_INTERVAL = 0.3

    # ====================================================
    # LED INDEXES
    # ====================================================

    MANUAL_AUTO = 0
    WIFI = 1
    CAMERA = 2
    ULTRASONIC = 3
    MOVING = 4
    DETECT = 5
    WATERING = 6
    ERROR_LED = 7

    # ====================================================
    # COLORS
    # ====================================================

    OFF = (0, 0, 0)

    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 0, 255)
    ORANGE = (255, 80, 0)

    # ====================================================
    # HARDWARE STATE (lazy — see initialize())
    # ====================================================

    enabled = False
    pixels = None

    # ====================================================
    # THREADING
    # ====================================================

    _pixel_lock = threading.Lock()

    _blinking_threads = {}
    _blinking_flags = {}

    # ====================================================
    # INITIALIZATION (call once at startup)
    # ====================================================

    @classmethod
    def initialize(cls):
        """Bring up the NeoPixel hardware. Safe to call once; degrades
        gracefully (enabled stays False) if libs/hardware/root missing."""
        if cls.enabled:
            return
        try:
            import board
            import neopixel

            pin = getattr(board, f"D{LED_PIN}")
            cls.pixels = neopixel.NeoPixel(
                pin,
                cls.LED_COUNT,
                brightness=cls.BRIGHTNESS,
                auto_write=cls.AUTO_WRITE,
            )
            cls.enabled = True
            logger.info(LedLog.INITIALISED.value,
                        LED_PIN, cls.LED_COUNT)
        except Exception as e:  # noqa: BLE001
            logger.warning(
                "SAQI_LEDS disabled (%s) — LED panel unavailable, "
                "system continues without it", e
            )

    # ====================================================
    # INTERNAL METHODS
    # ====================================================

    @classmethod
    def _set_led(cls, index: int, color: tuple):
        if not cls.enabled:
            return
        with cls._pixel_lock:
            cls.pixels[index] = color

    @classmethod
    def _turn_off(cls, index: int):
        if not cls.enabled:
            return
        with cls._pixel_lock:
            cls.pixels[index] = cls.OFF

    @classmethod
    def _stop_blink(cls, index: int):

        if index in cls._blinking_flags:
            cls._blinking_flags[index] = False

    @classmethod
    def _blink(
        cls,
        index: int,
        color: tuple,
        interval: float = 0.5
    ):
        if not cls.enabled:
            return

        cls._stop_blink(index)

        cls._blinking_flags[index] = True

        def blink_loop():

            while cls._blinking_flags.get(index):

                cls._set_led(index, color)
                time.sleep(interval)

                cls._turn_off(index)
                time.sleep(interval)

        thread = threading.Thread(
            target=blink_loop,
            daemon=True
        )

        cls._blinking_threads[index] = thread
        thread.start()

    @classmethod
    def _validate_state(
        cls,
        led_name: str,
        state: LEDState,
        allowed_states: set
    ):

        if state not in allowed_states:

            allowed = ", ".join(
                [s.name for s in allowed_states]
            )

            raise ValueError(
                f"[SAQI_LEDS] Invalid state '{state.name}' "
                f"for '{led_name}'. "
                f"Allowed states: {allowed}"
            )

    # ====================================================
    # GLOBAL METHODS
    # ====================================================

    @classmethod
    def all_off(cls):

        for i in range(cls.LED_COUNT):

            cls._stop_blink(i)
            cls._turn_off(i)

    @classmethod
    def close(cls):
        """Stop all blink threads and clear the panel (lifespan shutdown)."""
        cls.all_off()

    @classmethod
    def initialize_all(cls):

        cls.mode(LEDState.INITIALIZE)

        cls.wifi(LEDState.INITIALIZE)

        cls.camera(LEDState.INITIALIZE)

    @classmethod
    def startup_animation(cls):
        if not cls.enabled:
            return

        cls.all_off()

        for i in range(cls.LED_COUNT):

            cls._set_led(i, cls.BLUE)
            time.sleep(0.08)

        for i in range(cls.LED_COUNT):

            cls._turn_off(i)
            time.sleep(0.05)

    # ====================================================
    # 1. MANUAL / AUTO   (INITIALIZE | MANUAL | AUTO)
    # ====================================================

    @classmethod
    def mode(cls, state: LEDState):

        allowed_states = {
            LEDState.INITIALIZE,
            LEDState.MANUAL,
            LEDState.AUTO
        }

        cls._validate_state(
            "MODE",
            state,
            allowed_states
        )

        cls._stop_blink(cls.MANUAL_AUTO)

        if state == LEDState.INITIALIZE:

            cls._blink(
                cls.MANUAL_AUTO,
                cls.ORANGE,
                cls.BLINK_INTERVAL
            )

        elif state == LEDState.MANUAL:

            cls._set_led(
                cls.MANUAL_AUTO,
                cls.RED
            )

        elif state == LEDState.AUTO:

            cls._set_led(
                cls.MANUAL_AUTO,
                cls.GREEN
            )

    # ====================================================
    # 2. WIFI   (INITIALIZE | READY | ERROR)
    # ====================================================

    @classmethod
    def wifi(cls, state: LEDState):

        allowed_states = {
            LEDState.INITIALIZE,
            LEDState.READY,
            LEDState.ERROR
        }

        cls._validate_state(
            "WIFI",
            state,
            allowed_states
        )

        cls._stop_blink(cls.WIFI)

        if state == LEDState.INITIALIZE:

            cls._blink(
                cls.WIFI,
                cls.ORANGE,
                cls.BLINK_INTERVAL
            )

        elif state == LEDState.READY:

            cls._set_led(
                cls.WIFI,
                cls.GREEN
            )

        elif state == LEDState.ERROR:

            cls._set_led(
                cls.WIFI,
                cls.RED
            )

    # ====================================================
    # 3. CAMERA   (INITIALIZE | READY | ERROR)
    # ====================================================

    @classmethod
    def camera(cls, state: LEDState):

        allowed_states = {
            LEDState.INITIALIZE,
            LEDState.READY,
            LEDState.ERROR
        }

        cls._validate_state(
            "CAMERA",
            state,
            allowed_states
        )

        cls._stop_blink(cls.CAMERA)

        if state == LEDState.INITIALIZE:

            cls._blink(
                cls.CAMERA,
                cls.ORANGE,
                cls.BLINK_INTERVAL
            )

        elif state == LEDState.READY:

            cls._set_led(
                cls.CAMERA,
                cls.GREEN
            )

        elif state == LEDState.ERROR:

            cls._set_led(
                cls.CAMERA,
                cls.RED
            )

    # ====================================================
    # 4. ULTRASONIC
    # ====================================================

    @classmethod
    def ultrasonic(cls, enabled: bool):

        if enabled:

            cls._set_led(
                cls.ULTRASONIC,
                cls.BLUE
            )

        else:

            cls._turn_off(
                cls.ULTRASONIC
            )

    # ====================================================
    # 5. MOVING
    # ====================================================

    @classmethod
    def moving(cls, enabled: bool):

        if enabled:

            cls._set_led(
                cls.MOVING,
                cls.BLUE
            )

        else:

            cls._turn_off(
                cls.MOVING
            )

    # ====================================================
    # 6. DETECT
    # ====================================================

    @classmethod
    def detect(cls, enabled: bool):

        if enabled:

            cls._set_led(
                cls.DETECT,
                cls.BLUE
            )

        else:

            cls._turn_off(
                cls.DETECT
            )

    # ====================================================
    # 7. WATERING
    # ====================================================

    @classmethod
    def watering(cls, enabled: bool):

        if enabled:

            cls._set_led(
                cls.WATERING,
                cls.GREEN
            )

        else:

            cls._turn_off(
                cls.WATERING
            )

    # ====================================================
    # 8. ERROR
    # ====================================================

    @classmethod
    def error(cls, enabled: bool):

        cls._stop_blink(
            cls.ERROR_LED
        )

        if enabled:

            cls._blink(
                cls.ERROR_LED,
                cls.RED,
                cls.ERROR_BLINK_INTERVAL
            )

        else:

            cls._turn_off(
                cls.ERROR_LED
            )


# ========================================================
# TEST (python -m components.leds)
# ========================================================

if __name__ == "__main__":

    logging.basicConfig(level=logging.INFO)

    SAQI_LEDS.initialize()
    SAQI_LEDS.startup_animation()
    SAQI_LEDS.initialize_all()
    time.sleep(3)

    SAQI_LEDS.mode(LEDState.AUTO)
    SAQI_LEDS.wifi(LEDState.READY)
    SAQI_LEDS.camera(LEDState.READY)
    SAQI_LEDS.ultrasonic(True)
    SAQI_LEDS.moving(True)
    time.sleep(2)

    SAQI_LEDS.detect(True)
    time.sleep(2)

    SAQI_LEDS.watering(True)
    time.sleep(3)

    SAQI_LEDS.error(True)
    time.sleep(5)
    SAQI_LEDS.error(False)

    SAQI_LEDS.all_off()
