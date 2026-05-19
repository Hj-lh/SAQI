"""
Ultrasonic Sensor Component
===========================
HC-SR04 distance sensor via gpiozero.DistanceSensor (reuses the gpiozero
dependency already used by motor.py / waterpump.py — no new library).

Reading + LED only: this component measures distance and drives the
ULTRASONIC status LED (ON when an obstacle is within
ULTRASONIC_OBSTACLE_CM). It does NOT alter autonomous navigation.

Follows the project's graceful pattern: if the sensor / gpiozero pin
factory is unavailable, the component disables itself (``enabled`` False)
and the rest of the robot keeps running.

WIRING NOTE: HC-SR04 ECHO is 5 V — use a voltage divider to the Pi pin.
Pins come from components.config (SSOT): ULTRASONIC_TRIG_PIN / _ECHO_PIN.
"""

import logging
import threading
import time

from components.config import (
    ULTRASONIC_TRIG_PIN,
    ULTRASONIC_ECHO_PIN,
    ULTRASONIC_MAX_CM,
    ULTRASONIC_OBSTACLE_CM,
)
from components.leds import SAQI_LEDS

from components.logs import UltrasonicLog
logger = logging.getLogger(__name__)

_POLL_INTERVAL = 0.2  # seconds (~5 Hz)


class UltrasonicSensor:
    """HC-SR04 reader with a background poll that drives the ULTRASONIC LED."""

    def __init__(self):
        self.enabled = False
        self._sensor = None
        self._lock = threading.Lock()
        self._latest_cm: float | None = None

        self._thread = None
        self._stop_event = threading.Event()

        try:
            from gpiozero import DistanceSensor

            self._sensor = DistanceSensor(
                echo=ULTRASONIC_ECHO_PIN,
                trigger=ULTRASONIC_TRIG_PIN,
                max_distance=ULTRASONIC_MAX_CM / 100.0,  # metres
            )
            self.enabled = True
            logger.info(
                UltrasonicLog.INITIALISED.value,
                ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, ULTRASONIC_MAX_CM,
            )
        except Exception as e:  # noqa: BLE001
            logger.warning(
                "UltrasonicSensor disabled (%s) — distance unavailable, "
                "system continues without it", e,
            )

    # ------------------------------------------------------------------
    # Readings
    # ------------------------------------------------------------------

    def distance_cm(self) -> float | None:
        """Latest distance in cm, or None if disabled / no reading yet.

        If the background poll isn't running, reads on demand."""
        if not self.enabled:
            return None
        with self._lock:
            if self._latest_cm is not None:
                return self._latest_cm
        try:
            return round(self._sensor.distance * 100.0, 1)
        except Exception as e:  # noqa: BLE001
            logger.debug("Ultrasonic read error: %s", e)
            return None

    def is_obstacle(self) -> bool:
        """True if the latest distance is closer than ULTRASONIC_OBSTACLE_CM."""
        d = self.distance_cm()
        return d is not None and d < ULTRASONIC_OBSTACLE_CM

    # ------------------------------------------------------------------
    # Background poll → ULTRASONIC LED
    # ------------------------------------------------------------------

    def start(self):
        if not self.enabled or self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="ultrasonic-poll"
        )
        self._thread.start()

    def _loop(self):
        while not self._stop_event.is_set():
            try:
                cm = round(self._sensor.distance * 100.0, 1)
                with self._lock:
                    self._latest_cm = cm
                SAQI_LEDS.ultrasonic(cm < ULTRASONIC_OBSTACLE_CM)
            except Exception as e:  # noqa: BLE001
                logger.debug("Ultrasonic poll error: %s", e)
            if self._stop_event.wait(timeout=_POLL_INTERVAL):
                break

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        try:
            SAQI_LEDS.ultrasonic(False)
        except Exception:  # noqa: BLE001
            pass
        if self._sensor is not None:
            try:
                self._sensor.close()
            except Exception:  # noqa: BLE001
                pass
        logger.info(UltrasonicLog.CLOSED.value)


# ------------------------------------------------------------------
# Quick self-test (python -m components.ultrasonic)
# ------------------------------------------------------------------
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    u = UltrasonicSensor()
    print("enabled:", u.enabled)
    for _ in range(10):
        print("distance_cm:", u.distance_cm(), "obstacle:", u.is_obstacle())
        time.sleep(0.5)
    u.close()
