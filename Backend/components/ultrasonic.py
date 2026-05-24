"""
ESP32 ultrasonic sensor head over USB serial.

The ESP32 owns the HC-SR04 and servo. The Raspberry Pi sends line commands:
DIST, SCAN, CENTER. Manual mode logs what the sensor sees; AutoNavigator decides
whether to act on those readings.
"""

import json
import logging
import threading
import time

from components.config import (
    ULTRASONIC_OBSTACLE_CM,
    ULTRASONIC_POLL_INTERVAL,
    ULTRASONIC_SCAN_COOLDOWN,
    ULTRASONIC_SERIAL_BAUD,
    ULTRASONIC_SERIAL_PORT,
    ULTRASONIC_SERIAL_TIMEOUT,
)
from components.leds import SAQI_LEDS
from components.logs import UltrasonicLog

logger = logging.getLogger(__name__)


class UltrasonicSensor:
    """Serial client for an ESP32-mounted ultrasonic/servo sensor head."""

    def __init__(
        self,
        port: str = ULTRASONIC_SERIAL_PORT,
        baud: int = ULTRASONIC_SERIAL_BAUD,
        timeout: float = ULTRASONIC_SERIAL_TIMEOUT,
    ):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.enabled = False
        self._serial = None
        self._serial_lock = threading.Lock()
        self._lock = threading.Lock()
        self._latest_cm: float | None = None
        self._last_obstacle = False
        self._scan_logged = False
        self._auto_active = False

        self._thread = None
        self._stop_event = threading.Event()

        for candidate in self._candidate_ports(port):
            if self._open(candidate):
                break

    def _candidate_ports(self, preferred: str) -> list[str]:
        ports = [preferred, "/dev/ttyACM0", "/dev/ttyUSB0"]
        unique = []
        for item in ports:
            if item and item not in unique:
                unique.append(item)
        return unique

    def _open(self, port: str) -> bool:
        try:
            import serial

            self._serial = serial.Serial(port, self.baud, timeout=self.timeout)
            time.sleep(2.0)
            self._serial.reset_input_buffer()
            self.enabled = self.read_distance_cm() is not None
            if self.enabled:
                self.port = port
                logger.info(UltrasonicLog.INITIALISED.value, port)
                return True
            logger.warning("Ultrasonic ESP32 did not return a distance on %s", port)
            self._serial.close()
            self._serial = None
        except Exception as exc:  # noqa: BLE001
            logger.warning("Ultrasonic ESP32 serial unavailable on %s: %s", port, exc)
        return False

    def _command(self, command: str) -> dict | None:
        if self._serial is None:
            return None
        try:
            with self._serial_lock:
                self._serial.reset_input_buffer()
                self._serial.write((command.strip() + "\n").encode("utf-8"))
                self._serial.flush()
                line = self._serial.readline().decode("utf-8", errors="replace").strip()
            if not line:
                return None
            return json.loads(line)
        except Exception as exc:  # noqa: BLE001
            logger.debug("Ultrasonic serial command failed: %s", exc)
            return None

    def read_distance_cm(self) -> float | None:
        data = self._command("DIST")
        if not data or not data.get("ok"):
            return None
        cm = data.get("cm")
        if cm is None or cm < 0:
            return None
        return float(cm)

    def distance_cm(self) -> float | None:
        with self._lock:
            return self._latest_cm

    def is_obstacle(self) -> bool:
        cm = self.distance_cm()
        return cm is not None and cm < ULTRASONIC_OBSTACLE_CM

    def set_auto_active(self, active: bool):
        with self._lock:
            self._auto_active = active

    def scan(self) -> dict | None:
        data = self._command("SCAN")
        if not data or not data.get("ok"):
            return None
        return data

    def best_direction(self) -> str | None:
        scan = self.scan()
        if scan is None:
            return None

        right = scan.get("right_cm")
        left = scan.get("left_cm")
        front = scan.get("front_cm")
        logger.info(
            "Ultrasonic scan: front=%s cm, right=%s cm, left=%s cm",
            front, right, left,
        )

        best = scan.get("best")
        if best in ("left", "right"):
            return best
        if right is None and left is None:
            return None
        if right is None:
            return "left"
        if left is None:
            return "right"
        return "right" if right >= left else "left"

    def start(self):
        if not self.enabled or self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="ultrasonic-esp32-serial"
        )
        self._thread.start()

    def _loop(self):
        while not self._stop_event.is_set():
            cm = self.read_distance_cm()
            obstacle = cm is not None and cm < ULTRASONIC_OBSTACLE_CM

            with self._lock:
                self._latest_cm = cm
                auto_active = self._auto_active

            SAQI_LEDS.ultrasonic(obstacle)

            if cm is not None and obstacle != self._last_obstacle:
                state = "obstacle" if obstacle else "clear"
                logger.info("Ultrasonic front: %.1f cm (%s)", cm, state)
                self._last_obstacle = obstacle

            if obstacle and not auto_active and not self._scan_logged:
                scan = self.scan()
                if scan is not None:
                    logger.info(
                        "Ultrasonic manual scan log: front=%s cm, right=%s cm, left=%s cm",
                        scan.get("front_cm"), scan.get("right_cm"), scan.get("left_cm"),
                    )
                self._scan_logged = True
            elif not obstacle:
                self._scan_logged = False

            wait_seconds = ULTRASONIC_SCAN_COOLDOWN if obstacle else ULTRASONIC_POLL_INTERVAL
            if self._stop_event.wait(timeout=wait_seconds):
                break

    def close(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._command("CENTER")
        try:
            SAQI_LEDS.ultrasonic(False)
        except Exception:  # noqa: BLE001
            pass
        if self._serial is not None:
            self._serial.close()
        logger.info(UltrasonicLog.CLOSED.value)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    sensor = UltrasonicSensor()
    print("enabled:", sensor.enabled)
    print("distance_cm:", sensor.read_distance_cm())
    print("scan:", sensor.scan())
    sensor.close()
