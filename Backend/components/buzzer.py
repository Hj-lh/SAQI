"""
Passive buzzer controller.

Owns one GPIO PWM output and plays non-blocking audio cues for watering,
manual/automatic mode changes, and ultrasonic proximity readings.
"""

from __future__ import annotations

from collections import deque
import logging
import threading
import time

from components.config import (
    BUZZER_COLLISION_ALARM_SECONDS,
    BUZZER_COLLISION_CM,
    BUZZER_COLLISION_HIGH_HZ,
    BUZZER_COLLISION_LOW_HZ,
    BUZZER_COLLISION_NOTE_SECONDS,
    BUZZER_COLLISION_RESET_CM,
    BUZZER_DUTY_CYCLE,
    BUZZER_PIN,
    BUZZER_PROXIMITY_BEEP_SECONDS,
    BUZZER_PROXIMITY_FAST_INTERVAL,
    BUZZER_PROXIMITY_HIGH_HZ,
    BUZZER_PROXIMITY_LOW_HZ,
    BUZZER_PROXIMITY_SLOW_INTERVAL,
    BUZZER_PROXIMITY_START_CM,
    BUZZER_WATER_GAP_SECONDS,
    BUZZER_WATER_HIGH_HZ,
    BUZZER_WATER_LOW_HZ,
    BUZZER_WATER_NOTE_SECONDS,
)
from components.logs import BuzzerLog

logger = logging.getLogger(__name__)


class BuzzerController:
    """Play passive-buzzer cues without blocking robot control threads."""

    def __init__(
        self,
        pin: int = BUZZER_PIN,
        duty_cycle: float = BUZZER_DUTY_CYCLE,
        device=None,
    ):
        self.pin = pin
        self.duty_cycle = duty_cycle
        self.enabled = False
        self._device = None
        self._lock = threading.Lock()
        self._wake_event = threading.Event()
        self._stop_event = threading.Event()
        self._pump_active = False
        self._auto_active = False
        self._distance_cm: float | None = None
        self._collision_deadline: float | None = None
        self._collision_silenced = False
        self._mode_cues: deque[list[tuple[int | None, float]]] = deque()
        self._water_high_note = False
        self._collision_high_note = False
        self._thread = None

        try:
            if device is None:
                from gpiozero import PWMOutputDevice

                device = PWMOutputDevice(
                    pin,
                    initial_value=0.0,
                    frequency=BUZZER_WATER_LOW_HZ,
                )
            self._device = device
            self.enabled = True
            self._thread = threading.Thread(
                target=self._loop, daemon=True, name="passive-buzzer"
            )
            self._thread.start()
            logger.info(BuzzerLog.INITIALISED.value, pin)
        except Exception as exc:  # noqa: BLE001
            logger.warning(BuzzerLog.DISABLED.value, exc)

    # ------------------------------------------------------------------
    # Event notifications
    # ------------------------------------------------------------------

    def set_pump_active(self, active: bool):
        """Keep a gentle two-note loop playing while the water pump runs."""
        with self._lock:
            if self._pump_active == active:
                return
            self._pump_active = active
        logger.info(BuzzerLog.PUMP_ACTIVE.value, active)
        self._wake_event.set()

    def set_auto_active(self, active: bool):
        """Play a short rising/falling cue when the control mode changes."""
        with self._lock:
            if self._auto_active == active:
                return
            self._auto_active = active
            self._mode_cues.append(self._mode_cue(active))
        logger.info(BuzzerLog.MODE_CHANGED.value, "automatic" if active else "manual")
        self._wake_event.set()

    def update_distance(self, cm: float | None):
        """Update the latest front distance used by the proximity warning."""
        now = time.monotonic()
        with self._lock:
            self._distance_cm = cm
            if cm is None or cm > BUZZER_COLLISION_RESET_CM:
                self._collision_deadline = None
                self._collision_silenced = False
            elif (
                cm <= BUZZER_COLLISION_CM
                and self._collision_deadline is None
                and not self._collision_silenced
            ):
                self._collision_deadline = now + BUZZER_COLLISION_ALARM_SECONDS
                logger.info(
                    BuzzerLog.COLLISION_ALARM.value,
                    cm,
                    BUZZER_COLLISION_ALARM_SECONDS,
                )
        self._wake_event.set()

    # ------------------------------------------------------------------
    # Audio worker
    # ------------------------------------------------------------------

    @staticmethod
    def _mode_cue(auto_active: bool) -> list[tuple[int | None, float]]:
        notes = (880, 1175) if auto_active else (1175, 880)
        return [(notes[0], 0.09), (None, 0.04), (notes[1], 0.12)]

    def _loop(self):
        try:
            while not self._stop_event.is_set():
                if self._collision_alarm_active():
                    self._play_collision_step()
                    continue

                cue = self._next_mode_cue()
                if cue is not None:
                    self._play_pattern(cue)
                    continue

                with self._lock:
                    pump_active = self._pump_active
                    distance_cm = self._distance_cm
                    collision_silenced = self._collision_silenced

                if pump_active:
                    self._play_watering_step()
                elif (
                    distance_cm is not None
                    and BUZZER_COLLISION_CM < distance_cm < BUZZER_PROXIMITY_START_CM
                    and not collision_silenced
                ):
                    self._play_proximity_step(distance_cm)
                else:
                    self._tone(None)
                    self._wait()
        finally:
            self._tone(None)

    def _collision_alarm_active(self) -> bool:
        with self._lock:
            if self._collision_deadline is None:
                return False
            if time.monotonic() < self._collision_deadline:
                return True
            self._collision_deadline = None
            self._collision_silenced = True
        logger.info(BuzzerLog.COLLISION_SILENCED.value)
        return False

    def _next_mode_cue(self) -> list[tuple[int | None, float]] | None:
        with self._lock:
            if not self._mode_cues:
                return None
            return self._mode_cues.popleft()

    def _play_pattern(self, notes: list[tuple[int | None, float]]):
        for frequency, duration in notes:
            if self._stop_event.is_set():
                return
            self._tone(frequency)
            if self._stop_event.wait(timeout=duration):
                return

    def _play_watering_step(self):
        self._water_high_note = not self._water_high_note
        frequency = BUZZER_WATER_HIGH_HZ if self._water_high_note else BUZZER_WATER_LOW_HZ
        self._tone(frequency)
        if self._wait(BUZZER_WATER_NOTE_SECONDS, interruptible=False):
            return
        self._tone(None)
        self._wait(BUZZER_WATER_GAP_SECONDS)

    def _play_collision_step(self):
        self._collision_high_note = not self._collision_high_note
        frequency = (
            BUZZER_COLLISION_HIGH_HZ if self._collision_high_note else BUZZER_COLLISION_LOW_HZ
        )
        self._tone(frequency)
        self._wait(BUZZER_COLLISION_NOTE_SECONDS)

    def _play_proximity_step(self, cm: float):
        span = BUZZER_PROXIMITY_START_CM - BUZZER_COLLISION_CM
        closeness = (BUZZER_PROXIMITY_START_CM - cm) / span
        closeness = max(0.0, min(1.0, closeness))
        frequency = round(
            BUZZER_PROXIMITY_LOW_HZ
            + closeness * (BUZZER_PROXIMITY_HIGH_HZ - BUZZER_PROXIMITY_LOW_HZ)
        )
        interval = (
            BUZZER_PROXIMITY_SLOW_INTERVAL
            - closeness * (BUZZER_PROXIMITY_SLOW_INTERVAL - BUZZER_PROXIMITY_FAST_INTERVAL)
        )
        self._tone(frequency)
        if self._wait(BUZZER_PROXIMITY_BEEP_SECONDS, interruptible=False):
            return
        self._tone(None)
        self._wait(max(0.01, interval - BUZZER_PROXIMITY_BEEP_SECONDS))

    def _tone(self, frequency: int | None):
        if not self.enabled or self._device is None:
            return
        try:
            if frequency is None:
                self._device.value = 0.0
            else:
                self._device.frequency = frequency
                self._device.value = self.duty_cycle
        except Exception as exc:  # noqa: BLE001
            logger.warning(BuzzerLog.WRITE_FAILED.value, exc)
            self.enabled = False

    def _wait(self, seconds: float | None = None, interruptible: bool = True) -> bool:
        if not interruptible:
            return self._stop_event.wait(timeout=seconds)
        self._wake_event.wait(timeout=seconds)
        self._wake_event.clear()
        return self._stop_event.is_set()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self):
        self._stop_event.set()
        self._wake_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._tone(None)
        if self._device is not None:
            try:
                self._device.close()
            except Exception:  # noqa: BLE001
                pass
        logger.info(BuzzerLog.CLOSED.value)
