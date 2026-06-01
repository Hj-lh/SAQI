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
    BUZZER_COLLISION_HZ,
    BUZZER_COLLISION_RESET_CM,
    BUZZER_DUTY_CYCLE,
    BUZZER_PIN,
    BUZZER_PROXIMITY_BEEP_SECONDS,
    BUZZER_PROXIMITY_FAST_INTERVAL,
    BUZZER_PROXIMITY_HIGH_HZ,
    BUZZER_PROXIMITY_LOW_HZ,
    BUZZER_PROXIMITY_SLOW_INTERVAL,
    BUZZER_PROXIMITY_START_CM,
    BUZZER_WATER_BEEP_HZ,
    BUZZER_WATER_BEEP_SECONDS,
    BUZZER_WATER_INTERVAL,
    BUZZER_WATER_MUSIC_ENABLED,
    BUZZER_WATER_OFF_HIGH_HZ,
    BUZZER_WATER_OFF_LOW_HZ,
    BUZZER_WATER_ON_HIGH_HZ,
    BUZZER_WATER_ON_LOW_HZ,
    BUZZER_WATER_SONG,
    BUZZER_WATER_SONGS,
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
        self._event_cues: deque[list[tuple[int | None, float]]] = deque()
        self._song_name: str | None = None
        self._song_note_index = 0
        self._thread = None

        try:
            if device is None:
                from gpiozero import PWMOutputDevice

                device = PWMOutputDevice(
                    pin,
                    initial_value=0.0,
                    frequency=BUZZER_WATER_BEEP_HZ,
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
        """Play pump edge cues and keep a simple interval beep while it runs."""
        with self._lock:
            if self._pump_active == active:
                return
            self._pump_active = active
            self._song_note_index = 0
            self._event_cues.append(self._pump_cue(active))
        logger.info(BuzzerLog.PUMP_ACTIVE.value, active)
        if active and BUZZER_WATER_MUSIC_ENABLED:
            logger.info(BuzzerLog.WATER_SONG.value, BUZZER_WATER_SONG)
        self._wake_event.set()

    def set_auto_active(self, active: bool):
        """Play a short rising/falling cue when the control mode changes."""
        with self._lock:
            if self._auto_active == active:
                return
            self._auto_active = active
            self._event_cues.append(self._mode_cue(active))
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

    @staticmethod
    def _pump_cue(active: bool) -> list[tuple[int | None, float]]:
        notes = (
            (BUZZER_WATER_ON_LOW_HZ, BUZZER_WATER_ON_HIGH_HZ)
            if active
            else (BUZZER_WATER_OFF_HIGH_HZ, BUZZER_WATER_OFF_LOW_HZ)
        )
        return [(notes[0], 0.10), (None, 0.04), (notes[1], 0.14), (None, 0.05)]

    def _loop(self):
        try:
            while not self._stop_event.is_set():
                cue = self._next_event_cue()
                if cue is not None:
                    self._play_pattern(cue)
                    continue

                if self._collision_alarm_active():
                    self._play_collision_step()
                    continue

                with self._lock:
                    pump_active = self._pump_active
                    distance_cm = self._distance_cm
                    collision_silenced = self._collision_silenced

                if pump_active:
                    if BUZZER_WATER_MUSIC_ENABLED:
                        self._play_watering_song_step()
                    else:
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

    def _next_event_cue(self) -> list[tuple[int | None, float]] | None:
        with self._lock:
            if not self._event_cues:
                return None
            return self._event_cues.popleft()

    def _play_pattern(self, notes: list[tuple[int | None, float]]):
        for frequency, duration in notes:
            if self._stop_event.is_set():
                return
            self._tone(frequency)
            if self._stop_event.wait(timeout=duration):
                return

    def _play_watering_step(self):
        self._tone(BUZZER_WATER_BEEP_HZ)
        if self._wait(BUZZER_WATER_BEEP_SECONDS, interruptible=False):
            return
        self._tone(None)
        self._wait(max(0.01, BUZZER_WATER_INTERVAL - BUZZER_WATER_BEEP_SECONDS))

    def _play_watering_song_step(self):
        song_name = (
            BUZZER_WATER_SONG
            if BUZZER_WATER_SONG in BUZZER_WATER_SONGS
            else "ode_to_joy"
        )
        song = BUZZER_WATER_SONGS[song_name]
        notes = song["notes"]
        with self._lock:
            if self._song_name != song_name:
                self._song_name = song_name
                self._song_note_index = 0
            frequency, beats = notes[self._song_note_index]
            self._song_note_index = (self._song_note_index + 1) % len(notes)
        self._tone(frequency)
        self._wait(max(0.01, beats * song["beat_seconds"]), interruptible=False)

    def _play_collision_step(self):
        self._tone(BUZZER_COLLISION_HZ)
        with self._lock:
            deadline = self._collision_deadline
        remaining = max(0.0, deadline - time.monotonic()) if deadline is not None else 0.0
        self._wait(remaining)

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
