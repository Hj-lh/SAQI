"""
Automatic Navigation Component
==============================
Handles background autonomous navigation using YOLO + ByteTrack detections.

Key design:
  - The navigator consumes detections as soon as they arrive via a threading
    Event, so it never misses short-lived detections.
  - Scan direction reflects where the plant was *most recently* seen; turns
    are short so multiple loop iterations refine alignment without overshoot.
  - When transitioning from a scan into tracking, the navigator stops and
    re-evaluates with the next detection instead of stacking another turn.
  - Watered plants are remembered by ByteTrack ID *and* by appearance
    (CNN Re-ID) so they are not targeted again.
  - After watering, the robot reverses for a short window to clear the
    plant, watching for the *next* unwatered plant.

State machine:
  SCANNING    → target detected     → STOP + re-evaluate (no double-turn)
  TRACKING    → target in center    → APPROACHING
  TRACKING    → target left/right   → TURNING (short)
  TRACKING    → target lost briefly → HOLDING
  APPROACHING → centered box large enough → ULTRASONIC_HANDOFF
  ULTRASONIC_HANDOFF → distance close enough → WATERING
  WATERING    → done                → POST_WATER_REVERSING
  POST_WATER_REVERSING → quota met  → RETURNING
  POST_WATER_REVERSING → new target → TRACKING
  POST_WATER_REVERSING → no target  → SCANNING
  HOLDING     → target reappears    → TRACKING
  HOLDING     → lost too long       → SCANNING
"""

import time
import threading
import logging

import cv2

from components.motor import MotorController
from components.waterpump import WaterPumpController
from components.reid import PlantReID
from components.leds import SAQI_LEDS, LEDState
# Navigation tuning lives in components.config (single source of truth, so it
# can be surfaced/edited via the settings UI). These names become module-level
# globals here; AutoNavigator.start() re-binds the editable ones from the
# runtime settings before each auto run (see components.settings).
from components.config import (
    ULTRASONIC_AVOID_BACKWARD_SECONDS,
    ULTRASONIC_AVOID_FORWARD_SECONDS,
    ULTRASONIC_AVOID_SPEED,
    ULTRASONIC_AVOID_TURN_SECONDS,
    ULTRASONIC_WATER_DISTANCE_CM,
    WATER_HANDOFF_FORWARD_SECONDS,
    WATER_HANDOFF_FORWARD_SPEED,
    WATER_HANDOFF_TIMEOUT_SECONDS,
    YOLO_WATER_HANDOFF_AREA_RATIO,
    PLANTS_PER_RUN,
    BASE_QR_PAYLOAD,
    BASE_ARRIVAL_AREA_RATIO,
    BASE_ARRIVAL_DISTANCE_CM,
    BASE_SIGHTING_HOLD_SECONDS,
    BASE_SPIN_SPEED,
    BASE_SPIN_SECONDS,
    AUTO_SPEED_FORWARD,
    AUTO_SPEED_BACKWARD,
    AUTO_SPEED_TURN,
    AUTO_SPEED_TURN_GENTLE,
    MIN_TURN_SPEED,
    MOVE_TURN_DURATION,
    MOVE_FORWARD_DURATION,
    SLEEP_WAIT_YOLO,
    SCAN_TURN_DURATION,
    REID_CAPTURE_SAMPLES,
    REID_CAPTURE_INTERVAL,
    REID_RETREAT_INTERVAL,
    YOLO_CENTER_SIDE_MARGIN,
    ARRIVAL_AREA_RATIO,
    WATERING_DURATION,
    POST_WATER_REVERSE_SECONDS,
    RETREAT_POLL_PERIOD,
    LOST_THRESHOLD,
)

from components.logs import NavLog
logger = logging.getLogger(__name__)

# Safety: if the camera produces no frames for this long, the navigator holds
# (stops the motors) instead of driving or scanning blind.
CAMERA_STALE_SECONDS = 3.0


class AutoNavigator:
    def __init__(
        self,
        motor: MotorController,
        pump: WaterPumpController,
        camera,                       # RobotCamera — frame source for inference
        detector,                     # PlantDetector — YOLO inference
        ptz=None,                     # CameraPTZController | None
        reid: PlantReID | None = None,
        ultrasonic=None,
        buzzer=None,
    ):
        self.motor = motor
        self.pump = pump
        self.camera = camera
        self.detector = detector
        self.ptz = ptz                # may be None if camera control unavailable
        self.reid = reid              # may be None if torchvision unavailable
        self.ultrasonic = ultrasonic
        self.buzzer = buzzer

        self.is_active = False
        self._thread = None
        self._infer_thread = None

        # Latest annotated JPEG produced by the inference loop, read by the
        # MJPEG stream. None until the first frame is processed.
        self._annotated_lock = threading.Lock()
        self._latest_annotated_jpeg: bytes | None = None

        # Detection data (written by camera stream, read by navigator)
        self.latest_detections: list[dict] = []
        self.latest_frame = None
        self.frame_width: int = 640
        self.frame_height: int = 480
        self._lock = threading.Lock()

        # Cached ReID embeddings by track ID — avoids re-running the CNN
        # on every navigation cycle for already-seen tracks.
        self._embed_cache: dict[int, "np.ndarray"] = {}

        # Signals
        self._stop_event = threading.Event()
        self._detection_event = threading.Event()

        # Watering / labelling state — read by the MJPEG annotator
        self._watered_ids: set[int] = set()
        self._is_watering: bool = False

        # Navigation state
        self._frames_without_target = 0
        self._last_known_zone: str = "LEFT"
        self._just_finished_scan: bool = False  # Prevents double-turn after scan
        self._watering_handoff_target: dict | None = None
        self._watering_handoff_deadline = 0.0

        # Oscillation detection
        self._zone_history: list[str] = []
        self._oscillating = False

        # Camera liveness — set by the inference loop on every good frame
        self._last_frame_time = 0.0
        self._camera_warned = False

        # Return-to-base mission state
        self._qr = cv2.QRCodeDetector()
        self._watering_count = 0          # completed watering events this run
        self._returning = False           # True once heading home for the base
        self._base_lock = threading.Lock()
        # Latest base QR sighting: {"box", "payload", "w", "h"} or None
        self._latest_base: dict | None = None
        self._base_seen_time = 0.0          # monotonic time of last real sighting
        self._base_decoded_once = False     # confirmed the base payload this run?
        self._base_payload = BASE_QR_PAYLOAD or ""

    # ------------------------------------------------------------------
    # Public read-only accessors used by the video stream / annotator
    # ------------------------------------------------------------------

    @property
    def watered_ids(self) -> set[int]:
        return set(self._watered_ids)

    @property
    def is_watering(self) -> bool:
        return self._is_watering

    def get_annotated_jpeg(self) -> bytes | None:
        """Latest YOLO-annotated frame as JPEG bytes, or None if the
        inference loop has not produced a frame yet."""
        with self._annotated_lock:
            return self._latest_annotated_jpeg

    # ------------------------------------------------------------------
    # State reset
    # ------------------------------------------------------------------

    def _reset_state(self):
        """Reset all transient navigation state for a fresh run."""
        self._frames_without_target = 0
        self._last_known_zone = "LEFT"
        self._just_finished_scan = False
        self._watering_handoff_target = None
        self._watering_handoff_deadline = 0.0
        self._zone_history.clear()
        self._oscillating = False
        self._is_watering = False
        self._last_frame_time = time.monotonic()
        self._camera_warned = False
        self._watering_count = 0
        self._returning = False
        with self._base_lock:
            self._latest_base = None
            self._base_seen_time = 0.0
            self._base_decoded_once = False
            self._base_payload = BASE_QR_PAYLOAD or ""
        self._watered_ids.clear()
        self._embed_cache.clear()
        if self.reid is not None:
            self.reid.clear()
        self._detection_event.clear()

    # ------------------------------------------------------------------
    # Called by the camera stream thread
    # ------------------------------------------------------------------

    def update_detections(
        self,
        detections: list[dict],
        frame_width: int,
        frame_height: int = 480,
        frame=None,
    ):
        """Called by the MJPEG generator every frame with YOLO results.

        ``frame`` is the raw OpenCV frame the detections came from; the
        navigator holds a reference (no copy) so it can crop watered-plant
        regions when ReID is enabled.
        """
        with self._lock:
            self.latest_detections = list(detections)
            self.frame_width = frame_width
            self.frame_height = frame_height
            if frame is not None:
                self.latest_frame = frame

        if detections:
            self._detection_event.set()

    # ------------------------------------------------------------------
    # Inference loop — runs YOLO independently of any frontend client
    # ------------------------------------------------------------------

    def _inference_loop(self):
        """Continuously pull frames from the camera, run YOLO, feed the
        navigator, and cache an annotated frame for the MJPEG stream.

        This runs in its own thread so detection + navigation keep going
        even when no client is streaming /camera/feed.
        """
        logger.info(NavLog.INFER_LOOP_STARTED.value)
        while self.is_active:
            frame = self.camera.get_raw_frame()
            if frame is None:
                if self._stop_event.wait(timeout=0.05):
                    break
                continue

            # Mark the feed as alive so the navigation loop knows it can trust
            # detections (and won't drive/scan blind if the camera drops).
            self._last_frame_time = time.monotonic()

            if self._returning:
                # Mission's watering phase is done — hunt for the base QR
                # instead of plants so the stream and navigator agree.
                box, payload = self._detect_base(frame)
                now = time.monotonic()
                with self._base_lock:
                    if box is not None:
                        self._latest_base = {
                            "box": box, "payload": payload,
                            "w": frame.shape[1], "h": frame.shape[0],
                        }
                        self._base_seen_time = now
                    elif (
                        self._latest_base is not None
                        and now - self._base_seen_time > BASE_SIGHTING_HOLD_SECONDS
                    ):
                        self._latest_base = None   # held long enough; really lost
                    # else: keep the last sighting to ride out a brief miss
                    held = self._latest_base
                SAQI_LEDS.detect(held is not None)
                if held is not None:
                    annotated = self._annotate_base(frame, held["box"], held["payload"])
                else:
                    annotated = self._annotate_base(frame, None, None)
            else:
                detections = self.detector.detect(frame)
                SAQI_LEDS.detect(bool(detections))
                self.update_detections(
                    detections, frame.shape[1], frame.shape[0], frame=frame
                )
                annotated = self.detector.annotate_frame(
                    frame,
                    detections,
                    watered_ids=self.watered_ids,
                    watering=self.is_watering,
                )

            ok, jpeg = cv2.imencode(".jpg", annotated)
            if ok:
                with self._annotated_lock:
                    self._latest_annotated_jpeg = jpeg.tobytes()

        logger.info(NavLog.INFER_LOOP_EXITED.value)

    # ------------------------------------------------------------------
    # Start / Stop
    # ------------------------------------------------------------------

    def start(self):
        if self.is_active:
            return
        # Snapshot the latest user settings for this run (applies-on-Start).
        from components import settings as runtime_settings
        runtime_settings.apply_run_overrides()
        logger.info(NavLog.STARTING.value)
        self.motor.stop()
        self.pump.off()
        if self._ptz_available():
            self.ptz.look_center()
        if self.ultrasonic is not None:
            self.ultrasonic.set_auto_active(True)
        self._reset_state()
        self._stop_event.clear()
        self.is_active = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="auto-navigator")
        self._thread.start()
        self._infer_thread = threading.Thread(
            target=self._inference_loop, daemon=True, name="auto-inference"
        )
        self._infer_thread.start()
        SAQI_LEDS.mode(LEDState.AUTO)
        if self.buzzer is not None:
            self.buzzer.set_auto_active(True)

    def stop(self):
        if not self.is_active:
            if self.buzzer is not None:
                self.buzzer.set_auto_active(False)
            return
        logger.info(NavLog.STOPPING.value)
        self.is_active = False
        self._stop_event.set()
        self._detection_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._infer_thread:
            self._infer_thread.join(timeout=3.0)
        with self._annotated_lock:
            self._latest_annotated_jpeg = None
        self.motor.stop()
        self.pump.off()
        if self._ptz_available():
            self.ptz.look_center()
        if self.ultrasonic is not None:
            self.ultrasonic.set_auto_active(False)
        SAQI_LEDS.detect(False)
        SAQI_LEDS.mode(LEDState.MANUAL)
        if self.buzzer is not None:
            self.buzzer.set_auto_active(False)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_zone(self, obj_center_x: float, width: int) -> str:
        if obj_center_x < width * YOLO_CENTER_SIDE_MARGIN:
            return "LEFT"
        if obj_center_x > width * (1 - YOLO_CENTER_SIDE_MARGIN):
            return "RIGHT"
        return "CENTER"

    def _sleep(self, duration: float) -> bool:
        """Interruptible sleep. Returns True if we should stop."""
        return self._stop_event.wait(timeout=duration)

    def _camera_alive(self) -> bool:
        """True if the inference loop has produced a frame recently."""
        return (time.monotonic() - self._last_frame_time) < CAMERA_STALE_SECONDS

    def _wait_for_detections(self, timeout: float) -> bool:
        """Wait for new detections OR timeout. Returns True if stop requested."""
        self._detection_event.clear()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if self._detection_event.wait(timeout=min(remaining, 0.1)):
                self._detection_event.clear()
                break
        return self._stop_event.is_set()

    def _is_arrived(self, box: list[float], frame_width: int, frame_height: int) -> bool:
        ratio = self._box_area_ratio(box, frame_width, frame_height)
        logger.debug("Auto: box area ratio = %.2f (threshold=%.2f)", ratio, ARRIVAL_AREA_RATIO)
        return ratio >= ARRIVAL_AREA_RATIO

    @staticmethod
    def _box_area_ratio(box: list[float], frame_width: int, frame_height: int) -> float:
        x1, y1, x2, y2 = box
        box_area = (x2 - x1) * (y2 - y1)
        frame_area = frame_width * frame_height
        return box_area / frame_area if frame_area > 0 else 0.0

    def _ultrasonic_ready(self) -> bool:
        return (
            self.ultrasonic is not None
            and getattr(self.ultrasonic, "enabled", False)
        )

    def _start_watering_handoff(
        self,
        target: dict,
        frame_width: int,
        frame_height: int,
    ) -> bool:
        """Latch a centered close-up plant so ultrasonic can finish approach."""
        if not self._ultrasonic_ready():
            return False
        ratio = self._box_area_ratio(target["box"], frame_width, frame_height)
        if ratio < YOLO_WATER_HANDOFF_AREA_RATIO:
            logger.debug(
                "Auto: YOLO handoff gate - area=%.2f/%.2f; keep approaching",
                ratio,
                YOLO_WATER_HANDOFF_AREA_RATIO,
            )
            return False
        self._watering_handoff_target = dict(target)
        self._watering_handoff_deadline = (
            time.monotonic() + WATER_HANDOFF_TIMEOUT_SECONDS
        )
        logger.info(
            NavLog.WATER_HANDOFF_STARTED.value,
            target.get("id"),
            ratio,
            YOLO_WATER_HANDOFF_AREA_RATIO,
            WATER_HANDOFF_TIMEOUT_SECONDS,
        )
        return True

    def _watering_handoff_active(self) -> bool:
        if self._watering_handoff_target is None:
            return False
        if time.monotonic() < self._watering_handoff_deadline:
            return True
        self.motor.stop()
        self._clear_watering_handoff()
        logger.info(NavLog.WATER_HANDOFF_EXPIRED.value)
        return False

    def _clear_watering_handoff(self):
        self._watering_handoff_target = None
        self._watering_handoff_deadline = 0.0

    def _fresh_ultrasonic_distance_cm(self) -> float | None:
        """Get a fresh final-approach reading; use cache if the read raises."""
        try:
            return self.ultrasonic.read_distance_cm()
        except Exception as exc:  # noqa: BLE001
            logger.debug("Auto: fresh ultrasonic read failed during handoff: %s", exc)
            return self.ultrasonic.distance_cm()

    def _step_watering_handoff(self) -> bool:
        """Advance a YOLO-confirmed plant using ultrasonic alone.

        Returns True when the navigation loop should stop.
        """
        target = self._watering_handoff_target
        if target is None:
            return False
        cm = self._fresh_ultrasonic_distance_cm()
        if cm is None:
            self.motor.stop()
            logger.info(NavLog.WATER_HANDOFF_NO_READING.value)
            return self._sleep(0.1)
        if cm <= ULTRASONIC_WATER_DISTANCE_CM:
            logger.info(
                NavLog.WATER_HANDOFF_READY.value,
                cm,
                ULTRASONIC_WATER_DISTANCE_CM,
            )
            return self._water_target_and_recover(target)

        logger.info(
            NavLog.WATER_HANDOFF_APPROACH.value,
            cm,
            ULTRASONIC_WATER_DISTANCE_CM,
            WATER_HANDOFF_FORWARD_SPEED,
            WATER_HANDOFF_FORWARD_SECONDS,
        )
        self.motor.forward(WATER_HANDOFF_FORWARD_SPEED)
        if self._sleep(WATER_HANDOFF_FORWARD_SECONDS):
            self.motor.stop()
            return True
        self.motor.stop()
        return False

    def _get_turn_speed(self) -> float:
        speed = AUTO_SPEED_TURN_GENTLE if self._oscillating else AUTO_SPEED_TURN
        return max(speed, MIN_TURN_SPEED)

    def _update_oscillation(self, zone: str):
        """Track zone history and detect L↔R oscillation patterns."""
        self._zone_history.append(zone)
        if len(self._zone_history) > 4:
            self._zone_history.pop(0)

        if zone == "CENTER":
            if self._oscillating:
                logger.info(NavLog.PLANT_CENTERED.value)
            self._oscillating = False
            self._zone_history.clear()
            return

        if len(self._zone_history) >= 3:
            recent = self._zone_history[-3:]
            if all(z == recent[0] for z in recent):
                if self._oscillating:
                    logger.info(NavLog.ZONE_STABILISED.value, recent[0])
                self._oscillating = False
                return

            a, b, c = recent
            if a == c and a != b and a in ("LEFT", "RIGHT") and b in ("LEFT", "RIGHT"):
                if not self._oscillating:
                    logger.info(NavLog.OSCILLATION.value, a, b, c)
                self._oscillating = True

    # ------------------------------------------------------------------
    # Detection filtering — ignore plants we have already watered
    # ------------------------------------------------------------------

    def _reid_available(self) -> bool:
        return self.reid is not None and self.reid.enabled

    def _embedding_for(self, det: dict, frame):
        """Return the (cached) ReID embedding for a detection, or None."""
        if not self._reid_available() or frame is None:
            return None
        track_id = det.get("id")
        if track_id is not None and track_id in self._embed_cache:
            return self._embed_cache[track_id]
        emb = self.reid.embed(frame, det.get("box"))
        if track_id is not None and emb is not None:
            # Cap the cache so it can't grow without bound during long runs
            if len(self._embed_cache) >= 256:
                self._embed_cache.pop(next(iter(self._embed_cache)))
            self._embed_cache[track_id] = emb
        return emb

    def _is_known_watered(self, det: dict, frame) -> bool:
        """Detection has been watered if its track ID is remembered, OR if
        its appearance matches a stored watered embedding."""
        track_id = det.get("id")
        if track_id is not None and track_id in self._watered_ids:
            return True
        if not self._reid_available():
            return False
        emb = self._embedding_for(det, frame)
        if emb is None:
            return False
        if self.reid.is_watered(emb):
            # Also stamp the track ID so subsequent frames hit the fast path
            if track_id is not None:
                self._watered_ids.add(track_id)
                logger.info(NavLog.REID_MATCH_TAG.value, track_id)
            return True
        return False

    def _unwatered(self, detections: list[dict], frame=None) -> list[dict]:
        return [d for d in detections if not self._is_known_watered(d, frame)]

    def _snapshot_unwatered(self) -> tuple[list[dict], int, int]:
        with self._lock:
            dets = list(self.latest_detections)
            frame = self.latest_frame
            w, h = self.frame_width, self.frame_height
        return self._unwatered(dets, frame), w, h

    def _snapshot_with_frame(self) -> tuple[list[dict], "np.ndarray | None", int, int]:
        with self._lock:
            dets = list(self.latest_detections)
            frame = self.latest_frame
            w, h = self.frame_width, self.frame_height
        return dets, frame, w, h

    def _embed_target(self, target_id, fallback_box):
        """ReID-embed the freshest crop of the plant with ``target_id``.

        Uses the latest frame and the freshest detection box for that track
        ID; falls back to ``fallback_box`` if the ID isn't currently in
        view. Returns the unit embedding, or None.
        """
        with self._lock:
            frame = self.latest_frame
            box = fallback_box
            if target_id is not None:
                for d in self.latest_detections:
                    if d.get("id") == target_id:
                        box = d.get("box")
                        break
        return self.reid.embed(frame, box)

    # ------------------------------------------------------------------
    # Phase: watering
    # ------------------------------------------------------------------

    def _do_watering(self, target: dict):
        """Run the pump for WATERING_DURATION while the annotator shows a banner.

        After the pump turns off, the watered plant's appearance embedding
        is captured (if ReID is enabled) so the plant stays recognised even
        after its ByteTrack ID is lost.
        """
        target_id = target.get("id")
        logger.info(NavLog.ARRIVED.value, target_id)
        self.motor.stop()
        self._is_watering = True
        self.pump.on()
        try:
            for i in range(WATERING_DURATION):
                if not self.is_active:
                    break
                logger.info(NavLog.WATERING_PROGRESS.value, i + 1, WATERING_DURATION)
                if self._sleep(1.0):
                    break
        finally:
            self.pump.off()
            self._is_watering = False

        if target_id is not None:
            self._watered_ids.add(target_id)
            logger.info(NavLog.MARKED_WATERED.value,
                        target_id, len(self._watered_ids))
        else:
            logger.info(NavLog.WATERING_COMPLETE_NO_ID.value)

        # Count this watering event toward the mission target. The trigger is
        # the number of times we have watered (not distinct plants), per the
        # PLANTS_PER_RUN config knob.
        self._watering_count += 1
        logger.info(NavLog.WATERING_COUNT.value,
                    self._watering_count, PLANTS_PER_RUN)
        if PLANTS_PER_RUN > 0 and self._watering_count >= PLANTS_PER_RUN:
            self._returning = True
            with self._base_lock:
                self._latest_base = None
            logger.info(NavLog.ALL_PLANTS_WATERED.value,
                        PLANTS_PER_RUN, BASE_QR_PAYLOAD)

        # Capture several appearance embeddings (multi-view) so the same
        # physical plant is recognised later from a very different angle —
        # e.g. after the robot turns ~180° and comes back. The robot is
        # parked at the plant here, so the camera is naturally still.
        if self._reid_available():
            embeddings = []
            for s in range(REID_CAPTURE_SAMPLES):
                emb = self._embed_target(target_id, target.get("box"))
                if emb is not None:
                    embeddings.append(emb)
                if s < REID_CAPTURE_SAMPLES - 1 and self._sleep(REID_CAPTURE_INTERVAL):
                    break
            if embeddings:
                logger.info(NavLog.REID_CAPTURED.value,
                            len(embeddings))
                self.reid.register(embeddings)
            else:
                logger.warning("Auto: ReID embeddings unavailable for watered plant")

    def _water_target_and_recover(self, target: dict) -> bool:
        """Water one plant, then retreat or return to base.

        Returns True when the navigation loop should stop.
        """
        self._clear_watering_handoff()
        self._do_watering(target)
        if not self.is_active:
            return True

        found_next = self._reverse_after_watering(
            target.get("id"),
            target.get("box"),
            watch_for_plants=not self._returning,
        )
        if self._stop_event.is_set():
            return True

        # Mission target reached: after reversing, let the loop top look for
        # the base QR. Otherwise immediately retarget a plant spotted while
        # reversing or scan for one.
        if self._returning or found_next:
            return False

        return self._scan_for_target()

    # ------------------------------------------------------------------
    # Phase: retreat after watering
    # ------------------------------------------------------------------

    def _reverse_after_watering(
        self,
        target_id,
        fallback_box,
        watch_for_plants: bool,
    ) -> bool:
        """
        Always drive backward for POST_WATER_REVERSE_SECONDS after watering.
        While more plants are needed, watch for an unwatered plant so the main
        loop can immediately retarget it after the reverse completes. While
        returning, simply clear space so the loop can start looking for base QR.

        While retreating, extra ReID views of the just-watered plant are
        captured (gated on its ByteTrack id, so we never grab the wrong
        plant) and appended to its cluster. This teaches the cluster the
        farther/angled views, so once the plant is distant and its track id
        is lost it is still recognised as watered — preventing the
        re-approach/retreat ping-pong. Capture is throttled by
        REID_RETREAT_INTERVAL and lives inside this loop, so it scales with
        POST_WATER_REVERSE_SECONDS automatically.
        """
        destination = "another plant" if watch_for_plants else "the base QR"
        logger.info(
            NavLog.POST_WATER_REVERSING.value,
            POST_WATER_REVERSE_SECONDS,
            destination,
        )
        self.motor.backward(AUTO_SPEED_BACKWARD)
        deadline = time.monotonic() + POST_WATER_REVERSE_SECONDS
        capture = self._reid_available() and target_id is not None
        next_capture = time.monotonic() + REID_RETREAT_INTERVAL
        found = False

        try:
            while time.monotonic() < deadline:
                if self._stop_event.is_set():
                    break
                unwatered = []
                if watch_for_plants:
                    unwatered, _, _ = self._snapshot_unwatered()
                if unwatered and not found:
                    logger.info(NavLog.RETREAT_NEW_PLANT.value,
                                unwatered[0].get("id"))
                    found = True

                # Enrich the watered plant's ReID cluster with farther views
                # while it is still tracked (still ID-filtered as watered).
                if capture and time.monotonic() >= next_capture:
                    next_capture = time.monotonic() + REID_RETREAT_INTERVAL
                    with self._lock:
                        still_tracked = any(
                            d.get("id") == target_id for d in self.latest_detections
                        )
                    if still_tracked:
                        emb = self._embed_target(target_id, fallback_box)
                        if emb is not None:
                            logger.info(NavLog.RETREAT_REID_VIEW.value, target_id)
                            self.reid.extend_last([emb])

                if self._stop_event.wait(timeout=RETREAT_POLL_PERIOD):
                    break
        finally:
            self.motor.stop()

        return found

    # ------------------------------------------------------------------
    # Phase: scan
    # ------------------------------------------------------------------

    def _ptz_available(self) -> bool:
        return self.ptz is not None and getattr(self.ptz, "enabled", False)

    def _scan_for_target(self) -> bool:
        """Original simple scan: rotate toward the side the plant was most
        recently seen, briefly, then stop and wait for fresh detections.

        Short turns let multiple loop iterations refine alignment without
        overshoot. Returns True if a stop was requested mid-scan.
        """
        scan_direction = self._last_known_zone
        if scan_direction == "RIGHT":
            logger.debug("Auto: scanning RIGHT (last seen on right)")
            self.motor.right(AUTO_SPEED_TURN)
        else:
            logger.debug("Auto: scanning LEFT (last seen on left)")
            self.motor.left(AUTO_SPEED_TURN)

        if self._sleep(SCAN_TURN_DURATION):
            self.motor.stop()
            return True

        # Mark scan complete — the next detection cycle will stop and
        # re-evaluate instead of stacking another turn.
        self._just_finished_scan = True
        self.motor.stop()
        return self._wait_for_detections(SLEEP_WAIT_YOLO)

    # ------------------------------------------------------------------
    # Phase: return to base (QR code)
    # ------------------------------------------------------------------

    def _detect_base(self, frame) -> tuple[list[float] | None, str | None]:
        """Locate the base QR and, when possible, confirm its payload.

        ``detect()`` (locate only) survives motion blur far better than a full
        ``detectAndDecode()`` — the latter failing intermittently is what makes
        the approach flicker. We still decode to confirm identity, but once the
        base payload has been confirmed this run we accept a located quad even
        on frames where decoding fails, so the box stays stable while driving.

        Returns ``([x1, y1, x2, y2], payload)`` or ``(None, None)``.
        """
        if frame is None:
            return None, None
        try:
            ok, points = self._qr.detect(frame)
        except cv2.error:
            return None, None
        if not ok or points is None:
            return None, None

        pts = points.reshape(-1, 2)
        box = [
            float(pts[:, 0].min()), float(pts[:, 1].min()),
            float(pts[:, 0].max()), float(pts[:, 1].max()),
        ]

        data = ""
        try:
            data, _ = self._qr.decode(frame, points)
        except cv2.error:
            data = ""

        if data:
            if BASE_QR_PAYLOAD and data != BASE_QR_PAYLOAD:
                return None, None          # a different QR — not our base
            self._base_decoded_once = True
            self._base_payload = data
            return box, data

        # Decode failed this frame: trust the located quad only after the base
        # identity has already been confirmed at least once this run.
        if self._base_decoded_once:
            return box, self._base_payload
        return None, None

    def _annotate_base(self, frame, box, payload):
        """Draw the base QR box / a 'returning to base' banner on the stream."""
        annotated = frame.copy()
        h, w = annotated.shape[:2]
        cv2.rectangle(annotated, (0, 0), (w, 50), (0, 0, 0), -1)
        banner = (f"RETURNING TO BASE — '{payload}'" if box is not None
                  else "RETURNING TO BASE — searching for QR...")
        cv2.putText(annotated, banner, (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        if box is not None:
            x1, y1, x2, y2 = [int(v) for v in box]
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 200, 255), 3)
            cv2.putText(annotated, "BASE", (x1, max(65, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        return annotated

    def _scan_for_base(self):
        """Rotate toward the side the base was last seen, then settle so the
        inference loop can grab a fresh QR read. Searches indefinitely."""
        direction = self._last_known_zone
        logger.info(NavLog.RETURN_SCANNING.value, direction, BASE_QR_PAYLOAD)
        if direction == "RIGHT":
            self.motor.right(AUTO_SPEED_TURN)
        else:
            self.motor.left(AUTO_SPEED_TURN)
        if self._sleep(SCAN_TURN_DURATION):
            self.motor.stop()
            return
        self.motor.stop()
        self._sleep(SLEEP_WAIT_YOLO)

    def _base_arrived(self, area_ratio: float) -> bool:
        """Base reached when centered AND the front distance confirms it.

        Prefers the ultrasonic (centered QR + front <= BASE_ARRIVAL_DISTANCE_CM);
        falls back to QR box-area when the sensor is unavailable / silent.
        """
        if self._ultrasonic_ready():
            cm = self.ultrasonic.distance_cm()
            if cm is not None:
                arrived = cm <= BASE_ARRIVAL_DISTANCE_CM
                logger.info(
                    "Auto: base arrival gate — front=%.1f cm (threshold=%d cm) → %s",
                    cm, BASE_ARRIVAL_DISTANCE_CM, "ARRIVED" if arrived else "approach",
                )
                return arrived
            logger.debug("Auto: no ultrasonic reading at base — falling back to QR area")
        return area_ratio >= BASE_ARRIVAL_AREA_RATIO

    def _celebrate_and_spin(self):
        """Announce arrival on the buzzer, then spin ~180° in place to face away
        from the base before the mission ends."""
        if self.buzzer is not None:
            self.buzzer.announce_arrival()
        logger.info(NavLog.BASE_SPIN.value, BASE_SPIN_SPEED, BASE_SPIN_SECONDS)
        self.motor.left(BASE_SPIN_SPEED)
        self._sleep(BASE_SPIN_SECONDS)
        self.motor.stop()

    def _step_return_to_base(self) -> bool:
        """One step of the drive-home behaviour. Returns True once the robot
        has arrived at the base (mission complete)."""
        with self._base_lock:
            base = self._latest_base

        # Base not in view → scan for it (forever, per design).
        if base is None:
            self._scan_for_base()
            return False

        box, w, h = base["box"], base["w"], base["h"]
        x1, y1, x2, y2 = box
        obj_center_x = (x1 + x2) / 2.0
        zone = self._get_zone(obj_center_x, w)
        self._last_known_zone = "LEFT" if obj_center_x < w / 2.0 else "RIGHT"

        frame_area = w * h
        area_ratio = ((x2 - x1) * (y2 - y1)) / frame_area if frame_area > 0 else 0.0
        logger.info(NavLog.BASE_DETECTED.value, base["payload"], zone, area_ratio)

        # Arrived? Ultrasonic confirms physical proximity (centered + close).
        if zone == "CENTER" and self._base_arrived(area_ratio):
            self.motor.stop()
            logger.info(NavLog.ARRIVED_BASE.value, base["payload"], area_ratio)
            self._celebrate_and_spin()
            self.is_active = False
            return True

        if zone == "LEFT":
            logger.info(NavLog.BASE_TURN.value, "LEFT", "left",
                        AUTO_SPEED_TURN, MOVE_TURN_DURATION)
            self.motor.left(AUTO_SPEED_TURN)
            if self._sleep(MOVE_TURN_DURATION):
                return False
        elif zone == "RIGHT":
            logger.info(NavLog.BASE_TURN.value, "RIGHT", "right",
                        AUTO_SPEED_TURN, MOVE_TURN_DURATION)
            self.motor.right(AUTO_SPEED_TURN)
            if self._sleep(MOVE_TURN_DURATION):
                return False
        else:
            logger.info(NavLog.BASE_APPROACH.value, area_ratio, BASE_ARRIVAL_AREA_RATIO)
            self.motor.forward(AUTO_SPEED_FORWARD)
            if self._sleep(MOVE_FORWARD_DURATION):
                return False

        self.motor.stop()
        self._sleep(SLEEP_WAIT_YOLO)   # let the inference loop grab a fresh QR
        return False

    # ------------------------------------------------------------------
    # Obstacle avoidance
    # ------------------------------------------------------------------

    def _obstacle_detected(self) -> bool:
        return (
            self.ultrasonic is not None
            and getattr(self.ultrasonic, "enabled", False)
            and self.ultrasonic.is_obstacle()
        )

    def _acquire_watering_handoff_if_ready(self) -> bool:
        """Give a centered close-up YOLO plant priority over obstacle logic."""
        if self._returning or not self._ultrasonic_ready():
            return False
        detections, width, height = self._snapshot_unwatered()
        if not detections:
            return False
        best = max(detections, key=lambda d: d["confidence"])
        x1, _, x2, _ = best["box"]
        if self._get_zone((x1 + x2) / 2.0, width) != "CENTER":
            return False
        return self._start_watering_handoff(best, width, height)

    def _avoid_obstacle(self) -> bool:
        # ESP32 SCAN reads front, then left, then right, and re-centres.
        direction = self.ultrasonic.best_direction() or "right"
        logger.info(NavLog.OBSTACLE_AVOIDANCE.value, direction)

        self.motor.stop()
        logger.info(NavLog.OBSTACLE_BACKUP.value, ULTRASONIC_AVOID_BACKWARD_SECONDS)
        self.motor.backward(ULTRASONIC_AVOID_SPEED)
        if self._sleep(ULTRASONIC_AVOID_BACKWARD_SECONDS):
            return True

        turn = self.motor.right if direction == "right" else self.motor.left
        restore = self.motor.left if direction == "right" else self.motor.right

        self.motor.stop()
        logger.info(NavLog.OBSTACLE_SWING.value, direction, ULTRASONIC_AVOID_TURN_SECONDS)
        turn(ULTRASONIC_AVOID_SPEED)
        if self._sleep(ULTRASONIC_AVOID_TURN_SECONDS):
            return True

        self.motor.stop()
        logger.info(NavLog.OBSTACLE_FORWARD.value, ULTRASONIC_AVOID_FORWARD_SECONDS)
        self.motor.forward(ULTRASONIC_AVOID_SPEED)
        if self._sleep(ULTRASONIC_AVOID_FORWARD_SECONDS):
            return True

        self.motor.stop()
        restore_direction = "left" if direction == "right" else "right"
        logger.info(
            NavLog.OBSTACLE_RESTORE.value,
            restore_direction,
            ULTRASONIC_AVOID_TURN_SECONDS,
        )
        restore(ULTRASONIC_AVOID_SPEED)
        if self._sleep(ULTRASONIC_AVOID_TURN_SECONDS):
            return True

        self.motor.stop()
        # No re-scan here: the SCAN at the top of this method already re-centred
        # the servo, so another ~2s sweep would only add latency.
        return self._wait_for_detections(SLEEP_WAIT_YOLO)

    # ------------------------------------------------------------------
    # Main navigation loop
    # ------------------------------------------------------------------

    def _loop(self):
        try:
            while self.is_active:
                # Once YOLO has centered a sufficiently large plant box, let
                # ultrasonic own the final close approach. This survives YOLO
                # losing the oversized plant image and suppresses obstacle
                # avoidance only for the bounded handoff window.
                if self._watering_handoff_active():
                    if self._step_watering_handoff():
                        break
                    continue

                if self._camera_alive() and self._acquire_watering_handoff_if_ready():
                    if self._step_watering_handoff():
                        break
                    continue

                # Without a YOLO watering handoff, a near ultrasonic reading is
                # an obstacle: scan left/right, reverse, swing, go around, and
                # undo the swing before continuing.
                if self._obstacle_detected():
                    if self._avoid_obstacle():
                        break
                    continue

                # Camera dropped out — hold still instead of driving/scanning
                # blind. Resumes automatically when frames come back.
                if not self._camera_alive():
                    self.motor.stop()
                    if not self._camera_warned:
                        logger.warning(NavLog.CAMERA_STALE.value, CAMERA_STALE_SECONDS)
                        self._camera_warned = True
                    if self._wait_for_detections(SLEEP_WAIT_YOLO):
                        break
                    continue
                if self._camera_warned:
                    logger.info(NavLog.CAMERA_BACK.value)
                    self._camera_warned = False

                # Mission complete watering — drive home to the base QR.
                if self._returning:
                    if self._step_return_to_base():
                        break          # arrived at base; end the mission
                    continue

                # 1. Snapshot latest unwatered detections
                detections, width, height = self._snapshot_unwatered()

                # 2. Pick best detection
                best = max(detections, key=lambda d: d["confidence"]) if detections else None

                # 3. Act on detection
                if best is not None:
                    self._frames_without_target = 0
                    x1, y1, x2, y2 = best["box"]
                    obj_center_x = (x1 + x2) / 2.0
                    zone = self._get_zone(obj_center_x, width)

                    # Track which side the plant was last seen on, for scan bias
                    if obj_center_x < width / 2.0:
                        self._last_known_zone = "LEFT"
                    else:
                        self._last_known_zone = "RIGHT"

                    self._update_oscillation(zone)
                    turn_speed = self._get_turn_speed()

                    # If we just finished a scan, settle before stacking a turn
                    if self._just_finished_scan:
                        logger.info(NavLog.SCAN_TARGET_ACQUIRED.value)
                        self._just_finished_scan = False
                        self.motor.stop()
                        if self._wait_for_detections(SLEEP_WAIT_YOLO):
                            break
                        continue

                    # --- ARRIVED fallback: ultrasonic unavailable ---
                    if (
                        zone == "CENTER"
                        and not self._ultrasonic_ready()
                        and self._is_arrived([x1, y1, x2, y2], width, height)
                    ):
                        if self._water_target_and_recover(best):
                            break
                        continue

                    # --- TURN LEFT ---
                    if zone == "LEFT":
                        logger.debug("Auto: plant LEFT → turning left (speed=%.2f, %.1fs)",
                                     turn_speed, MOVE_TURN_DURATION)
                        self.motor.left(turn_speed)
                        if self._sleep(MOVE_TURN_DURATION):
                            break

                    # --- TURN RIGHT ---
                    elif zone == "RIGHT":
                        logger.debug("Auto: plant RIGHT → turning right (speed=%.2f, %.1fs)",
                                     turn_speed, MOVE_TURN_DURATION)
                        self.motor.right(turn_speed)
                        if self._sleep(MOVE_TURN_DURATION):
                            break

                    # --- CENTER but not arrived → approach ---
                    else:
                        logger.debug("Auto: plant CENTER → forward")
                        self.motor.forward(AUTO_SPEED_FORWARD)
                        if self._sleep(MOVE_FORWARD_DURATION):
                            break

                    self.motor.stop()
                    if self._wait_for_detections(SLEEP_WAIT_YOLO):
                        break

                # 4. Target lost briefly — hold and wait
                elif self._frames_without_target < LOST_THRESHOLD:
                    self._frames_without_target += 1
                    logger.debug("Auto: target lost briefly (%d/%d) — holding",
                                 self._frames_without_target, LOST_THRESHOLD)
                    self.motor.stop()
                    if self._wait_for_detections(SLEEP_WAIT_YOLO):
                        break

                # 5. Target genuinely lost — scan toward last-seen side
                else:
                    self._frames_without_target += 1
                    if self._scan_for_target():
                        break

        finally:
            self.motor.stop()
            self.pump.off()
            self._is_watering = False
            self._clear_watering_handoff()
            if self._ptz_available():
                try:
                    self.ptz.look_center()
                except Exception:  # noqa: BLE001
                    pass
            if self.ultrasonic is not None:
                self.ultrasonic.set_auto_active(False)
            self.is_active = False
            SAQI_LEDS.detect(False)
            SAQI_LEDS.mode(LEDState.MANUAL)
            if self.buzzer is not None:
                self.buzzer.set_auto_active(False)
            logger.info(NavLog.LOOP_EXITED.value)
