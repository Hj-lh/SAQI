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
  APPROACHING → box large enough    → WATERING
  WATERING    → done                → RETREATING (watch for next plant)
  RETREATING  → new target found    → TRACKING
  RETREATING  → nothing seen        → SCANNING
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
from components.config import (
    ULTRASONIC_AVOID_BACKWARD_SECONDS,
    ULTRASONIC_AVOID_FORWARD_SECONDS,
    ULTRASONIC_AVOID_SPEED,
    ULTRASONIC_AVOID_TURN_SECONDS,
)

from components.logs import NavLog
logger = logging.getLogger(__name__)

# --- Speeds ---
AUTO_SPEED_FORWARD     = 0.6
AUTO_SPEED_BACKWARD    = 0.5
AUTO_SPEED_TURN        = 0.75
AUTO_SPEED_TURN_GENTLE = 0.375
MIN_TURN_SPEED         = 0.375

# --- Durations (how long motors run per action) ---
MOVE_TURN_DURATION    = 0.5
MOVE_FORWARD_DURATION = 1.0

# --- After stopping, wait for fresh YOLO detections ---
SLEEP_WAIT_YOLO = 1.5

# --- Scanning ---
SCAN_TURN_DURATION = 1.0

# --- ReID multi-view capture (at watering time) ---
REID_CAPTURE_SAMPLES  = 5
REID_CAPTURE_INTERVAL = 0.2   # seconds between embedding samples

# --- ReID capture while retreating (far/angled views) ---
# Time-throttled INSIDE the retreat loop, which is bounded by
# RETREAT_DURATION — so shortening the retreat shortens capture too.
REID_RETREAT_INTERVAL = 0.4   # seconds between retreat-time ReID captures

# --- Geometry ---
CENTER_MARGIN = 0.33  # Middle third of the frame is "CENTER"

# --- Arrival ---
ARRIVAL_AREA_RATIO = 0.5

# --- Watering ---
WATERING_DURATION = 5  # seconds

# --- Retreat after watering ---
RETREAT_DURATION   = 4.0   # seconds the robot drives backward
RETREAT_POLL_PERIOD = 0.1  # how often to check for new targets while reversing

# --- How many cycles with no detection before switching to scan ---
LOST_THRESHOLD = 3


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
    ):
        self.motor = motor
        self.pump = pump
        self.camera = camera
        self.detector = detector
        self.ptz = ptz                # may be None if camera control unavailable
        self.reid = reid              # may be None if torchvision unavailable
        self.ultrasonic = ultrasonic

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

        # Oscillation detection
        self._zone_history: list[str] = []
        self._oscillating = False

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
        self._zone_history.clear()
        self._oscillating = False
        self._is_watering = False
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

    def stop(self):
        if not self.is_active:
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

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_zone(self, obj_center_x: float, width: int) -> str:
        if obj_center_x < width * CENTER_MARGIN:
            return "LEFT"
        if obj_center_x > width * (1 - CENTER_MARGIN):
            return "RIGHT"
        return "CENTER"

    def _sleep(self, duration: float) -> bool:
        """Interruptible sleep. Returns True if we should stop."""
        return self._stop_event.wait(timeout=duration)

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
        x1, y1, x2, y2 = box
        box_area = (x2 - x1) * (y2 - y1)
        frame_area = frame_width * frame_height
        ratio = box_area / frame_area if frame_area > 0 else 0
        logger.debug("Auto: box area ratio = %.2f (threshold=%.2f)", ratio, ARRIVAL_AREA_RATIO)
        return ratio >= ARRIVAL_AREA_RATIO

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

    # ------------------------------------------------------------------
    # Phase: retreat after watering
    # ------------------------------------------------------------------

    def _retreat_and_search(self, target_id, fallback_box) -> bool:
        """
        Drive backward for RETREAT_DURATION while watching for any unwatered
        plant to appear in frame. Returns True if a new target showed up so
        the main loop can immediately retarget; False if the window expired
        with nothing seen.

        While retreating, extra ReID views of the just-watered plant are
        captured (gated on its ByteTrack id, so we never grab the wrong
        plant) and appended to its cluster. This teaches the cluster the
        farther/angled views, so once the plant is distant and its track id
        is lost it is still recognised as watered — preventing the
        re-approach/retreat ping-pong. Capture is throttled by
        REID_RETREAT_INTERVAL and lives inside this loop, so it scales with
        RETREAT_DURATION automatically (nothing hardcoded to 4 s).
        """
        logger.info(NavLog.RETREATING.value, RETREAT_DURATION)
        self.motor.backward(AUTO_SPEED_BACKWARD)
        deadline = time.monotonic() + RETREAT_DURATION
        capture = self._reid_available() and target_id is not None
        next_capture = time.monotonic() + REID_RETREAT_INTERVAL
        found = False

        try:
            while time.monotonic() < deadline:
                if self._stop_event.is_set():
                    break
                unwatered, _, _ = self._snapshot_unwatered()
                if unwatered:
                    logger.info(NavLog.RETREAT_NEW_PLANT.value,
                                unwatered[0].get("id"))
                    found = True
                    break

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
    # Obstacle avoidance
    # ------------------------------------------------------------------

    def _obstacle_detected(self) -> bool:
        return (
            self.ultrasonic is not None
            and getattr(self.ultrasonic, "enabled", False)
            and self.ultrasonic.is_obstacle()
        )

    def _avoid_obstacle(self) -> bool:
        direction = self.ultrasonic.best_direction() or "right"
        logger.info(NavLog.OBSTACLE_AVOIDANCE.value, direction)

        self.motor.stop()
        self.motor.backward(ULTRASONIC_AVOID_SPEED)
        if self._sleep(ULTRASONIC_AVOID_BACKWARD_SECONDS):
            return True

        turn = self.motor.right if direction == "right" else self.motor.left
        restore = self.motor.left if direction == "right" else self.motor.right

        self.motor.stop()
        turn(ULTRASONIC_AVOID_SPEED)
        if self._sleep(ULTRASONIC_AVOID_TURN_SECONDS):
            return True

        self.motor.stop()
        self.motor.forward(ULTRASONIC_AVOID_SPEED)
        if self._sleep(ULTRASONIC_AVOID_FORWARD_SECONDS):
            return True

        self.motor.stop()
        restore(ULTRASONIC_AVOID_SPEED)
        if self._sleep(ULTRASONIC_AVOID_TURN_SECONDS):
            return True

        self.motor.stop()
        if self.ultrasonic is not None:
            self.ultrasonic.scan()
        return self._wait_for_detections(SLEEP_WAIT_YOLO)

    # ------------------------------------------------------------------
    # Main navigation loop
    # ------------------------------------------------------------------

    def _loop(self):
        try:
            while self.is_active:
                if self._obstacle_detected():
                    if self._avoid_obstacle():
                        break
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

                    # --- ARRIVED: water the plant ---
                    if zone == "CENTER" and self._is_arrived([x1, y1, x2, y2], width, height):
                        self._do_watering(best)
                        if not self.is_active:
                            break

                        # Retreat and look for the next unwatered plant
                        if self._retreat_and_search(best.get("id"), best.get("box")):
                            # New target spotted — go straight back into the
                            # loop, which will re-acquire on the next pass.
                            continue
                        if self._stop_event.is_set():
                            break

                        # Nothing seen during retreat — simple scan
                        if self._scan_for_target():
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
            if self._ptz_available():
                try:
                    self.ptz.look_center()
                except Exception:  # noqa: BLE001
                    pass
            if self.ultrasonic is not None:
                self.ultrasonic.set_auto_active(False)
            self.is_active = False
            logger.info(NavLog.LOOP_EXITED.value)
