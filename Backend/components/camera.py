"""
Camera Controller Component
============================
Continuously captures frames from the AXIS 213 network camera.

A background thread drains the camera buffer so get_frame() returns the most
recent frame with minimal latency.
"""

import logging
import threading
import time

import cv2

from components.logs import CameraLog

logger = logging.getLogger(__name__)

_DEFAULT_SOURCE = "http://169.254.138.53/axis-cgi/mjpg/video.cgi"
_RECONNECT_DELAY = 1


class RobotCamera:
    """Thread-safe singleton camera with a background capture loop."""

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
        return cls._instance

    def __init__(self, source: str = _DEFAULT_SOURCE):
        if self._initialized:
            return

        self.source = source
        self.cap = cv2.VideoCapture(self.source)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.latest_frame = None
        self._frame_lock = threading.Lock()
        self.running = True
        self._initialized = True

        self._thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="camera-capture"
        )
        self._thread.start()
        logger.info(CameraLog.INITIALISED.value, self.source)

    def _capture_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.flip(frame, -1)
                with self._frame_lock:
                    self.latest_frame = frame
                continue

            logger.warning("Camera read failed; reconnecting in %ds", _RECONNECT_DELAY)
            self.cap.release()
            time.sleep(_RECONNECT_DELAY)
            self.cap = cv2.VideoCapture(self.source)

    def get_frame(self) -> bytes | None:
        """Return the latest frame as JPEG bytes."""
        with self._frame_lock:
            if self.latest_frame is None:
                return None
            frame = self.latest_frame.copy()

        ok, jpeg = cv2.imencode(".jpg", frame)
        return jpeg.tobytes() if ok else None

    def get_raw_frame(self):
        """Return the latest raw OpenCV frame."""
        with self._frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def close(self):
        self.running = False
        self._thread.join(timeout=3)
        self.cap.release()
        logger.info(CameraLog.CLOSED.value)

        with self._lock:
            RobotCamera._instance = None
            self._initialized = False


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    camera = RobotCamera()

    print("Waiting for first frame...")
    for _ in range(30):
        frame = camera.get_frame()
        if frame:
            print(f"Got frame ({len(frame):,} bytes)")
            break
        time.sleep(0.5)
    else:
        print("No frame received after 15 s")

    camera.close()
