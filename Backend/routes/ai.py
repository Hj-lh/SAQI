"""AI detection routes."""

from fastapi import APIRouter
from fastapi.responses import JSONResponse

from routes.utils import component

router = APIRouter(prefix="/ai", tags=["ai"])


@router.get("/detect")
def ai_detect():
    detector = component("ai")
    camera = component("camera")

    if not detector.enabled:
        return JSONResponse(
            status_code=503,
            content={"status": "error", "message": "AI model not loaded"},
        )

    frame = camera.get_raw_frame()
    if frame is None:
        return JSONResponse(
            status_code=503,
            content={"status": "error", "message": "No camera frame available"},
        )

    detections = detector.detect(frame)
    return {"status": "ok", "count": len(detections), "detections": detections}
