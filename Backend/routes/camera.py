"""Camera stream and snapshot routes."""

import time

from fastapi import APIRouter, Query
from fastapi.responses import JSONResponse, StreamingResponse

from routes.utils import component

router = APIRouter(prefix="/camera", tags=["camera"])


def _mjpeg_generator(mode: str = "manual"):
    camera = component("camera")
    navigator = component("navigator")
    use_ai = mode == "automatic"

    while True:
        if use_ai:
            frame = navigator.get_annotated_jpeg() or camera.get_frame()
        else:
            frame = camera.get_frame()

        if frame is None:
            time.sleep(0.03)
            continue

        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        time.sleep(0.03)


@router.get("/feed")
def video_feed(mode: str = Query("manual", description="manual | automatic")):
    navigator = component("navigator")
    if mode == "automatic":
        navigator.start()
    elif mode == "manual":
        navigator.stop()
    else:
        return JSONResponse(
            status_code=400,
            content={"status": "error", "message": "mode must be manual or automatic"},
        )

    return StreamingResponse(
        _mjpeg_generator(mode),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@router.get("/snapshot")
def snapshot():
    frame = component("camera").get_frame()
    if frame is None:
        return JSONResponse(
            status_code=503,
            content={"status": "error", "message": "No frame available"},
        )
    return StreamingResponse(iter([frame]), media_type="image/jpeg")
