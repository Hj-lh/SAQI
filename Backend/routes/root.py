"""Root API metadata."""

from fastapi import APIRouter

router = APIRouter()


@router.get("/")
def index():
    return {
        "message": "AgriBot System Online",
        "endpoints": {
            "camera": ["/camera/feed", "/camera/snapshot"],
            "motor": ["/motor/move?direction=forward&speed=0.8", "/motor/stop"],
            "pump": ["/pump/on", "/pump/off", "/pump/status"],
            "ai": ["/ai/detect"],
            "ptz": [
                "/camera/ptz/pan?angle=45",
                "/camera/ptz/tilt?angle=-10",
                "/camera/ptz/move?direction=left&speed=50",
                "/camera/ptz/stop",
                "/camera/ptz/zoom?level=2000",
                "/camera/ptz/look?direction=right&degrees=90",
                "/camera/ptz/center",
                "/camera/ptz/home",
                "/camera/ptz/preset?number=1",
                "/camera/ptz/position",
            ],
        },
    }
