"""Motor control routes."""

import logging

from fastapi import APIRouter, Query
from fastapi.responses import JSONResponse

from app_state import app_state
from routes.utils import component

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/motor", tags=["motor"])


def _take_manual_control() -> bool:
    """Stop autonomous navigation if it is running, so a manual command isn't
    immediately overridden by the navigator's background loop. Returns True if
    auto mode was actually stopped."""
    nav = app_state.navigator
    if nav is not None and nav.is_active:
        logger.info("Manual motor command received — stopping autonomous mode")
        nav.stop()
        return True
    return False


@router.post("/move")
def move_robot(
    direction: str = Query(..., description="forward | backward | left | right"),
    speed: float = Query(0.8, ge=0.0, le=1.0, description="Speed 0.0 to 1.0"),
):
    motor = component("motor")
    actions = {
        "forward": motor.forward,
        "backward": motor.backward,
        "left": motor.left,
        "right": motor.right,
    }

    action = actions.get(direction)
    if action is None:
        return JSONResponse(
            status_code=400,
            content={"status": "error", "message": f"Invalid direction: {direction}"},
        )

    auto_stopped = _take_manual_control()
    action(speed)
    return {"status": "moving", "direction": direction, "speed": speed,
            "auto_stopped": auto_stopped}


@router.post("/stop")
def stop_robot():
    # A manual Stop must also end autonomous mode — otherwise the navigator's
    # background loop keeps driving/scanning a second later.
    auto_stopped = _take_manual_control()
    component("motor").stop()
    return {"status": "stopped", "auto_stopped": auto_stopped}
