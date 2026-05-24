"""Motor control routes."""

from fastapi import APIRouter, Query
from fastapi.responses import JSONResponse

from routes.utils import component

router = APIRouter(prefix="/motor", tags=["motor"])


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

    action(speed)
    return {"status": "moving", "direction": direction, "speed": speed}


@router.post("/stop")
def stop_robot():
    component("motor").stop()
    return {"status": "stopped"}
