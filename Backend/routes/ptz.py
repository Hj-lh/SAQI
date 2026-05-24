"""PTZ camera control routes."""

from fastapi import APIRouter, Query
from fastapi.responses import JSONResponse

from app_state import app_state

router = APIRouter(prefix="/camera/ptz", tags=["ptz"])


def _ptz_or_503():
    ptz = app_state.ptz
    if ptz is None or not ptz.ensure_enabled():
        return None, JSONResponse(
            status_code=503,
            content={"status": "error", "message": "PTZ camera control not available"},
        )
    return ptz, None


@router.post("/pan")
def ptz_pan(angle: float = Query(..., ge=-180.0, le=180.0)):
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ok = ptz.pan(angle)
    return {"status": "ok" if ok else "error", "pan": angle}


@router.post("/tilt")
def ptz_tilt(angle: float = Query(..., ge=-90.0, le=90.0)):
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ok = ptz.tilt(angle)
    return {"status": "ok" if ok else "error", "tilt": angle}


@router.post("/zoom")
def ptz_zoom(level: int = Query(..., ge=1, le=9999)):
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ok = ptz.zoom(level)
    return {"status": "ok" if ok else "error", "zoom": level}


@router.post("/move")
def ptz_move(
    direction: str = Query(..., description="left | right | up | down"),
    speed: int = Query(50, ge=1, le=100),
):
    ptz, err = _ptz_or_503()
    if err is not None:
        return err

    vectors = {
        "left": (-speed, 0),
        "right": (speed, 0),
        "up": (0, speed),
        "down": (0, -speed),
    }
    vector = vectors.get(direction)
    if vector is None:
        return JSONResponse(
            status_code=400,
            content={"status": "error", "message": f"Invalid direction: {direction}"},
        )

    ok = ptz.move_continuous(vector[0], vector[1])
    return {"status": "ok" if ok else "error", "direction": direction, "speed": speed}


@router.post("/stop")
def ptz_stop():
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ok = ptz.stop_movement()
    return {"status": "ok" if ok else "error"}


@router.post("/look")
def ptz_look(
    direction: str = Query(..., description="left | right | up | down | center"),
    degrees: float = Query(90.0, ge=0.0, le=180.0),
):
    ptz, err = _ptz_or_503()
    if err is not None:
        return err

    actions = {
        "left": lambda: ptz.look_left(degrees),
        "right": lambda: ptz.look_right(degrees),
        "up": lambda: ptz.look_up(degrees),
        "down": lambda: ptz.look_down(degrees),
        "center": ptz.look_center,
    }
    action = actions.get(direction)
    if action is None:
        return JSONResponse(
            status_code=400,
            content={"status": "error", "message": f"Invalid direction: {direction}"},
        )

    ok = action()
    return {"status": "ok" if ok else "error", "direction": direction, "degrees": degrees}


@router.post("/center")
def ptz_center():
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ptz.stop_movement()
    ok = ptz.look_center()
    return {"status": "ok" if ok else "error"}


@router.post("/home")
def ptz_home():
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    ok = ptz.home()
    return {"status": "ok" if ok else "error"}


@router.post("/preset")
def ptz_preset(
    name: str | None = Query(None),
    number: int | None = Query(None, ge=1),
):
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    if name is None and number is None:
        return JSONResponse(
            status_code=400,
            content={"status": "error", "message": "Provide either 'name' or 'number'"},
        )

    ok = ptz.goto_preset(name) if name is not None else ptz.goto_preset_number(number)
    return {"status": "ok" if ok else "error", "name": name, "number": number}


@router.get("/position")
def ptz_position():
    ptz, err = _ptz_or_503()
    if err is not None:
        return err
    data = ptz.get_position()
    if data is None:
        return JSONResponse(
            status_code=502,
            content={"status": "error", "message": "Could not query PTZ position"},
        )
    return {"status": "ok", "position": data}
