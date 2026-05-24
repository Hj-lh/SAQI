"""Water pump routes."""

from fastapi import APIRouter

from routes.utils import component

router = APIRouter(prefix="/pump", tags=["pump"])


@router.post("/on")
def pump_on():
    component("pump").on()
    return {"status": "pump_on"}


@router.post("/off")
def pump_off():
    component("pump").off()
    return {"status": "pump_off"}


@router.get("/status")
def pump_status():
    return {"is_on": component("pump").is_on}
