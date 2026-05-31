"""Ultrasonic sensor-head routes."""

from fastapi import APIRouter

from routes.utils import component

router = APIRouter(prefix="/ultrasonic", tags=["ultrasonic"])


def _fmt(value):
    """A non-negative number, else a dash for missing/invalid readings."""
    if isinstance(value, (int, float)) and value >= 0:
        return round(float(value), 1)
    return "-"


@router.get("/scan")
def ultrasonic_scan():
    """Sweep the head (front → right → left → center) and return distances.

    Any side the sensor could not read comes back as ``"-"``.
    """
    sensor = component("ultrasonic")
    scan = sensor.scan() if getattr(sensor, "enabled", False) else None
    if scan is None:
        return {"front": "-", "left": "-", "right": "-"}
    return {
        "front": _fmt(scan.get("front_cm")),
        "left": _fmt(scan.get("left_cm")),
        "right": _fmt(scan.get("right_cm")),
    }
