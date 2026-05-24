"""Small helpers shared by route modules."""

from fastapi import HTTPException

from app_state import app_state


def component(name: str):
    value = getattr(app_state, name)
    if value is None:
        raise HTTPException(status_code=503, detail=f"{name} is not available")
    return value
