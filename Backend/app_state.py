"""Typed application state shared by routes and startup/shutdown code."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from components.ai import PlantDetector
    from components.automatic import AutoNavigator
    from components.camera import RobotCamera
    from components.camera_control import CameraPTZController
    from components.motor import MotorController
    from components.reid import PlantReID
    from components.ultrasonic import UltrasonicSensor
    from components.waterpump import WaterPumpController


@dataclass
class AppState:
    motor: MotorController | None = None
    camera: RobotCamera | None = None
    pump: WaterPumpController | None = None
    ai: PlantDetector | None = None
    ptz: CameraPTZController | None = None
    reid: PlantReID | None = None
    navigator: AutoNavigator | None = None
    ultrasonic: UltrasonicSensor | None = None


app_state = AppState()
