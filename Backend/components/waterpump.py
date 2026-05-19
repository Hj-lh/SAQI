"""
Water Pump Controller Component
================================
Controls a water pump via a relay (gpiozero). The control pin and the
active-low setting come from components.config (single source of truth) —
see PUMP_PIN / PUMP_ACTIVE_HIGH there, not hardcoded here.
"""

import logging
from gpiozero import OutputDevice

from components.config import PUMP_PIN, PUMP_ACTIVE_HIGH

logger = logging.getLogger(__name__)


class WaterPumpController:
    """Simple on/off controller for a relay-driven water pump."""

    def __init__(self, pin: int = PUMP_PIN, active_high: bool = PUMP_ACTIVE_HIGH):
        self.relay = OutputDevice(pin, active_high=active_high, initial_value=False)
        self._is_on = False
        logger.info(
            "WaterPumpController initialised  (GPIO%d, active_high=%s)",
            pin, active_high,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def on(self):
        """Turn the water pump ON."""
        self.relay.on()
        self._is_on = True
        logger.info("Water pump ON")

    def off(self):
        """Turn the water pump OFF."""
        self.relay.off()
        self._is_on = False
        logger.info("Water pump OFF")

    @property
    def is_on(self) -> bool:
        """Return whether the pump is currently running."""
        return self._is_on

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self):
        """Turn off the pump and release GPIO resources."""
        self.off()
        self.relay.close()
        logger.info("WaterPumpController closed")


# ------------------------------------------------------------------
# Quick self-test (python -m components.waterpump)
# ------------------------------------------------------------------
if __name__ == "__main__":
    from time import sleep

    logging.basicConfig(level=logging.DEBUG)
    pump = WaterPumpController()

    try:
        print("▶ Pump ON for 2 s")
        pump.on()
        sleep(2)

        print("■ Pump OFF")
        pump.off()
    finally:
        pump.close()
