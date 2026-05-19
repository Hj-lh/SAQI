"""
Central Hardware Configuration
==============================
SINGLE SOURCE OF TRUTH for every GPIO pin and the wiring parameters that go
with them. Edit values HERE only — `motor.py` and `waterpump.py` import from
this module, so changing a pin in one place updates the whole system.

All pin numbers are BCM GPIO numbering.
"""

# ------------------------------------------------------------------
# Motors — 2x BTS7960 drivers via gpiozero Robot (hardware PWM)
# ------------------------------------------------------------------
MOTOR_LEFT_FWD_PIN   = 12     # left  RPWM (forward)
MOTOR_LEFT_BWD_PIN   = 13     # left  LPWM (backward)
MOTOR_LEFT_EN_PIN    = 5      # left  BTS7960 enable A
MOTOR_LEFT_EN_PIN1   = 17     # left  BTS7960 enable B

MOTOR_RIGHT_FWD_PIN  = 22     # right RPWM (forward)
MOTOR_RIGHT_BWD_PIN  = 23     # right LPWM (backward)
MOTOR_RIGHT_EN_PIN   = 6      # right BTS7960 enable A
MOTOR_RIGHT_EN_PIN1  = 27     # right BTS7960 enable B

MOTOR_PWM_FREQUENCY  = 1000   # Hz — overrides gpiozero default (anti-whine)

# ------------------------------------------------------------------
# Water pump — relay
# ------------------------------------------------------------------
PUMP_PIN         = 25         # relay control pin
PUMP_ACTIVE_HIGH = False      # relay board is active-LOW
