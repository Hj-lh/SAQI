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

# ------------------------------------------------------------------
# Status LEDs — WS2812 / NeoPixel strip (8 LEDs)
# ------------------------------------------------------------------
LED_PIN        = 18           # BCM data pin (PWM). Mapped to board.D18.
LED_COUNT      = 8
LED_BRIGHTNESS = 0.20         # 0.0 – 1.0

# ------------------------------------------------------------------
# Ultrasonic — HC-SR04 (gpiozero.DistanceSensor)
# ------------------------------------------------------------------
ULTRASONIC_TRIG_PIN    = 20   # ← set to your wiring
ULTRASONIC_ECHO_PIN    = 21   # ← set to your wiring (5V→3V3 divider on ECHO!)
ULTRASONIC_MAX_CM      = 200  # sensor max range used for scaling
ULTRASONIC_OBSTACLE_CM = 30   # ULTRASONIC LED turns ON when distance < this
