"""
Central hardware configuration.

Edit wiring and movement constants here so components do not hardcode them.
All Raspberry Pi pin numbers are BCM GPIO numbers.
"""

# Camera - AXIS 213 network camera MJPEG stream.
CAMERA_SOURCE = "http://169.254.138.53/axis-cgi/mjpg/video.cgi"

# Motors - 2x BTS7960 drivers via gpiozero Robot.
MOTOR_LEFT_FWD_PIN = 12
MOTOR_LEFT_BWD_PIN = 13
MOTOR_LEFT_EN_PIN = 5
MOTOR_LEFT_EN_PIN1 = 17

MOTOR_RIGHT_FWD_PIN = 22
MOTOR_RIGHT_BWD_PIN = 23
MOTOR_RIGHT_EN_PIN = 6
MOTOR_RIGHT_EN_PIN1 = 27

MOTOR_PWM_FREQUENCY = 1000

# Water pump relay.
PUMP_PIN = 25
PUMP_ACTIVE_HIGH = False

# Passive buzzer - direct GPIO PWM output.
BUZZER_PIN = 26
BUZZER_DUTY_CYCLE = 0.5

# Buzzer tunes. Watering starts and stops with distinct cues, with a selected
# song while the pump remains on. Set BUZZER_WATER_MUSIC_ENABLED to False to
# keep the simpler interval beep instead. The distance warning speeds up and
# rises in pitch as an obstacle approaches. At or below the collision distance,
# a steady high alarm sounds for two seconds, then stays quiet until the
# obstacle moves away far enough to re-arm it.
BUZZER_WATER_ON_LOW_HZ = 659
BUZZER_WATER_ON_HIGH_HZ = 988
BUZZER_WATER_OFF_HIGH_HZ = 784
BUZZER_WATER_OFF_LOW_HZ = 523
BUZZER_WATER_BEEP_HZ = 659
BUZZER_WATER_BEEP_SECONDS = 0.10
BUZZER_WATER_INTERVAL = 0.75
BUZZER_WATER_MUSIC_ENABLED = True
BUZZER_WATER_SONG = "ode_to_joy"

# Monophonic public-domain melodies for the passive buzzer. Each note is
# (frequency_hz, beats); use None for a rest. The settings UI exposes these
# keys as a dropdown while config.py stays the source of truth for defaults.
BUZZER_WATER_SONGS = {
    "ode_to_joy": {
        "label": "Ode to Joy",
        "beat_seconds": 0.18,
        "notes": (
            (659, 1), (659, 1), (698, 1), (784, 1),
            (784, 1), (698, 1), (659, 1), (587, 1),
            (523, 1), (523, 1), (587, 1), (659, 1),
            (659, 1.5), (587, 0.5), (587, 2), (None, 1),
            (659, 1), (659, 1), (698, 1), (784, 1),
            (784, 1), (698, 1), (659, 1), (587, 1),
            (523, 1), (523, 1), (587, 1), (659, 1),
            (587, 1.5), (523, 0.5), (523, 2), (None, 2),
        ),
    },
    "twinkle_twinkle": {
        "label": "Twinkle, Twinkle, Little Star",
        "beat_seconds": 0.20,
        "notes": (
            (523, 1), (523, 1), (784, 1), (784, 1),
            (880, 1), (880, 1), (784, 2),
            (698, 1), (698, 1), (659, 1), (659, 1),
            (587, 1), (587, 1), (523, 2), (None, 2),
        ),
    },
    "mary_had_a_little_lamb": {
        "label": "Mary Had a Little Lamb",
        "beat_seconds": 0.18,
        "notes": (
            (659, 1), (587, 1), (523, 1), (587, 1),
            (659, 1), (659, 1), (659, 2),
            (587, 1), (587, 1), (587, 2),
            (659, 1), (784, 1), (784, 2),
            (659, 1), (587, 1), (523, 1), (587, 1),
            (659, 1), (659, 1), (659, 1), (659, 1),
            (587, 1), (587, 1), (659, 1), (587, 1),
            (523, 2), (None, 2),
        ),
    },
    "happy_birthday": {
        "label": "Happy Birthday",
        "beat_seconds": 0.20,
        "notes": (
            (392, 0.75), (392, 0.25), (440, 1), (392, 1), (523, 1), (494, 2),
            (392, 0.75), (392, 0.25), (440, 1), (392, 1), (587, 1), (523, 2),
            (392, 0.75), (392, 0.25), (784, 1), (659, 1), (523, 1), (494, 1), (440, 2),
            (698, 0.75), (698, 0.25), (659, 1), (523, 1), (587, 1), (523, 2),
            (None, 2),
        ),
    },
    "jingle_bells": {
        "label": "Jingle Bells",
        "beat_seconds": 0.16,
        "notes": (
            (659, 1), (659, 1), (659, 2),
            (659, 1), (659, 1), (659, 2),
            (659, 1), (784, 1), (523, 1), (587, 1), (659, 3),
            (698, 1), (698, 1), (698, 1), (698, 1),
            (698, 1), (659, 1), (659, 1), (659, 0.5), (659, 0.5),
            (587, 1), (587, 1), (659, 1), (587, 2), (784, 2),
            (None, 2),
        ),
    },
}
BUZZER_PROXIMITY_START_CM = 35.0
BUZZER_COLLISION_CM = 17.0
BUZZER_COLLISION_RESET_CM = 20.0
BUZZER_PROXIMITY_LOW_HZ = 700
BUZZER_PROXIMITY_HIGH_HZ = 1400
BUZZER_PROXIMITY_SLOW_INTERVAL = 0.8
BUZZER_PROXIMITY_FAST_INTERVAL = 0.18
BUZZER_PROXIMITY_BEEP_SECONDS = 0.08
BUZZER_COLLISION_HZ = 1900
BUZZER_COLLISION_ALARM_SECONDS = 2.0

# Status LEDs - WS2812 / NeoPixel strip.
LED_PIN = 18
LED_COUNT = 8
LED_BRIGHTNESS = 0.20

# Ultrasonic sensor head - ESP32 + HC-SR04 + servo over USB serial.
ULTRASONIC_SERIAL_PORT = "/dev/ttyUSB0"
ULTRASONIC_SERIAL_BAUD = 115200
ULTRASONIC_SERIAL_TIMEOUT = 3.0  # > worst-case SCAN (~2s) so replies aren't cut off
ULTRASONIC_OBSTACLE_CM = 20
ULTRASONIC_POLL_INTERVAL = 0.5
ULTRASONIC_SCAN_COOLDOWN = 3.0
# In auto mode, a centered plant is watered once the front distance is within
# this range (physical confirmation that YOLO's "arrived" is real). Falls back
# to YOLO box-area when the sensor is unavailable. See AutoNavigator._ready_to_water.
WATER_DISTANCE_CM = 20

# Autonomous obstacle avoidance movement.
ULTRASONIC_AVOID_BACKWARD_SECONDS = 1.0
ULTRASONIC_AVOID_TURN_SECONDS = 0.8
ULTRASONIC_AVOID_FORWARD_SECONDS = 1.2
ULTRASONIC_AVOID_SPEED = 0.5

# --- Autonomous mission: return to base ---
# How many plants to water before heading home. After this many watering
# events the robot stops looking for plants and drives to the base QR code.
# Set to 0 to disable (water indefinitely, never return to base).
PLANTS_PER_RUN = 2

# Decoded text that identifies the home base. A QR code is only treated as
# the base when its payload matches this string exactly.
BASE_QR_PAYLOAD = "SAQI_BASE"

# The base is considered "reached" once its QR bounding box fills at least
# this fraction of the camera frame (same idea as ARRIVAL_AREA_RATIO). Used as
# a fallback when the ultrasonic is unavailable; otherwise distance wins.
BASE_ARRIVAL_AREA_RATIO = 0.35

# Ultrasonic confirmation for base arrival: once the base QR is centered AND the
# front distance is within this range, the robot is "home". Preferred over the
# area ratio above when the sensor is available.
BASE_ARRIVAL_DISTANCE_CM = 40

# Keep the last base sighting alive this long through brief detection misses, so
# the QR box (and the approach) stops flickering when a frame fails to decode.
BASE_SIGHTING_HOLD_SECONDS = 0.6

# On arrival the robot spins ~180° in place (to face away from the base) before
# the mission ends: turn left at this speed for this many seconds.
BASE_SPIN_SPEED = 0.60
BASE_SPIN_SECONDS = 3.75

# ---------------------------------------------------------------------------
# Autonomous navigation tuning
# These drive the AutoNavigator behaviour. They are read when an auto run
# starts, so changes take effect the next time you press Start.
# ---------------------------------------------------------------------------

# Motor speeds (0.0–1.0) used by the navigator.
AUTO_SPEED_FORWARD     = 0.6     # forward approach speed
AUTO_SPEED_BACKWARD    = 0.5     # reverse / retreat speed
AUTO_SPEED_TURN        = 0.75    # normal turn speed
AUTO_SPEED_TURN_GENTLE = 0.375   # slower turn used when oscillation is detected
MIN_TURN_SPEED         = 0.375   # floor so turns never stall the motors

# How long the motors run per discrete action (seconds).
MOVE_TURN_DURATION     = 0.5     # one left/right correction
MOVE_FORWARD_DURATION  = 1.0     # one forward approach step

# After stopping, how long to wait for fresh YOLO detections (seconds).
SLEEP_WAIT_YOLO        = 1.5

# Scanning: how long each rotate-to-search turn lasts (seconds).
SCAN_TURN_DURATION     = 1.0

# ReID multi-view capture at watering time.
REID_CAPTURE_SAMPLES   = 5       # embeddings captured while parked at a plant
REID_CAPTURE_INTERVAL  = 0.2     # seconds between those samples
REID_RETREAT_INTERVAL  = 0.4     # seconds between ReID captures while retreating

# Geometry: middle fraction of the frame treated as "CENTER".
CENTER_MARGIN          = 0.33

# Arrival: water once the plant box fills this fraction of the frame.
ARRIVAL_AREA_RATIO     = 0.5

# Watering run length (seconds the pump stays on per plant).
WATERING_DURATION      = 5

# Retreat after watering.
RETREAT_DURATION       = 4.0     # seconds the robot drives backward
RETREAT_POLL_PERIOD    = 0.1     # how often to check for new targets while reversing

# How many cycles with no detection before switching from holding to scanning.
LOST_THRESHOLD         = 3
