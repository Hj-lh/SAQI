"""
Motor Controller Component
==========================
Differential-drive motor controller using dual BTS7960 drivers.
Communicates via hardware PWM using the gpiozero library.
"""

import logging
from gpiozero import Robot, DigitalOutputDevice

from components.config import (
    MOTOR_LEFT_FWD_PIN, MOTOR_LEFT_BWD_PIN, MOTOR_LEFT_EN_PIN, MOTOR_LEFT_EN_PIN1,
    MOTOR_RIGHT_FWD_PIN, MOTOR_RIGHT_BWD_PIN, MOTOR_RIGHT_EN_PIN, MOTOR_RIGHT_EN_PIN1,
    MOTOR_PWM_FREQUENCY,
)
from components.leds import SAQI_LEDS
from components.logs import MotorLog

logger = logging.getLogger(__name__)

class MotorController:
    """High-level differential-drive controller via PWM."""

    def __init__(self,
                 left_fwd_pin=MOTOR_LEFT_FWD_PIN, left_bwd_pin=MOTOR_LEFT_BWD_PIN,
                 left_en_pin=MOTOR_LEFT_EN_PIN, left_en_pin1=MOTOR_LEFT_EN_PIN1,
                 right_fwd_pin=MOTOR_RIGHT_FWD_PIN, right_bwd_pin=MOTOR_RIGHT_BWD_PIN,
                 right_en_pin=MOTOR_RIGHT_EN_PIN, right_en_pin1=MOTOR_RIGHT_EN_PIN1,
                 pwm_frequency=MOTOR_PWM_FREQUENCY):

        # --------------------------------------------------------------
        # SOFTWARE TRIM (Hardcoded here to keep main.py clean)
        # Adjust these values (0.0 to 1.0) to make the robot drive straight
        # --------------------------------------------------------------
        self.left_trim = 1.0
        self.right_trim = 1.0

        try:
            # 1. SETUP ENABLE PINS
            self.left_en = DigitalOutputDevice(left_en_pin)
            self.right_en = DigitalOutputDevice(right_en_pin)
            self.left_en1 = DigitalOutputDevice(left_en_pin1)
            self.right_en1 = DigitalOutputDevice(right_en_pin1)

            # Turn the motor drivers "ON" (Awake)
            self.left_en.on()
            self.right_en.on()
            self.left_en1.on()
            self.right_en1.on()

            # 2. SETUP PWM PINS
            self.robot = Robot(left=(left_fwd_pin, left_bwd_pin),
                               right=(right_fwd_pin, right_bwd_pin))

            # 3. OVERRIDE FREQUENCY (Prevents motor whining)
            self.robot.left_motor.forward_device.frequency = pwm_frequency
            self.robot.left_motor.backward_device.frequency = pwm_frequency
            self.robot.right_motor.forward_device.frequency = pwm_frequency
            self.robot.right_motor.backward_device.frequency = pwm_frequency

            logger.info(MotorLog.INITIALIZED.value)

        except Exception as e:
            logger.error("Failed to initialize BTS7960 PWM pins: %s", e)
            raise

    # ------------------------------------------------------------------
    # Low-level helper
    # ------------------------------------------------------------------

    def _set_robot_value(self, left_speed: float, right_speed: float):
        """Applies trim multipliers and sets the gpiozero Robot value."""

        # Apply the trim multipliers
        l_cmd = left_speed * self.left_trim
        r_cmd = right_speed * self.right_trim

        # Ensure we don't accidentally exceed the -1.0 to 1.0 limit
        l_cmd = max(-1.0, min(1.0, l_cmd))
        r_cmd = max(-1.0, min(1.0, r_cmd))

        self.robot.value = (l_cmd, r_cmd)

    # ------------------------------------------------------------------
    # High-level movement API
    # ------------------------------------------------------------------

    def backward(self, speed: float = 0.5):
        speed = abs(speed)
        logger.info(MotorLog.BACKWARD.value, speed)
        self._set_robot_value(speed, speed)
        SAQI_LEDS.moving(True)

    def forward(self, speed: float = 0.5):
        speed = abs(speed)
        logger.info(MotorLog.FORWARD.value, speed)
        self._set_robot_value(-speed, -speed)
        SAQI_LEDS.moving(True)

    def left(self, speed: float = 0.5):
        speed = abs(speed) * 2.0
        if speed > 1.0:
            speed = 1.0
        logger.info(MotorLog.LEFT.value, speed)
        self._set_robot_value(speed, -speed)
        SAQI_LEDS.moving(True)

    def right(self, speed: float = 0.5):
        speed = abs(speed) * 2.0
        if speed > 1.0:
            speed = 1.0
        logger.info(MotorLog.RIGHT.value, speed)
        self._set_robot_value(-speed, speed)
        SAQI_LEDS.moving(True)

    def stop(self):
        """Immediately shut down both motors."""
        logger.info(MotorLog.STOP.value)
        self.robot.stop()
        SAQI_LEDS.moving(False)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self):
        self.stop()

        # Put the motor drivers to sleep safely
        if hasattr(self, 'left_en'):
            self.left_en.off()
        if hasattr(self, 'right_en'):
            self.right_en.off()
        if hasattr(self, 'left_en1'):
            self.left_en1.off()
        if hasattr(self, 'right_en1'):
            self.right_en1.off()

        if hasattr(self, 'robot'):
            self.robot.close()

        logger.info(MotorLog.CLOSED.value)


if __name__ == "__main__":
    from time import sleep

    logging.basicConfig(level=logging.DEBUG)
    motor = MotorController()

    try:
        print("▶ Forward 80% for 3 s")
        motor.forward(0.8)
        sleep(3)

        print("▶ Backward 50% for 3 s")
        motor.backward(0.5)
        sleep(3)

        print("■ Stop")
        motor.stop()
    finally:
        motor.close()
