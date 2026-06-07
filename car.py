import logging
import time
from typing import Optional

from smbus import SMBus


PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_PRESCALE = 0xFE
PCA9685_LED0_ON_L = 0x06
PCA9685_LED0_ON_H = 0x07
PCA9685_LED0_OFF_L = 0x08
PCA9685_LED0_OFF_H = 0x09

CHANNEL_STEERING = 0
CHANNEL_THROTTLE = 1
CHANNEL_LIFT_DIR = 2
CHANNEL_LIFT_SPEED = 3
CHANNEL_DIFF_LOCK = 4

STEERING_CENTER_DEG = 90.0
STEERING_MAX_REL_DEG = 45.0
SPEED_MIN_PERCENT = -100.0
SPEED_MAX_PERCENT = 100.0

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


class PCA9685:
    """PCA9685 PWM driver."""

    def __init__(self, bus_num: int = 7, address: int = 0x40, frequency: int = 50):
        self.bus = SMBus(bus_num)
        self.address = address
        self.frequency = frequency
        self.pwm_range = 4096
        self._reset()
        self._set_pwm_frequency(frequency)

    def _reset(self) -> None:
        self.bus.write_byte_data(self.address, PCA9685_MODE1, 0x00)
        self.bus.write_byte_data(self.address, PCA9685_MODE2, 0x04)
        time.sleep(0.005)

    def _set_pwm_frequency(self, frequency: int) -> None:
        prescale_value = 25000000.0 / self.pwm_range / float(frequency) - 1.0
        prescale = int(round(prescale_value))

        old_mode = self.bus.read_byte_data(self.address, PCA9685_MODE1)
        new_mode = (old_mode & 0x7F) | 0x10
        self.bus.write_byte_data(self.address, PCA9685_MODE1, new_mode)
        self.bus.write_byte_data(self.address, PCA9685_PRESCALE, prescale)
        self.bus.write_byte_data(self.address, PCA9685_MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, PCA9685_MODE1, old_mode | 0x80)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        self.bus.write_byte_data(self.address, PCA9685_LED0_ON_L + 4 * channel, on & 0xFF)
        self.bus.write_byte_data(self.address, PCA9685_LED0_ON_H + 4 * channel, on >> 8)
        self.bus.write_byte_data(self.address, PCA9685_LED0_OFF_L + 4 * channel, off & 0xFF)
        self.bus.write_byte_data(self.address, PCA9685_LED0_OFF_H + 4 * channel, off >> 8)


class ServoController:
    """Steering servo and ESC controller."""

    def __init__(self, initialize_esc: bool = True):
        self.pca = PCA9685(bus_num=7, address=0x40, frequency=50)

        self.servo_min = 102
        self.servo_max = 512

        self.esc_min = 100
        self.esc_mid = 300
        self.esc_max = 500

        if initialize_esc:
            self.initialize_outputs()

    def initialize_outputs(self) -> None:
        logger.info("Initializing ESCs and servos")
        self.set_motor_speed(CHANNEL_THROTTLE, -100)
        time.sleep(1.0)
        self.set_motor_speed(CHANNEL_THROTTLE, 100)
        time.sleep(0.5)
        self.set_motor_speed(CHANNEL_THROTTLE, 0)
        time.sleep(0.5)

        self.set_motor_speed(CHANNEL_LIFT_SPEED, -100)
        time.sleep(1.0)
        self.set_motor_speed(CHANNEL_LIFT_SPEED, 100)
        time.sleep(0.5)
        self.set_motor_speed(CHANNEL_LIFT_SPEED, 0)
        time.sleep(0.5)

        self.set_servo_angle(CHANNEL_STEERING, STEERING_CENTER_DEG)
        self.set_servo_angle(CHANNEL_LIFT_DIR, 90)
        self.set_servo_angle(CHANNEL_DIFF_LOCK, 90)
        logger.info("ESCs and servos initialized")

    def angle_to_pulse(self, angle: float) -> int:
        angle = clamp(angle, 0.0, 180.0)
        return int(self.servo_min + (self.servo_max - self.servo_min) * angle / 180.0)

    def speed_to_pulse(self, speed: float) -> int:
        speed = clamp(speed, SPEED_MIN_PERCENT, SPEED_MAX_PERCENT)
        if speed >= 0.0:
            return int(self.esc_mid + (self.esc_max - self.esc_mid) * speed / 100.0)
        return int(self.esc_mid + (self.esc_mid - self.esc_min) * speed / 100.0)

    def set_servo_angle(self, channel: int, angle: float) -> None:
        pulse = self.angle_to_pulse(angle)
        self.pca.set_pwm(channel, 0, pulse)
        logger.debug("servo channel=%d angle=%.3f pulse=%d", channel, angle, pulse)

    def set_motor_speed(self, channel: int, speed: float) -> None:
        pulse = self.speed_to_pulse(speed)
        self.pca.set_pwm(channel, 0, pulse)
        logger.debug("motor channel=%d speed=%.3f%% pulse=%d", channel, speed, pulse)

    def set_drive(self, steer_angle: float, speed_percent: float) -> tuple[float, float]:
        steering_servo_angle = STEERING_CENTER_DEG + clamp(
            steer_angle,
            -STEERING_MAX_REL_DEG,
            STEERING_MAX_REL_DEG,
        )
        speed_percent = clamp(speed_percent, SPEED_MIN_PERCENT, SPEED_MAX_PERCENT)
        self.set_servo_angle(CHANNEL_STEERING, steering_servo_angle)
        self.set_motor_speed(CHANNEL_THROTTLE, speed_percent)
        return steering_servo_angle, speed_percent

    def stop(self) -> None:
        self.set_servo_angle(CHANNEL_STEERING, STEERING_CENTER_DEG)
        self.set_motor_speed(CHANNEL_THROTTLE, 0)

    def reset_auxiliary_outputs(self) -> None:
        self.set_servo_angle(CHANNEL_LIFT_DIR, 90)
        self.set_motor_speed(CHANNEL_LIFT_SPEED, 0)
        self.set_servo_angle(CHANNEL_DIFF_LOCK, 90)


_controller: Optional[ServoController] = None


def get_controller(initialize_esc: bool = True) -> ServoController:
    global _controller
    if _controller is None:
        _controller = ServoController(initialize_esc=initialize_esc)
    return _controller


def car_control(steer_angle: float, speed_percent: float) -> tuple[float, float]:
    """Control real-car steering and throttle.

    Args:
        steer_angle: Signed steering angle in degrees relative to center.
            Negative and positive values steer in opposite directions. The value
            is clipped to +/- STEERING_MAX_REL_DEG and then mapped to the servo
            range around the 90-degree center.
        speed_percent: Signed speed command in percent, clipped to [-100, 100].
            Positive drives forward, negative drives reverse/brake according to
            the ESC calibration.

    Returns:
        A tuple of (steering_servo_angle_deg, clipped_speed_percent) actually
        sent to the hardware.
    """
    return get_controller().set_drive(steer_angle, speed_percent)


def stop_car() -> None:
    get_controller().stop()


def reset_auxiliary_outputs() -> None:
    get_controller().reset_auxiliary_outputs()
