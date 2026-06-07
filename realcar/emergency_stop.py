#!/usr/bin/python3
"""Immediately stop real-car motor outputs without starting ROS."""

import argparse

from car import emergency_stop_car


def main() -> None:
    parser = argparse.ArgumentParser(description="Hold neutral, then disable the real-car motor PWM outputs.")
    parser.add_argument("--hold-seconds", type=float, default=1.0, help="Seconds to repeatedly send neutral before disabling PWM.")
    parser.add_argument(
        "--disable-motor-pwm",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Disable throttle and lift PWM after holding neutral.",
    )
    args = parser.parse_args()
    emergency_stop_car(hold_seconds=args.hold_seconds, disable_motor_pwm=args.disable_motor_pwm)


if __name__ == "__main__":
    main()
