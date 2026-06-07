from dataclasses import dataclass

import numpy as np


DEFAULT_STABLE_PID = {
    "sample_time": 0.05,
    "preview_time": 0.8,
    "wheelbase": 2.35,
    "steer_limit_deg": 45.0,
    "kp_yaw": 0.12,
    "ki_yaw": 0.0,
    "kd_yaw": 0.02,
    "kp_preview": 0.25,
    "ki_preview": 0.0,
    "kd_preview": 0.0,
    "kff_curvature": 0.0,
    "min_preview_distance": 0.35,
}


@dataclass
class PidController:
    preview_distance: float
    steer_limit_deg: float
    target_speed: float
    sample_time: float
    wheelbase: float
    kp_yaw: float
    ki_yaw: float
    kd_yaw: float
    kp_preview: float
    ki_preview: float
    kd_preview: float
    kff_curvature: float
    yaw_integral: float = 0.0
    preview_integral: float = 0.0
    last_yaw_error_deg: float = 0.0
    last_preview_error: float = 0.0
    integral_limit: float = 8.0


def build_pid_controller(
    speed: float = 10.0,
    sample_time: float = DEFAULT_STABLE_PID["sample_time"],
    preview_time: float = DEFAULT_STABLE_PID["preview_time"],
    wheelbase: float = DEFAULT_STABLE_PID["wheelbase"],
    steer_limit_deg: float = DEFAULT_STABLE_PID["steer_limit_deg"],
    kp_yaw: float = DEFAULT_STABLE_PID["kp_yaw"],
    ki_yaw: float = DEFAULT_STABLE_PID["ki_yaw"],
    kd_yaw: float = DEFAULT_STABLE_PID["kd_yaw"],
    kp_preview: float = DEFAULT_STABLE_PID["kp_preview"],
    ki_preview: float = DEFAULT_STABLE_PID["ki_preview"],
    kd_preview: float = DEFAULT_STABLE_PID["kd_preview"],
    kff_curvature: float = DEFAULT_STABLE_PID["kff_curvature"],
    min_preview_distance: float = DEFAULT_STABLE_PID["min_preview_distance"],
) -> PidController:
    return PidController(
        preview_distance=max(min_preview_distance, speed * preview_time),
        steer_limit_deg=steer_limit_deg,
        target_speed=speed,
        sample_time=sample_time,
        wheelbase=wheelbase,
        kp_yaw=kp_yaw,
        ki_yaw=ki_yaw,
        kd_yaw=kd_yaw,
        kp_preview=kp_preview,
        ki_preview=ki_preview,
        kd_preview=kd_preview,
        kff_curvature=kff_curvature,
    )


def compute_pid_command(yaw_error_deg: float, preview_error: float, ref_curvature: float, controller: PidController) -> float:
    dt = controller.sample_time
    next_yaw_integral = float(
        np.clip(controller.yaw_integral + yaw_error_deg * dt, -controller.integral_limit, controller.integral_limit)
    )
    next_preview_integral = float(
        np.clip(controller.preview_integral + preview_error * dt, -controller.integral_limit, controller.integral_limit)
    )
    yaw_derivative = (yaw_error_deg - controller.last_yaw_error_deg) / dt
    preview_derivative = (preview_error - controller.last_preview_error) / dt

    yaw_term = controller.kp_yaw * yaw_error_deg + controller.ki_yaw * next_yaw_integral + controller.kd_yaw * yaw_derivative
    preview_term = (
        controller.kp_preview * preview_error
        + controller.ki_preview * next_preview_integral
        + controller.kd_preview * preview_derivative
    )
    feedforward = np.degrees(np.arctan(controller.wheelbase * ref_curvature))
    steer_unsat = -(yaw_term + preview_term) + controller.kff_curvature * float(feedforward)
    steer_cmd = float(np.clip(steer_unsat, -controller.steer_limit_deg, controller.steer_limit_deg))

    if abs(steer_cmd - steer_unsat) < 1e-9:
        controller.yaw_integral = next_yaw_integral
        controller.preview_integral = next_preview_integral

    controller.last_yaw_error_deg = yaw_error_deg
    controller.last_preview_error = preview_error
    return steer_cmd
