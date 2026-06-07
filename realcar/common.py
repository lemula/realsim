import math
from dataclasses import dataclass


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class VehicleState:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    yaw_rate: float
    beta: float
    steering_cmd_deg: float


@dataclass
class TrackingError:
    path_index: int
    preview_x: float
    preview_y: float
    ref_x: float
    ref_y: float
    ref_yaw: float
    ref_curvature: float
    beta: float
    heading_error: float
    preview_error: float
    yaw_rate: float
