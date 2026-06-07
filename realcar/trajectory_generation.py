import math
from typing import Dict, Tuple

import numpy as np

from common import wrap_to_pi


def generate_trajectory(scene_id: int = 1, x_res: float = 0.1, **kwargs: float) -> Dict[str, np.ndarray]:
    car_params = {"speed": 25.0, "wheelbase": 2.70, "delta_max": math.radians(6.0)}
    car_params.update({key: kwargs[key] for key in ("speed", "wheelbase", "delta_max") if key in kwargs})

    if scene_id == 1:
        params = {"x_cycle": 290.0, "amplitude": 5.5, "cycle_num": 2.0}
        params.update(kwargs)
        ref_x, ref_y = gen_truck_sin(x_res, params["x_cycle"], params["amplitude"], int(params["cycle_num"]))
        mapyaw, curv = traj_postproc(ref_x, ref_y)
    elif scene_id == 2:
        params = {
            "pre": 0.0,
            "length": 2.0,
            "post": 0.0,
            "distance": 0.5,
        }
        params.update(kwargs)
        ref_x, ref_y = gen_smooth_lane_change(
            x_res=x_res,
            pre_length=params["pre"],
            transition_length=params["length"],
            post_length=params["post"],
            distance=params["distance"],
        )
        mapyaw, curv = traj_postproc(ref_x, ref_y)
    elif scene_id == 3:
        params = {"size": 360.0}
        params.update(kwargs)
        ref_x, ref_y = gen_closed_figure_eight(x_res, params["size"])
        mapyaw, curv = traj_postproc(ref_x, ref_y)
    elif scene_id == 4:
        params = {"pre": 40.0, "mid": 40.0, "post": 120.0, "radius": 180.0}
        params.update(kwargs)
        ref_x, ref_y, mapyaw, curv = gen_left90_right90_straight(x_res, params["pre"], params["mid"], params["post"], params["radius"])
    elif scene_id == 5:
        params = {"straight_length": 20.0, "radius": 100.0}
        params.update(kwargs)
        ref_x, ref_y, mapyaw, curv = gen_straight_upper_straight_lower_straight(x_res, params["straight_length"], params["radius"])
    elif scene_id == 6:
        params = {"pre": 10.0, "mid": 7.0, "post": 10.0, "radius": 80.0}
        params.update(kwargs)
        ref_x, ref_y, mapyaw, curv = gen_left90_left90_straight(x_res, params["pre"], params["mid"], params["post"], params["radius"])
    elif scene_id == 7:
        params = {"straight_length": 200.0, "diameter": 360.0}
        params.update(kwargs)
        ref_x, ref_y, mapyaw, curv = gen_closed_racetrack(x_res, params["straight_length"], params["diameter"])
    elif scene_id == 8:
        params = {"scale": 1.0}
        params.update(kwargs)
        ref_x, ref_y = gen_scene8_closed_contour(x_res, params["scale"])
        mapyaw, curv = traj_postproc(ref_x, ref_y)
    else:
        raise ValueError("scene_id must be 1/2/3/4/5/6/7/8")

    limit_check = check_vehicle_limits(curv, car_params)
    ds = np.hypot(np.diff(ref_x), np.diff(ref_y))
    s = np.concatenate(([0.0], np.cumsum(ds)))
    return {"x": ref_x, "y": ref_y, "yaw": mapyaw, "curvature": curv, "s": s, "limit_check": limit_check}


def gen_truck_sin(x_res: float, x_cycle: float, amplitude: float, cycle_num: int) -> Tuple[np.ndarray, np.ndarray]:
    x_max = cycle_num * x_cycle
    ref_x = np.arange(0.0, x_max + x_res, x_res)
    ref_y = amplitude * np.sin(2.0 * math.pi * ref_x / x_cycle)
    return ref_x, ref_y


def gen_closed_figure_eight(ds: float, size: float) -> Tuple[np.ndarray, np.ndarray]:
    t_candidates = np.linspace(-math.pi, math.pi, 5000, endpoint=False)
    x_candidates = size * np.cos(t_candidates) / (1.0 + np.sin(t_candidates) ** 2)
    y_candidates = size * np.sin(t_candidates) * np.cos(t_candidates) / (1.0 + np.sin(t_candidates) ** 2)
    dx_candidates = np.gradient(x_candidates, t_candidates)
    dy_candidates = np.gradient(y_candidates, t_candidates)
    heading_candidates = np.arctan2(dy_candidates, dx_candidates)
    heading_error = np.abs(np.array([wrap_to_pi(val) for val in heading_candidates]))
    start_idx = int(np.argmin(np.where(dx_candidates > 0.0, heading_error, np.inf)))
    t_start = float(t_candidates[start_idx])

    t = np.linspace(t_start, t_start + 2.0 * math.pi, 10000, endpoint=False)
    x = size * np.cos(t) / (1.0 + np.sin(t) ** 2)
    y = size * np.sin(t) * np.cos(t) / (1.0 + np.sin(t) ** 2)
    x = x - x[0]
    y = -(y - y[0])
    return resample_polyline(x, y, ds)


def gen_smooth_lane_change(
    x_res: float,
    pre_length: float,
    transition_length: float,
    post_length: float,
    distance: float,
) -> Tuple[np.ndarray, np.ndarray]:
    total_length = pre_length + transition_length + post_length
    ref_x = np.arange(0.0, total_length + x_res, x_res)
    ref_y = np.zeros_like(ref_x)

    x1 = pre_length
    x2 = x1 + transition_length

    for i, xval in enumerate(ref_x):
        if xval < x1:
            ref_y[i] = 0.0
        elif xval < x2:
            s = (xval - x1) / transition_length
            ref_y[i] = 0.5 * distance * (1.0 - math.cos(math.pi * s))
        else:
            ref_y[i] = distance
    return ref_x, ref_y


def gen_straight_upper_straight_lower_straight(ds: float, straight_length: float, radius: float):
    points = []
    x = y = psi = 0.0
    x, y, psi = append_straight(points, x, y, psi, straight_length, ds)
    x, y, psi = append_arc(points, x, y, psi, radius, math.pi, ds, left_turn=True)
    x, y, psi = append_straight(points, x, y, psi, straight_length, ds)
    x, y, psi = append_arc(points, x, y, psi, radius, math.pi, ds, left_turn=False)
    x, y, psi = append_straight(points, x, y, psi, straight_length, ds)
    return points_to_arrays(points)


def gen_left90_right90_straight(ds: float, pre_length: float, mid_length: float, post_length: float, radius: float):
    points = []
    x = y = psi = 0.0
    x, y, psi = append_straight(points, x, y, psi, pre_length, ds)
    x, y, psi = append_arc(points, x, y, psi, radius, 0.5 * math.pi, ds, left_turn=True)
    x, y, psi = append_straight(points, x, y, psi, mid_length, ds)
    x, y, psi = append_arc(points, x, y, psi, radius, 0.5 * math.pi, ds, left_turn=False)
    x, y, psi = append_straight(points, x, y, psi, post_length, ds)
    return points_to_arrays(points)


def gen_left90_left90_straight(ds: float, pre_length: float, mid_length: float, post_length: float, radius: float):
    points = []
    x = y = psi = 0.0
    x, y, psi = append_straight(points, x, y, psi, pre_length, ds)
    x, y, psi = append_arc(points, x, y, psi, radius, 0.5 * math.pi, ds, left_turn=True)
    x, y, psi = append_straight(points, x, y, psi, mid_length, ds)
    x, y, psi = append_arc(points, x, y, psi, radius, 0.5 * math.pi, ds, left_turn=True)
    x, y, psi = append_straight(points, x, y, psi, post_length, ds)
    return points_to_arrays(points)


def gen_closed_racetrack(ds: float, straight_length: float, diameter: float):
    points = []
    x = y = psi = 0.0
    radius = 0.5 * diameter
    x, y, psi = append_straight(points, x, y, psi, straight_length, ds)
    x, y, psi = append_arc(points, x, y, psi, radius, math.pi, ds, left_turn=True)
    x, y, psi = append_straight(points, x, y, psi, straight_length, ds)
    x, y, psi = append_arc(points, x, y, psi, radius, math.pi, ds, left_turn=True)
    return points_to_arrays(points)


def gen_scene8_closed_contour(ds: float, scale: float = 1.0) -> Tuple[np.ndarray, np.ndarray]:
    control_points = np.array(
        [
            (-150.0, 0.0),
            (-150.0, 105.0),
            (-90.0, 122.0),
            (40.0, 115.0),
            (115.0, 70.0),
            (165.0, 55.0),
            (195.0, 120.0),
            (270.0, 125.0),
            (305.0, 70.0),
            (290.0, -25.0),
            (270.0, -145.0),
            (220.0, -190.0),
            (60.0, -192.0),
            (-110.0, -188.0),
            (-155.0, -140.0),
            (-155.0, -55.0),
            (-115.0, -30.0),
            (-60.0, -35.0),
            (-35.0, -110.0),
            (30.0, -115.0),
            (120.0, -138.0),
            (185.0, -120.0),
            (190.0, -35.0),
            (130.0, -8.0),
            (30.0, 0.0),
        ],
        dtype=float,
    )
    control_points *= float(scale)
    x, y = sample_closed_catmull_rom(control_points, ds)
    return smooth_closed_polyline(x, y, window=101, passes=3)


def integrate_pose(x: float, y: float, psi: float, curvature: float, ds: float):
    psi = psi + curvature * ds
    x = x + math.cos(psi) * ds
    y = y + math.sin(psi) * ds
    return x, y, psi


def append_straight(points, x: float, y: float, psi: float, length: float, ds: float):
    steps = max(1, int(math.ceil(length / ds)))
    for step in range(steps):
        ds_i = min(ds, length - step * ds)
        if ds_i <= 0.0:
            break
        x, y, psi = integrate_pose(x, y, psi, 0.0, ds_i)
        points.append((x, y, wrap_to_pi(psi), 0.0))
    return x, y, psi


def append_arc(points, x: float, y: float, psi: float, radius: float, total_angle: float, ds: float, left_turn: bool):
    curvature = (1.0 / radius) if left_turn else (-1.0 / radius)
    arc_length = abs(total_angle) * radius
    steps = max(1, int(math.ceil(arc_length / ds)))
    for step in range(steps):
        ds_i = min(ds, arc_length - step * ds)
        if ds_i <= 0.0:
            break
        x, y, psi = integrate_pose(x, y, psi, curvature, ds_i)
        points.append((x, y, wrap_to_pi(psi), curvature))
    return x, y, psi


def points_to_arrays(points):
    return (
        np.array([p[0] for p in points], dtype=float),
        np.array([p[1] for p in points], dtype=float),
        np.array([p[2] for p in points], dtype=float),
        np.array([p[3] for p in points], dtype=float),
    )


def sample_closed_catmull_rom(control_points: np.ndarray, ds: float, samples_per_seg: int = 40) -> Tuple[np.ndarray, np.ndarray]:
    if control_points.ndim != 2 or control_points.shape[1] != 2:
        raise ValueError("control_points must have shape (N, 2)")
    if control_points.shape[0] < 4:
        raise ValueError("At least four control points are required for a closed spline")

    pts = np.asarray(control_points, dtype=float)
    curve_segments = []
    count = pts.shape[0]
    for idx in range(count):
        p0 = pts[(idx - 1) % count]
        p1 = pts[idx % count]
        p2 = pts[(idx + 1) % count]
        p3 = pts[(idx + 2) % count]
        t_values = np.linspace(0.0, 1.0, samples_per_seg, endpoint=False)
        t2 = t_values * t_values
        t3 = t2 * t_values
        segment = 0.5 * (
            (2.0 * p1)
            + (-p0 + p2) * t_values[:, None]
            + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2[:, None]
            + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3[:, None]
        )
        curve_segments.append(segment)

    curve = np.vstack(curve_segments)
    curve = np.vstack((curve, curve[0]))
    x, y = resample_polyline(curve[:, 0], curve[:, 1], ds)
    return x, y


def smooth_closed_polyline(x: np.ndarray, y: np.ndarray, window: int = 21, passes: int = 1) -> Tuple[np.ndarray, np.ndarray]:
    if window < 3 or window % 2 == 0:
        raise ValueError("window must be an odd integer greater than or equal to 3")
    if x.size != y.size:
        raise ValueError("x and y must have the same size")
    if x.size < window:
        raise ValueError("Polyline is too short for the requested smoothing window")

    kernel = np.ones(window, dtype=float) / float(window)
    x_inner = np.asarray(x[:-1], dtype=float).copy()
    y_inner = np.asarray(y[:-1], dtype=float).copy()
    pad = window // 2
    for _ in range(max(1, int(passes))):
        x_inner = np.convolve(np.pad(x_inner, (pad, pad), mode="wrap"), kernel, mode="valid")
        y_inner = np.convolve(np.pad(y_inner, (pad, pad), mode="wrap"), kernel, mode="valid")
    return np.append(x_inner, x_inner[0]), np.append(y_inner, y_inner[0])


def resample_polyline(x: np.ndarray, y: np.ndarray, ds: float):
    s = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))
    sample_s = np.arange(0.0, s[-1] + ds, ds)
    return np.interp(sample_s, s, x), np.interp(sample_s, s, y)


def traj_postproc(ref_x: np.ndarray, ref_y: np.ndarray):
    dx = np.gradient(ref_x)
    dy = np.gradient(ref_y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    mapyaw = np.arctan2(dy, dx)
    den = np.power(dx * dx + dy * dy, 1.5)
    curv = (dx * ddy - dy * ddx) / np.maximum(den, 1e-9)
    if mapyaw.size > 1:
        mapyaw[0] = mapyaw[1]
        curv[0] = curv[1]
    mapyaw = np.array([wrap_to_pi(val) for val in mapyaw])
    return mapyaw, curv


def check_vehicle_limits(curv: np.ndarray, vehicle_params: Dict[str, float]) -> Dict[str, float]:
    curv_max = float(np.max(np.abs(curv)))
    wheelbase = float(vehicle_params["wheelbase"])
    delta_max = float(vehicle_params["delta_max"])
    speed = float(vehicle_params["speed"])
    delta_req_max = math.atan(wheelbase * curv_max)
    ay_max = speed * speed * curv_max
    rmin = 1.0 / max(curv_max, 1e-9)
    exceeds = delta_req_max > delta_max
    print(f"--- Car limit check (v={speed:.1f}, L={wheelbase:.2f}m, delta_max={math.degrees(delta_max):.2f}deg) ---")
    print(f"max|curv| = {curv_max:.6f} 1/m (Rmin={rmin:.1f} m)")
    print(f"required max|delta| = {math.degrees(delta_req_max):.2f} deg")
    print(f"approx max ay = {ay_max:.3f}")
    print("warning: required steering exceeds current steering limit assumption" if exceeds else "reference curvature is within the steering limit assumption")
    return {
        "curv_max": curv_max,
        "rmin": rmin,
        "delta_req_max": delta_req_max,
        "delta_max": delta_max,
        "ay_max": ay_max,
        "within_limit": 0.0 if exceeds else 1.0,
    }
