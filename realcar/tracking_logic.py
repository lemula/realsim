import csv
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from common import TrackingError, VehicleState, wrap_to_pi


def find_preview_index(
    ref_x: np.ndarray,
    ref_y: np.ndarray,
    ref_yaw: np.ndarray,
    ref_curvature: np.ndarray,
    ref_s: np.ndarray,
    preview_x: float,
    preview_y: float,
    preview_heading: float,
    last_index: int,
    search_window: int = 1200,
    backtrack_window: int = 5,
    heading_gate: float = 0.5 * math.pi,
    heading_weight: float = 6.0,
) -> Tuple[int, float, float, float, float]:
    start = max(0, last_index - backtrack_window)
    stop = min(ref_x.size - 1, last_index + search_window)
    best = None
    best_score = float("inf")
    best_fallback = None
    best_fallback_score = float("inf")

    for i in range(start, stop):
        x0 = float(ref_x[i])
        y0 = float(ref_y[i])
        x1 = float(ref_x[i + 1])
        y1 = float(ref_y[i + 1])
        seg_dx = x1 - x0
        seg_dy = y1 - y0
        seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy
        tau = 0.0 if seg_len2 <= 1e-12 else min(1.0, max(0.0, ((preview_x - x0) * seg_dx + (preview_y - y0) * seg_dy) / seg_len2))
        proj_x = x0 + tau * seg_dx
        proj_y = y0 + tau * seg_dy
        dist2 = (proj_x - preview_x) ** 2 + (proj_y - preview_y) ** 2
        seg_heading = math.atan2(seg_dy, seg_dx) if seg_len2 > 1e-12 else float(ref_yaw[i])
        heading_diff = abs(wrap_to_pi(preview_heading - seg_heading))
        score = dist2 + (heading_weight * heading_diff) ** 2
        interp_curvature = float((1.0 - tau) * ref_curvature[i] + tau * ref_curvature[i + 1])
        interp_s = float((1.0 - tau) * ref_s[i] + tau * ref_s[i + 1])
        candidate = (i, proj_x, proj_y, seg_heading, interp_curvature, interp_s)
        if score < best_fallback_score:
            best_fallback_score = score
            best_fallback = candidate
        if heading_diff <= heading_gate and score < best_score:
            best_score = score
            best = candidate

    if best is None:
        best = best_fallback
    if best is None:
        idx = min(max(last_index, 0), ref_x.size - 1)
        return idx, float(ref_x[idx]), float(ref_y[idx]), float(ref_yaw[idx]), float(ref_curvature[idx])

    seg_idx, proj_x, proj_y, proj_heading, proj_curvature, proj_s = best
    idx = int(np.searchsorted(ref_s, proj_s, side="left"))
    idx = min(max(idx, seg_idx), ref_x.size - 1)
    return idx, proj_x, proj_y, proj_heading, proj_curvature


def compute_tracking_error(state: VehicleState, path: Dict[str, np.ndarray], preview_distance: float, last_index: int) -> TrackingError:
    preview_heading = state.yaw + state.beta
    preview_x = state.x + preview_distance * math.cos(preview_heading)
    preview_y = state.y + preview_distance * math.sin(preview_heading)
    idx, ref_x, ref_y, ref_yaw, ref_curvature = find_preview_index(
        path["x"], path["y"], path["yaw"], path["curvature"], path["s"], preview_x, preview_y, preview_heading, last_index
    )
    return TrackingError(
        path_index=idx,
        preview_x=preview_x,
        preview_y=preview_y,
        ref_x=ref_x,
        ref_y=ref_y,
        ref_yaw=ref_yaw,
        ref_curvature=ref_curvature,
        beta=state.beta,
        heading_error=wrap_to_pi(state.yaw - ref_yaw),
        preview_error=-(preview_x - ref_x) * math.sin(ref_yaw) + (preview_y - ref_y) * math.cos(ref_yaw),
        yaw_rate=state.yaw_rate,
    )


def write_csv(path: str, rows: List[Dict[str, float]]) -> None:
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def save_trajectory_plot(
    path: Dict[str, np.ndarray],
    rows: List[Dict[str, float]],
    output_path: Optional[str] = None,
    show_plot: bool = True,
) -> None:
    if not rows:
        return
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed, skip plot export")
        return

    actual_x = np.array([row["x"] for row in rows], dtype=float)
    actual_y = np.array([row["y"] for row in rows], dtype=float)
    ref_x = np.array(path["x"], dtype=float)
    ref_y = np.array(path["y"], dtype=float)
    plt.figure("Lateral Trajectory Tracking", figsize=(10, 6))
    plt.plot(ref_x, ref_y, label="reference", linewidth=2.0)
    plt.plot(actual_x, actual_y, label="vehicle", linewidth=2.0)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Trajectory Tracking")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
    if show_plot:
        plt.show()
    plt.close()
