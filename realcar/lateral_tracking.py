#!/usr/bin/python3
# -*- coding: utf-8 -*-
import argparse

import math
import os
import shlex
import subprocess
import sys
import time
from typing import Optional

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from common import VehicleState
from pid_controller import DEFAULT_STABLE_PID, build_pid_controller, compute_pid_command
from tracking_logic import compute_tracking_error, save_trajectory_plot, write_csv
from trajectory_generation import generate_trajectory


CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))


def start_odom_state_reader(args: argparse.Namespace) -> Optional[subprocess.Popen]:
    if not args.start_odom_reader:
        return None

    if args.odom_reader_command:
        command = shlex.split(args.odom_reader_command)
    else:
        command = [
            sys.executable,
            os.path.join(CURRENT_DIR, "odom_state_reader.py"),
            f"_odom_topic:={args.odom_topic}",
            f"_state_topic:={args.state_topic}",
            f"_speed_mode:={args.odom_speed_mode}",
            f"_debug_every:={args.odom_reader_debug_every}",
        ]
        if args.odom_reader_csv:
            command.append(f"_csv_path:={args.odom_reader_csv}")

    rospy.loginfo("starting odom_state_reader: %s", " ".join(command))
    return subprocess.Popen(command)


def stop_odom_state_reader(process: Optional[subprocess.Popen]) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=1.0)


class RosStateBackend:
    """Bridge odom_state_reader.py output into the VehicleState used by PID."""

    def __init__(
        self,
        state_topic: str,
        control_topic: str,
        sensor_timeout: float,
        direct_car_control: bool,
    ) -> None:
        self.state_topic = state_topic
        self.sensor_timeout = sensor_timeout
        self.last_state: Optional[VehicleState] = None
        self.last_state_stamp = 0.0
        self.last_msg_wall_time = 0.0
        self.last_steer_cmd_deg = 0.0
        self.emergency_stopped = False
        self.car_control = None
        self.stop_car = None
        self.emergency_stop_car = None

        if direct_car_control:
            from car import car_control, emergency_stop_car, stop_car

            self.car_control = car_control
            self.stop_car = stop_car
            self.emergency_stop_car = emergency_stop_car

        self.control_pub = rospy.Publisher(control_topic, Float64MultiArray, queue_size=20)
        self.state_sub = rospy.Subscriber(state_topic, Float64MultiArray, self.state_cb, queue_size=100)

    def state_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 6:
            rospy.logwarn_throttle(1.0, "state message on %s has %d values, expected 6", self.state_topic, len(msg.data))
            return

        timestamp, x, y, speed, yaw, yaw_rate = [float(value) for value in msg.data[:6]]
        self.last_state = VehicleState(
            x=x,
            y=y,
            yaw=yaw,
            vx=speed,
            vy=0.0,
            yaw_rate=yaw_rate,
            beta=0.0,
            steering_cmd_deg=self.last_steer_cmd_deg,
        )
        self.last_state_stamp = timestamp
        self.last_msg_wall_time = time.monotonic()

    def wait_for_state(self, timeout: float, state_source_process: Optional[subprocess.Popen] = None) -> VehicleState:
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.last_state is None and time.monotonic() < deadline:
            if state_source_process is not None and state_source_process.poll() is not None:
                raise RuntimeError(
                    f"odom_state_reader exited with code {state_source_process.returncode} "
                    f"before publishing state on {self.state_topic}"
                )
            rate.sleep()
        if self.last_state is None:
            raise TimeoutError(
                f"no state received on {self.state_topic} within {timeout:.2f}s; "
                "check that the configured odometry input topic is active"
            )
        return self.last_state

    def state(self) -> VehicleState:
        if self.last_state is None:
            raise RuntimeError(f"no state has been received on {self.state_topic}")
        return self.last_state

    def sensor_is_fresh(self) -> bool:
        if self.last_state is None:
            return False
        return (time.monotonic() - self.last_msg_wall_time) <= self.sensor_timeout

    def publish_control(self, steer_cmd_deg: float, speed_cmd: float) -> None:
        self.last_steer_cmd_deg = float(steer_cmd_deg)
        out = Float64MultiArray()
        out.layout.dim.append(MultiArrayDimension(label="control", size=3, stride=3))
        out.data = [rospy.Time.now().to_sec(), float(steer_cmd_deg), float(speed_cmd)]
        self.control_pub.publish(out)

    def set_control(self, steer_cmd_deg: float, speed_cmd: float) -> None:
        self.publish_control(steer_cmd_deg, speed_cmd)
        if self.car_control is not None:
            self.emergency_stopped = False
            self.car_control(steer_cmd_deg, speed_cmd)

    def stop(self) -> None:
        self.publish_control(0.0, 0.0)
        if self.stop_car is not None:
            self.stop_car()

    def emergency_stop(self) -> None:
        if self.emergency_stopped:
            return
        self.emergency_stopped = True
        if self.emergency_stop_car is not None:
            try:
                self.emergency_stop_car()
            except Exception as exc:
                rospy.logerr("failed to stop direct-control hardware: %s", exc)
        for _ in range(10):
            try:
                self.publish_control(0.0, 0.0)
            except Exception as exc:
                rospy.logerr("failed to publish emergency stop: %s", exc)
                break
            time.sleep(0.05)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Real-car PID trajectory tracking using odom_state_reader.py state output.")
    parser.add_argument("--state-topic", default="/realcar/state", help="Float64MultiArray state topic from odom_state_reader.py.")
    parser.add_argument("--control-topic", default="/realcar/control_cmd", help="Float64MultiArray command topic: [time, steer_deg, speed].")
    parser.add_argument(
        "--start-odom-reader",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Start odom_state_reader.py as a child ROS node before waiting for vehicle state.",
    )
    parser.add_argument("--odom-topic", default="/Odometry", help="Odometry topic consumed by odom_state_reader.py.")
    parser.add_argument("--odom-speed-mode", choices=("x", "xy_norm", "norm"), default="norm", help="Speed parsing mode for odom_state_reader.py.")
    parser.add_argument("--odom-reader-csv", default="", help="Optional CSV path written by odom_state_reader.py.")
    parser.add_argument("--odom-reader-debug-every", type=int, default=20, help="Debug print interval for odom_state_reader.py.")
    parser.add_argument(
        "--odom-reader-command",
        default="",
        help="Optional full command used to start odom_state_reader.py. Overrides the local-script command.",
    )
    parser.add_argument(
        "--direct-car-control",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Call car.car_control() directly while also publishing the control topic.",
    )
    parser.add_argument("--node-name", default="realcar_lateral_tracking", help="ROS node name.")
    parser.add_argument("--wait-state-timeout", type=float, default=5.0, help="Seconds to wait for the first state message.")
    parser.add_argument("--sensor-timeout", type=float, default=0.25, help="Emergency stop threshold for stale state messages.")
    parser.add_argument("--max-preview-error", type=float, default=1.5, help="Emergency stop threshold for lateral preview error, meters.")
    parser.add_argument("--max-heading-error-deg", type=float, default=45.0, help="Emergency stop threshold for heading error, degrees.")
    parser.add_argument("--max-steps", type=int, default=None, help="Optional maximum control iterations.")
    parser.add_argument("--output", default=None, help="CSV trace output path. Defaults to sceneX.csv in the project directory.")
    parser.add_argument("--plot-output", default=None, help="Optional trajectory plot output path.")
    parser.add_argument("--show-plot", action=argparse.BooleanOptionalAction, default=False, help="Show trajectory plot after exit.")

    parser.add_argument("--scene-id", type=int, choices=(1, 2, 3, 4, 5, 6, 7, 8), default=2, help="Trajectory scene id.")
    parser.add_argument("--x-res", type=float, default=0.1, help="Path sampling resolution in meters.")
    parser.add_argument("--scene3-size", type=float, default=360.0, help="Scale parameter for scene 3 closed figure-eight.")
    parser.add_argument("--scene2-distance", type=float, default=0.5, help="Lateral lane-change distance for scene 2, meters.")
    parser.add_argument("--scene2-length", type=float, default=5.0, help="Longitudinal lane-change length for scene 2, meters.")
    parser.add_argument("--scene4-radius", type=float, default=180.0, help="Turn radius for scene 4.")
    parser.add_argument("--scene5-straight-length", type=float, default=20.0, help="Straight length for scene 5.")
    parser.add_argument("--scene5-radius", type=float, default=100.0, help="Arc radius for scene 5.")
    parser.add_argument("--scene6-pre", type=float, default=10.0, help="Entry straight length for scene 6 U-turn.")
    parser.add_argument("--scene6-mid", type=float, default=7.0, help="Longitudinal connection length for scene 6 U-turn.")
    parser.add_argument("--scene6-post", type=float, default=10.0, help="Exit straight length for scene 6.")
    parser.add_argument("--scene6-radius", type=float, default=80.0, help="Turn radius for scene 6 U-turn.")
    parser.add_argument("--scene7-straight-length", type=float, default=200.0, help="Straight length for scene 7.")
    parser.add_argument("--scene7-diameter", type=float, default=360.0, help="Semicircle diameter for scene 7.")

    parser.add_argument("--target-speed", type=float, default=0.1, help="Real-car target speed, m/s.")
    parser.add_argument("--fixed-speed-percent", type=float, default=20.0, help="Fixed motor speed command in percent during tracking.")
    parser.add_argument("--sample-time", type=float, default=DEFAULT_STABLE_PID["sample_time"], help="Controller sample time in seconds.")
    parser.add_argument("--preview-time", type=float, default=DEFAULT_STABLE_PID["preview_time"], help="Preview time in seconds.")
    parser.add_argument("--min-preview-distance", type=float, default=DEFAULT_STABLE_PID["min_preview_distance"], help="Minimum preview distance in meters.")
    parser.add_argument("--wheelbase", type=float, default=DEFAULT_STABLE_PID["wheelbase"], help="Vehicle wheelbase in meters.")
    parser.add_argument("--steer-limit-deg", type=float, default=DEFAULT_STABLE_PID["steer_limit_deg"], help="Steering limit in degrees.")
    parser.add_argument("--kp-yaw", type=float, default=DEFAULT_STABLE_PID["kp_yaw"], help="Yaw error proportional gain.")
    parser.add_argument("--ki-yaw", type=float, default=DEFAULT_STABLE_PID["ki_yaw"], help="Yaw error integral gain.")
    parser.add_argument("--kd-yaw", type=float, default=DEFAULT_STABLE_PID["kd_yaw"], help="Yaw error derivative gain.")
    parser.add_argument("--kp-preview", type=float, default=DEFAULT_STABLE_PID["kp_preview"], help="Preview error proportional gain.")
    parser.add_argument("--ki-preview", type=float, default=DEFAULT_STABLE_PID["ki_preview"], help="Preview error integral gain.")
    parser.add_argument("--kd-preview", type=float, default=DEFAULT_STABLE_PID["kd_preview"], help="Preview error derivative gain.")
    parser.add_argument("--kff-curvature", type=float, default=DEFAULT_STABLE_PID["kff_curvature"], help="Curvature feedforward gain.")
    return parser.parse_args()


def build_path(args: argparse.Namespace):
    path_kwargs = {
        "scene_id": args.scene_id,
        "x_res": args.x_res,
        "speed": args.target_speed,
        "wheelbase": args.wheelbase,
        "delta_max": math.radians(args.steer_limit_deg),
    }
    if args.scene_id == 2:
        path_kwargs.update({"distance": args.scene2_distance, "length": args.scene2_length})
    elif args.scene_id == 3:
        path_kwargs.update({"size": args.scene3_size})
    elif args.scene_id == 4:
        path_kwargs.update({"radius": args.scene4_radius})
    elif args.scene_id == 5:
        path_kwargs.update({"straight_length": args.scene5_straight_length, "radius": args.scene5_radius})
    elif args.scene_id == 6:
        path_kwargs.update({"pre": args.scene6_pre, "mid": args.scene6_mid, "post": args.scene6_post, "radius": args.scene6_radius})
    elif args.scene_id == 7:
        path_kwargs.update({"straight_length": args.scene7_straight_length, "diameter": args.scene7_diameter})
    return generate_trajectory(**path_kwargs)


def resolve_output_path(args: argparse.Namespace) -> str:
    if args.output:
        return args.output
    return os.path.join(CURRENT_DIR, f"scene{args.scene_id}.csv")


def build_controller(args: argparse.Namespace):
    return build_pid_controller(
        speed=args.target_speed,
        sample_time=args.sample_time,
        preview_time=args.preview_time,
        min_preview_distance=args.min_preview_distance,
        wheelbase=args.wheelbase,
        steer_limit_deg=args.steer_limit_deg,
        kp_yaw=args.kp_yaw,
        ki_yaw=args.ki_yaw,
        kd_yaw=args.kd_yaw,
        kp_preview=args.kp_preview,
        ki_preview=args.ki_preview,
        kd_preview=args.kd_preview,
        kff_curvature=args.kff_curvature,
    )


def stop_reason_for(args: argparse.Namespace, backend: RosStateBackend, path_size: int, path_index: int, preview_error: float, heading_error: float):
    if not backend.sensor_is_fresh():
        return "sensor_timeout"
    if abs(preview_error) > args.max_preview_error:
        return "preview_error_limit"
    if abs(math.degrees(heading_error)) > args.max_heading_error_deg:
        return "heading_error_limit"
    if path_index >= path_size - 5:
        return "path_finished"
    return None


def run_tracking(args: argparse.Namespace, backend: RosStateBackend, controller, path) -> None:
    rows = []
    last_index = 0
    stop_reason = None
    rate = rospy.Rate(1.0 / controller.sample_time)
    step = 0

    while not rospy.is_shutdown() and (args.max_steps is None or step < args.max_steps):
        state = backend.state()
        error = compute_tracking_error(state, path, controller.preview_distance, last_index)
        last_index = error.path_index
        speed_cmd = max(-100.0, min(100.0, float(args.fixed_speed_percent)))
        steer_cmd_deg = compute_pid_command(
            yaw_error_deg=math.degrees(error.heading_error),
            preview_error=error.preview_error,
            ref_curvature=error.ref_curvature,
            controller=controller,
        )

        stop_reason = stop_reason_for(args, backend, path["x"].size, last_index, error.preview_error, error.heading_error)
        if stop_reason is None:
            backend.set_control(steer_cmd_deg, speed_cmd)
        else:
            backend.stop()

        rows.append(
            {
                "timestamp": round(rospy.Time.now().to_sec(), 6),
                "state_timestamp": round(backend.last_state_stamp, 6),
                "scene_id": args.scene_id,
                "step": step,
                "actual_x": state.x,
                "actual_y": state.y,
                "actual_yaw": state.yaw,
                "actual_yaw_deg": math.degrees(state.yaw),
                "actual_speed": state.vx,
                "actual_vx": state.vx,
                "actual_vy": state.vy,
                "actual_yaw_rate": state.yaw_rate,
                "actual_beta": state.beta,
                "actual_steering_cmd_deg": state.steering_cmd_deg,
                "reference_x": error.ref_x,
                "reference_y": error.ref_y,
                "reference_yaw": error.ref_yaw,
                "reference_yaw_deg": math.degrees(error.ref_yaw),
                "reference_curvature": error.ref_curvature,
                "preview_x": error.preview_x,
                "preview_y": error.preview_y,
                "x": state.x,
                "y": state.y,
                "yaw": state.yaw,
                "yaw_rate": state.yaw_rate,
                "speed": state.vx,
                "preview_error": error.preview_error,
                "heading_error": error.heading_error,
                "heading_error_deg": math.degrees(error.heading_error),
                "ref_x": error.ref_x,
                "ref_y": error.ref_y,
                "ref_yaw": error.ref_yaw,
                "ref_curvature": error.ref_curvature,
                "path_index": last_index,
                "steer_cmd_deg": steer_cmd_deg,
                "speed_cmd": speed_cmd,
                "stop_reason": stop_reason or "",
            }
        )

        if step % 20 == 0:
            rospy.loginfo(
                "[PID_TRACK] step=%d x=%.3f y=%.3f speed=%.3f yaw_err=%.3fdeg e_preview=%.3f steer=%.3f speed_cmd=%.3f",
                step,
                state.x,
                state.y,
                state.vx,
                math.degrees(error.heading_error),
                error.preview_error,
                steer_cmd_deg,
                speed_cmd,
            )

        if stop_reason is not None:
            rospy.logwarn("real-car tracking stopped: %s", stop_reason)
            break

        step += 1
        rate.sleep()

    backend.emergency_stop()
    output_path = resolve_output_path(args)
    write_csv(output_path, rows)
    save_trajectory_plot(path, rows, output_path=args.plot_output, show_plot=args.show_plot)
    rospy.loginfo("tracking finished, samples=%d, results=%s", len(rows), output_path)


def main() -> None:
    args = parse_args()
    rospy.init_node(args.node_name)
    odom_reader_process = start_odom_state_reader(args)
    path = build_path(args)
    if path["limit_check"]["within_limit"] < 0.5:
        rospy.logwarn("selected trajectory exceeds the steering limit assumption and may saturate")
    controller = build_controller(args)
    backend = RosStateBackend(
        state_topic=args.state_topic,
        control_topic=args.control_topic,
        sensor_timeout=args.sensor_timeout,
        direct_car_control=args.direct_car_control,
    )
    rospy.on_shutdown(backend.emergency_stop)
    rospy.on_shutdown(lambda: stop_odom_state_reader(odom_reader_process))
    try:
        backend.wait_for_state(args.wait_state_timeout, odom_reader_process)
        rospy.loginfo(
            "real-car PID tracking start: odom_topic=%s state_topic=%s control_topic=%s target_speed=%.3f direct_car_control=%s",
            args.odom_topic,
            args.state_topic,
            args.control_topic,
            controller.target_speed,
            args.direct_car_control,
        )
        run_tracking(args, backend, controller, path)
    finally:
        try:
            backend.emergency_stop()
        except Exception as exc:
            rospy.logerr("failed to send emergency stop during shutdown: %s", exc)
        finally:
            stop_odom_state_reader(odom_reader_process)


if __name__ == "__main__":
    main()
