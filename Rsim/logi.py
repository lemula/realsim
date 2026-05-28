"""Logitech wheel control example for RSim dynamic_vehicle.

This script does not create a pygame display window. It reads the first
Logitech-compatible joystick/wheel and sends steering, throttle, and brake
commands to dynamic_vehicle.

Default mapping:
  wheel axis     0 -> steering
  pedal axis     2 -> throttle
  pedal axis     3 -> brake
  Ctrl+C           -> quit
"""

from __future__ import annotations

import functools
import subprocess
import sys
import time
from pathlib import Path
from dataclasses import dataclass

import rsim

try:
    import pygame
except ImportError:
    pygame = None


print = functools.partial(print, flush=True)


RSIM_HOST = "192.168.2.181"
RSIM_PORT = 9000
VEHICLE_HOST = "192.168.2.181"
VEHICLE_PORT = 9030
SCENARIO_ID = 1075
SEED = 0
MAP_ID = 1173

CONNECT_TIMEOUT_SECONDS = 30.0
CONNECT_RETRY_INTERVAL_SECONDS = 0.5
WAIT_FOR_INIT_SECONDS = 6.0
CONTROL_HZ = 20.0
MAX_RUN_SECONDS = 120.0
PRINT_EVERY_SECONDS = 1.0

JOYSTICK_INDEX = 0
LOGITECH_STEER_AXIS = 0
LOGITECH_THROTTLE_AXIS = 2
LOGITECH_BRAKE_AXIS = 3
LOGITECH_DEADZONE = 0.05
LOGITECH_INVERT_STEER = False
LOGITECH_INVERT_THROTTLE = True
LOGITECH_INVERT_BRAKE = True

MAX_STEERING = 0.45
STEER_SIGN = -1.0
FORWARD_GEAR = 1
FORWARD_MODE_TRANS = 2

ENABLE_FORCE_FEEDBACK = True
FFB_SCRIPT = Path(__file__).with_name("FFB.py")
# Leave empty to let FFB.py auto-detect, or set /dev/input/eventX here.
FFB_DEVICE = ""
FFB_RATE_HZ = 100
FFB_PRINT_HZ = 0.0
# Example: ["--gain", "0.7", "--max-torque", "0.8"]
FFB_EXTRA_ARGS: list[str] = []


@dataclass
class ControlState:
    throttle: float = 0.0
    brake: float = 0.0
    steering: float = 0.0


class LogitechController:
    def __init__(self) -> None:
        if pygame is None:
            raise RuntimeError("pygame is not installed. Install it with: pip install pygame")

        pygame.init()
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count <= 0:
            raise RuntimeError("no Logitech/joystick device detected")
        if JOYSTICK_INDEX >= joystick_count:
            raise RuntimeError(
                f"JOYSTICK_INDEX={JOYSTICK_INDEX} out of range, detected {joystick_count} device(s)"
            )

        self.joystick = pygame.joystick.Joystick(JOYSTICK_INDEX)
        self.joystick.init()
        print(
            "joystick enabled: "
            f"{self.joystick.get_name()}, axes={self.joystick.get_numaxes()}, "
            f"buttons={self.joystick.get_numbuttons()}"
        )

    def close(self) -> None:
        if pygame is not None:
            pygame.quit()

    def poll(self) -> ControlState:
        try:
            pygame.event.pump()
        except Exception:
            pass

        steer_axis = read_axis(self.joystick, LOGITECH_STEER_AXIS, LOGITECH_INVERT_STEER)
        throttle_axis = read_axis(
            self.joystick,
            LOGITECH_THROTTLE_AXIS,
            LOGITECH_INVERT_THROTTLE,
        )
        brake_axis = read_axis(
            self.joystick,
            LOGITECH_BRAKE_AXIS,
            LOGITECH_INVERT_BRAKE,
        )

        steer = 0.0 if abs(steer_axis) < LOGITECH_DEADZONE else steer_axis
        throttle = axis_to_pedal(throttle_axis)
        brake = axis_to_pedal(brake_axis)
        throttle = 0.0 if throttle < LOGITECH_DEADZONE else throttle
        brake = 0.0 if brake < LOGITECH_DEADZONE else brake

        return ControlState(
            throttle=clamp(throttle, 0.0, 1.0),
            brake=clamp(brake, 0.0, 1.0),
            steering=clamp(steer * MAX_STEERING, -MAX_STEERING, MAX_STEERING),
        )


def main() -> int:
    try:
        controller = LogitechController()
    except RuntimeError as exc:
        print(f"logitech init failed: {exc}")
        return 1

    ffb_process = None

    print_control_help()

    client = connect_rsim_client()
    print(f"version: {client.get_version()}")

    is_ready = client.wait_for_init(WAIT_FOR_INIT_SECONDS)
    print(f"is_ready: {is_ready}")
    if not is_ready:
        print("rsim init timeout")
        stop_force_feedback(ffb_process)
        controller.close()
        return 1

    sr_client = client.get_scene_runner_client()
    bind_vehicle_client(client, sr_client)

    has_3d = client.has_world_3d()
    print(f"has_world_3d: {has_3d}")
    if has_3d:
        init_3d_world(client)

    print(f"loading trial: {SCENARIO_ID}")
    set_task_param(client)
    client.load_trial(SCENARIO_ID, SEED)
    print(f"loaded trial: {SCENARIO_ID}")

    ego_names = list(sr_client.get_ego_names())
    print(f"ego names: {ego_names}")
    ego_name = ego_names[0] if ego_names else None
    if has_3d:
        reset_spectator(client, sr_client, ego_name)

    vehicle_client = client.get_dynamic_vehicle_client()
    vehicle_status = {"count": 0, "latest": None}

    def on_vehicle_status(status):
        vehicle_status["count"] += 1
        vehicle_status["latest"] = status
        count = vehicle_status["count"]
        if count == 1 or count % 100 == 0:
            print(f"vehicle status callback count={count}")

    vehicle_client.on_vehicle_status(on_vehicle_status)

    exit_code = 0

    try:
        client.run()
        ffb_process = start_force_feedback()
        print("run requested")

        start_time = time.monotonic()
        last_control_time = start_time
        last_print_time = start_time
        last_control = ControlState()

        while True:
            now = time.monotonic()
            elapsed = now - start_time
            dt = now - last_control_time
            if dt < 1.0 / CONTROL_HZ:
                time.sleep(0.001)
                continue
            last_control_time = now

            state = client.get_state()
            if state == rsim.ModuleState.FINISHED:
                print("rsim finished")
                break
            if state == rsim.ModuleState.RUNNING_ERROR:
                print("rsim running error")
                exit_code = 1
                break

            last_control = controller.poll()
            cmd = make_control_cmd(last_control)
            vehicle_client.set_control_cmd(cmd)

            if now - last_print_time >= PRINT_EVERY_SECONDS:
                last_print_time = now
                print_runtime_status(
                    elapsed,
                    state,
                    last_control,
                    vehicle_status["latest"],
                    vehicle_status["count"],
                )

            if elapsed >= MAX_RUN_SECONDS:
                print(f"max run seconds {MAX_RUN_SECONDS:.1f} reached")
                break
    except KeyboardInterrupt:
        print("interrupted by user")
    finally:
        send_stop_command(vehicle_client)
        stop_force_feedback(ffb_process)
        controller.close()

    return exit_code


def start_force_feedback():
    if not ENABLE_FORCE_FEEDBACK:
        return None
    if not FFB_SCRIPT.exists():
        print(f"force feedback skipped: {FFB_SCRIPT} not found")
        return None

    cmd = [
        sys.executable,
        str(FFB_SCRIPT),
        "--rate",
        str(FFB_RATE_HZ),
        "--print-hz",
        str(FFB_PRINT_HZ),
    ]
    if FFB_DEVICE:
        cmd.extend(["--device", FFB_DEVICE])
    cmd.extend(FFB_EXTRA_ARGS)

    try:
        process = subprocess.Popen(cmd)
    except Exception as exc:
        print(f"force feedback skipped: {exc}")
        return None

    print(f"force feedback started: pid={process.pid}, script={FFB_SCRIPT}")
    return process


def stop_force_feedback(process) -> None:
    if process is None:
        return
    if process.poll() is not None:
        print(f"force feedback exited: code={process.returncode}")
        return

    process.terminate()
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)
    print("force feedback stopped")


def make_control_cmd(control: ControlState):
    cmd = rsim.RPCControlCmd()
    cmd.throttle = float(clamp(control.throttle, 0.0, 1.0))
    cmd.brake = float(clamp(control.brake, 0.0, 1.0))
    cmd.steering = float(clamp(control.steering * STEER_SIGN, -MAX_STEERING, MAX_STEERING))
    cmd.gear_location = FORWARD_GEAR
    if hasattr(cmd, "IMP_MODE_TRANS"):
        cmd.IMP_MODE_TRANS = FORWARD_MODE_TRANS
    return cmd


def send_stop_command(vehicle_client) -> None:
    cmd = rsim.RPCControlCmd()
    cmd.throttle = 0.0
    cmd.brake = 1.0
    cmd.steering = 0.0
    cmd.gear_location = FORWARD_GEAR
    if hasattr(cmd, "IMP_MODE_TRANS"):
        cmd.IMP_MODE_TRANS = FORWARD_MODE_TRANS
    try:
        vehicle_client.set_control_cmd(cmd)
    except Exception as exc:
        print(f"failed to send stop command: {exc}")


def bind_vehicle_client(client, sr_client) -> None:
    print(f"set_vehicle_client: {VEHICLE_HOST}:{VEHICLE_PORT}")
    if hasattr(client, "set_vehicle_client"):
        client.set_vehicle_client(VEHICLE_HOST, VEHICLE_PORT)
        return
    sr_client.set_vehicle_client(VEHICLE_HOST, VEHICLE_PORT)


def set_task_param(client) -> None:
    if not hasattr(client, "set_task_param") or not hasattr(rsim, "TaskParam"):
        print("set_task_param skipped: API not available on this build")
        return
    task_param = rsim.TaskParam()
    task_param.task_id = 0
    task_param.map.map_id = MAP_ID
    print(f"set_task_param: map_id={MAP_ID}")
    client.set_task_param(task_param)


def connect_rsim_client():
    deadline = time.monotonic() + CONNECT_TIMEOUT_SECONDS
    last_error = None
    while time.monotonic() < deadline:
        try:
            return rsim.RSimClient(RSIM_HOST, RSIM_PORT)
        except RuntimeError as exc:
            last_error = exc
            time.sleep(CONNECT_RETRY_INTERVAL_SECONDS)
    raise RuntimeError(f"failed to connect RSim {RSIM_HOST}:{RSIM_PORT}: {last_error}")


def reset_spectator(client, sr_client, ego_name) -> None:
    if not ego_name:
        print("reset spectator skipped: no ego")
        return

    world = open_3d_world(client, "reset spectator")
    if world is None:
        return

    try:
        ego_actor_id = sr_client.get_actor_id_in_world(ego_name)
        follow_mode = rsim.SpectatorFollowMode.God
        follow_transform = rsim.Transform(
            rsim.Location(x=-15.0, y=0.0, z=10.0),
            rsim.Rotation(pitch=-15.0, yaw=0.0, roll=0.0),
        )
        actor_ok = world.set_spectator_follow_actor(ego_actor_id)
        transform_ok = world.set_spectator_follow_transform(follow_mode, follow_transform)
        mode_ok = world.set_spectator_follow_mode(follow_mode)
        print(
            "reset spectator follow-actor: "
            f"ego_actor_id={ego_actor_id} actor_ok={actor_ok} "
            f"transform_ok={transform_ok} mode_ok={mode_ok}"
        )
    except Exception as exc:
        print(f"reset spectator follow-actor skipped: {exc}")


def init_3d_world(client) -> None:
    world = open_3d_world(client, "init 3d world")
    if world is None:
        return

    try:
        world.show_cursor(True)
    except Exception as exc:
        print(f"init 3d world show_cursor skipped: {exc}")

    try:
        world.set_spectator_follow_mode(rsim.SpectatorFollowMode.God)
        print("init 3d world: spectator follow_mode=God")
    except Exception as exc:
        print(f"init 3d world set_spectator_follow_mode skipped: {exc}")


def open_3d_world(client, tag: str):
    try:
        world_client = client.get_world_client()
    except Exception as exc:
        print(f"{tag} skipped: get_world_client failed: {exc}")
        return None

    if world_client is None:
        print(f"{tag} skipped: 3d world client is not available")
        return None

    try:
        return world_client.get_world()
    except Exception as exc:
        print(f"{tag} skipped: get_world failed: {exc}")
        return None


def read_axis(joystick, axis_index: int, invert: bool) -> float:
    if axis_index < 0 or axis_index >= joystick.get_numaxes():
        return 0.0
    value = float(joystick.get_axis(axis_index))
    return -value if invert else value


def axis_to_pedal(value: float) -> float:
    return clamp((value + 1.0) * 0.5, 0.0, 1.0)


def print_runtime_status(
    elapsed: float,
    state,
    control: ControlState,
    status,
    status_count: int,
) -> None:
    if status is not None:
        pose = status.rear_axle_center
        position = pose.position
        velocity = status.vel_body
        print(
            f"t={elapsed:6.1f}s state={state} "
            f"throttle={control.throttle:.2f} brake={control.brake:.2f} "
            f"steer={control.steering:.3f} "
            f"pos=({position.x:.2f}, {position.y:.2f}, {position.z:.2f}) "
            f"vel=({velocity.x:.2f}, {velocity.y:.2f}, {velocity.z:.2f}) "
            f"vehicle_status_count={status_count}"
        )
        return

    print(
        f"t={elapsed:6.1f}s state={state} "
        f"throttle={control.throttle:.2f} brake={control.brake:.2f} "
        f"steer={control.steering:.3f} vehicle_status_count={status_count}"
    )


def print_control_help() -> None:
    print("control help:")
    print(f"  steer axis={LOGITECH_STEER_AXIS}, throttle axis={LOGITECH_THROTTLE_AXIS}, brake axis={LOGITECH_BRAKE_AXIS}")
    print(f"  invert steer={LOGITECH_INVERT_STEER}, throttle={LOGITECH_INVERT_THROTTLE}, brake={LOGITECH_INVERT_BRAKE}")
    print(f"  deadzone={LOGITECH_DEADZONE}, max_steering={MAX_STEERING}, steer_sign={STEER_SIGN}")
    print("  quit with Ctrl+C")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


if __name__ == "__main__":
    sys.exit(main())
