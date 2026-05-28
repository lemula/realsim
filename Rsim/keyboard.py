"""Keyboard-only control example for RSim dynamic_vehicle.

Arrow keys:
  Up       throttle
  Down     brake
  Left     steer left
  Right    steer right
  Space    parking brake
  Esc      quit
"""

from __future__ import annotations

import functools
import sys
import time
from dataclasses import dataclass

import rsim

try:
    import pygame
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
except ImportError:
    pygame = None


print = functools.partial(print, flush=True)


RSIM_HOST = "127.0.0.1"
RSIM_PORT = 9000
VEHICLE_HOST = "127.0.0.1"
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

THROTTLE_VALUE = 0.35
BRAKE_VALUE = 0.65
STEER_INCREMENT_PER_SECOND = 1.6
STEER_RETURN_PER_SECOND = 2.4
MAX_STEERING = 0.45
STEER_SIGN = -1.0
FORWARD_GEAR = 1
FORWARD_MODE_TRANS = 2


@dataclass
class ControlState:
    throttle: float = 0.0
    brake: float = 0.0
    steering: float = 0.0
    parking_brake: bool = False
    quit_requested: bool = False


class KeyboardController:
    def __init__(self) -> None:
        if pygame is None:
            raise RuntimeError("pygame is not installed, please install pygame first")
        self._steering = 0.0

    def poll(self, dt: float) -> ControlState:
        quit_requested = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit_requested = True
            elif event.type == pygame.KEYUP and event.key == K_ESCAPE:
                quit_requested = True

        keys = pygame.key.get_pressed()
        throttle = THROTTLE_VALUE if keys[K_UP] else 0.0
        brake = BRAKE_VALUE if keys[K_DOWN] else 0.0
        if brake > 0.0:
            throttle = 0.0

        steer_step = STEER_INCREMENT_PER_SECOND * dt
        return_step = STEER_RETURN_PER_SECOND * dt
        if keys[K_LEFT]:
            self._steering -= steer_step
        elif keys[K_RIGHT]:
            self._steering += steer_step
        elif self._steering > 0.0:
            self._steering = max(0.0, self._steering - return_step)
        elif self._steering < 0.0:
            self._steering = min(0.0, self._steering + return_step)

        self._steering = clamp(self._steering, -MAX_STEERING, MAX_STEERING)
        return ControlState(
            throttle=throttle,
            brake=brake,
            steering=self._steering,
            parking_brake=bool(keys[K_SPACE]),
            quit_requested=quit_requested,
        )


def main() -> int:
    if pygame is None:
        print("pygame is not installed. Install it with: pip install pygame")
        return 1

    pygame.init()
    pygame.display.set_mode((520, 160))
    pygame.display.set_caption("RSim keyboard vehicle control")
    draw_help_window()
    print_control_help()

    client = connect_rsim_client()
    print(f"version: {client.get_version()}")

    is_ready = client.wait_for_init(WAIT_FOR_INIT_SECONDS)
    print(f"is_ready: {is_ready}")
    if not is_ready:
        print("rsim init timeout")
        pygame.quit()
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

    controller = KeyboardController()
    client.run()
    print("run requested")

    start_time = time.monotonic()
    last_control_time = start_time
    last_print_time = start_time
    exit_code = 0

    try:
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

            control = controller.poll(dt)
            if control.quit_requested:
                print("quit requested")
                break

            cmd = make_control_cmd(control)
            vehicle_client.set_control_cmd(cmd)

            if now - last_print_time >= PRINT_EVERY_SECONDS:
                last_print_time = now
                print_runtime_status(
                    elapsed,
                    state,
                    control,
                    vehicle_status["latest"],
                    vehicle_status["count"],
                )

            if elapsed >= MAX_RUN_SECONDS:
                print(f"max run seconds {MAX_RUN_SECONDS:.1f} reached")
                break
    finally:
        send_stop_command(vehicle_client)
        pygame.quit()

    return exit_code


def make_control_cmd(control: ControlState):
    cmd = rsim.RPCControlCmd()
    cmd.throttle = float(clamp(control.throttle, 0.0, 1.0))
    cmd.brake = float(clamp(control.brake, 0.0, 1.0))
    cmd.steering = float(clamp(control.steering * STEER_SIGN, -MAX_STEERING, MAX_STEERING))
    cmd.gear_location = FORWARD_GEAR
    if hasattr(cmd, "IMP_MODE_TRANS"):
        cmd.IMP_MODE_TRANS = FORWARD_MODE_TRANS
    if hasattr(cmd, "parking_brake"):
        cmd.parking_brake = int(control.parking_brake)
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
            f"steer={control.steering:.3f} parking={int(control.parking_brake)} "
            f"pos=({position.x:.2f}, {position.y:.2f}, {position.z:.2f}) "
            f"vel=({velocity.x:.2f}, {velocity.y:.2f}, {velocity.z:.2f}) "
            f"vehicle_status_count={status_count}"
        )
        return

    print(
        f"t={elapsed:6.1f}s state={state} "
        f"throttle={control.throttle:.2f} brake={control.brake:.2f} "
        f"steer={control.steering:.3f} parking={int(control.parking_brake)} "
        f"vehicle_status_count={status_count}"
    )


def draw_help_window() -> None:
    surface = pygame.display.get_surface()
    if surface is None:
        return

    surface.fill((245, 247, 250))
    font = pygame.font.SysFont(None, 22)
    lines = [
        "RSim keyboard vehicle control",
        "Up: throttle    Down: brake",
        "Left: steer left    Right: steer right",
        "Space: parking brake    Esc: quit",
    ]
    for idx, line in enumerate(lines):
        color = (30, 38, 50) if idx == 0 else (65, 74, 88)
        text = font.render(line, True, color)
        surface.blit(text, (18, 18 + idx * 30))
    pygame.display.flip()


def print_control_help() -> None:
    print("control help:")
    print("  Up throttle")
    print("  Down brake")
    print("  Left steer left")
    print("  Right steer right")
    print("  Space parking brake")
    print("  Esc quit")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


if __name__ == "__main__":
    sys.exit(main())
