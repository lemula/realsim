"""dynamic_vehicle smoke example.

流程:
  1. 连接本地 RSim 引擎(127.0.0.1:9000)
  2. 绑定外部 dynamic_vehicle RPC server(127.0.0.1:9030)
  3. 以 external ego control 加载 scenario_id=335
  4. 调用 run() 连续运行
  5. 通过 vehicle status stream callback 观察 ego 状态

当 RSim 服务端提供 3D world (client.has_world_3d() == True) 时,额外做:
  - load_trial 之前: world.show_cursor(True) + 把 spectator 切到 God 模式
  - load_trial 之后: 把 spectator 绑到 ego(follow actor + 默认 God transform)
"""

from __future__ import annotations

import functools
import sys
import time

import rsim


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
MAX_RUN_SECONDS = 30.0
PRINT_EVERY_SECONDS = 1.0

THROTTLE = 0.2
FORWARD_GEAR = 1
FORWARD_MODE_TRANS = 2  # FMU IMP_MODE_TRANS: -1 倒挡 / 0 空挡 / 1 手动挡 / 2 自动挡前进


def main() -> int:
    client = connect_rsim_client()
    print(f"version: {client.get_version()}")

    is_ready = client.wait_for_init(WAIT_FOR_INIT_SECONDS)
    print(f"is_ready: {is_ready}")
    if not is_ready:
        print("rsim init timeout")
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
    # if ego_name is not None:
    #     sr_client.set_external_control(ego_name)
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

    # 构造前进控制命令; run() 后在循环里持续发送给 dynamic_vehicle。
    forward_cmd = rsim.RPCControlCmd()
    forward_cmd.throttle = THROTTLE  # 油门开度, 无量纲 0~1
    forward_cmd.brake = 0.0  # 制动开度, 无量纲 0~1
    forward_cmd.steering = 0.0  # 车轮等效转角, rad
    forward_cmd.IMP_MODE_TRANS = FORWARD_MODE_TRANS  # 变速箱模式枚举: 2=自动挡前进
    forward_cmd.gear_location = FORWARD_GEAR  # 档位枚举: 1=前进挡

    client.run()
    print(f"run requested (throttle={THROTTLE:.2f}, gear=forward)")

    start_time = time.monotonic()
    last_print_time = start_time
    while True:
        now = time.monotonic()
        elapsed = now - start_time
        state = client.get_state()

        if state == rsim.ModuleState.FINISHED:
            print("rsim finished")
            return 0
        if state == rsim.ModuleState.RUNNING_ERROR:
            print("rsim running error")
            return 1

        vehicle_client.set_control_cmd(forward_cmd)

        if now - last_print_time >= PRINT_EVERY_SECONDS:
            last_print_time = now
            print_runtime_status(elapsed, state, vehicle_status["latest"], vehicle_status["count"])

        if elapsed >= MAX_RUN_SECONDS:
            print(f"max run seconds {MAX_RUN_SECONDS:.1f} reached")
            return 0

        time.sleep(0.05)


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


def print_runtime_status(elapsed: float, state, status, status_count: int) -> None:
    if status is not None:
        pose = status.rear_axle_center
        position = pose.position
        rotation = status.ang_body
        velocity = status.vel_body
        print(
            f"t={elapsed:6.1f}s state={state} "
            f"pos=({position.x:.2f}, {position.y:.2f}, {position.z:.2f}) "
            f"rot=({rotation.x:.3f}, {rotation.y:.3f}, {rotation.z:.3f}) "
            f"vel=({velocity.x:.2f}, {velocity.y:.2f}, {velocity.z:.2f}) "
            f"vehicle_status_count={status_count}"
        )
        return
    print(f"t={elapsed:6.1f}s state={state} vehicle_status_count={status_count}")


if __name__ == "__main__":
    sys.exit(main())
