"""dynamic_vehicle live-debug example.

流程:
  1. 连接本地 RSim 引擎(127.0.0.1:9000)
  2. 绑定 dynamic_vehicle RPC server(127.0.0.1:9030)
  3. 以 external ego control 加载 scenario_id=1075
  4. 调用 run() 让 RSim 连续运行
  5. 默认支持 keyboard / Logitech 控制来源; scripted 控制可按需打开

当 RSim 服务端提供 3D world (client.has_world_3d() == True) 时,额外做:
  - load_trial 之前: world.show_cursor(True) + 把 spectator 切到 God 模式
  - load_trial 之后: 把 spectator 绑到 ego(follow actor + 默认 God transform)
"""

from __future__ import annotations

import math
import socket
import struct
import sys
import functools
import time
from dataclasses import dataclass
from typing import Iterable

import rsim

try:
    import pygame
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    pygame = None 


print = functools.partial(print, flush=True)


# ---- Global example configuration -----------------------------------------

RSIM_HOST = "127.0.0.1"
RSIM_PORT = 9000
VEHICLE_HOST = "127.0.0.1"
VEHICLE_PORT = 9030
SCENARIO_ID = 1075
SEED = 0
MAP_ID = 1173

WAIT_FOR_INIT_SECONDS = 6.0
CONNECT_TIMEOUT_SECONDS = 30.0
CONNECT_RETRY_INTERVAL_SECONDS = 0.5
CONTROL_HZ = 20.0
MAX_RUN_SECONDS = 120.0
PRINT_EVERY_SECONDS = 1.0

ENABLE_3D = True  # 只控制 3D UI/spectator 初始化; 是否有 3D 由 client.has_world_3d() 判定
ENABLE_SCRIPTED_CONTROLLER = False
ENABLE_KEYBOARD_CONTROLLER = True
ENABLE_LOGITECH_CONTROLLER = True
ENABLE_SIMULATED_KEYBOARD_TEST = False

# Simulated keyboard smoke test: time window in seconds, plus active keys.
SIMULATED_KEYBOARD_SEQUENCE = [
    (0.0, 4.0, {"w", "a"}),
    (4.0, 6.0, {"w"}),
]
HEADING_CHANGE_THRESHOLD = 0.02
SIMULATED_TEST_MAX_SECONDS = 8.0

# Control tuning. Positive steering follows the dynamic_vehicle API; if the
# physical vehicle turns opposite to your input device, flip STEER_SIGN.
THROTTLE_VALUE = 0.35
BRAKE_VALUE = 0.6
KEYBOARD_STEER_INCREMENT_PER_SECOND = 1.6
KEYBOARD_STEER_RETURN_PER_SECOND = 2.4
MAX_STEERING = 0.45
STEER_SIGN = -1.0
FORWARD_GEAR = 1
REVERSE_GEAR = -1
FORWARD_MODE_TRANS = 2
REVERSE_MODE_TRANS = -1
ENABLE_DIRECT_FMI_CONTROL = True

# Logitech/default joystick axis mapping. Adjust these globals for the local
# device if the detected wheel reports different axes.
LOGITECH_STEER_AXIS = 0
LOGITECH_THROTTLE_AXIS = 2
LOGITECH_BRAKE_AXIS = 3
LOGITECH_DEADZONE = 0.05
LOGITECH_INVERT_STEER = False
LOGITECH_INVERT_THROTTLE = True
LOGITECH_INVERT_BRAKE = True


@dataclass
class ControlState:
    throttle: float = 0.0
    brake: float = 0.0
    steering: float = 0.0
    parking_brake: bool = False
    reverse: bool = False
    active: bool = False
    quit_requested: bool = False
    source: str = "none"


class ScriptedController:
    def __init__(self) -> None:
        self.enabled = ENABLE_SCRIPTED_CONTROLLER

    def poll(self, elapsed_seconds: float, _: float) -> ControlState:
        if not self.enabled:
            return ControlState()

        if elapsed_seconds < 4.0:
            steering = 0.18
        elif elapsed_seconds < 8.0:
            steering = -0.18
        else:
            steering = 0.0

        return ControlState(
            throttle=0.25,
            steering=steering,
            active=True,
            source="scripted",
        )


class KeyboardController:
    def __init__(self) -> None:
        self.enabled = (
            ENABLE_KEYBOARD_CONTROLLER
            and not ENABLE_SIMULATED_KEYBOARD_TEST
            and pygame is not None
        )
        self._steer_cache = 0.0
        self._reverse = False
        if ENABLE_KEYBOARD_CONTROLLER and pygame is None:
            print("keyboard skipped: pygame is not installed")

    def poll(self, _: float, dt: float) -> ControlState:
        if not self.enabled:
            return ControlState()

        quit_requested = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit_requested = True
            elif event.type == pygame.KEYUP:
                if event.key == K_ESCAPE:
                    quit_requested = True
                elif event.key == K_q:
                    self._reverse = not self._reverse
                    gear = "reverse" if self._reverse else "forward"
                    print(f"keyboard gear toggled: {gear}")

        keys = pygame.key.get_pressed()
        throttle = THROTTLE_VALUE if keys[K_UP] or keys[K_w] else 0.0
        brake = BRAKE_VALUE if keys[K_DOWN] or keys[K_s] else 0.0

        steer_step = KEYBOARD_STEER_INCREMENT_PER_SECOND * dt
        return_step = KEYBOARD_STEER_RETURN_PER_SECOND * dt
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_step
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_step
        elif self._steer_cache > 0.0:
            self._steer_cache = max(0.0, self._steer_cache - return_step)
        elif self._steer_cache < 0.0:
            self._steer_cache = min(0.0, self._steer_cache + return_step)

        self._steer_cache = clamp(self._steer_cache, -MAX_STEERING, MAX_STEERING)
        # 倒挡是持续状态; 即使没有踏板/转向输入,也要继续下发 reverse,
        # 否则空控制帧会退回默认前进挡。
        active = bool(throttle or brake or keys[K_SPACE] or abs(self._steer_cache) > 1e-3 or self._reverse)
        return ControlState(
            throttle=throttle,
            brake=brake,
            steering=self._steer_cache,
            parking_brake=bool(keys[K_SPACE]),
            reverse=self._reverse,
            active=active,
            quit_requested=quit_requested,
            source="keyboard",
        )


class LogitechController:
    def __init__(self) -> None:
        self.enabled = False
        self.joystick = None
        if not ENABLE_LOGITECH_CONTROLLER or ENABLE_SIMULATED_KEYBOARD_TEST:
            return
        if pygame is None:
            print("logitech skipped: pygame is not installed")
            return
        if pygame.joystick.get_count() <= 0:
            print("logitech skipped: no joystick detected")
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.enabled = True
        print(
            "joystick enabled: "
            f"{self.joystick.get_name()}, axes={self.joystick.get_numaxes()}, "
            f"buttons={self.joystick.get_numbuttons()}"
        )

    def poll(self, _: float, __: float) -> ControlState:
        if not self.enabled or self.joystick is None:
            return ControlState()

        steer = read_axis(self.joystick, LOGITECH_STEER_AXIS, LOGITECH_INVERT_STEER)
        throttle = axis_to_pedal(
            read_axis(self.joystick, LOGITECH_THROTTLE_AXIS, LOGITECH_INVERT_THROTTLE)
        )
        brake = axis_to_pedal(
            read_axis(self.joystick, LOGITECH_BRAKE_AXIS, LOGITECH_INVERT_BRAKE)
        )

        steer = 0.0 if abs(steer) < LOGITECH_DEADZONE else steer
        throttle = 0.0 if throttle < LOGITECH_DEADZONE else throttle
        brake = 0.0 if brake < LOGITECH_DEADZONE else brake

        active = bool(abs(steer) > 1e-3 or throttle > 1e-3 or brake > 1e-3)
        return ControlState(
            throttle=throttle,
            brake=brake,
            steering=clamp(steer * MAX_STEERING, -MAX_STEERING, MAX_STEERING),
            active=active,
            source="logitech",
        )


class SimulatedKeyboardController:
    def __init__(self) -> None:
        self.enabled = ENABLE_SIMULATED_KEYBOARD_TEST
        self._steer_cache = 0.0

    def poll(self, elapsed_seconds: float, dt: float) -> ControlState:
        if not self.enabled:
            return ControlState()

        keys = simulated_keys(elapsed_seconds)
        throttle = THROTTLE_VALUE if "w" in keys or "up" in keys else 0.0
        brake = BRAKE_VALUE if "s" in keys or "down" in keys else 0.0

        steer_step = KEYBOARD_STEER_INCREMENT_PER_SECOND * dt
        if "a" in keys or "left" in keys:
            self._steer_cache -= steer_step
        elif "d" in keys or "right" in keys:
            self._steer_cache += steer_step
        else:
            self._steer_cache = 0.0

        self._steer_cache = clamp(self._steer_cache, -MAX_STEERING, MAX_STEERING)
        active = bool(keys)
        return ControlState(
            throttle=throttle,
            brake=brake,
            steering=self._steer_cache,
            parking_brake="space" in keys,
            active=active,
            source="simulated-keyboard",
        )


def main() -> int:
    if pygame is not None and not ENABLE_SIMULATED_KEYBOARD_TEST:
        pygame.init()
        pygame.joystick.init()
        if ENABLE_KEYBOARD_CONTROLLER or ENABLE_LOGITECH_CONTROLLER:
            pygame.display.set_mode((620, 220))
            pygame.display.set_caption("RSim dynamic_vehicle control")
            draw_control_help_window()

    print_control_help()

    client = connect_rsim_client()
    print(f"version: {client.get_version()}")

    is_ready = client.wait_for_init(WAIT_FOR_INIT_SECONDS)
    print(f"is_ready: {is_ready}")
    if not is_ready:
        print("rsim init timeout")
        return 1

    sr_client = client.get_scene_runner_client()

    bind_vehicle_client(client, sr_client)

    has_3d = ENABLE_3D and client.has_world_3d()
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
    # rsim_main 已在 load_trial 后把主车控制权交给 dynamic_vehicle。
    # 这里不再调用 set_external_control,避免覆盖 dynamic_vehicle 控制归属。
    if has_3d:
        reset_spectator(client, sr_client, ego_name)

    vehicle_client = client.get_dynamic_vehicle_client()
    fmi_client = connect_direct_fmi_client() if ENABLE_DIRECT_FMI_CONTROL else None
    status_counter = {"count": 0, "latest": None}

    def on_vehicle_status(status):
        status_counter["count"] += 1
        status_counter["latest"] = status
        count = status_counter["count"]
        if count == 1 or count % 100 == 0:
            print(f"vehicle status callback count={count}")

    vehicle_client.on_vehicle_status(on_vehicle_status)

    controllers = [
        SimulatedKeyboardController(),
        LogitechController(),
        KeyboardController(),
        ScriptedController(),
    ]

    initial_yaw = read_heading(sr_client, ego_name, status_counter["latest"])
    client.run()
    print("run requested")

    start_time = time.monotonic()
    last_control_time = start_time
    last_print_time = start_time
    final_yaw = initial_yaw
    last_control = ControlState()
    exit_code = 0

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

        control = choose_control(controllers, elapsed, dt)
        if control.quit_requested:
            print("quit requested")
            break

        cmd = make_control_cmd(control)
        send_control_command(vehicle_client, fmi_client, cmd, control)
        last_control = control

        latest_status = status_counter["latest"]
        final_yaw = read_heading(sr_client, ego_name, latest_status)

        if now - last_print_time >= PRINT_EVERY_SECONDS:
            last_print_time = now
            print_status(elapsed, state, last_control, final_yaw, status_counter["count"])

        if ENABLE_SIMULATED_KEYBOARD_TEST and elapsed >= SIMULATED_TEST_MAX_SECONDS:
            break
        if elapsed >= MAX_RUN_SECONDS:
            print(f"max run seconds {MAX_RUN_SECONDS:.1f} reached")
            break

    send_stop_command(vehicle_client, fmi_client)

    if fmi_client is not None:
        fmi_client.close()

    if ENABLE_SIMULATED_KEYBOARD_TEST:
        passed = print_simulated_keyboard_result(initial_yaw, final_yaw)
        if not passed:
            exit_code = 1

    if pygame is not None:
        pygame.quit()

    return exit_code


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


def choose_control(
    controllers: Iterable[object],
    elapsed_seconds: float,
    dt: float,
) -> ControlState:
    fallback = ControlState()
    for controller in controllers:
        control = controller.poll(elapsed_seconds, dt)
        if control.quit_requested:
            return control
        if control.active:
            return control
        if control.source == "scripted":
            fallback = control
    return fallback


def make_control_cmd(control: ControlState):
    cmd = rsim.RPCControlCmd()
    cmd.throttle = float(clamp(control.throttle, 0.0, 1.0))
    cmd.brake = float(clamp(control.brake, 0.0, 1.0))
    cmd.steering = float(clamp(control.steering * STEER_SIGN, -MAX_STEERING, MAX_STEERING))
    cmd.gear_location = REVERSE_GEAR if control.reverse else FORWARD_GEAR
    if hasattr(cmd, "IMP_MODE_TRANS"):
        cmd.IMP_MODE_TRANS = REVERSE_MODE_TRANS if control.reverse else FORWARD_MODE_TRANS
    if hasattr(cmd, "parking_brake"):
        cmd.parking_brake = int(control.parking_brake)
    return cmd


def send_control_command(vehicle_client, fmi_client, cmd, control: ControlState) -> None:
    vehicle_client.set_control_cmd(cmd)

    if fmi_client is None:
        return

    mode_trans = REVERSE_MODE_TRANS if control.reverse else FORWARD_MODE_TRANS
    codes = fmi_client.call(
        "fmi_set_double",
        ["IMP_THROTTLE_ENGINE", "IMP_BK_STAT", "IMP_STEER_SW", "IMP_MODE_TRANS"],
        [cmd.throttle, cmd.brake, cmd.steering, float(mode_trans)],
    )
    failed = [int(code) for code in codes]
    if any(failed):
        print(f"fmi_set_double control codes: {failed}")


def send_stop_command(vehicle_client, fmi_client) -> None:
    cmd = rsim.RPCControlCmd()
    cmd.throttle = 0.0
    cmd.brake = 1.0
    cmd.steering = 0.0
    cmd.gear_location = FORWARD_GEAR
    try:
        send_control_command(vehicle_client, fmi_client, cmd, ControlState(brake=1.0, active=True, source="stop"))
    except Exception as exc:
        print(f"failed to send stop command: {exc}")


class DirectFmiClient:
    def __init__(self, host: str, port: int, timeout_seconds: float) -> None:
        self._sock = socket.create_connection((host, port), timeout=timeout_seconds)
        self._sock.settimeout(timeout_seconds)
        self._next_msgid = 0
        self._buffer = b""

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass

    def call(self, method: str, *args):
        msgid = self._next_msgid
        self._next_msgid = (self._next_msgid + 1) & 0xFFFFFFFF
        self._sock.sendall(pack_msgpack([0, msgid, method, [[False], *args]]))

        while True:
            message = self._read_message()
            if isinstance(message, list) and len(message) == 4 and message[0] == 1 and message[1] == msgid:
                _, _, error, result = message
                if error is not None:
                    raise RuntimeError(f"RPC {method} error: {error!r}")
                return unwrap_bindsync_response(result)

    def _read_message(self):
        while True:
            try:
                value, offset = unpack_msgpack(self._buffer, 0)
                self._buffer = self._buffer[offset:]
                return value
            except NeedMoreData:
                chunk = self._sock.recv(8192)
                if not chunk:
                    raise ConnectionError("direct fmi rpc server closed connection")
                self._buffer += chunk


class NeedMoreData(Exception):
    pass


def connect_direct_fmi_client() -> DirectFmiClient:
    print(f"direct fmi control: {VEHICLE_HOST}:{VEHICLE_PORT}")
    return DirectFmiClient(VEHICLE_HOST, VEHICLE_PORT, CONNECT_TIMEOUT_SECONDS)


def unwrap_bindsync_response(result):
    if not isinstance(result, list) or len(result) != 1:
        return result
    data = result[0]
    if data == [False]:
        return None
    if not isinstance(data, list) or len(data) != 2:
        return result
    index, payload = data
    if index == 0:
        raise RuntimeError(payload[0] if isinstance(payload, list) and payload else payload)
    if index == 1:
        return payload
    return result


def pack_msgpack(value) -> bytes:
    if value is None:
        return b"\xc0"
    if value is False:
        return b"\xc2"
    if value is True:
        return b"\xc3"
    if isinstance(value, int):
        return pack_msgpack_int(value)
    if isinstance(value, float):
        return b"\xcb" + struct.pack(">d", value)
    if isinstance(value, str):
        data = value.encode("utf-8")
        size = len(data)
        if size < 32:
            return bytes([0xA0 | size]) + data
        if size <= 0xFF:
            return b"\xd9" + struct.pack(">B", size) + data
        if size <= 0xFFFF:
            return b"\xda" + struct.pack(">H", size) + data
        return b"\xdb" + struct.pack(">I", size) + data
    if isinstance(value, (list, tuple)):
        size = len(value)
        if size < 16:
            prefix = bytes([0x90 | size])
        elif size <= 0xFFFF:
            prefix = b"\xdc" + struct.pack(">H", size)
        else:
            prefix = b"\xdd" + struct.pack(">I", size)
        return prefix + b"".join(pack_msgpack(item) for item in value)
    raise TypeError(f"unsupported msgpack value: {type(value)!r}")


def pack_msgpack_int(value: int) -> bytes:
    if 0 <= value <= 0x7F:
        return bytes([value])
    if -32 <= value < 0:
        return struct.pack("b", value)
    if 0 <= value <= 0xFF:
        return b"\xcc" + struct.pack(">B", value)
    if 0 <= value <= 0xFFFF:
        return b"\xcd" + struct.pack(">H", value)
    if 0 <= value <= 0xFFFFFFFF:
        return b"\xce" + struct.pack(">I", value)
    if 0 <= value <= 0xFFFFFFFFFFFFFFFF:
        return b"\xcf" + struct.pack(">Q", value)
    if -0x80 <= value < 0:
        return b"\xd0" + struct.pack(">b", value)
    if -0x8000 <= value < 0:
        return b"\xd1" + struct.pack(">h", value)
    if -0x80000000 <= value < 0:
        return b"\xd2" + struct.pack(">i", value)
    return b"\xd3" + struct.pack(">q", value)


def unpack_msgpack(data: bytes, offset: int):
    if offset >= len(data):
        raise NeedMoreData()
    prefix = data[offset]
    offset += 1

    if prefix <= 0x7F:
        return prefix, offset
    if 0x80 <= prefix <= 0x8F:
        size = prefix & 0x0F
        return unpack_msgpack_map(data, offset, size)
    if 0x90 <= prefix <= 0x9F:
        size = prefix & 0x0F
        return unpack_msgpack_array(data, offset, size)
    if 0xA0 <= prefix <= 0xBF:
        size = prefix & 0x1F
        return unpack_msgpack_str(data, offset, size)
    if prefix >= 0xE0:
        return prefix - 0x100, offset
    if prefix == 0xC0:
        return None, offset
    if prefix == 0xC2:
        return False, offset
    if prefix == 0xC3:
        return True, offset
    if prefix == 0xCA:
        return unpack_struct(data, offset, ">f", 4)
    if prefix == 0xCB:
        return unpack_struct(data, offset, ">d", 8)
    if prefix == 0xCC:
        return unpack_struct(data, offset, ">B", 1)
    if prefix == 0xCD:
        return unpack_struct(data, offset, ">H", 2)
    if prefix == 0xCE:
        return unpack_struct(data, offset, ">I", 4)
    if prefix == 0xCF:
        return unpack_struct(data, offset, ">Q", 8)
    if prefix == 0xD0:
        return unpack_struct(data, offset, ">b", 1)
    if prefix == 0xD1:
        return unpack_struct(data, offset, ">h", 2)
    if prefix == 0xD2:
        return unpack_struct(data, offset, ">i", 4)
    if prefix == 0xD3:
        return unpack_struct(data, offset, ">q", 8)
    if prefix == 0xC4:
        size, offset = unpack_struct(data, offset, ">B", 1)
        return unpack_msgpack_bin(data, offset, size)
    if prefix == 0xC5:
        size, offset = unpack_struct(data, offset, ">H", 2)
        return unpack_msgpack_bin(data, offset, size)
    if prefix == 0xC6:
        size, offset = unpack_struct(data, offset, ">I", 4)
        return unpack_msgpack_bin(data, offset, size)
    if prefix == 0xD9:
        size, offset = unpack_struct(data, offset, ">B", 1)
        return unpack_msgpack_str(data, offset, size)
    if prefix == 0xDA:
        size, offset = unpack_struct(data, offset, ">H", 2)
        return unpack_msgpack_str(data, offset, size)
    if prefix == 0xDB:
        size, offset = unpack_struct(data, offset, ">I", 4)
        return unpack_msgpack_str(data, offset, size)
    if prefix == 0xDC:
        size, offset = unpack_struct(data, offset, ">H", 2)
        return unpack_msgpack_array(data, offset, size)
    if prefix == 0xDD:
        size, offset = unpack_struct(data, offset, ">I", 4)
        return unpack_msgpack_array(data, offset, size)
    if prefix == 0xDE:
        size, offset = unpack_struct(data, offset, ">H", 2)
        return unpack_msgpack_map(data, offset, size)
    if prefix == 0xDF:
        size, offset = unpack_struct(data, offset, ">I", 4)
        return unpack_msgpack_map(data, offset, size)
    raise ValueError(f"unsupported msgpack prefix: 0x{prefix:02x}")


def unpack_struct(data: bytes, offset: int, fmt: str, size: int):
    if offset + size > len(data):
        raise NeedMoreData()
    return struct.unpack(fmt, data[offset:offset + size])[0], offset + size


def unpack_msgpack_str(data: bytes, offset: int, size: int):
    if offset + size > len(data):
        raise NeedMoreData()
    return data[offset:offset + size].decode("utf-8"), offset + size


def unpack_msgpack_bin(data: bytes, offset: int, size: int):
    if offset + size > len(data):
        raise NeedMoreData()
    return data[offset:offset + size], offset + size


def unpack_msgpack_array(data: bytes, offset: int, size: int):
    result = []
    for _ in range(size):
        value, offset = unpack_msgpack(data, offset)
        result.append(value)
    return result, offset


def unpack_msgpack_map(data: bytes, offset: int, size: int):
    result = {}
    for _ in range(size):
        key, offset = unpack_msgpack(data, offset)
        value, offset = unpack_msgpack(data, offset)
        result[key] = value
    return result, offset


def print_status(
    elapsed: float,
    state,
    control: ControlState,
    yaw: float | None,
    vehicle_status_count: int,
) -> None:
    yaw_text = "n/a" if yaw is None else f"{yaw:.4f}"
    print(
        f"t={elapsed:6.1f}s state={state} source={control.source} "
        f"throttle={control.throttle:.2f} brake={control.brake:.2f} "
        f"steer={control.steering:.3f} yaw={yaw_text} "
        f"vehicle_status_count={vehicle_status_count}"
    )


def print_control_help() -> None:
    print("control help:")
    print("  keyboard: W/Up throttle, S/Down brake, A/Left steer left, D/Right steer right")
    print("  keyboard: Space parking brake, Q toggle forward/reverse, Esc quit")
    print(
        "  logitech: "
        f"steer axis={LOGITECH_STEER_AXIS}, throttle axis={LOGITECH_THROTTLE_AXIS}, "
        f"brake axis={LOGITECH_BRAKE_AXIS}; gear/parking/quit use keyboard"
    )
    print(
        "  config: "
        f"STEER_SIGN={STEER_SIGN}, throttle={THROTTLE_VALUE}, brake={BRAKE_VALUE}, "
        f"max_steering={MAX_STEERING}"
    )


def draw_control_help_window() -> None:
    if pygame is None:
        return

    surface = pygame.display.get_surface()
    if surface is None:
        return

    surface.fill((245, 247, 250))
    font = pygame.font.SysFont(None, 22)
    lines = [
        "RSim dynamic_vehicle control",
        "Keyboard: W/Up throttle  S/Down brake",
        "Keyboard: A/Left steer left  D/Right steer right",
        "Keyboard: Space parking brake  Q forward/reverse  Esc quit",
        (
            "Logitech: "
            f"steer axis {LOGITECH_STEER_AXIS}, throttle axis {LOGITECH_THROTTLE_AXIS}, "
            f"brake axis {LOGITECH_BRAKE_AXIS}"
        ),
    ]
    for idx, line in enumerate(lines):
        color = (30, 38, 50) if idx == 0 else (65, 74, 88)
        text = font.render(line, True, color)
        surface.blit(text, (18, 18 + idx * 34))
    pygame.display.flip()


def read_heading(sr_client, ego_name: str | None, latest_status) -> float | None:
    if ego_name:
        try:
            transform = sr_client.get_actor_transform(ego_name)
            return float(transform.rotation.yaw)
        except Exception:
            pass
    if latest_status is not None:
        try:
            return float(latest_status.ang_body.z)
        except Exception:
            return None
    return None


def print_simulated_keyboard_result(initial_yaw: float | None, final_yaw: float | None) -> bool:
    if initial_yaw is None or final_yaw is None:
        print("simulated keyboard left-turn test: yaw unavailable, FAIL")
        return False

    delta = normalize_angle(final_yaw - initial_yaw)
    passed = abs(delta) > HEADING_CHANGE_THRESHOLD
    result = "PASS" if passed else "FAIL"
    print(
        "simulated keyboard left-turn test: "
        f"initial_yaw={initial_yaw:.4f}, final_yaw={final_yaw:.4f}, "
        f"delta={delta:.4f}, {result}"
    )
    return passed


def simulated_keys(elapsed_seconds: float) -> set[str]:
    keys = set()
    for start, end, window_keys in SIMULATED_KEYBOARD_SEQUENCE:
        if start <= elapsed_seconds < end:
            keys.update(window_keys)
    return keys


def read_axis(joystick, axis_index: int, invert: bool) -> float:
    if axis_index < 0 or axis_index >= joystick.get_numaxes():
        return 0.0
    value = float(joystick.get_axis(axis_index))
    return -value if invert else value


def axis_to_pedal(value: float) -> float:
    # Some Logitech pedals report released=1 and pressed=-1; after optional
    # inversion this maps the active direction to 0..1.
    return clamp((value + 1.0) * 0.5, 0.0, 1.0)


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


if __name__ == "__main__":
    sys.exit(main())
