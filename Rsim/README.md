# dynamic_vehicle examples

本目录包含两个 Python example:

- `dynamic_vehicle.py`: 基础 smoke,验证 RSim 绑定外部 dynamic_vehicle 后 `load_trial(1075)` + `run()` 可持续运行,并观察 vehicle status stream callback。
- `logistic_example.py`: 控制输入 demo,在基础链路上默认支持 scripted / keyboard / Logitech 三种控制来源。

两个脚本都不使用命令行参数。端口、场景、控制器配置都在脚本顶部全局变量里改。
脚本会在 `load_trial(SCENARIO_ID, SEED)` 后绑定 dynamic_vehicle 控制链路;当前不额外调用 `scene_runner.set_external_control(ego)`。
是否启用 3D 由 `rsim launch rsim --enable_3d ...` 决定；两个脚本里的 `ENABLE_3D`
只用于 3D 已启动时的 spectator 调整。

## 启动服务

```bash
conda run -n realsim rsim stop rsim --port 9000
conda run -n realsim rsim launch rsim --port 9000 --enable_3d false
conda run -n realsim rsim launch rsim_vehicle --rpc_port 9030
```

默认连接:

- RSim: `192.168.2.181:9000`
- dynamic_vehicle: `192.168.2.181:9030`
- scenario: `1075`
- map: `1173`

## Example 1: dynamic_vehicle.py

```bash
conda run -n realsim python Rsim/dynamic_vehicle.py
```

预期现象:

```text
enable_world_3d:0
set_vehicle_client: 192.168.2.181:9030
loaded trial: 1075
run requested
vehicle status callback count=1
vehicle status callback count=100
...
```

RSim / scene_runner / dynamic_vehicle 侧应能看到:

```text
load_trial 1075 READY
state RUNNING
RSimVehicleRemoteController received vehicle status callback count=...
vehicle status data sent to sensor stream
```

## Example 2: logistic_example.py

```bash
conda run -n realsim python Rsim/logistic_example.py
```

控制源默认同时启用,每一轮按优先级选择:

```text
Logitech active > keyboard active > scripted default
```

脚本默认打开 `ENABLE_DIRECT_FMI_CONTROL = True`。这是为了补齐当前 Python `RPCControlCmd`
未暴露 `IMP_MODE_TRANS` 的限制,让 D/R 挡能实际写入 dynamic_vehicle。该开关也是脚本顶部全局变量,
不需要命令行参数。

键盘控制:

```text
W / Up       throttle
S / Down     brake
A / Left     steer left
D / Right    steer right
Space        parking brake
Q            forward/reverse toggle
Esc          quit
```

Logitech 方向盘通过 `pygame.joystick` 自动检测。未检测到设备时只打印 skipped,不影响 keyboard/scripted 控制。

如需在已经用 `rsim launch rsim --enable_3d true ...` 启动的 3D 模式下调整 spectator,
把对应脚本顶部改为:

```python
ENABLE_3D = True
```

该开关只会在 3D 已启动时 reset spectator;不会额外调用 scene_runner client 的 3D 同步配置。

如需自动验证左转输入会改变 vehicle heading,把 `logistic_example.py` 顶部改为:

```python
ENABLE_SIMULATED_KEYBOARD_TEST = True
```

脚本会模拟前几秒 `W + A`,然后输出:

```text
simulated keyboard left-turn test: initial_yaw=..., final_yaw=..., delta=..., PASS
```
