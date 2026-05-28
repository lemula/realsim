#!/usr/bin/env python3
import argparse
import os
import signal
import sys
import time
from dataclasses import dataclass

from evdev import InputDevice, ecodes, ff, list_devices


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def find_ffb_device(preferred_name=None, fallback="/dev/input/event6"):
    override = os.environ.get("FFB_DEVICE")
    if override:
        return override
    try:
        for path in list_devices():
            try:
                dev = InputDevice(path)
                caps = dev.capabilities().get(ecodes.EV_FF, [])
                if ecodes.FF_CONSTANT not in caps:
                    continue
                if preferred_name and preferred_name.lower() not in (dev.name or "").lower():
                    continue
                return path
            except Exception:
                continue
    except Exception:
        pass
    return fallback


@dataclass
class Config:
    # device / loop
    device: str = "/dev/input/event6"
    hz: int = 100
    print_hz: float = 10.0

    # feedback curve (FFF style)
    deadband: float = 0.02
    max_torque: float = 1
    min_torque: float = 0.1
    soft_zone: float = 0.05
    linear_gain: float = 0.4
    cubic_gain: float = 1
    damping: float = 0.08
    smooth: float = 0.2

    # speed scaling
    speed: float = 0.0
    speed_gain: float = 1.0
    max_speed: float = 30.0

    # output scaling
    gain: float = 1

    # hardware settings
    hw_autocenter: float = 0.0


def normalize_abs_x(dev: InputDevice, raw_value: int) -> float:
    absinfo = dev.absinfo(ecodes.ABS_X)
    if not absinfo:
        return clamp(raw_value / 32767.0, -1.0, 1.0)
    axis_min = absinfo.min
    axis_max = absinfo.max
    mid = 0.5 * (axis_max + axis_min)
    half = 0.5 * (axis_max - axis_min)
    if half <= 1e-9:
        return 0.0
    x = (raw_value - mid) / half
    return clamp(x, -1.0, 1.0)


def build_constant_effect(effect_id: int, torque_norm: float) -> ff.Effect:
    torque_norm = clamp(torque_norm, -1.0, 1.0)
    level = int(round(torque_norm * 0x7FFF))
    level = clamp(level, -0x7FFF, 0x7FFF)

    envelope = ff.Envelope(attack_length=0, attack_level=0, fade_length=0, fade_level=0)
    replay = ff.Replay(length=0x7FFF, delay=0)
    effect_type = ff.EffectType(ff_constant_effect=ff.Constant(level=level, envelope=envelope))

    return ff.Effect(
        ecodes.FF_CONSTANT,
        effect_id,
        0x4000,
        ff.Trigger(0, 0),
        replay,
        effect_type,
    )


def configure_device(dev: InputDevice, cfg: Config) -> None:
    try:
        dev.write(ecodes.EV_FF, ecodes.FF_GAIN, 0x7FFF)
    except Exception:
        pass
    try:
        dev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, int(round(clamp(cfg.hw_autocenter, 0.0, 1.0) * 0x7FFF)))
    except Exception:
        pass


def compute_align_torque(pos: float, vel: float, cfg: Config) -> float:
    if abs(pos) < cfg.deadband:
        return 0.0

    pos_abs = abs(pos)
    if pos_abs <= cfg.soft_zone:
        align_mag = 0.0
    else:
        x = (pos_abs - cfg.soft_zone) / max(1e-6, (1.0 - cfg.soft_zone))
        shaped = cfg.linear_gain * x + cfg.cubic_gain * (x ** 3)
        align_mag = cfg.min_torque + (cfg.max_torque - cfg.min_torque) * shaped
        align_mag = clamp(align_mag, 0.0, cfg.max_torque)

    speed_norm = 0.0 if cfg.max_speed <= 0.0 else clamp(cfg.speed / cfg.max_speed, 0.0, 1.0)
    align_mag *= (1.0 + cfg.speed_gain * speed_norm)
    align_mag = clamp(align_mag, 0.0, cfg.max_torque)

    damping_term = cfg.damping * vel
    torque = ((align_mag * (1.0 if pos > 0.0 else -1.0)) + damping_term) * cfg.gain
    return clamp(torque, -cfg.max_torque, cfg.max_torque)


def parse_args() -> Config:
    default_device = find_ffb_device()
    ap = argparse.ArgumentParser(description="合并版力反馈：参数集中配置 + 实时角度/力显示")
    ap.add_argument("-d", "--device", default=default_device, help="Device path /dev/input/eventX")
    ap.add_argument("-r", "--rate", type=int, default=Config.hz, help="Update rate Hz")
    ap.add_argument("--print-hz", type=float, default=Config.print_hz, help="Print angle/torque rate (Hz)")

    ap.add_argument("--deadband", type=float, default=Config.deadband, help="Deadband around center")
    ap.add_argument("--max-torque", type=float, default=Config.max_torque, help="Max torque [0,1]")
    ap.add_argument("--min-torque", type=float, default=Config.min_torque, help="Min torque outside soft zone")
    ap.add_argument("--soft-zone", type=float, default=Config.soft_zone, help="Soft zone around center")
    ap.add_argument("--linear-gain", type=float, default=Config.linear_gain, help="Linear shape gain")
    ap.add_argument("--cubic-gain", type=float, default=Config.cubic_gain, help="Cubic shape gain")
    ap.add_argument("--damping", type=float, default=Config.damping, help="Damping gain (velocity)")
    ap.add_argument("--smooth", type=float, default=Config.smooth, help="Torque smoothing alpha (0-1)")

    ap.add_argument("--speed", type=float, default=Config.speed, help="Vehicle speed (same unit as max-speed)")
    ap.add_argument("--speed-gain", type=float, default=Config.speed_gain, help="Speed scaling gain")
    ap.add_argument("--max-speed", type=float, default=Config.max_speed, help="Max speed for scaling")

    ap.add_argument("--gain", type=float, default=Config.gain, help="Overall torque scale [0,1]")
    ap.add_argument("--hw-autocenter", type=float, default=Config.hw_autocenter, help="Enable device autocenter [0,1]")

    args = ap.parse_args()
    return Config(
        device=args.device,
        hz=max(1, args.rate),
        print_hz=max(0.0, args.print_hz),
        deadband=max(0.0, args.deadband),
        max_torque=clamp(args.max_torque, 0.0, 1.0),
        min_torque=clamp(args.min_torque, 0.0, clamp(args.max_torque, 0.0, 1.0)),
        soft_zone=clamp(args.soft_zone, 0.0, 0.5),
        linear_gain=max(0.0, args.linear_gain),
        cubic_gain=max(0.0, args.cubic_gain),
        damping=max(0.0, args.damping),
        smooth=clamp(args.smooth, 0.0, 1.0),
        speed=max(0.0, args.speed),
        speed_gain=max(0.0, args.speed_gain),
        max_speed=max(0.0, args.max_speed),
        gain=clamp(args.gain, 0.0, 1.0),
        hw_autocenter=clamp(args.hw_autocenter, 0.0, 1.0),
    )


def main() -> int:
    cfg = parse_args()

    dev = InputDevice(cfg.device)
    caps = dev.capabilities()
    ff_caps = caps.get(ecodes.EV_FF, [])
    if ecodes.FF_CONSTANT not in ff_caps:
        print("ERROR: Device does not support FF_CONSTANT.", file=sys.stderr)
        return 1

    try:
        os.set_blocking(dev.fd, False)
    except Exception:
        pass

    configure_device(dev, cfg)

    running = True

    def _sigint(_signo, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    current_pos = 0.0
    last_pos = 0.0
    last_time = time.time()

    effect_id = -1
    eff0 = build_constant_effect(effect_id, 0.0)
    effect_id = dev.upload_effect(eff0)
    try:
        dev.write(ecodes.EV_FF, effect_id, 1)
    except Exception:
        pass

    print(
        f"Using device: {cfg.device} | hz={cfg.hz} | gain={cfg.gain:.2f} | speed={cfg.speed:.2f}"
    )

    period = 1.0 / cfg.hz
    print_period = 1.0 / cfg.print_hz if cfg.print_hz > 0 else 0.0
    last_print = time.time()

    try:
        while running:
            try:
                for ev in dev.read():
                    if ev.type == ecodes.EV_ABS and ev.code == ecodes.ABS_X:
                        current_pos = normalize_abs_x(dev, ev.value)
            except BlockingIOError:
                pass

            now = time.time()
            dt = max(1e-4, now - last_time)
            vel = (current_pos - last_pos) / dt
            last_pos = current_pos
            last_time = now

            torque_cmd = compute_align_torque(current_pos, vel, cfg)

            if not hasattr(main, "_torque_f"):
                main._torque_f = torque_cmd
            else:
                main._torque_f = (1.0 - cfg.smooth) * main._torque_f + cfg.smooth * torque_cmd
            torque_cmd = main._torque_f

            eff = build_constant_effect(effect_id, torque_cmd)
            effect_id = dev.upload_effect(eff)
            try:
                dev.write(ecodes.EV_FF, effect_id, 1)
            except Exception:
                pass

            if print_period > 0:
                now_p = time.time()
                if now_p - last_print >= print_period:
                    sys.stdout.write(
                        f"\rangle={current_pos:+.3f}  torque={torque_cmd:+.3f}  vel={vel:+.3f}   "
                    )
                    sys.stdout.flush()
                    last_print = now_p

            time.sleep(period)

    finally:
        try:
            effz = build_constant_effect(effect_id, 0.0)
            dev.upload_effect(effz)
            dev.write(ecodes.EV_FF, effect_id, 1)
        except Exception:
            pass
        try:
            dev.erase_effect(effect_id)
        except Exception:
            pass
        try:
            dev.close()
        except Exception:
            pass
        print("\nExit.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())