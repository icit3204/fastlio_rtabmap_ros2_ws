#!/usr/bin/env python3
import argparse
import select
import socket
import struct
import sys
import termios
import time
import tty


CAN_EFF_FLAG = socket.CAN_EFF_FLAG
DEFAULT_CAN_ID = 0x801400


STOP_COMMAND = ("stop", 0, 10000, 0)


def build_commands(args):
    return {
        "w": ("forward", args.forward_vel, args.straight_radius, 0),
        "s": ("backward", args.backward_vel, args.straight_radius, 0),
        "a": ("slow_left", args.turn_vel, args.turn_radius, 0),
        "d": ("slow_right", args.turn_vel, -args.turn_radius, 0),
        "q": ("fast_left", args.fast_turn_vel, args.fast_turn_radius, 0),
        "e": ("fast_right", args.fast_turn_vel, -args.fast_turn_radius, 0),
        "z": ("spin_left", -args.spin_vel, 0, 0),
        "c": ("spin_right", args.spin_vel, 0, 0),
        "x": STOP_COMMAND,
        " ": STOP_COMMAND,
        "p": STOP_COMMAND,
    }


def clamp_int16(value: int, limit: int) -> int:
    limit = min(abs(limit), 32767)
    return max(-limit, min(limit, value))


def calc_wheel_speeds(vel: int, radius: int, velocity_limit: int, wheel_radius: int):
    vel = clamp_int16(vel, velocity_limit)
    if vel == 0:
        return 0, 0
    if radius == 0:
        return vel, -vel
    if radius >= 10000:
        return -vel, -vel

    left = -int((radius + wheel_radius) * vel / radius + 0.5)
    right = -int((radius - wheel_radius) * vel / radius + 0.5)
    return clamp_int16(left, velocity_limit), clamp_int16(right, velocity_limit)


def make_can_frame(can_id: int, vel: int, radius: int, distance: int, velocity_limit: int, wheel_radius: int):
    left, right = calc_wheel_speeds(vel, radius, velocity_limit, wheel_radius)
    data = struct.pack("<hhhh", left, right, clamp_int16(distance, 32767), clamp_int16(distance, 32767))
    frame_id = can_id | CAN_EFF_FLAG
    frame = struct.pack("=IB3x8s", frame_id, 8, data)
    return frame, left, right, data


def send_command(sock, can_id: int, name: str, vel: int, radius: int, distance: int, velocity_limit: int, wheel_radius: int, quiet: bool):
    frame, left, right, data = make_can_frame(can_id, vel, radius, distance, velocity_limit, wheel_radius)
    sock.send(frame)
    if not quiet:
        print(
            f"\r{name:<11} vel={vel:>6} radius={radius:>6} "
            f"left={left:>6} right={right:>6} data={data.hex(' ').upper()}",
            end="",
            flush=True,
        )


def print_help(args, can_id: int):
    mode = "latch" if args.latch else f"deadman timeout={args.hold_timeout_ms:.0f}ms"
    print(
        f"""
Keyboard CAN control
interface={args.interface} id=0x{can_id:X} period={args.period_ms:.1f}ms mode={mode}

  W  forward      vel={args.forward_vel:>6} radius={args.straight_radius:>6}
  S  backward     vel={args.backward_vel:>6} radius={args.straight_radius:>6}
  A  slow left    vel={args.turn_vel:>6} radius={args.turn_radius:>6}
  D  slow right   vel={args.turn_vel:>6} radius={-args.turn_radius:>6}
  Q  fast left    vel={args.fast_turn_vel:>6} radius={args.fast_turn_radius:>6}
  E  fast right   vel={args.fast_turn_vel:>6} radius={-args.fast_turn_radius:>6}
  Z  spin left    vel={-args.spin_vel:>6} radius={0:>6}
  C  spin right   vel={args.spin_vel:>6} radius={0:>6}
  X / Space / P   stop
  H               show this help
  Esc / Ctrl+C    stop and exit

Default mode is deadman control: motion stops automatically when no motion key
is received within --hold-timeout-ms. Use --latch to restore old behavior.
"""
    )


def main():
    parser = argparse.ArgumentParser(description="Keyboard teleop for wheelchair CAN control on SocketCAN.")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface, default: can0")
    parser.add_argument("--can-id", default="0x801400", help="extended CAN id, default: 0x801400")
    parser.add_argument("--period-ms", type=float, default=20.0, help="send period in ms, default: 20")
    parser.add_argument("--velocity-limit", type=int, default=16380, help="absolute int16 velocity limit")
    parser.add_argument("--wheel-radius", type=int, default=300, help="same WHEEL_RADIUS value as Can2026")
    parser.add_argument("--hold-timeout-ms", type=float, default=250.0, help="deadman timeout after last motion key, default: 250")
    parser.add_argument("--latch", action="store_true", help="old behavior: keep sending the selected command until another key is pressed")
    parser.add_argument("--forward-vel", type=int, default=1500, help="W velocity, lower is safer")
    parser.add_argument("--backward-vel", type=int, default=-1000, help="S velocity, should normally be negative")
    parser.add_argument("--turn-vel", type=int, default=1000, help="A/D arc-turn velocity")
    parser.add_argument("--fast-turn-vel", type=int, default=1500, help="Q/E larger-radius turn velocity")
    parser.add_argument("--spin-vel", type=int, default=700, help="Z/C in-place spin velocity magnitude")
    parser.add_argument("--straight-radius", type=int, default=10000, help="radius treated as straight driving")
    parser.add_argument("--turn-radius", type=int, default=1800, help="A/D turn radius in mm; smaller is sharper")
    parser.add_argument("--fast-turn-radius", type=int, default=3000, help="Q/E turn radius in mm; larger is gentler")
    parser.add_argument("--quiet", action="store_true", help="do not print every transmitted frame")
    args = parser.parse_args()

    can_id = int(str(args.can_id), 0)
    commands = build_commands(args)

    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.interface,))

    old_term = termios.tcgetattr(sys.stdin)
    current = STOP_COMMAND
    last_sent = 0.0
    last_motion_key = 0.0
    stop_reported = True
    period = max(args.period_ms / 1000.0, 0.001)
    hold_timeout = max(args.hold_timeout_ms / 1000.0, period)

    print_help(args, can_id)
    print("Current: stop")

    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            now = time.monotonic()
            if (
                not args.latch
                and current != STOP_COMMAND
                and now - last_motion_key > hold_timeout
            ):
                current = STOP_COMMAND
                stop_reported = True
                send_command(sock, can_id, *current, args.velocity_limit, args.wheel_radius, True)
                last_sent = now
                if not args.quiet:
                    print("\nAuto stop")

            if now - last_sent >= period:
                send_command(sock, can_id, *current, args.velocity_limit, args.wheel_radius, args.quiet)
                last_sent = now

            readable, _, _ = select.select([sys.stdin], [], [], 0.01)
            if not readable:
                continue

            ch = sys.stdin.read(1)
            if ch == "\x1b":
                break
            key = ch.lower()
            if key == "h":
                print()
                print_help(args, can_id)
                continue
            if key in commands:
                current = commands[key]
                if current != STOP_COMMAND:
                    last_motion_key = time.monotonic()
                    stop_reported = False
                elif not stop_reported:
                    stop_reported = True
                print(f"\nSelected: {current[0]}")

    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping...")
        try:
            for _ in range(5):
                send_command(sock, can_id, *STOP_COMMAND, args.velocity_limit, args.wheel_radius, True)
                time.sleep(0.02)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
            sock.close()
            print("Stopped.")


if __name__ == "__main__":
    main()
