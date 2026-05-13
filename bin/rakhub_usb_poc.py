#!/usr/bin/env python3
"""Minimal RAKSensorHub USB text POC helper.

This tool sends the same RAKHUB text lines that are used manually over USB CDC.
It intentionally does not use Meshtastic protobuf/admin APIs.
"""

import argparse
import sys
import time
from typing import Dict, Optional

try:
    import serial
except ImportError:  # pragma: no cover - environment hint
    print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
    raise


def parse_task_spec(spec: str) -> Dict[str, str]:
    values: Dict[str, str] = {}
    for item in spec.split(","):
        if not item:
            continue
        key, sep, value = item.partition("=")
        if not sep:
            raise ValueError(f"task spec item must be key=value: {item}")
        values[key.strip()] = value.strip()
    return values


def value(args: argparse.Namespace, overrides: Dict[str, str], name: str) -> str:
    return overrides.get(name, str(getattr(args, name)))


def build_rs485(args: argparse.Namespace, slot: Optional[int] = None, overrides: Optional[Dict[str, str]] = None) -> str:
    overrides = overrides or {}
    slot_part = f"slot={slot} " if slot is not None else ""
    return (
        "RAKHUB RS485 "
        f"baud={value(args, overrides, 'baud')} databit={value(args, overrides, 'databit')} "
        f"stop={value(args, overrides, 'stop')} parity={value(args, overrides, 'parity')} "
        f"{slot_part}task={value(args, overrides, 'task')} hex={value(args, overrides, 'hex')} "
        f"period={value(args, overrides, 'period')} timeout={value(args, overrides, 'timeout')} "
        f"retry={value(args, overrides, 'retry')} scale={value(args, overrides, 'scale')} "
        f"ipso={value(args, overrides, 'ipso')} dtype={value(args, overrides, 'dtype')} name={value(args, overrides, 'name')}"
    )


def build_aic(args: argparse.Namespace) -> str:
    return (
        "RAKHUB AIC "
        f"ch={args.ch} ipso={args.ipso} min={args.min} max={args.max} off={args.off} name={args.name}"
    )


def send_line(port: serial.Serial, line: str) -> None:
    print(f"> {line}")
    port.write((line + "\n").encode("ascii"))
    port.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description="Send RAKHUB USB text POC commands")
    parser.add_argument("--port", required=True, help="USB CDC serial port, e.g. COM7 or /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200, help="USB CDC baudrate")
    parser.add_argument("--apply", action="store_true", help="Send RAKHUB APPLY after profile command")
    parser.add_argument("--pid", default="", help="Optional hex PID for APPLY, e.g. 01")

    sub = parser.add_subparsers(dest="cmd", required=True)

    rs485 = sub.add_parser("rs485", help="Send RAKHUB RS485 profile")
    rs485.add_argument("--baud", type=int, default=4800)
    rs485.add_argument("--databit", type=int, default=8)
    rs485.add_argument("--stop", type=int, default=1)
    rs485.add_argument("--parity", type=int, default=0)
    rs485.add_argument("--task", type=int, default=1)
    rs485.add_argument("--hex", default="010300120001", help="Modbus request bytes without CRC")
    rs485.add_argument("--period", type=int, default=60)
    rs485.add_argument("--timeout", type=int, default=5000)
    rs485.add_argument("--retry", type=int, default=2)
    rs485.add_argument("--scale", default="0.1")
    rs485.add_argument("--ipso", type=int, default=112)
    rs485.add_argument("--dtype", type=int, default=6)
    rs485.add_argument("--name", default="GE")
    rs485.add_argument(
        "--task-spec",
        action="append",
        default=[],
        help=(
            "Append one RS485 task as comma key=value list. Example: "
            "task=2,hex=010300010001,ipso=103,scale=0.1,name=temp"
        ),
    )

    aic = sub.add_parser("aic", help="Send RAKHUB AIC profile")
    aic.add_argument("--ch", type=int, default=1)
    aic.add_argument("--ipso", type=int, default=130)
    aic.add_argument("--min", type=int, default=0)
    aic.add_argument("--max", type=int, default=5)
    aic.add_argument("--off", default="0")
    aic.add_argument("--name", default="ULB16_05")

    status = sub.add_parser("status", help="Send RAKHUB STATUS")
    status.set_defaults()

    builtin = sub.add_parser("builtin", help="Select built-in template id, then optionally apply")
    builtin.add_argument("id", type=int, choices=range(0, 5), help="0=clear RS485 placeholder, 4=AIC, others RS485 presets")

    args = parser.parse_args()

    with serial.Serial(args.port, args.baudrate, timeout=0.2) as port:
        if args.cmd == "rs485":
            if args.task_spec:
                for slot, spec in enumerate(args.task_spec):
                    if slot >= 4:
                        raise ValueError("at most 4 RS485 task-spec entries are supported")
                    send_line(port, build_rs485(args, slot=slot, overrides=parse_task_spec(spec)))
                    time.sleep(0.05)
            else:
                send_line(port, build_rs485(args))
        elif args.cmd == "aic":
            send_line(port, build_aic(args))
        elif args.cmd == "status":
            send_line(port, "RAKHUB STATUS")
        elif args.cmd == "builtin":
            send_line(port, f"RAKHUB BUILTIN {args.id}")

        if args.apply and args.cmd in {"rs485", "aic", "builtin"}:
            time.sleep(0.2)
            apply = "RAKHUB APPLY" + (f" {args.pid}" if args.pid else "")
            send_line(port, apply)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
