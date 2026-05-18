#!/usr/bin/env python3
"""Minimal RAKSensorHub USB text POC helper.

This tool sends the same RAKHUB text lines that are used manually over USB CDC.
It intentionally does not use Meshtastic protobuf/admin APIs.

Debug:
  - Global ``--listen-after SEC``: after most subcommands, print incoming serial lines for SEC seconds.
  - Global ``--pid HEX``: optional probe PID for **CLEARAIC** only (may appear before the subcommand). ``RAKHUB APPLY`` is sent **without** a PID suffix; firmware resolves the probe.
  - ``json --apply``: by default prints up to **20** serial log lines after APPLY (``--post-apply-lines N`` / ``0`` to disable).
  - ``reboot``: sends ``RAKHUB REBOOT`` only; firmware picks probe PID.
  - ``monitor``: stream RX to stdout until Ctrl+C or ``--duration``.

WisToolBox-style JSON (single object) can be loaded to emit equivalent RAKHUB lines
for Interface RS485 (ATC+IO_CFG + ATC+IO_ADDPOLL) and Interface AI (atc+io_decode for
AIC / channel 1). Other ATC lines are ignored in POC with a stderr note.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from typing import Any, Dict, List, Optional, Tuple, Union

try:
    import serial
except ImportError:  # pragma: no cover - environment hint
    print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
    raise

JsonDoc = Union[Dict[str, Any], List[Any]]


# Default WisToolBox placeholder values (override with --subst KEY=VAL)
DEFAULT_PLACEHOLDERS: Dict[str, str] = {
    "PRB_ID": "01",
    "TASK_ID1": "1",
    "TASK_ID2": "2",
    "DEV_ADDR": "01",
    "PROFILE_NAME": "GE",
    "SNSR_INTV": "60",
}


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


def build_rs485_line(
    baud: int,
    databit: int,
    stop: int,
    parity: int,
    fields: Dict[str, str],
    slot: Optional[int] = None,
) -> str:
    """One RAKHUB RS485 line from WisToolBox IO_ADDPOLL fields + serial params."""
    slot_part = f"slot={slot} " if slot is not None else ""
    return (
        "RAKHUB RS485 "
        f"baud={baud} databit={databit} stop={stop} parity={parity} "
        f"{slot_part}"
        f"task={fields['task']} hex={fields['hex']} "
        f"period={fields['period']} timeout={fields['timeout']} retry={fields['retry']} "
        f"scale={fields['scale']} ipso={fields['ipso']} dtype={fields['dtype']} name={fields['name']}"
    )


def build_rs485(
    args: argparse.Namespace,
    slot: Optional[int] = None,
    overrides: Optional[Dict[str, str]] = None,
) -> str:
    overrides = overrides or {}
    fields = {
        "task": value(args, overrides, "task"),
        "hex": value(args, overrides, "hex"),
        "period": value(args, overrides, "period"),
        "timeout": value(args, overrides, "timeout"),
        "retry": value(args, overrides, "retry"),
        "scale": value(args, overrides, "scale"),
        "ipso": value(args, overrides, "ipso"),
        "dtype": value(args, overrides, "dtype"),
        "name": value(args, overrides, "name"),
    }
    return build_rs485_line(
        int(value(args, overrides, "baud")),
        int(value(args, overrides, "databit")),
        int(value(args, overrides, "stop")),
        int(value(args, overrides, "parity")),
        fields,
        slot=slot,
    )


def build_aic(
    ch: int,
    ipso: int,
    min_v: int,
    max_v: int,
    off: str,
    name: str,
) -> str:
    return f"RAKHUB AIC ch={ch} ipso={ipso} min={min_v} max={max_v} off={off} name={name}"


def send_line(port: serial.Serial, line: str) -> None:
    print(f"> {line}")
    port.write((line + "\n").encode("ascii"))
    port.flush()


def listen_serial(port: serial.Serial, duration_s: float, prefix: str = "< ") -> None:
    """Decode LF-terminated lines from the USB CDC port for debug (stdout)."""
    if duration_s <= 0:
        return
    deadline = time.monotonic() + duration_s
    buf = bytearray()
    prev_timeout = port.timeout
    port.timeout = 0.05
    try:
        while time.monotonic() < deadline:
            chunk = port.read(4096)
            if chunk:
                buf.extend(chunk)
                while True:
                    nl = buf.find(b"\n")
                    if nl < 0:
                        break
                    raw = buf[:nl]
                    del buf[: nl + 1]
                    text = raw.decode("utf-8", errors="replace").rstrip("\r")
                    print(f"{prefix}{text}")
            else:
                time.sleep(0.02)
    finally:
        port.timeout = prev_timeout


def listen_serial_lines(
    port: serial.Serial,
    max_lines: int,
    prefix: str = "< ",
    idle_timeout_s: float = 1.5,
    hard_cap_s: float = 45.0,
) -> None:
    """Print up to ``max_lines`` LF-terminated RX lines (for quick post-config verification)."""
    if max_lines <= 0:
        return
    buf = bytearray()
    prev_timeout = port.timeout
    port.timeout = 0.08
    lines_done = 0
    t0 = time.monotonic()
    last_data = t0
    try:
        while lines_done < max_lines and (time.monotonic() - t0) < hard_cap_s:
            chunk = port.read(4096)
            now = time.monotonic()
            if chunk:
                last_data = now
                buf.extend(chunk)
                while lines_done < max_lines:
                    nl = buf.find(b"\n")
                    if nl < 0:
                        break
                    raw = buf[:nl]
                    del buf[: nl + 1]
                    text = raw.decode("utf-8", errors="replace").rstrip("\r")
                    print(f"{prefix}{text}")
                    lines_done += 1
            else:
                if lines_done > 0 and (now - last_data) >= idle_timeout_s:
                    break
                time.sleep(0.02)
    finally:
        port.timeout = prev_timeout


def cmd_monitor(args: argparse.Namespace, port: serial.Serial) -> None:
    """Print serial RX until Ctrl+C or optional duration."""
    print("# RAKHUB monitor: RX → stdout (Ctrl+C to stop)", file=sys.stderr)
    buf = bytearray()
    port.timeout = 0.1
    deadline = None if args.duration is None else time.monotonic() + float(args.duration)
    try:
        while deadline is None or time.monotonic() < deadline:
            chunk = port.read(4096)
            if chunk:
                buf.extend(chunk)
                while True:
                    nl = buf.find(b"\n")
                    if nl < 0:
                        break
                    raw = buf[:nl]
                    del buf[: nl + 1]
                    text = raw.decode("utf-8", errors="replace").rstrip("\r")
                    print(text)
            else:
                time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n# monitor stopped", file=sys.stderr)


def cmd_reboot(args: argparse.Namespace, port: serial.Serial) -> None:
    """Tell firmware to arm OneWire CONTROL reboot; wire line is always ``RAKHUB REBOOT`` (no PID suffix)."""
    send_line(port, "RAKHUB REBOOT")
    listen_serial(port, float(args.listen))


def parse_subst_entries(pairs: List[str]) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for p in pairs:
        key, sep, val = p.partition("=")
        if not sep:
            raise ValueError(f"--subst must be KEY=VALUE, got: {p}")
        out[key.strip().upper()] = val.strip()
    return out


def merge_placeholders(defaults: Dict[str, str], cli: Dict[str, str]) -> Dict[str, str]:
    m = dict(defaults)
    m.update(cli)
    return m


def substitute_line(line: str, ph: Dict[str, str]) -> str:
    s = line
    for k, v in ph.items():
        s = s.replace("{" + k + "}", v)
    return s


def load_wistool_template(path: str, index: Optional[int] = None) -> Dict[str, Any]:
    """Load one WisToolBox template from JSON.

    - Root **object**: returned as-is; if ``index`` is set and not ``0``, raises.
    - Root **array**: pick ``index`` (default ``0``); must be a non-empty array of objects.
    """
    with open(path, encoding="utf-8") as f:
        doc: JsonDoc = json.load(f)
    if isinstance(doc, dict):
        if index is not None and index != 0:
            raise ValueError("JSON root is a single object; omit --index or use --index 0")
        return doc
    if isinstance(doc, list):
        if not doc:
            raise ValueError("JSON array is empty")
        idx = 0 if index is None else index
        if idx < 0 or idx >= len(doc):
            raise ValueError(f"--index {idx} out of range (len={len(doc)})")
        item = doc[idx]
        if not isinstance(item, dict):
            raise ValueError(f"array[{idx}] must be an object")
        return item
    raise ValueError("JSON root must be an object or an array of objects")


# --- WisToolBox → RAKHUB (POC) ---

_RE_IO_CFG = re.compile(
    r"^ATC\+IO_CFG=([^:]+):RS485:(\d+):(\d+):(\d+):(\d+)\s*$",
    re.IGNORECASE,
)
# IO_ADDPOLL=prb:RS485:task:HEX:period:timeout:retry:dtype:scale:ipso:name
_RE_IO_ADDPOLL = re.compile(
    r"^ATC\+IO_ADDPOLL=([^:]+):RS485:([^:]+):([^:]+):(\d+):(\d+):(\d+):(\d+):([^:]+):(\d+):(.+?)\s*$",
    re.IGNORECASE,
)
# io_decode=prb:ai:ch:ipso:max:min:name:off  (WisToolBox / core examples)
_RE_IO_DECODE_AI = re.compile(
    r"^ATC\+IO_DECODE=([^:]+):AI:(\d+):(\d+):(-?\d+):(-?\d+):([^:]*):(-?[\d.]+)\s*$",
    re.IGNORECASE,
)
# io_decode=prb:di:ch:ipso:trigger_mode:debounce_ms:name  (DI_01 JSON template)
_RE_IO_DECODE_DI = re.compile(
    r"^ATC\+IO_DECODE=([^:]+):DI:(\d+):(\d+):(\d+):(\d+):([^:]*)\s*$",
    re.IGNORECASE,
)


def parse_io_cfg_line(line: str) -> Optional[Tuple[int, int, int, int]]:
    m = _RE_IO_CFG.match(line.strip())
    if not m:
        return None
    _prb, baud, db, stop, parity = m.groups()
    return int(baud), int(db), int(stop), int(parity)


def parse_io_addpoll_line(line: str) -> Optional[Dict[str, str]]:
    # Only strip outer whitespace; hex payload must keep internal spacing if present.
    m = _RE_IO_ADDPOLL.match(line.strip())
    if not m:
        return None
    _prb, task, hexbytes, period, timeout, retry, dtype, scale, ipso, name = m.groups()
    return {
        "task": task,
        "hex": hexbytes.lower(),
        "period": period,
        "timeout": timeout,
        "retry": retry,
        "dtype": dtype,
        "scale": scale,
        "ipso": ipso,
        "name": name,
    }


def parse_io_decode_ai_line(line: str) -> Optional[Dict[str, Any]]:
    m = _RE_IO_DECODE_AI.match(line.strip())
    if not m:
        return None
    _prb, ch, ipso, mx, mn, name, off = m.groups()
    return {
        "ch": int(ch),
        "ipso": int(ipso),
        "min": int(mn),
        "max": int(mx),
        "off": str(off).strip(),
        "name": (name.strip() or "GE"),
    }


def parse_io_decode_di_line(line: str) -> Optional[Dict[str, Any]]:
    """Parse atc+io_decode=PRB:di:ch:ipso:trigger_mode:debounce_ms:name (DI_01 template)."""
    m = _RE_IO_DECODE_DI.match(line.strip())
    if not m:
        return None
    _prb, ch, ipso, trig, debounce, name = m.groups()
    return {
        "ch": int(ch),
        "ipso": int(ipso),
        "trigger_mode": int(trig),
        "debounce_ms": int(debounce),
        "name": (name.strip() or "GE"),
    }


def build_di(ch: int, ipso: int, trigger_mode: int, debounce_ms: int, name: str) -> str:
    return f"RAKHUB DI ch={ch} ipso={ipso} trigger={trigger_mode} debounce={debounce_ms} name={name}"


def wistool_template_to_rakhub(
    template: Dict[str, Any],
    ph: Dict[str, str],
) -> Tuple[List[str], List[str]]:
    """Return (rakhub_lines, skip_notes)."""
    sensor = template.get("SensorName", "?")
    iface = str(template.get("Interface", "")).strip().upper()
    content = template.get("Content", [])
    if not isinstance(content, list):
        raise ValueError("Content must be a JSON array of strings")

    rakhub: List[str] = []
    notes: List[str] = []

    substituted = [substitute_line(str(x), ph) for x in content]

    if iface == "RS485":
        baud, databit, stop, parity = 9600, 8, 1, 0
        tasks: List[Dict[str, str]] = []
        for raw in substituted:
            line = raw.strip()
            u = line.upper()
            if u.startswith("ATC+IO_CFG="):
                cfg = parse_io_cfg_line(line)
                if cfg:
                    baud, databit, stop, parity = cfg[0], cfg[1], cfg[2], cfg[3]
                else:
                    notes.append(f"IO_CFG not parsed (expected ATC+IO_CFG=…:RS485:baud:8:1:0): {line[:80]}")
            elif "IO_ADDPOLL" in u and "RS485" in u:
                parsed = parse_io_addpoll_line(line)
                if parsed:
                    tasks.append(parsed)
                else:
                    notes.append(f"IO_ADDPOLL not parsed: {line[:100]}")
        if not tasks:
            raise ValueError(
                f"Interface RS485 but no parsable ATC+IO_ADDPOLL=...:RS485:... for {sensor!r}"
            )
        for i, fields in enumerate(tasks):
            slot = i if len(tasks) > 1 else None
            rakhub.append(build_rs485_line(baud, databit, stop, parity, fields, slot=slot))
        return rakhub, notes

    if iface == "AI":
        decoded = False
        for raw in substituted:
            line = raw.strip()
            ul = line.upper()
            if "IO_DECODE" in ul and ":AI:" in ul:
                d = parse_io_decode_ai_line(line)
                if d:
                    rakhub.append(build_aic(d["ch"], d["ipso"], d["min"], d["max"], d["off"], d["name"]))
                    decoded = True
                else:
                    notes.append(f"io_decode (ai) not parsed: {line[:100]}")
            elif "IO_DECODE" in ul and ":DI:" in ul:
                d = parse_io_decode_di_line(line)
                if d:
                    rakhub.append(build_di(d["ch"], d["ipso"], d["trigger_mode"], d["debounce_ms"], d["name"]))
                    decoded = True
                else:
                    notes.append(f"io_decode (di) not parsed: {line[:100]}")
        if not decoded:
            raise ValueError(
                f"Interface AI but no parsable atc+io_decode=...:ai/di:... line for {sensor!r}"
            )
        return rakhub, notes

    raise ValueError(f"Unsupported Interface for POC: {iface!r} (sensor={sensor!r})")


def cmd_json(args: argparse.Namespace, port: serial.Serial) -> None:
    subst = merge_placeholders(DEFAULT_PLACEHOLDERS, parse_subst_entries(args.subst))
    tmpl = load_wistool_template(args.file, args.index)

    print(f"# SensorName={tmpl.get('SensorName')} Interface={tmpl.get('Interface')} ", file=sys.stderr)

    lines, notes = wistool_template_to_rakhub(tmpl, subst)
    for n in notes:
        print(f"# skip: {n}", file=sys.stderr)

    for line in lines:
        send_line(port, line)
        time.sleep(0.05)

    if args.apply:
        time.sleep(0.2)
        send_line(port, "RAKHUB APPLY")
        if args.post_apply_lines > 0:
            print(f"# tail: up to {args.post_apply_lines} serial lines after APPLY", file=sys.stderr)
            listen_serial_lines(port, args.post_apply_lines)


def cmd_clear(args: argparse.Namespace, port: serial.Serial) -> None:
    """Clear all ProbeIO IOC defaults/tasks via downlink (builtin clear-only template)."""
    send_line(port, "RAKHUB BUILTIN 0")
    time.sleep(0.05)
    send_line(port, "RAKHUB APPLY")


def cmd_clearaic(args: argparse.Namespace, port: serial.Serial) -> None:
    """Clear AIC decode defaults on ProbeIO (no APPLY required)."""
    cmd = "RAKHUB CLEARAIC" + (f" {args.pid}" if args.pid else "")
    send_line(port, cmd)


def main() -> int:
    parser = argparse.ArgumentParser(description="Send RAKHUB USB text POC commands")
    parser.add_argument("--port", required=True, help="USB CDC serial port, e.g. COM7 or /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200, help="USB CDC baudrate")
    parser.add_argument(
        "--listen-after",
        type=float,
        default=0.0,
        metavar="SEC",
        help="After subcommand(s), print incoming serial lines for SEC seconds (debug; skipped for monitor)",
    )
    parser.add_argument("--apply", action="store_true", help="Send RAKHUB APPLY after profile command")
    parser.add_argument(
        "--pid",
        default="",
        metavar="HEX",
        help="Optional hex probe PID for CLEARAIC only (may appear before the subcommand). APPLY is always sent without PID.",
    )

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

    di = sub.add_parser("di", help="Send RAKHUB DI (digital input) profile")
    di.add_argument("--ch", type=int, default=1, help="DI channel (1-based)")
    di.add_argument("--ipso", type=int, default=0, help="IPSO object id (default 0 = RAK_IPSO_DIGITAL_INPUT)")
    di.add_argument("--trigger", type=int, default=2, help="Trigger mode: 0=rising 1=falling 2=both (default 2)")
    di.add_argument("--debounce", type=int, default=100, help="Debounce time in ms (default 100)")
    di.add_argument("--name", default="GE", help="Profile name (max 16 chars, default GE)")
    di.add_argument("--psm", type=int, default=1, help="Enable IO_PSM before decode (1=yes, default 1)")
    di.add_argument("--if-psm", type=int, default=9, dest="if_psm")
    di.add_argument("--psw", type=int, default=1)
    di.add_argument("--warmup", type=int, default=10000)
    di.add_argument("--pmode", type=int, default=0)
    di.add_argument("--method", type=int, default=1)

    status = sub.add_parser("status", help="Send RAKHUB STATUS")
    status.set_defaults()

    builtin = sub.add_parser("builtin", help="Select built-in template id, then optionally apply")
    builtin.add_argument("id", type=int, choices=range(0, 10),
                         help="0=clear-only 1=JXBS3001-EC 2=SDSIN 3=JXBS4001-PH 4=AIC 5=SDI12 6=DI 7=RS232 8=DO 9=AIV")

    jsn = sub.add_parser(
        "json",
        help="Load one WisToolBox-style JSON template and emit RAKHUB RS485 or AI (POC)",
    )
    jsn.add_argument("--file", required=True, help="Path to JSON file (single object, or array + --index)")
    jsn.add_argument(
        "--index",
        type=int,
        default=None,
        help="If root is an array, pick this template index (default: use first object / first array element)",
    )
    jsn.add_argument(
        "--subst",
        action="append",
        default=[],
        metavar="KEY=VAL",
        help="Placeholder overrides e.g. PROFILE_NAME=GE DEV_ADDR=01 (keys matched upper-case as {PROFILE_NAME})",
    )
    jsn.add_argument(
        "--post-apply-lines",
        type=int,
        default=20,
        metavar="N",
        help="With --apply: print up to N serial log lines after RAKHUB APPLY (0=skip)",
    )

    clr = sub.add_parser(
        "clear",
        help="Clear all ProbeIO IOC tasks/defaults (sends RAKHUB BUILTIN 0 + APPLY)",
    )

    clearaic = sub.add_parser(
        "clearaic",
        help="Clear ProbeIO AIC decode defaults (sends RAKHUB CLEARAIC [pid]; no APPLY)",
    )

    reboot = sub.add_parser(
        "reboot",
        help="ProbeIO software reset via firmware (sends RAKHUB REBOOT only); requires RAK_SENSORHUB_USB_PROFILE build",
    )
    reboot.add_argument(
        "--listen",
        type=float,
        default=3.0,
        metavar="SEC",
        help="Seconds to print serial RX after command (0 = do not listen)",
    )

    mon = sub.add_parser("monitor", help="Debug: print USB CDC RX lines until Ctrl+C or --duration")
    mon.add_argument(
        "--duration",
        type=float,
        default=None,
        metavar="SEC",
        help="Stop after SEC seconds (default: run until Ctrl+C)",
    )

    args = parser.parse_args()

    with serial.Serial(args.port, args.baudrate, timeout=0.2) as port:
        if args.cmd == "monitor":
            cmd_monitor(args, port)
        elif args.cmd == "json":
            cmd_json(args, port)
        elif args.cmd == "clear":
            cmd_clear(args, port)
        elif args.cmd == "clearaic":
            cmd_clearaic(args, port)
        elif args.cmd == "reboot":
            cmd_reboot(args, port)
        elif args.cmd == "rs485":
            if args.task_spec:
                for slot, spec in enumerate(args.task_spec):
                    if slot >= 4:
                        raise ValueError("at most 4 RS485 task-spec entries are supported")
                    send_line(port, build_rs485(args, slot=slot, overrides=parse_task_spec(spec)))
                    time.sleep(0.05)
            else:
                send_line(port, build_rs485(args))
        elif args.cmd == "aic":
            send_line(port, build_aic(args.ch, args.ipso, args.min, args.max, args.off, args.name))
        elif args.cmd == "di":
            line = (
                f"RAKHUB DI ch={args.ch} ipso={args.ipso} trigger={args.trigger}"
                f" debounce={args.debounce} name={args.name}"
                f" psm={args.psm} if_psm={args.if_psm} psw={args.psw}"
                f" warmup={args.warmup} pmode={args.pmode} method={args.method}"
            )
            send_line(port, line)
        elif args.cmd == "status":
            send_line(port, "RAKHUB STATUS")
        elif args.cmd == "builtin":
            send_line(port, f"RAKHUB BUILTIN {args.id}")

        if args.apply and args.cmd in {"rs485", "aic", "builtin"}:
            time.sleep(0.2)
            send_line(port, "RAKHUB APPLY")

        if args.cmd != "monitor" and args.listen_after > 0:
            listen_serial(port, args.listen_after)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
