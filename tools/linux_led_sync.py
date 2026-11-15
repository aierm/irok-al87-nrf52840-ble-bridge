#!/usr/bin/env python3
"""Sync Linux lock LED state to a HID keyboard via hidraw.

This helper is meant for BLE keyboard bridges where the Linux host updates its
own logical Num/Caps/Scroll state, but that state is not forwarded back to the
keyboard over the standard HID output report path.
"""

from __future__ import annotations

import argparse
import fcntl
import os
import re
import sys
import time
from pathlib import Path


NUM_LOCK_MASK = 0x01
CAPS_LOCK_MASK = 0x02
SCROLL_LOCK_MASK = 0x04
VALID_LED_MASK = NUM_LOCK_MASK | CAPS_LOCK_MASK | SCROLL_LOCK_MASK

LED_NAME_TO_MASK = {
    "num": NUM_LOCK_MASK,
    "caps": CAPS_LOCK_MASK,
    "scroll": SCROLL_LOCK_MASK,
}

SYSFS_HIDRAW_ROOT = Path("/sys/class/hidraw")
SYSFS_LED_ROOT = Path("/sys/class/leds")

IOC_NRBITS = 8
IOC_TYPEBITS = 8
IOC_SIZEBITS = 14
IOC_DIRBITS = 2
IOC_NRSHIFT = 0
IOC_TYPESHIFT = IOC_NRSHIFT + IOC_NRBITS
IOC_SIZESHIFT = IOC_TYPESHIFT + IOC_TYPEBITS
IOC_DIRSHIFT = IOC_SIZESHIFT + IOC_SIZEBITS
IOC_WRITE = 1
IOC_READ = 2


def ioc(direction: int, ioc_type: str, nr: int, size: int) -> int:
    return (
        (direction << IOC_DIRSHIFT)
        | (ord(ioc_type) << IOC_TYPESHIFT)
        | (nr << IOC_NRSHIFT)
        | (size << IOC_SIZESHIFT)
    )


def hidiocsoutput(length: int) -> int:
    return ioc(IOC_WRITE | IOC_READ, "H", 0x0B, length)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Write keyboard output reports to a hidraw device and optionally "
            "follow Linux lock LED state from sysfs."
        ),
        epilog=(
            "Examples:\n"
            "  %(prog)s --list\n"
            "  sudo %(prog)s --hidraw /dev/hidraw2 --bits caps\n"
            "  sudo %(prog)s --hidraw /dev/hidraw2 --watch\n"
            "  sudo %(prog)s --hidraw /dev/hidraw2 --watch --input input18\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List hidraw devices and their related input/LED sysfs nodes, then exit.",
    )
    parser.add_argument(
        "--hidraw",
        help="Target hidraw device, for example /dev/hidraw2.",
    )
    parser.add_argument(
        "--input",
        dest="input_name",
        help="Explicit sysfs input node to read LED state from, for example input18.",
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Continuously poll Linux LED state and mirror it to the hidraw device.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Read the current Linux LED state once, write it, then exit.",
    )
    parser.add_argument(
        "--bits",
        help=(
            "Manual LED bits to write once. Accepts names like caps,num or a "
            "raw value such as 0x02."
        ),
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.05,
        help="Polling interval in seconds for --watch (default: 0.05).",
    )
    parser.add_argument(
        "--report-id",
        type=lambda x: int(x, 0),
        default=1,
        help=(
            "Prefix a non-zero report ID before the LED byte. "
            "Default is 1, which matches this bridge sample."
        ),
    )
    parser.add_argument(
        "--prefix-zero-report-id",
        action="store_true",
        dest="prefix_zero_report_id",
        help=(
            "Prefix a leading 0x00 report number before the LED byte. "
            "This is the default on Linux hidraw."
        ),
    )
    parser.add_argument(
        "--no-prefix-zero-report-id",
        action="store_false",
        dest="prefix_zero_report_id",
        help=(
            "Write only the LED byte with no leading report number. "
            "Only use this if your stack rejects the standard hidraw format."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would be written without touching hidraw.",
    )
    parser.add_argument(
        "--transport",
        choices=("write", "ioctl-output", "both"),
        default="write",
        help=(
            "How to send the output report to hidraw. "
            "Use ioctl-output or both if plain write() does not reach the BLE bridge."
        ),
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print extra progress information.",
    )
    parser.set_defaults(prefix_zero_report_id=True)
    args = parser.parse_args()

    if args.list:
        return args

    if not args.hidraw:
        parser.error("--hidraw is required unless --list is used.")

    manual_mode = args.bits is not None
    auto_mode = args.watch or args.once or not manual_mode
    if manual_mode and (args.watch or args.once):
        parser.error("--bits is a manual write mode; do not combine it with --watch or --once.")

    if not manual_mode and not auto_mode:
        parser.error("Choose --bits, --once, or --watch.")

    return args


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace").strip()


def parse_uevent(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    if not path.exists():
        return data

    for line in read_text(path).splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key] = value
    return data


def hidraw_sysfs_dir(devnode: str) -> Path:
    hidraw_name = Path(devnode).name
    if hidraw_name.lower() == "hidrawx":
        raise FileNotFoundError(
            "You passed the placeholder /dev/hidrawX. Replace X with the real number from --list, "
            "for example /dev/hidraw5."
        )
    path = SYSFS_HIDRAW_ROOT / hidraw_name
    if not path.exists():
        raise FileNotFoundError(f"hidraw sysfs entry not found for {devnode}")
    return path


def find_input_dirs_for_hidraw(devnode: str) -> list[Path]:
    device_dir = hidraw_sysfs_dir(devnode) / "device"
    if not device_dir.exists():
        return []

    candidates: list[Path] = []
    seen: set[Path] = set()
    for path in device_dir.rglob("input*"):
        if not path.is_dir() or not re.fullmatch(r"input\d+", path.name):
            continue
        resolved = path.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        candidates.append(resolved)
    candidates.sort(key=lambda item: item.name)
    return candidates


def led_brightness_path(input_name: str, led_name: str) -> Path:
    return SYSFS_LED_ROOT / f"{input_name}::{led_name}" / "brightness"


def input_led_support(input_name: str) -> dict[str, bool]:
    return {
        "numlock": led_brightness_path(input_name, "numlock").exists(),
        "capslock": led_brightness_path(input_name, "capslock").exists(),
        "scrolllock": led_brightness_path(input_name, "scrolllock").exists(),
    }


def read_led_bits_from_input(input_name: str) -> int:
    bits = 0
    if led_brightness_path(input_name, "numlock").exists() and read_text(
        led_brightness_path(input_name, "numlock")
    ) == "1":
        bits |= NUM_LOCK_MASK
    if led_brightness_path(input_name, "capslock").exists() and read_text(
        led_brightness_path(input_name, "capslock")
    ) == "1":
        bits |= CAPS_LOCK_MASK
    if led_brightness_path(input_name, "scrolllock").exists() and read_text(
        led_brightness_path(input_name, "scrolllock")
    ) == "1":
        bits |= SCROLL_LOCK_MASK
    return bits


def select_input_name(devnode: str, explicit_input_name: str | None) -> str:
    if explicit_input_name:
        support = input_led_support(explicit_input_name)
        if not any(support.values()):
            raise RuntimeError(
                f"{explicit_input_name} has no lock LED sysfs nodes under {SYSFS_LED_ROOT}"
            )
        return explicit_input_name

    input_dirs = find_input_dirs_for_hidraw(devnode)
    if not input_dirs:
        raise RuntimeError(
            f"No input sysfs node found for {devnode}. Try --list and then pass --input inputX."
        )

    best_match: str | None = None
    best_score = -1
    for input_dir in input_dirs:
        support = input_led_support(input_dir.name)
        score = sum(1 for enabled in support.values() if enabled)
        if score > best_score:
            best_score = score
            best_match = input_dir.name

    if not best_match or best_score <= 0:
        joined = ", ".join(path.name for path in input_dirs)
        raise RuntimeError(
            f"Found input nodes for {devnode} ({joined}) but none expose lock LED sysfs state."
        )

    return best_match


def format_led_bits(bits: int) -> str:
    names = [name for name, mask in LED_NAME_TO_MASK.items() if bits & mask]
    if not names:
        return "none"
    return ",".join(names)


def parse_manual_bits(text: str) -> int:
    value = text.strip().lower()
    if not value:
        raise ValueError("empty LED bit string")

    if value == "none":
        return 0

    if value.startswith("0x") or value.isdigit():
        return int(value, 0) & VALID_LED_MASK

    bits = 0
    for part in value.split(","):
        name = part.strip().lower()
        if not name:
            continue
        if name not in LED_NAME_TO_MASK:
            raise ValueError(f"unknown LED name: {name}")
        bits |= LED_NAME_TO_MASK[name]
    return bits & VALID_LED_MASK


class HidrawWriter:
    def __init__(
        self,
        devnode: str,
        report_id: int,
        prefix_zero_report_id: bool,
        dry_run: bool,
        transport: str,
        verbose: bool,
    ) -> None:
        self.devnode = devnode
        self.report_id = report_id & 0xFF
        self.prefix_zero_report_id = prefix_zero_report_id
        self.dry_run = dry_run
        self.transport = transport
        self.verbose = verbose
        self.fd: int | None = None

        if not dry_run:
            self.fd = os.open(devnode, os.O_RDWR | os.O_CLOEXEC)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def payload_for_bits(self, led_bits: int) -> bytes:
        led_bits &= VALID_LED_MASK
        if self.report_id != 0:
            return bytes([self.report_id, led_bits])
        if self.prefix_zero_report_id:
            return bytes([0x00, led_bits])
        return bytes([led_bits])

    def write_bits(self, led_bits: int) -> None:
        payload = self.payload_for_bits(led_bits)
        if self.verbose or self.dry_run:
            print(
                f"write {self.devnode}: bits=0x{led_bits:02X} ({format_led_bits(led_bits)}) "
                f"payload={' '.join(f'{b:02X}' for b in payload)} transport={self.transport}"
            )

        if self.dry_run:
            return

        assert self.fd is not None
        if self.transport in ("write", "both"):
            written = os.write(self.fd, payload)
            if written != len(payload):
                raise OSError(f"short write to {self.devnode}: {written}/{len(payload)} bytes")

        if self.transport in ("ioctl-output", "both"):
            fcntl.ioctl(self.fd, hidiocsoutput(len(payload)), payload)


def list_hidraw_devices() -> int:
    entries = sorted(SYSFS_HIDRAW_ROOT.glob("hidraw*"), key=lambda item: item.name)
    if not entries:
        print(f"No hidraw entries found under {SYSFS_HIDRAW_ROOT}", file=sys.stderr)
        return 1

    for entry in entries:
        devnode = f"/dev/{entry.name}"
        uevent = parse_uevent(entry / "device" / "uevent")
        hid_name = uevent.get("HID_NAME", "(unknown)")
        inputs = find_input_dirs_for_hidraw(devnode)
        input_names = ", ".join(path.name for path in inputs) if inputs else "-"
        led_summary_parts = []
        for input_dir in inputs:
            support = input_led_support(input_dir.name)
            enabled_names = [name for name, enabled in support.items() if enabled]
            if enabled_names:
                led_summary_parts.append(f"{input_dir.name}[{','.join(enabled_names)}]")
        led_summary = ", ".join(led_summary_parts) if led_summary_parts else "-"
        print(f"{devnode}: {hid_name}")
        print(f"  inputs: {input_names}")
        print(f"  leds:   {led_summary}")
    return 0


def run_manual_write(args: argparse.Namespace) -> int:
    try:
        led_bits = parse_manual_bits(args.bits)
    except ValueError as exc:
        print(f"Invalid --bits value: {exc}", file=sys.stderr)
        return 2

    writer = HidrawWriter(
        devnode=args.hidraw,
        report_id=args.report_id,
        prefix_zero_report_id=args.prefix_zero_report_id,
        dry_run=args.dry_run,
        transport=args.transport,
        verbose=True,
    )
    try:
        writer.write_bits(led_bits)
    finally:
        writer.close()
    return 0


def run_auto_sync(args: argparse.Namespace) -> int:
    input_name = select_input_name(args.hidraw, args.input_name)
    target_inputs = {path.name for path in find_input_dirs_for_hidraw(args.hidraw)}
    writer = HidrawWriter(
        devnode=args.hidraw,
        report_id=args.report_id,
        prefix_zero_report_id=args.prefix_zero_report_id,
        dry_run=args.dry_run,
        transport=args.transport,
        verbose=args.verbose,
    )

    if args.verbose:
        print(f"Using Linux LED source: {input_name}")
    if input_name in target_inputs:
        print(
            f"Warning: {input_name} belongs to the same HID device as {args.hidraw}. "
            "That can create a feedback loop. Prefer another source input node if available.",
            file=sys.stderr,
        )

    try:
        last_seen_bits: int | None = None
        while True:
            current_bits = read_led_bits_from_input(input_name)
            if current_bits != last_seen_bits:
                writer.write_bits(current_bits)
                last_seen_bits = current_bits

            if args.once:
                break

            time.sleep(args.interval)
    except KeyboardInterrupt:
        return 130
    finally:
        writer.close()

    return 0


def main() -> int:
    args = parse_args()

    if args.list:
        return list_hidraw_devices()

    try:
        if args.bits is not None:
            return run_manual_write(args)
        return run_auto_sync(args)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except PermissionError:
        print(
            "Permission denied. Try running with sudo or adjust hidraw device permissions.",
            file=sys.stderr,
        )
        return 1
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"I/O error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
