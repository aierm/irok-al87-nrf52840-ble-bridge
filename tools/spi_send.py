#!/usr/bin/env python3
import argparse
import subprocess
import sys
import time

try:
    import spidev
except Exception as exc:  # pragma: no cover - runtime import
    print("Failed to import spidev. Install with: pip3 install spidev", file=sys.stderr)
    raise

# USB HID keyboard usage IDs (subset)
HID_KEYCODES = {
    "a": 0x04,
    "b": 0x05,
    "c": 0x06,
    "d": 0x07,
    "e": 0x08,
    "f": 0x09,
    "g": 0x0A,
    "h": 0x0B,
    "i": 0x0C,
    "j": 0x0D,
    "k": 0x0E,
    "l": 0x0F,
    "m": 0x10,
    "n": 0x11,
    "o": 0x12,
    "p": 0x13,
    "q": 0x14,
    "r": 0x15,
    "s": 0x16,
    "t": 0x17,
    "u": 0x18,
    "v": 0x19,
    "w": 0x1A,
    "x": 0x1B,
    "y": 0x1C,
    "z": 0x1D,
    "1": 0x1E,
    "2": 0x1F,
    "3": 0x20,
    "4": 0x21,
    "5": 0x22,
    "6": 0x23,
    "7": 0x24,
    "8": 0x25,
    "9": 0x26,
    "0": 0x27,
    "enter": 0x28,
    "esc": 0x29,
    "backspace": 0x2A,
    "tab": 0x2B,
    "space": 0x2C,
}

MODIFIERS = {
    "lctrl": 0x01,
    "lshift": 0x02,
    "lalt": 0x04,
    "lmeta": 0x08,
    "rctrl": 0x10,
    "rshift": 0x20,
    "ralt": 0x40,
    "rmeta": 0x80,
}

CTRL_COMMANDS = {0x01, 0x02, 0x03, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8}
CTRL_HEADER = (0xC3, 0x3C)
CTRL_TAIL = 0x69
ACK_HEADER = (0xA5, 0x5A)
ACK_MAGIC_MASK = 0xE0
ACK_MAGIC_VALUE = 0xA0
FRAME_LEN = 48
FRAME_HEADER = (0xB4, 0x4B)
FRAME_VERSION = 1
FRAME_TYPE_BOOT_KEYBOARD = 0x01
FRAME_PAYLOAD_INDEX = 8
FRAME_CRC_INDEX = 46
FRAME_TAIL_INDEX = 47
FRAME_TAIL = 0x96


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send versioned 48-byte keyboard bridge frames over SPI (master)"
    )
    parser.add_argument("--bus", type=int, default=3, help="SPI bus number")
    parser.add_argument("--cs", type=int, default=0, help="SPI chip select (device)")
    parser.add_argument("--speed", type=int, default=1_000_000, help="SPI speed (Hz)")
    parser.add_argument("--delay", type=float, default=0.02, help="Delay between press/release (s)")
    parser.add_argument("--repeat", type=int, default=3, help="Repeat count")
    parser.add_argument(
        "--key",
        default="a",
        help="Key name (a-z, 0-9, enter, esc, backspace, tab, space)",
    )
    parser.add_argument(
        "--mods",
        default="lshift",
        help="Comma-separated modifiers: lctrl,lshift,lalt,lmeta,rctrl,rshift,ralt,rmeta",
    )
    parser.add_argument(
        "--ctrl",
        type=lambda x: int(x, 0),
        default=0,
        help="Bridge control command (e.g. 0xF7 or 0x01).",
    )
    parser.add_argument(
        "--soft-cs-wpi",
        type=int,
        default=-1,
        help="Use software CS via `gpio` tool with wiringPi pin index (e.g. 13).",
    )
    return parser.parse_args()


def crc8(data: list[int]) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_report(modifier: int, keycode: int, sequence: int) -> list[int]:
    payload = [modifier & 0xFF, 0x00, keycode & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00]
    frame = [0x00] * FRAME_LEN
    frame[0:2] = FRAME_HEADER
    frame[2] = FRAME_TYPE_BOOT_KEYBOARD
    frame[3] = FRAME_VERSION
    frame[4] = sequence & 0xFF
    frame[5] = len(payload)
    frame[FRAME_PAYLOAD_INDEX : FRAME_PAYLOAD_INDEX + len(payload)] = payload
    frame[FRAME_CRC_INDEX] = crc8(frame[:FRAME_CRC_INDEX])
    frame[FRAME_TAIL_INDEX] = FRAME_TAIL
    return frame


def build_control_frame(ctrl_cmd: int, sequence: int) -> list[int]:
    control = [
        CTRL_HEADER[0],
        CTRL_HEADER[1],
        ctrl_cmd & 0xFF,
        sequence & 0xFF,
        (~ctrl_cmd) & 0xFF,
        (~sequence) & 0xFF,
    ]
    control.extend([crc8(control), CTRL_TAIL])
    frame = [0x00] * FRAME_LEN
    frame[: len(control)] = control
    return frame


def control_ack_matches(data: list[int], ctrl_cmd: int, sequence: int) -> bool:
    return (
        len(data) >= 8
        and data[0] == ACK_HEADER[0]
        and data[1] == ACK_HEADER[1]
        and data[5] & ACK_MAGIC_MASK == ACK_MAGIC_VALUE
        and data[6] == sequence
        and data[7] == ctrl_cmd
    )


def report_ack_matches(data: list[int], sequence: int) -> bool:
    return (
        len(data) >= 8
        and data[0] == ACK_HEADER[0]
        and data[1] == ACK_HEADER[1]
        and data[5] & ACK_MAGIC_MASK == ACK_MAGIC_VALUE
        and data[6] == sequence
        and data[7] == 0x00
    )


def parse_modifiers(mods: str) -> int:
    if not mods:
        return 0
    value = 0
    for part in mods.split(","):
        part = part.strip().lower()
        if not part:
            continue
        if part not in MODIFIERS:
            raise ValueError(f"Unknown modifier: {part}")
        value |= MODIFIERS[part]
    return value


def fmt_hex(data: list[int]) -> str:
    return " ".join(f"{b:02X}" for b in data)


def gpio_cmd(*args: str) -> None:
    subprocess.run(["gpio", *args], check=True)


def spi_transfer(spi: spidev.SpiDev, data: list[int], soft_cs_wpi: int) -> list[int]:
    if soft_cs_wpi < 0:
        return spi.xfer2(data)

    gpio_cmd("write", str(soft_cs_wpi), "0")
    try:
        return spi.xfer2(data)
    finally:
        gpio_cmd("write", str(soft_cs_wpi), "1")


def main() -> int:
    args = parse_args()
    key = args.key.lower().strip()
    if key not in HID_KEYCODES:
        print(f"Unknown key: {args.key}", file=sys.stderr)
        return 2

    try:
        modifier = parse_modifiers(args.mods)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    keycode = HID_KEYCODES[key]

    spi = spidev.SpiDev()
    spi.open(args.bus, args.cs)
    spi.max_speed_hz = args.speed
    spi.mode = 0
    spi.bits_per_word = 8
    spi.cshigh = False
    spi.lsbfirst = False
    sw_cs_active = args.soft_cs_wpi >= 0
    if sw_cs_active:
        try:
            spi.no_cs = True
        except OSError as exc:
            print(
                f"Warning: SPI_NO_CS unsupported ({exc}); keep hardware CS as-is, "
                "still driving software CS.",
                file=sys.stderr,
            )

    ctrl_cmd = args.ctrl & 0xFF
    if ctrl_cmd and ctrl_cmd not in CTRL_COMMANDS:
        print(f"Unsupported bridge control command: 0x{ctrl_cmd:02X}", file=sys.stderr)
        return 2

    sequence = (int(time.monotonic() * 1000) & 0xFF) or 1
    ctrl_sequence = sequence
    ctrl_frame = build_control_frame(ctrl_cmd, ctrl_sequence) if ctrl_cmd else []
    ctrl_ack_received = False

    print(f"SPI /dev/spidev{args.bus}.{args.cs} speed={args.speed}Hz mode=0")
    if sw_cs_active:
        print(f"Software CS enabled on wiringPi pin {args.soft_cs_wpi}")
        gpio_cmd("mode", str(args.soft_cs_wpi), "out")
        gpio_cmd("write", str(args.soft_cs_wpi), "1")
    if ctrl_cmd:
        print(
            f"Sending CTRL command 0x{ctrl_cmd:02X} sequence={ctrl_sequence} "
            f"frame=[{fmt_hex(ctrl_frame)}]"
        )
    else:
        print(f"Sending key '{key}' mods='{args.mods}' with matched report ACKs")

    try:
        for i in range(args.repeat):
            if ctrl_cmd:
                rx_first = spi_transfer(spi, ctrl_frame, args.soft_cs_wpi)
                time.sleep(args.delay)
                rx_ack = spi_transfer(spi, ctrl_frame, args.soft_cs_wpi)
                matched = control_ack_matches(rx_ack, ctrl_cmd, ctrl_sequence)
                print(
                    f"Sent {i + 1}/{args.repeat}  "
                    f"RX1=[{fmt_hex(rx_first)}] RX2=[{fmt_hex(rx_ack)}] "
                    f"ACK={'matched' if matched else 'pending'}"
                )
                if matched:
                    ctrl_ack_received = True
                    break
                time.sleep(args.delay)
                continue

            sequence = ((sequence + 1) & 0xFF) or 1
            press = build_report(modifier, keycode, sequence)
            rx_press_first = spi_transfer(spi, press, args.soft_cs_wpi)
            time.sleep(0.002)
            rx_press_ack = spi_transfer(spi, press, args.soft_cs_wpi)
            press_ok = report_ack_matches(rx_press_ack, sequence)
            if not press_ok:
                print("Press report ACK not received", file=sys.stderr)
                return 1

            time.sleep(args.delay)

            sequence = ((sequence + 1) & 0xFF) or 1
            release = build_report(0, 0, sequence)
            rx_release_first = spi_transfer(spi, release, args.soft_cs_wpi)
            time.sleep(0.002)
            rx_release_ack = spi_transfer(spi, release, args.soft_cs_wpi)
            release_ok = report_ack_matches(rx_release_ack, sequence)
            if not release_ok:
                print("Release report ACK not received", file=sys.stderr)
                return 1

            time.sleep(args.delay)
            print(
                f"Sent {i + 1}/{args.repeat}  "
                f"press ACK=[{fmt_hex(rx_press_ack[:8])}] "
                f"release ACK=[{fmt_hex(rx_release_ack[:8])}]"
            )
    finally:
        if sw_cs_active:
            gpio_cmd("write", str(args.soft_cs_wpi), "1")
        spi.close()

    if ctrl_cmd and not ctrl_ack_received:
        print("Control ACK not received", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
