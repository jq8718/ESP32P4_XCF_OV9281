"""Read the ESP32-P4 diagnostic serial port without toggling DTR/RTS."""

import argparse
import time
from pathlib import Path

import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM57")
    parser.add_argument("--seconds", type=float, default=20)
    parser.add_argument("--output", type=Path, default=Path("docs/decode_stages/board_timing.log"))
    args = parser.parse_args()
    port = serial.Serial()
    port.port = args.port
    port.baudrate = 115200
    port.timeout = 0.2
    port.dtr = False
    port.rts = False
    port.open()
    lines = []
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            line = port.readline()
            if line:
                decoded = line.decode("utf-8", errors="replace")
                lines.append(decoded)
                print(decoded, end="")
    finally:
        port.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("".join(lines), encoding="utf-8")
    print(f"Saved {len(lines)} lines to {args.output}")


if __name__ == "__main__":
    main()
