"""Shared command-line setup for the tools in this folder."""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from bioloid import DynamixelBus, Humanoid, SerialTransport, SimTransport  # noqa: E402


def parser(description: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description)
    hw = p.add_argument_group("connection")
    hw.add_argument("--port", help="serial port, e.g. /dev/ttyUSB0, /dev/tty.usbserial-XXXX or COM3")
    hw.add_argument("--baud", type=int, default=1_000_000,
                    help="1000000 for U2D2/USB2Dynamixel (default); 57600 is typical through the CM-530")
    hw.add_argument("--echo", action="store_true", help="adapter echoes transmitted bytes (half-duplex)")
    hw.add_argument("--sim", action="store_true", help="use the built-in simulator instead of hardware")
    p.add_argument("--config", type=Path, default=ROOT / "config" / "humanoid_type_a.json",
                   help="joint map / limits file")
    return p


def connect(args, realtime: bool = True):
    """Return (robot, transport). Exits with a helpful message if no port is given."""
    if args.sim:
        transport = SimTransport(realtime=realtime)
    else:
        if not args.port:
            sys.exit("error: give --port for hardware, or --sim to use the simulator")
        transport = SerialTransport(args.port, args.baud)
    bus = DynamixelBus(transport, echo=args.echo)
    return Humanoid.from_config(bus, args.config), transport
