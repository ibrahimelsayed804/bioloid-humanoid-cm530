#!/usr/bin/env python3
"""Live joint table: angle, voltage, temperature, load and warnings.

Useful for checking joint directions: with torque off, move a limb by hand
and watch which way its angle changes.

    python tools/monitor.py --port /dev/ttyUSB0
    python tools/monitor.py --sim --once
"""

import time

from _common import connect, parser


def render(report) -> str:
    lines = [f"{'ID':>3} {'joint':<17} {'angle':>7} {'volt':>5} {'temp':>5} {'load':>6}  warnings",
             "-" * 62]
    for h in report:
        lines.append(f"{h.id:>3} {h.name:<17} {h.angle_deg:>6.1f}° {h.voltage_v:>4.1f}V "
                     f"{h.temperature_c:>4}C {h.load:>+6.2f}  {', '.join(h.warnings)}")
    return "\n".join(lines)


def main() -> int:
    p = parser(__doc__)
    p.add_argument("--once", action="store_true", help="print one table and exit")
    p.add_argument("--interval", type=float, default=0.5, help="refresh period in seconds")
    args = p.parse_args()
    robot, _ = connect(args)

    try:
        while True:
            table = render(robot.health())
            if args.once:
                print(table)
                return 0
            print("\033[2J\033[H" + table + "\n\nCtrl+C to quit", flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
