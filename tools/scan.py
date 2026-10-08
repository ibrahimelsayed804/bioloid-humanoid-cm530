#!/usr/bin/env python3
"""Find every servo on the bus and check it against the joint map.

    python tools/scan.py --port /dev/ttyUSB0
    python tools/scan.py --sim
"""

from _common import connect, parser


def main() -> int:
    p = parser(__doc__)
    p.add_argument("--max-id", type=int, default=30, help="highest id to ping (default 30)")
    args = p.parse_args()
    robot, _ = connect(args)

    found = robot.bus.scan(range(0, args.max_id + 1))
    expected = robot.by_id
    print(f"Found {len(found)} servo(s):")
    for sid, model in sorted(found.items()):
        name = expected[sid].name if sid in expected else "-- not in joint map --"
        print(f"  ID {sid:>2}  {model:<8} {name}")

    missing = sorted(set(expected) - set(found))
    if missing:
        print("\nMissing from the bus (check cables and ID settings):")
        for sid in missing:
            print(f"  ID {sid:>2}  {expected[sid].name}")
        return 1
    print("\nAll joints in the map are responding.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
