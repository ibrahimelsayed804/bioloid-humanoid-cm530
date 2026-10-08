#!/usr/bin/env python3
"""Play a motion file on the robot (or in the simulator).

    python tools/play.py motions/wave_right.json --port /dev/ttyUSB0
    python tools/play.py motions/wave_right.json --sim --mirror

Always start from the 'ready' motion and keep a hand near the robot the
first time you run anything new.
"""

from _common import connect, parser

from bioloid import Motion, MotionPlayer


def main() -> int:
    p = parser(__doc__)
    p.add_argument("motion", help="motion file (.json)")
    p.add_argument("--mirror", action="store_true", help="swap left and right")
    p.add_argument("--slow", type=float, default=1.0,
                   help="time stretch factor, e.g. 2 plays at half speed (recommended for first runs)")
    p.add_argument("--relax", action="store_true", help="switch torque off when finished")
    p.add_argument("--yes", action="store_true", help="skip the safety prompt")
    args = p.parse_args()

    motion = Motion.load(args.motion)
    if args.mirror:
        motion = motion.mirrored()
    for kf in motion.keyframes:
        kf.duration *= args.slow
        kf.hold *= args.slow

    robot, transport = connect(args, realtime=False)
    robot.validate_pose({k: 0 for kf in motion.keyframes for k in kf.pose})

    if not args.sim and not args.yes:
        input(f"About to play '{motion.name}' ({motion.total_time:.1f}s). "
              "Robot on a flat surface, area clear? Enter to continue, Ctrl+C to abort.")

    sleep = transport.advance if args.sim else None
    player = MotionPlayer(robot, sleep=sleep) if sleep else MotionPlayer(robot)
    robot.torque_on()
    final = player.play(motion)

    if args.sim:
        actual = robot.read_pose()
        worst = max(abs(actual[n] - final[n]) for n in final)
        print(f"[sim] played '{motion.name}': {len(motion.keyframes)} keyframes, "
              f"{motion.total_time:.2f}s, worst final tracking error {worst:.1f}°")
    else:
        print(f"Played '{motion.name}'.")
    if args.relax:
        robot.torque_off()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
