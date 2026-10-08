#!/usr/bin/env python3
"""Record a motion by posing the robot by hand (teach mode).

Torque is switched off so you can move the limbs. Press Enter to capture each
keyframe, then type 'q' to finish and save. This is the same workflow as the
RoboPlus Motion editor, but it produces a plain JSON file you can edit,
version and replay with tools/play.py.

    python tools/teach.py --port /dev/ttyUSB0 --out motions/my_motion.json

Support the robot while recording: with torque off it cannot stand.
"""

from _common import connect, parser

from bioloid import Keyframe, Motion


def main() -> int:
    p = parser(__doc__)
    p.add_argument("--out", required=True, help="output motion file (.json)")
    p.add_argument("--name", help="motion name (defaults to the file name)")
    p.add_argument("--duration", type=float, default=0.6, help="seconds between keyframes")
    args = p.parse_args()
    robot, _ = connect(args)

    robot.torque_off()
    print("Torque OFF. Pose the robot, press Enter to capture, 'q' + Enter to finish.")
    motion = Motion(name=args.name or args.out.rsplit("/", 1)[-1].removesuffix(".json"))
    while True:
        cmd = input(f"[{len(motion.keyframes)} keyframes] > ").strip().lower()
        if cmd == "q":
            break
        pose = robot.read_pose()
        motion.keyframes.append(Keyframe(pose=pose, duration=args.duration))
        print("  captured: " + ", ".join(f"{k}={v:+.0f}" for k, v in pose.items()))

    if not motion.keyframes:
        print("Nothing captured; no file written.")
        return 1
    motion.save(args.out)
    print(f"Saved {len(motion.keyframes)} keyframes to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
