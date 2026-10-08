import unittest

from bioloid import DynamixelBus, Humanoid, Keyframe, Motion, MotionPlayer, SimTransport, ax12
from bioloid.motion import ease, interpolate


def make_robot(ids=range(1, 19)):
    transport = SimTransport(ids=ids)
    return Humanoid.from_config(DynamixelBus(transport)), transport


class Conversions(unittest.TestCase):
    def test_centre_and_limits(self):
        self.assertEqual(ax12.deg_to_position(0), 512)
        self.assertEqual(ax12.deg_to_position(1000), 1023)
        self.assertEqual(ax12.deg_to_position(-1000), 0)
        self.assertAlmostEqual(ax12.position_to_deg(ax12.deg_to_position(45)), 45, delta=0.3)

    def test_load_sign(self):
        self.assertAlmostEqual(ax12.decode_load(1023), 1.0)
        self.assertAlmostEqual(ax12.decode_load(1024 + 1023), -1.0)


class Bus(unittest.TestCase):
    def test_scan_finds_all_servos(self):
        robot, _ = make_robot()
        self.assertEqual(sorted(robot.bus.scan(range(0, 25))), list(range(1, 19)))

    def test_missing_servo_times_out(self):
        robot, _ = make_robot(ids=[1, 2])
        self.assertFalse(robot.bus.ping(5))
        with self.assertRaises(TimeoutError):
            robot.bus.read_word(5, ax12.PRESENT_POSITION)


class Joints(unittest.TestCase):
    def test_mirrored_joints_get_opposite_ticks(self):
        robot, _ = make_robot()
        r, l = robot.joints["r_knee"], robot.joints["l_knee"]
        self.assertEqual(r.to_ticks(30) - 512, -(l.to_ticks(30) - 512))

    def test_limits_are_enforced(self):
        robot, _ = make_robot()
        ankle = robot.joints["r_ankle_roll"]
        self.assertEqual(ankle.to_ticks(500), ankle.to_ticks(ankle.max_deg))

    def test_unknown_joint_rejected(self):
        robot, _ = make_robot()
        with self.assertRaises(KeyError):
            robot.set_pose({"tail": 10})

    def test_torque_on_does_not_jump(self):
        robot, transport = make_robot()
        transport.servos[13].move_by_hand(700)    # limb moved while relaxed
        robot.torque_on()
        transport.advance(1.0)
        self.assertEqual(robot.bus.read_word(13, ax12.PRESENT_POSITION), 700)

    def test_health_flags_hot_servo(self):
        robot, transport = make_robot()
        transport.servos[4].table[ax12.PRESENT_TEMPERATURE] = 70
        report = {h.id: h for h in robot.health()}
        self.assertIn("hot", report[4].warnings)
        self.assertEqual(report[3].warnings, [])


class Motions(unittest.TestCase):
    def test_ease_endpoints(self):
        self.assertEqual(ease(0), 0)
        self.assertEqual(ease(1), 1)
        self.assertAlmostEqual(ease(0.5), 0.5)

    def test_interpolate_midpoint(self):
        self.assertAlmostEqual(interpolate({"a": 0}, {"a": 10}, 0.5)["a"], 5)

    def test_mirror_swaps_sides(self):
        m = Motion("x", [Keyframe({"r_elbow": 40, "l_knee": 10})]).mirrored()
        self.assertEqual(m.keyframes[0].pose, {"l_elbow": 40, "r_knee": 10})

    def test_invalid_keyframe(self):
        with self.assertRaises(ValueError):
            Keyframe({"r_elbow": 0}, duration=0)

    def test_playback_reaches_target(self):
        robot, transport = make_robot()
        robot.torque_on()
        target = {"r_elbow": 45, "l_elbow": 45, "r_knee": 60, "l_knee": 60}
        MotionPlayer(robot, sleep=transport.advance).play(Motion("t", [Keyframe(target, 1.0, 0.2)]))
        pose = robot.read_pose()
        for name, deg in target.items():
            self.assertAlmostEqual(pose[name], deg, delta=0.5)

    def test_playback_is_smooth(self):
        robot, transport = make_robot()
        robot.torque_on()
        frames = []
        player = MotionPlayer(robot, rate_hz=50, sleep=transport.advance,
                              on_frame=lambda t, p: frames.append(p["r_knee"]))
        player.play(Motion("t", [Keyframe({"r_knee": 90}, duration=1.0)]))
        steps = [b - a for a, b in zip(frames, frames[1:])]
        self.assertTrue(all(s >= 0 for s in steps))          # monotonic
        self.assertLess(steps[0], steps[len(steps) // 2])    # eases in
        self.assertLess(steps[-1], steps[len(steps) // 2])   # eases out


if __name__ == "__main__":
    unittest.main()
