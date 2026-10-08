import unittest
from pathlib import Path

from bioloid import DynamixelBus, Humanoid, Motion, MotionPlayer, SimTransport

MOTIONS = sorted((Path(__file__).resolve().parent.parent / "motions").glob("*.json"))


class MotionFiles(unittest.TestCase):
    def test_there_are_motion_files(self):
        self.assertTrue(MOTIONS)

    def test_every_motion_plays_in_simulation(self):
        for path in MOTIONS:
            with self.subTest(motion=path.name):
                transport = SimTransport()
                robot = Humanoid.from_config(DynamixelBus(transport))
                robot.torque_on()
                motion = Motion.load(path)
                final = MotionPlayer(robot, sleep=transport.advance).play(motion)
                actual = robot.read_pose()
                for name, deg in final.items():
                    joint = robot.joints[name]
                    self.assertGreaterEqual(deg, joint.min_deg, f"{name} below limit")
                    self.assertLessEqual(deg, joint.max_deg, f"{name} above limit")
                    self.assertAlmostEqual(actual[name], deg, delta=0.5)


if __name__ == "__main__":
    unittest.main()
