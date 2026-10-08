"""Keyframe motions: load, mirror, interpolate and play back.

A motion is a list of keyframes. Each keyframe gives target angles for some
joints, how long to take getting there, and how long to hold afterwards.
Joints not mentioned in a keyframe keep their previous target. This is the
same idea as a RoboPlus Motion page, stored as readable JSON.

    {
      "name": "wave",
      "keyframes": [
        {"pose": {"r_shoulder_roll": -80}, "duration": 0.8},
        {"pose": {"r_elbow": 40}, "duration": 0.3, "hold": 0.1}
      ]
    }
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Mapping, Optional

from .robot import Humanoid

Pose = Dict[str, float]


@dataclass
class Keyframe:
    pose: Pose
    duration: float = 0.5
    hold: float = 0.0

    def __post_init__(self) -> None:
        if self.duration <= 0:
            raise ValueError("keyframe duration must be > 0")
        if self.hold < 0:
            raise ValueError("keyframe hold must be >= 0")


@dataclass
class Motion:
    name: str
    keyframes: List[Keyframe] = field(default_factory=list)
    description: str = ""

    @classmethod
    def load(cls, path: Path | str) -> "Motion":
        data = json.loads(Path(path).read_text())
        return cls(
            name=data.get("name", Path(path).stem),
            description=data.get("description", ""),
            keyframes=[Keyframe(**kf) for kf in data["keyframes"]],
        )

    def save(self, path: Path | str) -> None:
        data = {
            "name": self.name,
            "description": self.description,
            "keyframes": [{"pose": kf.pose, "duration": kf.duration, "hold": kf.hold}
                          for kf in self.keyframes],
        }
        Path(path).write_text(json.dumps(data, indent=2) + "\n")

    @property
    def total_time(self) -> float:
        return sum(kf.duration + kf.hold for kf in self.keyframes)

    def mirrored(self) -> "Motion":
        """Swap left and right joints, e.g. turn a right-hand wave into a left-hand one."""
        def swap(name: str) -> str:
            if name.startswith("r_"):
                return "l_" + name[2:]
            if name.startswith("l_"):
                return "r_" + name[2:]
            return name
        return Motion(
            name=f"{self.name}_mirrored",
            description=f"Mirror of {self.name}",
            keyframes=[Keyframe({swap(k): v for k, v in kf.pose.items()}, kf.duration, kf.hold)
                       for kf in self.keyframes],
        )


def ease(t: float) -> float:
    """Cosine ease-in/ease-out on 0..1. Zero velocity at both ends reduces jerk on the servos."""
    t = max(0.0, min(1.0, t))
    return 0.5 - 0.5 * math.cos(math.pi * t)


def interpolate(start: Mapping[str, float], end: Mapping[str, float], t: float) -> Pose:
    s = ease(t)
    return {name: start.get(name, target) + (target - start.get(name, target)) * s
            for name, target in end.items()}


class MotionPlayer:
    """Streams interpolated poses to the robot at a fixed rate.

    ``sleep`` is injectable so the same player runs in real time on hardware
    and in fast, deterministic simulated time in tests.
    """

    def __init__(self, robot: Humanoid, rate_hz: float = 50.0,
                 sleep: Callable[[float], None] = time.sleep,
                 on_frame: Optional[Callable[[float, Pose], None]] = None):
        self.robot = robot
        self.dt = 1.0 / rate_hz
        self.sleep = sleep
        self.on_frame = on_frame
        self.target: Pose = {}

    def play(self, motion: Motion, start_pose: Optional[Pose] = None) -> Pose:
        for kf in motion.keyframes:
            self.robot.validate_pose(kf.pose)
        current: Pose = dict(start_pose) if start_pose is not None else self.robot.read_pose()
        elapsed = 0.0
        for kf in motion.keyframes:
            begin = {name: current.get(name, 0.0) for name in kf.pose}
            steps = max(1, round(kf.duration / self.dt))
            for i in range(1, steps + 1):
                frame = interpolate(begin, kf.pose, i / steps)
                # Speed for each joint: just fast enough to reach this frame's
                # target within one tick, with 50% headroom so it never lags.
                speeds = {n: max(1.0, abs(frame[n] - current.get(n, frame[n])) / self.dt / 6.0 * 1.5)
                          for n in frame}
                self.robot.set_pose(frame, speeds)
                current.update(frame)
                elapsed += self.dt
                if self.on_frame:
                    self.on_frame(elapsed, dict(current))
                self.sleep(self.dt)
            if kf.hold:
                self.sleep(kf.hold)
                elapsed += kf.hold
        self.target = current
        return current
