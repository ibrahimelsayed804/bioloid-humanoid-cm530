"""Whole-robot interface: named joints, safety limits and health checks."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional

from . import ax12
from .bus import DynamixelBus

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config" / "humanoid_type_a.json"


@dataclass(frozen=True)
class Joint:
    id: int
    name: str
    direction: int = 1
    trim_deg: float = 0.0
    min_deg: float = -150.0
    max_deg: float = 150.0

    def clamp(self, deg: float) -> float:
        return max(self.min_deg, min(self.max_deg, deg))

    def to_ticks(self, deg: float) -> int:
        """Joint angle (robot convention) -> raw servo goal position."""
        return ax12.deg_to_position(self.direction * (self.clamp(deg) + self.trim_deg))

    def from_ticks(self, ticks: int) -> float:
        return self.direction * ax12.position_to_deg(ticks) - self.trim_deg + 0.0  # avoid -0.0


@dataclass
class JointHealth:
    id: int
    name: str
    angle_deg: float
    voltage_v: float
    temperature_c: int
    load: float
    warnings: List[str]


class Humanoid:
    def __init__(self, bus: DynamixelBus, joints: Iterable[Joint], *,
                 torque_limit_percent: int = 80, max_temperature_c: int = 65,
                 min_voltage_v: float = 10.0):
        self.bus = bus
        self.joints: Dict[str, Joint] = {j.name: j for j in joints}
        self.by_id: Dict[int, Joint] = {j.id: j for j in self.joints.values()}
        self.torque_limit_percent = torque_limit_percent
        self.max_temperature_c = max_temperature_c
        self.min_voltage_v = min_voltage_v

    @classmethod
    def from_config(cls, bus: DynamixelBus, path: Optional[Path] = None) -> "Humanoid":
        cfg = json.loads(Path(path or DEFAULT_CONFIG).read_text())
        joints = [Joint(**{k: v for k, v in j.items() if not k.startswith("_")}) for j in cfg["joints"]]
        return cls(bus, joints,
                   torque_limit_percent=cfg.get("torque_limit_percent", 80),
                   max_temperature_c=cfg.get("max_temperature_c", 65),
                   min_voltage_v=cfg.get("min_voltage_v", 10.0))

    # -- torque ----------------------------------------------------------
    def torque_on(self) -> None:
        """Enable torque without the robot jumping.

        The goal register is first set to each joint's present position, so
        the servos hold where they are instead of snapping to a stale goal.
        """
        present = {sid: self.bus.read_word(sid, ax12.PRESENT_POSITION) for sid in self.by_id}
        limit = round(1023 * self.torque_limit_percent / 100)
        self.bus.sync_write_words(ax12.TORQUE_LIMIT, {sid: limit for sid in self.by_id})
        self.bus.sync_write_words(ax12.GOAL_POSITION, present)
        self.bus.sync_write_bytes(ax12.TORQUE_ENABLE, {sid: 1 for sid in self.by_id})

    def torque_off(self) -> None:
        """Relax every joint (the robot will collapse if standing, so support it)."""
        self.bus.sync_write_bytes(ax12.TORQUE_ENABLE, {sid: 0 for sid in self.by_id})

    # -- motion ----------------------------------------------------------
    def validate_pose(self, pose: Mapping[str, float]) -> None:
        unknown = set(pose) - set(self.joints)
        if unknown:
            raise KeyError(f"unknown joints: {', '.join(sorted(unknown))}")

    def set_pose(self, pose: Mapping[str, float], speeds_rpm: Optional[Mapping[str, float]] = None) -> None:
        """Command joint angles (degrees) in one sync-write packet."""
        self.validate_pose(pose)
        if speeds_rpm:
            self.bus.sync_write_words(ax12.MOVING_SPEED, {
                self.joints[n].id: ax12.rpm_to_speed(rpm) for n, rpm in speeds_rpm.items()})
        self.bus.sync_write_words(ax12.GOAL_POSITION, {
            self.joints[n].id: self.joints[n].to_ticks(deg) for n, deg in pose.items()})

    def read_pose(self) -> Dict[str, float]:
        return {j.name: round(j.from_ticks(self.bus.read_word(j.id, ax12.PRESENT_POSITION)), 1) + 0.0
                for j in self.joints.values()}

    # -- health ----------------------------------------------------------
    def health(self) -> List[JointHealth]:
        report = []
        for j in self.joints.values():
            pos = self.bus.read_word(j.id, ax12.PRESENT_POSITION)
            load = ax12.decode_load(self.bus.read_word(j.id, ax12.PRESENT_LOAD))
            volts = ax12.decode_voltage(self.bus.read_byte(j.id, ax12.PRESENT_VOLTAGE))
            temp = self.bus.read_byte(j.id, ax12.PRESENT_TEMPERATURE)
            warnings = []
            if temp >= self.max_temperature_c:
                warnings.append("hot")
            if volts < self.min_voltage_v:
                warnings.append("low battery")
            if abs(load) > 0.9:
                warnings.append("overload")
            report.append(JointHealth(j.id, j.name, round(j.from_ticks(pos), 1) + 0.0, volts, temp,
                                      round(load, 2), warnings))
        return report
