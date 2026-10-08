"""A software model of the robot's servo bus.

``SimTransport`` behaves like a serial port with a chain of AX-12A servos on
it. It decodes instruction packets, keeps a control table per servo, moves
each joint towards its goal at the commanded speed, and replies with real
status packets. Every tool in this repo accepts ``--sim``, which lets you
develop and test motions before risking the hardware.
"""

from __future__ import annotations

import time
from typing import Dict, Iterable, Optional

from . import ax12, protocol


class SimServo:
    def __init__(self, servo_id: int, model: int = 12):
        self.table = bytearray(50)
        self.table[ax12.MODEL_NUMBER : ax12.MODEL_NUMBER + 2] = bytes(protocol.to_word(model))
        self.table[ax12.FIRMWARE_VERSION] = 24
        self.table[ax12.ID] = servo_id
        self.table[ax12.BAUD_RATE] = 1
        self.table[ax12.CCW_ANGLE_LIMIT : ax12.CCW_ANGLE_LIMIT + 2] = bytes(protocol.to_word(1023))
        self.table[ax12.TEMPERATURE_LIMIT] = 70
        self.table[ax12.MAX_TORQUE : ax12.MAX_TORQUE + 2] = bytes(protocol.to_word(1023))
        self.table[ax12.STATUS_RETURN_LEVEL] = 2
        self.table[ax12.TORQUE_LIMIT : ax12.TORQUE_LIMIT + 2] = bytes(protocol.to_word(1023))
        self.table[ax12.PRESENT_VOLTAGE] = 111   # 11.1 V LiPo
        self.table[ax12.PRESENT_TEMPERATURE] = 32
        self.position = float(ax12.POSITION_CENTER)
        self.goal = ax12.POSITION_CENTER
        # On power-up a real servo's goal register equals its present position.
        self.table[ax12.GOAL_POSITION : ax12.GOAL_POSITION + 2] = bytes(protocol.to_word(self.goal))
        self._sync_present()

    def word(self, address: int) -> int:
        return protocol.from_word(self.table[address], self.table[address + 1])

    def _sync_present(self) -> None:
        pos = int(round(self.position))
        self.table[ax12.PRESENT_POSITION : ax12.PRESENT_POSITION + 2] = bytes(protocol.to_word(pos))
        self.table[ax12.MOVING] = 1 if abs(self.position - self.goal) > 0.5 else 0

    def step(self, dt: float) -> None:
        if self.table[ax12.TORQUE_ENABLE]:
            speed = self.word(ax12.MOVING_SPEED) or ax12.MAX_SPEED_UNIT
            ticks_per_s = speed * ax12.RPM_PER_SPEED_UNIT * 6.0 / ax12.DEG_PER_UNIT
            delta = self.goal - self.position
            move = max(-ticks_per_s * dt, min(ticks_per_s * dt, delta))
            self.position += move
        self._sync_present()

    def write(self, address: int, data: Iterable[int]) -> None:
        data = list(data)
        for offset, value in enumerate(data):
            self.table[address + offset] = value
        if address <= ax12.GOAL_POSITION + 1 and address + len(data) > ax12.GOAL_POSITION:
            self.goal = max(0, min(1023, self.word(ax12.GOAL_POSITION)))

    def move_by_hand(self, ticks: int) -> None:
        """Simulate someone posing a limb while torque is off (teach mode)."""
        if not self.table[ax12.TORQUE_ENABLE]:
            self.position = float(ticks)
            self._sync_present()


class SimTransport:
    """Drop-in replacement for SerialTransport backed by simulated servos."""

    def __init__(self, ids: Iterable[int] = range(1, 19), realtime: bool = False):
        self.servos: Dict[int, SimServo] = {sid: SimServo(sid) for sid in ids}
        self.realtime = realtime
        self._rx = bytearray()
        self._clock = time.monotonic()
        self.sim_time = 0.0

    # Simulated time advances with real time when realtime=True, otherwise
    # callers advance it explicitly with advance().
    def advance(self, dt: float) -> None:
        self.sim_time += dt
        for servo in self.servos.values():
            servo.step(dt)

    def _tick(self) -> None:
        if self.realtime:
            now = time.monotonic()
            self.advance(now - self._clock)
            self._clock = now

    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def read(self, n: int) -> bytes:
        out, self._rx = bytes(self._rx[:n]), self._rx[n:]
        return out

    def write(self, data: bytes) -> None:
        self._tick()
        try:
            status = protocol.parse_status(data)  # same framing as instruction packets
        except protocol.ProtocolError:
            return  # corrupted packets are ignored, as real servos do
        sid, instruction, params = status.servo_id, status.error, status.params
        self._handle(sid, instruction, params)

    def _reply(self, sid: int, params=()) -> None:
        servo: Optional[SimServo] = self.servos.get(sid)
        if servo and sid != protocol.BROADCAST_ID and servo.table[ax12.STATUS_RETURN_LEVEL] >= 1:
            self._rx += protocol.build_status(sid, 0, params)

    def _handle(self, sid: int, instruction: int, params) -> None:
        if instruction == protocol.SYNC_WRITE:
            address, length = params[0], params[1]
            body = params[2:]
            for i in range(0, len(body), length + 1):
                target = self.servos.get(body[i])
                if target:
                    target.write(address, body[i + 1 : i + 1 + length])
            return
        servo = self.servos.get(sid)
        if servo is None:
            return  # nobody answers: timeout on the caller's side
        if instruction == protocol.PING:
            self._reply(sid)
        elif instruction == protocol.READ_DATA:
            address, length = params[0], params[1]
            self._reply(sid, servo.table[address : address + length])
        elif instruction == protocol.WRITE_DATA:
            servo.write(params[0], params[1:])
            if servo.table[ax12.STATUS_RETURN_LEVEL] >= 2:
                self._reply(sid)

    def close(self) -> None:
        pass
