"""Dynamixel Protocol 1.0 packet encoding and decoding.

The AX-12A servos in the Bioloid kit speak Dynamixel Protocol 1.0 over a
half-duplex TTL serial bus. Every packet has the same shape:

    Instruction:  0xFF 0xFF  ID  LENGTH  INSTRUCTION  PARAM_1 ... PARAM_N  CHECKSUM
    Status:       0xFF 0xFF  ID  LENGTH  ERROR        PARAM_1 ... PARAM_N  CHECKSUM

    LENGTH   = number of parameters + 2
    CHECKSUM = ~(ID + LENGTH + INSTRUCTION/ERROR + sum(PARAMS)) & 0xFF

This module is pure (no I/O) so it can be unit-tested without hardware.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, List, Sequence

HEADER = bytes([0xFF, 0xFF])
BROADCAST_ID = 0xFE

# Instruction set
PING = 0x01
READ_DATA = 0x02
WRITE_DATA = 0x03
REG_WRITE = 0x04
ACTION = 0x05
RESET = 0x06
SYNC_WRITE = 0x83

# Status-packet error bits
ERROR_BITS = {
    0x01: "input voltage",
    0x02: "angle limit",
    0x04: "overheating",
    0x08: "range",
    0x10: "checksum",
    0x20: "overload",
    0x40: "instruction",
}


class ProtocolError(Exception):
    """Raised when a status packet is malformed or reports an error."""


def checksum(body: Iterable[int]) -> int:
    """Checksum over ID, LENGTH, INSTRUCTION/ERROR and parameters."""
    return (~sum(body)) & 0xFF


def build_packet(servo_id: int, instruction: int, params: Sequence[int] = ()) -> bytes:
    """Encode an instruction packet."""
    if not 0 <= servo_id <= 0xFE:
        raise ValueError(f"servo id out of range: {servo_id}")
    params = [int(p) & 0xFF for p in params]
    body = [servo_id, len(params) + 2, instruction, *params]
    return HEADER + bytes(body + [checksum(body)])


def ping(servo_id: int) -> bytes:
    return build_packet(servo_id, PING)


def read(servo_id: int, address: int, length: int) -> bytes:
    return build_packet(servo_id, READ_DATA, [address, length])


def write(servo_id: int, address: int, data: Sequence[int]) -> bytes:
    return build_packet(servo_id, WRITE_DATA, [address, *data])


def sync_write(address: int, data_len: int, entries: dict[int, Sequence[int]]) -> bytes:
    """Write the same register block on many servos in a single packet.

    ``entries`` maps servo id -> list of ``data_len`` bytes. Sync write is what
    makes smooth whole-body motion possible: all 18 joints are updated in one
    bus transaction instead of 18.
    """
    params: List[int] = [address, data_len]
    for servo_id, data in entries.items():
        if len(data) != data_len:
            raise ValueError(f"servo {servo_id}: expected {data_len} bytes, got {len(data)}")
        params.append(servo_id)
        params.extend(data)
    return build_packet(BROADCAST_ID, SYNC_WRITE, params)


def to_word(value: int) -> List[int]:
    """Split a 16-bit value into [low, high] bytes (little endian)."""
    value = int(value)
    return [value & 0xFF, (value >> 8) & 0xFF]


def from_word(low: int, high: int) -> int:
    return low | (high << 8)


@dataclass
class StatusPacket:
    servo_id: int
    error: int
    params: List[int] = field(default_factory=list)

    @property
    def error_names(self) -> List[str]:
        return [name for bit, name in ERROR_BITS.items() if self.error & bit]

    def raise_for_error(self) -> None:
        if self.error:
            raise ProtocolError(f"servo {self.servo_id} error: {', '.join(self.error_names)}")


def parse_status(raw: bytes) -> StatusPacket:
    """Decode a status packet, validating header, length and checksum."""
    start = raw.find(HEADER)
    if start < 0:
        raise ProtocolError("no packet header found")
    raw = raw[start:]
    if len(raw) < 6:
        raise ProtocolError("packet too short")
    servo_id, length, error = raw[2], raw[3], raw[4]
    total = 4 + length
    if len(raw) < total:
        raise ProtocolError(f"incomplete packet: need {total} bytes, have {len(raw)}")
    params = list(raw[5 : 3 + length])
    expected = checksum([servo_id, length, error, *params])
    if raw[total - 1] != expected:
        raise ProtocolError(f"bad checksum: got 0x{raw[total - 1]:02X}, expected 0x{expected:02X}")
    return StatusPacket(servo_id, error, params)


def build_status(servo_id: int, error: int = 0, params: Sequence[int] = ()) -> bytes:
    """Encode a status packet (used by the simulator)."""
    params = list(params)
    body = [servo_id, len(params) + 2, error, *params]
    return HEADER + bytes(body + [checksum(body)])
