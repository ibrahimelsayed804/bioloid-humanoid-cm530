"""Serial bus access for Dynamixel AX servos.

Two ways to connect a PC to the servo bus (see docs/hardware.md):

* U2D2 / USB2Dynamixel adapter on the TTL bus: 1,000,000 bps (AX-12A default).
* Through the CM-530 controller's USB port in pass-through mode: usually
  57,600 bps on the PC side. Check this against your controller firmware.

The bus talks to a *transport*, any object with ``write(bytes)``,
``read(n) -> bytes`` and ``reset_input_buffer()``. ``SerialTransport`` wraps
pyserial for real hardware. ``bioloid.sim.SimTransport`` emulates a full
robot so everything can be developed and tested without hardware.
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Sequence

from . import ax12, protocol
from .protocol import ProtocolError, StatusPacket


class SerialTransport:
    """pyserial-backed transport for real hardware."""

    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 0.05):
        try:
            import serial  # pyserial
        except ImportError as exc:  # pragma: no cover - depends on environment
            raise RuntimeError("pyserial is required for hardware: pip install pyserial") from exc
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

    def write(self, data: bytes) -> None:
        self._ser.write(data)
        self._ser.flush()

    def read(self, n: int) -> bytes:
        return self._ser.read(n)

    def reset_input_buffer(self) -> None:
        self._ser.reset_input_buffer()

    def close(self) -> None:
        self._ser.close()


class DynamixelBus:
    """High-level read/write helpers on top of Protocol 1.0."""

    def __init__(self, transport, retries: int = 2, echo: bool = False):
        """
        :param transport: object with write/read/reset_input_buffer
        :param retries: extra attempts after a timeout or checksum error
        :param echo: set True for half-duplex adapters that echo what is sent
        """
        self.transport = transport
        self.retries = retries
        self.echo = echo

    # -- low level -------------------------------------------------------
    def _send(self, packet: bytes) -> None:
        self.transport.reset_input_buffer()
        self.transport.write(packet)
        if self.echo:
            self.transport.read(len(packet))

    def _receive(self) -> StatusPacket:
        head = self.transport.read(4)
        if len(head) < 4:
            raise TimeoutError("no response from servo")
        if head[:2] != protocol.HEADER:
            raise ProtocolError(f"bad header: {head.hex()}")
        rest = self.transport.read(head[3])
        if len(rest) < head[3]:
            raise TimeoutError("truncated response")
        return protocol.parse_status(head + rest)

    def transact(self, packet: bytes) -> StatusPacket:
        last_exc: Optional[Exception] = None
        for _ in range(self.retries + 1):
            try:
                self._send(packet)
                status = self._receive()
                status.raise_for_error()
                return status
            except (TimeoutError, ProtocolError) as exc:
                last_exc = exc
                time.sleep(0.002)
        assert last_exc is not None
        raise last_exc

    # -- instructions ----------------------------------------------------
    def ping(self, servo_id: int) -> bool:
        try:
            self.transact(protocol.ping(servo_id))
            return True
        except (TimeoutError, ProtocolError):
            return False

    def read_bytes(self, servo_id: int, address: int, length: int) -> List[int]:
        status = self.transact(protocol.read(servo_id, address, length))
        if len(status.params) != length:
            raise ProtocolError(f"servo {servo_id}: expected {length} bytes, got {len(status.params)}")
        return status.params

    def read_byte(self, servo_id: int, address: int) -> int:
        return self.read_bytes(servo_id, address, 1)[0]

    def read_word(self, servo_id: int, address: int) -> int:
        low, high = self.read_bytes(servo_id, address, 2)
        return protocol.from_word(low, high)

    def write_bytes(self, servo_id: int, address: int, data: Sequence[int]) -> None:
        self.transact(protocol.write(servo_id, address, data))

    def write_byte(self, servo_id: int, address: int, value: int) -> None:
        self.write_bytes(servo_id, address, [value])

    def write_word(self, servo_id: int, address: int, value: int) -> None:
        self.write_bytes(servo_id, address, protocol.to_word(value))

    def sync_write_words(self, address: int, values: Dict[int, int]) -> None:
        """Broadcast a 16-bit register to many servos. No status reply is sent."""
        entries = {sid: protocol.to_word(v) for sid, v in values.items()}
        self._send(protocol.sync_write(address, 2, entries))

    def sync_write_bytes(self, address: int, values: Dict[int, int]) -> None:
        entries = {sid: [v & 0xFF] for sid, v in values.items()}
        self._send(protocol.sync_write(address, 1, entries))

    # -- convenience -----------------------------------------------------
    def scan(self, ids: Sequence[int] = range(0, 31)) -> Dict[int, str]:
        """Ping a range of ids and return {id: model name} for the servos that answer."""
        found: Dict[int, str] = {}
        for sid in ids:
            if self.ping(sid):
                model = self.read_word(sid, ax12.MODEL_NUMBER)
                found[sid] = ax12.MODEL_NAMES.get(model, f"model {model}")
        return found
