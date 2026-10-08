"""AX-12A control table and unit conversions.

Reference: ROBOTIS e-Manual, AX-12/AX-12A control table.
Position range is 0..1023 over 300 degrees, with 512 at the centre.
"""

# EEPROM area
MODEL_NUMBER = 0        # 2 bytes
FIRMWARE_VERSION = 2
ID = 3
BAUD_RATE = 4
RETURN_DELAY_TIME = 5
CW_ANGLE_LIMIT = 6      # 2 bytes
CCW_ANGLE_LIMIT = 8     # 2 bytes
TEMPERATURE_LIMIT = 11
MIN_VOLTAGE_LIMIT = 12
MAX_VOLTAGE_LIMIT = 13
MAX_TORQUE = 14         # 2 bytes
STATUS_RETURN_LEVEL = 16
ALARM_LED = 17
ALARM_SHUTDOWN = 18

# RAM area
TORQUE_ENABLE = 24
LED = 25
CW_COMPLIANCE_MARGIN = 26
CCW_COMPLIANCE_MARGIN = 27
CW_COMPLIANCE_SLOPE = 28
CCW_COMPLIANCE_SLOPE = 29
GOAL_POSITION = 30      # 2 bytes
MOVING_SPEED = 32       # 2 bytes
TORQUE_LIMIT = 34       # 2 bytes
PRESENT_POSITION = 36   # 2 bytes
PRESENT_SPEED = 38      # 2 bytes
PRESENT_LOAD = 40       # 2 bytes
PRESENT_VOLTAGE = 42
PRESENT_TEMPERATURE = 43
REGISTERED = 44
MOVING = 46
LOCK = 47
PUNCH = 48              # 2 bytes

MODEL_NAMES = {12: "AX-12A", 18: "AX-18A", 300: "AX-12W"}

POSITION_MIN = 0
POSITION_MAX = 1023
POSITION_CENTER = 512
RANGE_DEG = 300.0
DEG_PER_UNIT = RANGE_DEG / 1024.0     # ~0.293 deg per tick
RPM_PER_SPEED_UNIT = 0.111            # moving-speed register unit
MAX_SPEED_UNIT = 1023


def deg_to_position(deg: float) -> int:
    """Angle relative to centre (degrees) -> goal position ticks, clamped."""
    ticks = round(POSITION_CENTER + deg / DEG_PER_UNIT)
    return max(POSITION_MIN, min(POSITION_MAX, ticks))


def position_to_deg(ticks: int) -> float:
    return (ticks - POSITION_CENTER) * DEG_PER_UNIT


def rpm_to_speed(rpm: float) -> int:
    """RPM -> moving-speed register value. Note: 0 means 'max speed, no control'."""
    unit = round(abs(rpm) / RPM_PER_SPEED_UNIT)
    return max(1, min(MAX_SPEED_UNIT, unit))


def decode_load(raw: int) -> float:
    """Present-load register -> signed fraction of max torque (-1.0..1.0)."""
    magnitude = (raw & 0x3FF) / 1023.0
    return -magnitude if raw & 0x400 else magnitude


def decode_voltage(raw: int) -> float:
    return raw / 10.0
