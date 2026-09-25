"""Roborock Q7 Port 6001 Diagnostic Protocol and Telemetry Decoder.

Handles communication over TCP port 6001, UDP port 8899 discovery,
binary header framing (magic 0x51589158), nanopb Protobuf decoding of
RobotCleaner_Info_msg telemetry frames (magic 0xabbaccdd), and loading of
private capture dumps.
"""

from collections import Counter
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import errno
import json
import os
from pathlib import Path
import re
import socket
import struct
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

# Port 6001 Framing
TCP_MAGIC = 0x51589158
TCP_HEADER = struct.Struct("<IIII")  # magic, command << 8, length, reserved
MAX_PAYLOAD = 65536

# Telemetry Protobuf Framing
PASSIVE_MAGIC = bytes.fromhex("abbaccdd")
DISCOVERY_MAGIC = 0x00ABCDEF

# Commands
CMD_PASSIVE_TELEMETRY = 0x65
CMD_WRAPPER_ROBOT_PACKET = 0x70
CMD_AUDIO_TEST = 0x81
CMD_LIDAR_CONTROL = 0x83
CMD_MODEL = 0xEB
CMD_SKU = 0xF8
CMD_LANGUAGE = 0xF4
CMD_AUDIO_META = 0xF3
CMD_CERT_META = 0xEC
CMD_READ_PASSWORD = 0xF7

# Inner MCU Commands (sent wrapped in TCP command 0x70)
CMD_MCU_RESET_ERROR = 0x0A         # Reset active fault/error codes
CMD_MCU_CLIFF_READ = 0x19          # Read cached cliff ADC & thresholds
CMD_MCU_CLIFF_BACK = 0x1A          # Rear cliff sensor enable/disable
CMD_MCU_MOTOR_CONTROL_TYPE = 0x65  # Motor control mode (1=manual/test mode, 0=navigation mode)
CMD_MCU_MOTOR_SPEED = 0x67         # Wheels: left_speed_s32, right_speed_s32 (mm/s)
CMD_MCU_BLOWER_SPEED = 0x68        # Suction fan: speed_s16 (speed // 10)
CMD_MCU_SIDE_BRUSH = 0x69          # Side brush: speed_s8 (0-100)
CMD_MCU_MAIN_BRUSH = 0x6A          # Main roller brush: speed_s8 (0-100)
CMD_MCU_WATER_PUMP = 0x6B          # Water pump: speed_s8 (0-100)
CMD_MCU_SONIC_MOP = 0x6C           # VibraRise vibrating mop: 3-byte TSonicTank (speed, amp, enable)
CMD_MCU_CLIFF_IR_VALID = 0x77      # Cliff IR valid (0=disable cliff brake lockout, 1=enable)
CMD_MCU_BLOWER_CONTROL_MODE = 0x84 # Blower control mode (4-byte int 1=manual/test, 0=normal)
CMD_MCU_BLOWER_PWM = 0x85          # Suction blower fan: 1-byte PWM (0-100%)
CMD_MCU_BUTTON_LED = 0x8D          # White / Red button LEDs (0=off, 1=white, 2=red, 3=pulse)
CMD_MCU_WIFI_LED = 0x8E            # Blue Wi-Fi / Status indicator LED (0=off, 1=on, 2=blink)
CMD_MCU_DUST_COLLECT = 0x90        # Base station dust collection cycle
CMD_MCU_LIDAR_POWER = 0x97         # LiDAR motor power (1=on, 0=off)



def build_inner_mcu_packet(cmd: int, payload: bytes = b"") -> bytes:
    """Builds a CRobotPacket conforming to Shenzhen 3irobotix B01 / Everest OS framing.

    Header: 0xFA, 0xFB
    Length: 1 byte (number of following bytes up to and including checksum)
    Command: 1 byte
    Payload: N bytes
    Checksum: 1 or 2 bytes
    """
    body = bytes([cmd]) + payload
    n = len(body)
    if n <= 1:
        body_with_dummy = body + (b"\x00" if len(payload) == 0 else b"")
        chk = body_with_dummy[0] ^ body_with_dummy[1]
        pkt_len = len(body_with_dummy) + 1
        return b"\xfa\xfb" + bytes([pkt_len]) + body_with_dummy + bytes([chk])

    acc = 0
    i = 0
    while i + 1 < n:
        word = (body[i] << 8) | body[i + 1]
        acc = (acc + word) & 0xFFFF
        i += 2
    if i < n:
        acc ^= body[i]

    chk_hi = (acc >> 8) & 0xFF
    chk_lo = acc & 0xFF
    pkt_len = len(body) + 2
    return b"\xfa\xfb" + bytes([pkt_len]) + body + bytes([chk_hi, chk_lo])


# Inner Cliff Getter Packet (fafb03190019)
CLIFF_REQUEST_PAYLOAD = bytes.fromhex("fafb03190019")
CLIFF_SENSOR_NAMES = (
    "left",
    "left_front",
    "right_front",
    "right",
    "left_back",
    "right_back",
)

DIGEST_NAMES = (
    "device.json",
    "device.key",
    "device.crt",
    "rriot_ca.crt",
    "device_cfg.json",
)

PROTOBUF_TAG_METADATA: Dict[int, Dict[str, str]] = {
    1: {"name": "dustbox_state", "type": "uint32", "desc": "Dustbox Presence (0=missing, 1=inserted)"},
    2: {"name": "waterbox_state", "type": "uint32", "desc": "Water Tank Presence (0=missing, 1=inserted)"},
    3: {"name": "mop_bracket_state", "type": "uint32", "desc": "Mop Cloth Bracket Presence (0=missing, 1=attached)"},
    4: {"name": "power_state", "type": "uint32", "desc": "Power & Operational State"},
    5: {"name": "clean_mode", "type": "uint32", "desc": "Cleaning Preset / Fan Speed Mode"},
    6: {"name": "side_brush", "type": "Brush_msg", "desc": "Side Brush Motor (enabled, current mA, stall)"},
    7: {"name": "rolling_brush", "type": "RollingBrush_msg", "desc": "Main Roller Brush Motor (enabled, current mA, stall)"},
    8: {"name": "blower", "type": "Blower_msg", "desc": "Suction Blower Motor (enabled, level, RPM, current)"},
    9: {"name": "left_wheel", "type": "Wheel_msg", "desc": "Left Drive Wheel (status, speed, current, stall)"},
    10: {"name": "right_wheel", "type": "Wheel_msg", "desc": "Right Drive Wheel (status, speed, current, stall)"},
    11: {"name": "battery", "type": "Battery_msg", "desc": "Battery Pack (voltage raw, temp °C, current mA)"},
    12: {"name": "front_bumper", "type": "FrontBumper_msg", "desc": "Front Bumper Microswitches (left, right, lidar)"},
    13: {"name": "keys", "type": "Key_msg", "desc": "Top Panel Physical Buttons (Power, Dock, Spot)"},
    14: {"name": "fault_code", "type": "uint32", "desc": "MCU Active Fault / Error Code"},
    15: {"name": "ahrs", "type": "AHRS_msg", "desc": "6-Axis IMU (accel X/Y/Z, gyro X/Y/Z, pitch, roll, yaw)"},
    16: {"name": "detect_ground", "type": "DetectGround_msg", "desc": "Optical Cliff Sensors (LF, RF, L, R drop triggers)"},
    17: {"name": "charge_contact", "type": "uint32", "desc": "Dock Charging Contact State (0=open, 1=docked)"},
    18: {"name": "follow_wall", "type": "FollowWallDetect_msg", "desc": "Wall Follow TOF Optical Distance (mm)"},
    19: {"name": "front_touch", "type": "FrontTouch_msg", "desc": "Front Obstacle Optical Proximity Touch"},
    20: {"name": "edge_touch", "type": "EdgeTouch_msg", "desc": "Edge Optical Touch / Proximity"},
    21: {"name": "mop_motor_1", "type": "MopMotor_msg", "desc": "VibraRise Mop Lift & Vibration Actuator"},
    22: {"name": "mop_motor_2", "type": "MopMotor_msg", "desc": "Dual Rotary Mop Drive Motor"},
    23: {"name": "reserved_tag_23", "type": "submessage", "desc": "Nanopb Submessage Slot 23"},
    24: {"name": "reserved_tag_24", "type": "submessage", "desc": "Nanopb Submessage Slot 24"},
    25: {"name": "basestation_ir", "type": "Basestation_IR_msg", "desc": "Dock IR Beacon Homing Receivers (L, R, C)"},
    26: {"name": "slam_pose", "type": "SlamPoseReport_msg", "desc": "Live SLAM Pose Coordinates (X mm, Y mm, θ deg)"},
    27: {"name": "ultrasonic", "type": "UltraSonic_msg", "desc": "Carpet Acoustic Ultrasonic Sensor (carpet detect, signal)"},
    28: {"name": "wifi_rssi", "type": "WiFi_RSSI_msg", "desc": "Wi-Fi Signal Strength (dBm)"},
    29: {"name": "virtual_wall", "type": "VirtualWall_msg", "desc": "Magnetic Strip Virtual Wall Sensor"},
    30: {"name": "device_info", "type": "DeviceInfo_msg", "desc": "Device Hardware Identity & Subsystem Flags"},
    31: {"name": "lidar_bumper", "type": "LadarBumper_msg", "desc": "LiDAR Turret Collision Microswitch Trigger"},
    32: {"name": "led_mode", "type": "uint32", "desc": "Button LED Pattern Mode Index"},
    33: {"name": "dustbox_motor", "type": "DustboxMotor_msg", "desc": "Auto-Empty Dock Port Actuator Motor"},
    34: {"name": "water_pump", "type": "WaterPump_msg", "desc": "Peristaltic Electronic Water Pump Rate"},
    35: {"name": "sequence", "type": "uint32", "desc": "Protobuf Packet Sequence Counter"},
    36: {"name": "timestamp", "type": "uint32", "desc": "MCU Internal Uptime Timestamp (ms)"},
}


def _varint_decode(data: bytes, p: int) -> Tuple[int, int]:
    """Decode variable-length int from bytes starting at index p."""
    v = 0
    shift = 0
    while p < len(data):
        b = data[p]
        p += 1
        v |= (b & 0x7F) << shift
        if b < 0x80:
            return v, p
        shift += 7
        if shift > 70:
            raise ValueError("Varint too long")
    raise ValueError("Unexpected end of data reading varint")


def _decode_protobuf_fields(data: bytes) -> Dict[int, Any]:
    """Decode raw protobuf wire types into tag-indexed dictionary."""
    fields: Dict[int, Any] = {}
    p = 0
    while p < len(data):
        key, p = _varint_decode(data, p)
        tag = key >> 3
        wire_type = key & 7
        if wire_type == 0:  # Varint
            val, p = _varint_decode(data, p)
        elif wire_type == 2:  # Length-delimited
            length, p = _varint_decode(data, p)
            val = data[p : p + length]
            p += length
        elif wire_type == 5:  # 32-bit fixed
            val = struct.unpack("<I", data[p : p + 4])[0]
            p += 4
        elif wire_type == 1:  # 64-bit fixed
            val = struct.unpack("<Q", data[p : p + 8])[0]
            p += 8
        else:
            raise ValueError(f"Unsupported wire type {wire_type}")
        fields[tag] = val
    return fields


@dataclass
class CliffSensorData:
    left_adc: Optional[int] = None
    left_front_adc: Optional[int] = None
    right_front_adc: Optional[int] = None
    right_adc: Optional[int] = None
    left_back_adc: Optional[int] = None
    right_back_adc: Optional[int] = None

    left_threshold: Optional[int] = None
    left_front_threshold: Optional[int] = None
    right_front_threshold: Optional[int] = None
    right_threshold: Optional[int] = None
    left_back_threshold: Optional[int] = None
    right_back_threshold: Optional[int] = None

    left_triggered: bool = False
    left_front_triggered: bool = False
    right_front_triggered: bool = False
    right_triggered: bool = False
    left_back_triggered: bool = False
    right_back_triggered: bool = False


@dataclass
class BumperData:
    left_pressed: bool = False
    right_pressed: bool = False
    center_pressed: bool = False
    left_drop: bool = False
    right_drop: bool = False
    lidar_pressed: bool = False


@dataclass
class BatteryData:
    voltage_raw: Optional[int] = None  # Tag 1 (e.g. 146 -> ~14.6V)
    estimated_voltage_v: Optional[float] = None
    temperature_raw: Optional[int] = None  # Tag 2 (e.g. 26 -> 26C)
    current_raw: Optional[int] = None  # Tag 3 (charge/discharge current)
    state_raw: Optional[int] = None  # Tag 4
    is_charging: bool = False  # Tag 5
    percentage: Optional[int] = None  # Tag 6 (0-100%)
    cycle_count: Optional[int] = None  # Tag 7
    health_pct: Optional[int] = None  # Tag 8


@dataclass
class WheelData:
    left_status: Optional[int] = None
    left_speed: Optional[int] = None
    left_current: Optional[int] = None
    left_stalled: bool = False
    left_wheel_up: bool = False
    right_status: Optional[int] = None
    right_speed: Optional[int] = None
    right_current: Optional[int] = None
    right_stalled: bool = False
    right_wheel_up: bool = False


@dataclass
class KeyData:
    power_pressed: bool = False
    dock_pressed: bool = False
    spot_pressed: bool = False


@dataclass
class BrushData:
    side_enabled: bool = False
    side_stalled: bool = False
    side_speed_raw: Optional[int] = None
    side_current_ma: Optional[int] = None
    side_status: Optional[int] = None

    rolling_enabled: bool = False
    rolling_stalled: bool = False
    rolling_speed_raw: Optional[int] = None
    rolling_current_ma: Optional[int] = None
    rolling_status: Optional[int] = None


@dataclass
class BlowerData:
    enabled: bool = False
    is_stalled: bool = False
    level: Optional[int] = None
    speed_rpm: Optional[int] = None
    current_ma: Optional[int] = None


@dataclass
class AHRSData:
    accel_x: Optional[int] = None
    accel_y: Optional[int] = None
    accel_z: Optional[int] = None
    gyro_x: Optional[int] = None
    gyro_y: Optional[int] = None
    gyro_z: Optional[int] = None
    mag_x: Optional[int] = None
    mag_y: Optional[int] = None
    mag_z: Optional[int] = None
    pitch: Optional[int] = None
    roll: Optional[int] = None
    yaw: Optional[int] = None


@dataclass
class MopData:
    waterbox_present: bool = False
    mop_bracket_present: bool = False
    mop_motor_enabled: bool = False
    mop_motor_speed: Optional[int] = None
    mop_motor_current: Optional[int] = None
    mop_lift_status: Optional[int] = None
    mop_motor_stalled: bool = False
    mop_limit_switch_down: bool = False
    mop_limit_switch_up: bool = False
    mop_motor_2_enabled: bool = False
    water_pump_enabled: bool = False
    water_pump_speed: Optional[int] = None
    water_pump_current: Optional[int] = None


@dataclass
class SlamPoseData:
    x_mm: Optional[int] = None
    y_mm: Optional[int] = None
    theta_deg: Optional[float] = None
    map_id: Optional[int] = None


@dataclass
class EnvironmentData:
    wall_distance_raw: Optional[int] = None
    front_proximity_raw: Optional[int] = None
    carpet_detected: bool = False
    carpet_signal: Optional[int] = None
    wifi_rssi_dbm: Optional[int] = None
    virtual_wall_detected: bool = False
    charge_contact: bool = False
    dock_ir_left: bool = False
    dock_ir_right: bool = False
    dock_ir_center: bool = False
    led_mode: Optional[int] = None
    dustbox_motor_enabled: bool = False


@dataclass
class TelemetryFrame:
    sequence: Optional[int] = None
    timestamp_raw: Optional[int] = None
    received_at: str = ""
    dustbox_present: bool = True
    waterbox_present: bool = False
    mop_bracket_present: bool = False
    power_state: int = 1
    clean_mode: int = 0
    fault_code: int = 0
    bumper: BumperData = field(default_factory=BumperData)
    cliff: CliffSensorData = field(default_factory=CliffSensorData)
    battery: BatteryData = field(default_factory=BatteryData)
    wheel: WheelData = field(default_factory=WheelData)
    key: KeyData = field(default_factory=KeyData)
    brush: BrushData = field(default_factory=BrushData)
    blower: BlowerData = field(default_factory=BlowerData)
    ahrs: AHRSData = field(default_factory=AHRSData)
    mop: MopData = field(default_factory=MopData)
    slam_pose: SlamPoseData = field(default_factory=SlamPoseData)
    environment: EnvironmentData = field(default_factory=EnvironmentData)
    raw_tags: Dict[int, Any] = field(default_factory=dict)
    raw_hex: str = ""


def decode_robot_telemetry_payload(payload: bytes) -> TelemetryFrame:
    """Decodes a nanopb RobotCleaner_Info_msg payload preceded by 0xabbaccdd."""
    if len(payload) < 4:
        raise ValueError("Payload too short for telemetry frame")

    # Verify or strip magic
    if payload[:4] == PASSIVE_MAGIC:
        pb_data = payload[4:]
    else:
        pb_data = payload

    top_fields = _decode_protobuf_fields(pb_data)

    raw_tags_dict: Dict[int, Any] = {}
    for tag, val in top_fields.items():
        meta = PROTOBUF_TAG_METADATA.get(tag, {})
        raw_tags_dict[tag] = {
            "tag": tag,
            "name": meta.get("name", f"tag_{tag}"),
            "type": meta.get("type", "unknown"),
            "desc": meta.get("desc", ""),
            "value": val.hex() if isinstance(val, bytes) else val,
            "is_bytes": isinstance(val, bytes),
        }

    frame = TelemetryFrame(
        received_at=datetime.now(timezone.utc).isoformat(),
        raw_tags=raw_tags_dict,
        raw_hex=payload.hex(),
    )

    # Tag 1: Dustbox state
    if 1 in top_fields:
        frame.dustbox_present = bool(top_fields[1])

    # Tag 2: Waterbox state
    if 2 in top_fields:
        frame.waterbox_present = bool(top_fields[2])
        frame.mop.waterbox_present = bool(top_fields[2])

    # Tag 3: Mopping cloth bracket presence
    if 3 in top_fields:
        frame.mop_bracket_present = bool(top_fields[3])
        frame.mop.mop_bracket_present = bool(top_fields[3])

    # Tag 4: System power / general state
    if 4 in top_fields:
        frame.power_state = int(top_fields[4])

    # Tag 5: Clean mode / fan speed preset
    if 5 in top_fields:
        frame.clean_mode = int(top_fields[5])

    # Tag 6: Side Brush (Brush_msg)
    if 6 in top_fields and isinstance(top_fields[6], bytes):
        sub = _decode_protobuf_fields(top_fields[6])
        frame.brush.side_enabled = bool(sub.get(1, 0))
        frame.brush.side_stalled = bool(sub.get(2, 0))
        frame.brush.side_speed_raw = sub.get(3)
        frame.brush.side_current_ma = sub.get(4)
        frame.brush.side_status = sub.get(5)

    # Tag 7: Main Rolling Brush (RollingBrush_msg)
    if 7 in top_fields and isinstance(top_fields[7], bytes):
        sub = _decode_protobuf_fields(top_fields[7])
        frame.brush.rolling_enabled = bool(sub.get(1, 0))
        frame.brush.rolling_stalled = bool(sub.get(2, 0))
        frame.brush.rolling_speed_raw = sub.get(3)
        frame.brush.rolling_current_ma = sub.get(4)
        frame.brush.rolling_status = sub.get(5)

    # Tag 8: Blower (Blower_msg)
    if 8 in top_fields and isinstance(top_fields[8], bytes):
        sub = _decode_protobuf_fields(top_fields[8])
        frame.blower.enabled = bool(sub.get(1, 0))
        frame.blower.is_stalled = bool(sub.get(2, 0))
        frame.blower.level = sub.get(3)
        frame.blower.speed_rpm = sub.get(4)
        frame.blower.current_ma = sub.get(5)

    # Tag 9: Left Wheel (Wheel_msg)
    if 9 in top_fields and isinstance(top_fields[9], bytes):
        wheel_sub = _decode_protobuf_fields(top_fields[9])
        frame.wheel.left_status = wheel_sub.get(1) if 1 in wheel_sub else wheel_sub.get(4)
        frame.wheel.left_speed = wheel_sub.get(2)
        frame.wheel.left_current = wheel_sub.get(3)
        frame.wheel.left_stalled = bool(wheel_sub.get(5, 0))
        frame.wheel.left_wheel_up = bool(wheel_sub.get(6, 0))

    # Tag 10: Right Wheel (Wheel_msg)
    if 10 in top_fields and isinstance(top_fields[10], bytes):
        wheel_sub = _decode_protobuf_fields(top_fields[10])
        frame.wheel.right_status = wheel_sub.get(1) if 1 in wheel_sub else wheel_sub.get(4)
        frame.wheel.right_speed = wheel_sub.get(2)
        frame.wheel.right_current = wheel_sub.get(3)
        frame.wheel.right_stalled = bool(wheel_sub.get(5, 0))
        frame.wheel.right_wheel_up = bool(wheel_sub.get(6, 0))

    # Tag 11: Battery (Battery_msg)
    if 11 in top_fields and isinstance(top_fields[11], bytes):
        bat_sub = _decode_protobuf_fields(top_fields[11])
        frame.battery.voltage_raw = bat_sub.get(1)
        if frame.battery.voltage_raw is not None:
            frame.battery.estimated_voltage_v = round(frame.battery.voltage_raw * 0.1, 2)
        frame.battery.temperature_raw = bat_sub.get(2)
        frame.battery.current_raw = bat_sub.get(3)
        frame.battery.state_raw = bat_sub.get(4)
        frame.battery.is_charging = bool(bat_sub.get(5, 0))
        frame.battery.percentage = bat_sub.get(6)
        frame.battery.cycle_count = bat_sub.get(7)
        frame.battery.health_pct = bat_sub.get(8)

    # Tag 12: FrontBumper (FrontBumper_msg)
    if 12 in top_fields and isinstance(top_fields[12], bytes):
        bumper_sub = _decode_protobuf_fields(top_fields[12])
        frame.bumper.left_pressed = bool(bumper_sub.get(1, 0))
        frame.bumper.right_pressed = bool(bumper_sub.get(2, 0))
        frame.bumper.center_pressed = bool(bumper_sub.get(3, 0))
        frame.bumper.left_drop = bool(bumper_sub.get(4, 0))
        frame.bumper.right_drop = bool(bumper_sub.get(5, 0))
        if 6 in bumper_sub:
            frame.bumper.lidar_pressed = bool(bumper_sub.get(6, 0))
        elif 3 in bumper_sub and len(bumper_sub) <= 3:
            frame.bumper.lidar_pressed = bool(bumper_sub.get(3, 0))

    # Tag 13: Key switches (Key_msg)
    if 13 in top_fields and isinstance(top_fields[13], bytes):
        key_sub = _decode_protobuf_fields(top_fields[13])
        frame.key.power_pressed = bool(key_sub.get(1, 0))
        frame.key.dock_pressed = bool(key_sub.get(2, 0))
        frame.key.spot_pressed = bool(key_sub.get(3, 0))

    # Tag 14: Fault / Error code
    if 14 in top_fields:
        frame.fault_code = int(top_fields[14])

    # Tag 15: AHRS / IMU (AHRS_msg)
    if 15 in top_fields and isinstance(top_fields[15], bytes):
        sub = _decode_protobuf_fields(top_fields[15])
        def _to_s32(val: Optional[int]) -> Optional[int]:
            if val is None:
                return None
            return val - 0x100000000 if val > 0x7FFFFFFF else val
        frame.ahrs.accel_x = _to_s32(sub.get(1))
        frame.ahrs.accel_y = _to_s32(sub.get(2))
        frame.ahrs.accel_z = _to_s32(sub.get(3))
        frame.ahrs.gyro_x = _to_s32(sub.get(4))
        frame.ahrs.gyro_y = _to_s32(sub.get(5))
        frame.ahrs.gyro_z = _to_s32(sub.get(6))
        frame.ahrs.mag_x = _to_s32(sub.get(7))
        frame.ahrs.mag_y = _to_s32(sub.get(8))
        frame.ahrs.mag_z = _to_s32(sub.get(9))
        frame.ahrs.pitch = _to_s32(sub.get(10))
        frame.ahrs.roll = _to_s32(sub.get(11)) if 11 in sub else _to_s32(sub.get(9))
        frame.ahrs.yaw = _to_s32(sub.get(12))

    # Tag 16: DetectGround -> DetectGround_Status (Cliff sensor triggers)
    if 16 in top_fields and isinstance(top_fields[16], bytes):
        dg_sub = _decode_protobuf_fields(top_fields[16])
        if 1 in dg_sub and isinstance(dg_sub[1], bytes):
            status_sub = _decode_protobuf_fields(dg_sub[1])
            frame.cliff.left_front_triggered = bool(status_sub.get(1, 0))
            frame.cliff.right_front_triggered = bool(status_sub.get(2, 0))
            frame.cliff.left_triggered = bool(status_sub.get(3, 0))
            frame.cliff.right_triggered = bool(status_sub.get(4, 0))
            frame.cliff.left_back_triggered = bool(status_sub.get(5, 0))
            frame.cliff.right_back_triggered = bool(status_sub.get(6, 0))
        if 2 in dg_sub and isinstance(dg_sub[2], bytes):
            val_sub = _decode_protobuf_fields(dg_sub[2])
            frame.cliff.left_front_adc = val_sub.get(1)
            frame.cliff.right_front_adc = val_sub.get(2)
            frame.cliff.left_adc = val_sub.get(3)
            frame.cliff.right_adc = val_sub.get(4)
            frame.cliff.left_back_adc = val_sub.get(5)
            frame.cliff.right_back_adc = val_sub.get(6)
        if 3 in dg_sub and isinstance(dg_sub[3], bytes):
            dem_sub = _decode_protobuf_fields(dg_sub[3])
            frame.cliff.left_front_threshold = dem_sub.get(1)
            frame.cliff.right_front_threshold = dem_sub.get(2)
            frame.cliff.left_threshold = dem_sub.get(3)
            frame.cliff.right_threshold = dem_sub.get(4)
            frame.cliff.left_back_threshold = dem_sub.get(5)
            frame.cliff.right_back_threshold = dem_sub.get(6)

    # Tag 17: Charging pole electrical contact
    if 17 in top_fields:
        frame.environment.charge_contact = bool(top_fields[17])

    # Tag 18: FollowWallDetect (Wall TOF sensor)
    if 18 in top_fields and isinstance(top_fields[18], bytes):
        sub = _decode_protobuf_fields(top_fields[18])
        frame.environment.wall_distance_raw = sub.get(1)

    # Tag 19: FrontTouch (Front optical proximity)
    if 19 in top_fields and isinstance(top_fields[19], bytes):
        sub = _decode_protobuf_fields(top_fields[19])
        frame.environment.front_proximity_raw = sub.get(1)

    # Tag 21: MopMotor 1 (VibraRise / lifting actuator)
    if 21 in top_fields and isinstance(top_fields[21], bytes):
        sub = _decode_protobuf_fields(top_fields[21])
        frame.mop.mop_motor_enabled = bool(sub.get(1, 0))
        frame.mop.mop_motor_speed = sub.get(2)
        frame.mop.mop_motor_current = sub.get(3)
        frame.mop.mop_lift_status = sub.get(4)
        frame.mop.mop_motor_stalled = bool(sub.get(5, 0))
        frame.mop.mop_limit_switch_down = bool(sub.get(6, 0))
        frame.mop.mop_limit_switch_up = bool(sub.get(7, 0))

    # Tag 22: MopMotor 2 (Dual rotary spin mop)
    if 22 in top_fields and isinstance(top_fields[22], bytes):
        sub = _decode_protobuf_fields(top_fields[22])
        frame.mop.mop_motor_2_enabled = bool(sub.get(1, 0))

    # Tag 25: Basestation IR receivers
    if 25 in top_fields and isinstance(top_fields[25], bytes):
        sub = _decode_protobuf_fields(top_fields[25])
        frame.environment.dock_ir_left = bool(sub.get(1, 0))
        frame.environment.dock_ir_right = bool(sub.get(2, 0))
        frame.environment.dock_ir_center = bool(sub.get(3, 0))

    # Tag 26: SlamPoseReport (Live SLAM robot pose)
    if 26 in top_fields and isinstance(top_fields[26], bytes):
        sub = _decode_protobuf_fields(top_fields[26])
        if 1 in sub and isinstance(sub[1], bytes):
            pose_sub = _decode_protobuf_fields(sub[1])
            def _to_s16(val: Optional[int]) -> Optional[int]:
                if val is None:
                    return None
                return val - 0x10000 if val > 0x7FFF else val
            frame.slam_pose.x_mm = _to_s16(pose_sub.get(1))
            frame.slam_pose.y_mm = _to_s16(pose_sub.get(2))
        if 2 in sub:
            frame.slam_pose.theta_deg = round(float(sub[2]) * 0.1, 1)

    # Tag 27: UltraSonic (Carpet acoustic transducer)
    if 27 in top_fields and isinstance(top_fields[27], bytes):
        sub = _decode_protobuf_fields(top_fields[27])
        frame.environment.carpet_detected = bool(sub.get(1, 0))
        frame.environment.carpet_signal = sub.get(2)

    # Tag 28: WiFi RSSI
    if 28 in top_fields and isinstance(top_fields[28], bytes):
        sub = _decode_protobuf_fields(top_fields[28])
        rssi_val = sub.get(1)
        if rssi_val is not None:
            frame.environment.wifi_rssi_dbm = rssi_val - 256 if rssi_val > 127 else rssi_val

    # Tag 29: VirtualWall (Magnetic strip sensor)
    if 29 in top_fields and isinstance(top_fields[29], bytes):
        sub = _decode_protobuf_fields(top_fields[29])
        frame.environment.virtual_wall_detected = bool(sub.get(1, 0))

    # Tag 31: LadarBumper (LiDAR top bumper)
    if 31 in top_fields and isinstance(top_fields[31], bytes):
        ladar_sub = _decode_protobuf_fields(top_fields[31])
        if any(ladar_sub.get(tag, 0) for tag in (1, 2, 3, 4)):
            frame.bumper.lidar_pressed = True

    # Tag 32: LED mode index
    if 32 in top_fields:
        frame.environment.led_mode = int(top_fields[32])

    # Tag 33: Dustbox auto-empty motor
    if 33 in top_fields and isinstance(top_fields[33], bytes):
        sub = _decode_protobuf_fields(top_fields[33])
        frame.environment.dustbox_motor_enabled = bool(sub.get(1, 0))

    # Tag 34: Electronic Water Pump
    if 34 in top_fields and isinstance(top_fields[34], bytes):
        sub = _decode_protobuf_fields(top_fields[34])
        frame.mop.water_pump_enabled = bool(sub.get(1, 0))
        frame.mop.water_pump_speed = sub.get(2)

    # Tag 35: Sequence counter
    if 35 in top_fields:
        frame.sequence = int(top_fields[35])

    # Tag 36: Timestamp
    if 36 in top_fields:
        frame.timestamp_raw = int(top_fields[36])

    return frame


def decode_cliff_adc_reply(payload: bytes) -> Dict[str, Any]:
    """Decodes inner 0x19 reply from 0x70 command (length 38 bytes)."""
    if len(payload) != 38 or payload[:2] != b"\xfa\xfb" or payload[3] != 0x19:
        raise ValueError("Invalid cliff ADC response frame")

    adcs = struct.unpack_from("<6I", payload, 4)
    thresholds = payload[28:34]

    adc_dict = dict(zip(CLIFF_SENSOR_NAMES, adcs))
    threshold_dict = dict(zip(CLIFF_SENSOR_NAMES, thresholds))

    # Check triggers (reading <= threshold means cliff triggered)
    triggers = {
        name: (adc_dict[name] <= threshold_dict[name]) for name in CLIFF_SENSOR_NAMES[:4]
    }

    return {
        "adc": adc_dict,
        "threshold": threshold_dict,
        "triggers": triggers,
    }


def parse_clean_text(data: bytes) -> Optional[str]:
    """Helper to extract clean ascii null-terminated string."""
    raw = data.split(b"\0", 1)[0]
    return raw.decode("ascii") if all(32 <= b < 127 for b in raw) else None


def decode_getter_response(command: int, payload: bytes) -> Dict[str, Any]:
    """Decodes raw responses for model, sku, language, audio, certs, password."""
    if command in (CMD_MODEL, CMD_SKU, CMD_LANGUAGE):
        return {"value": parse_clean_text(payload)}

    if command == CMD_READ_PASSWORD:
        pwd = payload.rstrip(b"\0").decode("ascii", errors="replace")
        return {"ssh_password": pwd}

    if command == CMD_AUDIO_META:
        return {
            "project": parse_clean_text(payload[0:10]),
            "audio_region": parse_clean_text(payload[10:30]),
            "audio_version": parse_clean_text(payload[30:50]),
            "audio_package": parse_clean_text(payload[50:90]),
        }

    if command == CMD_CERT_META:
        checksums = {}
        for index, name in enumerate(DIGEST_NAMES):
            digest_bytes = payload[index * 50 : (index + 1) * 50]
            val = parse_clean_text(digest_bytes)
            checksums[name] = val if val and re.fullmatch(r"[0-9a-fA-F]{32}", val) else None

        cfg_prefix = ""
        if len(payload) > 250:
            raw_cfg = payload[250:].split(b"\0", 1)[0]
            cfg_prefix = raw_cfg.decode("ascii", errors="replace")

        return {
            "checksums": checksums,
            "configuration_prefix": cfg_prefix,
        }

    return {"raw_hex": payload.hex()}


def receive_exact(sock: socket.socket, count: int, deadline: float) -> bytes:
    """Reads exact number of bytes from socket before deadline."""
    out = bytearray()
    while len(out) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Socket receive timed out")
        sock.settimeout(remaining)
        block = sock.recv(count - len(out))
        if not block:
            raise ConnectionError("Socket closed prematurely")
        out.extend(block)
    return bytes(out)


def receive_frame(sock: socket.socket, deadline: float) -> Tuple[int, bytes]:
    """Reads 16-byte header and full payload of a TCP 6001 frame."""
    header = receive_exact(sock, TCP_HEADER.size, deadline)
    magic, word, length, _reserved = TCP_HEADER.unpack(header)
    if magic != TCP_MAGIC or length > MAX_PAYLOAD:
        raise ValueError(f"Invalid frame header (magic=0x{magic:x}, len={length})")
    payload = receive_exact(sock, length, deadline)
    return (word >> 8), payload


def build_request_frame(command: int, payload: bytes = b"") -> bytes:
    """Constructs a 16-byte TCP header + payload frame."""
    return TCP_HEADER.pack(TCP_MAGIC, command << 8, len(payload), 0) + payload


class Q7DiagnosticClient:
    """Client for connecting to Q7 port 6001 and executing diagnostics."""

    def __init__(self, host: str = "192.168.5.1", port: int = 6001):
        self.host = host
        self.port = port
        self._is_streaming = False
        self._stream_thread: Optional[threading.Thread] = None
        self._listeners: List[Callable[[TelemetryFrame], None]] = []
        self._lock = threading.Lock()
        self.last_frame: Optional[TelemetryFrame] = None
        self.frame_history: List[TelemetryFrame] = []
        self.max_history = 100

    def add_listener(self, cb: Callable[[TelemetryFrame], None]) -> None:
        with self._lock:
            self._listeners.append(cb)

    def remove_listener(self, cb: Callable[[TelemetryFrame], None]) -> None:
        with self._lock:
            if cb in self._listeners:
                self._listeners.remove(cb)

    def _broadcast(self, frame: TelemetryFrame) -> None:
        with self._lock:
            self.last_frame = frame
            self.frame_history.append(frame)
            if len(self.frame_history) > self.max_history:
                self.frame_history.pop(0)
            listeners_copy = list(self._listeners)
        for cb in listeners_copy:
            try:
                cb(frame)
            except Exception:
                pass

    def check_ports(self, ports: Tuple[int, ...] = (22, 23, 5555, 6001, 58867), timeout: float = 2.0) -> Dict[str, Any]:
        """Probes connectivity for common robot ports."""
        res: Dict[str, Any] = {}
        for p in ports:
            started = time.monotonic()
            info: Dict[str, Any] = {"port": p, "status": "closed"}
            try:
                with socket.create_connection((self.host, p), timeout=timeout):
                    info["status"] = "open"
            except (socket.timeout, TimeoutError):
                info["status"] = "timeout"
            except OSError as e:
                info["status"] = "refused" if getattr(e, "errno", None) in (errno.ECONNREFUSED, 10061) else "error"
            info["elapsed_ms"] = round((time.monotonic() - started) * 1000)
            res[str(p)] = info
        return res

    def query_discovery_8899(self, timeout: float = 3.0) -> Dict[str, Any]:
        """Sends UDP discovery probe on port 8899."""
        request = struct.pack("<I", DISCOVERY_MAGIC) + os.urandom(4)
        result: Dict[str, Any] = {"status": "unavailable", "port": 8899}
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.settimeout(timeout)
                sock.connect((self.host, 8899))
                sock.send(request)
                data = sock.recv(2048)
            if len(data) == 304 and data[:8] == request:
                body = data[8:]
                result.update(
                    status="reply",
                    versions={
                        "software": parse_clean_text(body[4:36]),
                        "hardware": parse_clean_text(body[36:68]),
                        "mcu": parse_clean_text(body[100:132]),
                        "robot_release": parse_clean_text(body[164:196]),
                    },
                    serial=parse_clean_text(body[228:260]),
                    mac=parse_clean_text(body[272:292]),
                    raw_hex=data.hex(),
                )
        except Exception as e:
            result["error"] = str(e)
        return result

    def query_getter(self, command: int, expected_len: Optional[int] = None, timeout: float = 4.0) -> Dict[str, Any]:
        """Sends an active getter command to port 6001 and parses response."""
        req_frame = build_request_frame(command)
        result: Dict[str, Any] = {"command": hex(command), "status": "unavailable"}
        deadline = time.monotonic() + timeout
        try:
            with socket.create_connection((self.host, self.port), timeout=timeout) as sock:
                sock.sendall(req_frame)
                for _ in range(64):
                    cmd, payload = receive_frame(sock, deadline)
                    if cmd == command:
                        result["status"] = "reply"
                        result["response_bytes"] = len(payload)
                        result["decoded"] = decode_getter_response(command, payload)
                        result["raw_hex"] = payload.hex()
                        break
        except Exception as e:
            result["error"] = str(e)
        return result

    def query_cliff_adc(self, timeout: float = 4.0) -> Dict[str, Any]:
        """Queries cached cliff ADC & thresholds via inner command 0x19."""
        req_frame = build_request_frame(CMD_WRAPPER_ROBOT_PACKET, CLIFF_REQUEST_PAYLOAD)
        result: Dict[str, Any] = {"command": "0x70/0x19", "status": "unavailable"}
        deadline = time.monotonic() + timeout
        try:
            with socket.create_connection((self.host, self.port), timeout=timeout) as sock:
                sock.sendall(req_frame)
                for _ in range(64):
                    cmd, payload = receive_frame(sock, deadline)
                    if cmd == CMD_PASSIVE_TELEMETRY and len(payload) == 38 and payload[:2] == b"\xfa\xfb":
                        result["status"] = "reply"
                        result["data"] = decode_cliff_adc_reply(payload)
                        result["raw_hex"] = payload.hex()
                        break
        except Exception as e:
            result["error"] = str(e)
        return result

    def start_streaming(self) -> None:
        """Starts background streaming thread on TCP 6001."""
        if self._is_streaming:
            return
        self._is_streaming = True
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

    def stop_streaming(self) -> None:
        """Stops background streaming."""
        self._is_streaming = False
        if self._stream_thread and self._stream_thread.is_alive():
            self._stream_thread.join(timeout=1.0)
        self._stream_thread = None

    def _stream_loop(self) -> None:
        while self._is_streaming:
            try:
                with socket.create_connection((self.host, self.port), timeout=5.0) as sock:
                    deadline = time.monotonic() + 10.0
                    while self._is_streaming:
                        cmd, payload = receive_frame(sock, deadline)
                        deadline = time.monotonic() + 10.0
                        if cmd == CMD_PASSIVE_TELEMETRY and payload.startswith(PASSIVE_MAGIC):
                            frame = decode_robot_telemetry_payload(payload)
                            self._broadcast(frame)
            except Exception:
                time.sleep(1.0)

    def send_mcu_packet(self, packet: bytes, timeout: float = 3.0) -> bool:
        """Sends an inner CRobotPacket wrapped in TCP command 0x70 to the MCU."""
        req_frame = build_request_frame(CMD_WRAPPER_ROBOT_PACKET, packet)
        try:
            with socket.create_connection((self.host, self.port), timeout=timeout) as sock:
                sock.sendall(req_frame)
            return True
        except Exception:
            return False

    def send_raw_command(self, command: int, payload: bytes = b"", timeout: float = 3.0) -> bool:
        """Sends an arbitrary TCP frame on port 6001."""
        req_frame = build_request_frame(command, payload)
        try:
            with socket.create_connection((self.host, self.port), timeout=timeout) as sock:
                sock.sendall(req_frame)
            return True
        except Exception:
            return False

    def actuate_motor(
        self,
        component: str,
        value: int = 50,
        duration_s: float = 2.0,
    ) -> Dict[str, Any]:
        """Actuates a hardware component on-demand with automatic safety timeout shutdown.

        Supported components:
          - 'main_brush': Main roller brush (value: 0-100% speed, default 50)
          - 'side_brush': Side brush (value: 0-100% speed, default 50)
          - 'blower': Suction blower fan (value: 0-100 PWM % or RPM, default 40)
          - 'water_pump': Water pump (value: 0-100% speed, default 50)
          - 'sonic_mop': VibraRise vibrating mop motor (value: 0-100%, default 50)
          - 'wheels', 'wheels_fwd': Drive wheels forward (value: 5-80 mm/s, default 30)
          - 'wheels_rev': Drive wheels in reverse (value: 5-80 mm/s, default 30)
          - 'wheels_left': Spin in place turn left (left -30, right +30 mm/s)
          - 'wheels_right': Spin in place turn right (left +30, right -30 mm/s)
          - 'wheel_left_only': Left drive wheel only (value: 5-80 mm/s)
          - 'wheel_right_only': Right drive wheel only (value: 5-80 mm/s)
          - 'lidar': LiDAR spin motor (value: 0=stop, 1=spin)
          - 'led': Button LEDs (value: 0=off, 1=white, 2=red, 3=pulse)
          - 'wifi_led': Wi-Fi / status LED (value: 0=off, 1=on, 2=blink)
          - 'audio_test': Speaker test tone trigger
          - 'stop': Immediate emergency stop of all motors
        """
        duration_s = max(0.2, min(duration_s, 5.0))

        pkt_start: Optional[bytes] = None
        pkt_stop: Optional[bytes] = None
        is_wheel = False
        is_blower = False

        if component == "main_brush":
            spd = max(0, min(value, 100))
            self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_RESET_ERROR))
            pkt_start = build_inner_mcu_packet(CMD_MCU_MAIN_BRUSH, bytes([spd]))
            pkt_stop = build_inner_mcu_packet(CMD_MCU_MAIN_BRUSH, bytes([0]))
        elif component == "side_brush":
            spd = max(0, min(value, 100))
            self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_RESET_ERROR))
            pkt_start = build_inner_mcu_packet(CMD_MCU_SIDE_BRUSH, bytes([spd]))
            pkt_stop = build_inner_mcu_packet(CMD_MCU_SIDE_BRUSH, bytes([0]))
        elif component == "blower":
            is_blower = True
            # Support both PWM percentage (0-100) or raw RPM/level (e.g. 1200-7500)
            if value > 100:
                pwm_pct = min(100, max(10, int(value / 75)))
                spd_param = value // 10
            else:
                pwm_pct = max(0, min(value, 100))
                spd_param = pwm_pct * 50
            # Pre-requisite: Put MCU blower into manual control mode
            self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_RESET_ERROR))
            self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_BLOWER_CONTROL_MODE, struct.pack("<I", 1)))
            # Send both PWM byte (0x85) and speed short (0x68)
            self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_BLOWER_SPEED, struct.pack("<h", spd_param)))
            pkt_start = build_inner_mcu_packet(CMD_MCU_BLOWER_PWM, bytes([pwm_pct]))
            pkt_stop = build_inner_mcu_packet(CMD_MCU_BLOWER_PWM, bytes([0]))
        elif component == "water_pump":
            spd = max(0, min(value, 100))
            pkt_start = build_inner_mcu_packet(CMD_MCU_WATER_PUMP, bytes([spd]))
            pkt_stop = build_inner_mcu_packet(CMD_MCU_WATER_PUMP, bytes([0]))
        elif component == "sonic_mop":
            spd = max(0, min(value, 100))
            self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_RESET_ERROR))
            # TSonicTank requires 3 bytes: speed, amplitude, enable
            pkt_start = build_inner_mcu_packet(CMD_MCU_SONIC_MOP, bytes([spd, 1, 1]))
            pkt_stop = build_inner_mcu_packet(CMD_MCU_SONIC_MOP, bytes([0, 0, 0]))
        elif component in ("wheels", "wheels_fwd", "wheels_rev", "wheels_left", "wheels_right", "wheel_left_only", "wheel_right_only"):
            is_wheel = True
            # Pre-requisites for MCU wheel actuation in test mode:
            # 1. Reset any active error code / drop flag
            self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_RESET_ERROR))
            # 2. Disable optical cliff brake latch so wheels can spin on test bench
            self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_CLIFF_IR_VALID, bytes([0])))
            # 3. Switch MCU to manual test motor control mode (cmd 0x65 = 1)
            self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_MOTOR_CONTROL_TYPE, bytes([1])))

            if component in ("wheels", "wheels_fwd"):
                mm_s = max(5, min(value, 80))
                l_spd, r_spd = mm_s, mm_s
            elif component == "wheels_rev":
                mm_s = -max(5, min(abs(value), 80))
                l_spd, r_spd = mm_s, mm_s
            elif component == "wheels_left":
                spd = max(10, min(abs(value), 50))
                l_spd, r_spd = -spd, spd
            elif component == "wheels_right":
                spd = max(10, min(abs(value), 50))
                l_spd, r_spd = spd, -spd
            elif component == "wheel_left_only":
                l_spd, r_spd = max(5, min(value, 80)), 0
            else:  # wheel_right_only
                l_spd, r_spd = 0, max(5, min(value, 80))

            pkt_start = build_inner_mcu_packet(CMD_MCU_MOTOR_SPEED, struct.pack("<ii", l_spd, r_spd))
            pkt_stop = build_inner_mcu_packet(CMD_MCU_MOTOR_SPEED, struct.pack("<ii", 0, 0))
        elif component == "lidar":
            val = 1 if value else 0
            self.send_raw_command(CMD_LIDAR_CONTROL, bytes([val]))
            pkt_start = build_inner_mcu_packet(CMD_MCU_LIDAR_POWER, bytes([1 if val else 0]))
            pkt_stop = build_inner_mcu_packet(CMD_MCU_LIDAR_POWER, bytes([0]))
            if val == 0:
                self.send_raw_command(CMD_LIDAR_CONTROL, bytes([0]))
        elif component == "led":
            val = max(0, min(value, 3))
            pkt_start = build_inner_mcu_packet(CMD_MCU_BUTTON_LED, bytes([val]))
            pkt_stop = build_inner_mcu_packet(CMD_MCU_BUTTON_LED, bytes([0]))
        elif component == "wifi_led":
            val = max(0, min(value, 2))
            pkt_start = build_inner_mcu_packet(CMD_MCU_WIFI_LED, bytes([val]))
            pkt_stop = build_inner_mcu_packet(CMD_MCU_WIFI_LED, bytes([0]))
        elif component == "audio_test":
            self.send_raw_command(CMD_AUDIO_TEST, bytes([1]))
            return {"success": True, "action": "audio_test"}
        elif component == "stop":
            self.send_raw_command(CMD_LIDAR_CONTROL, bytes([0]), timeout=0.5)
            for cmd, payload in (
                (CMD_MCU_MAIN_BRUSH, bytes([0])),
                (CMD_MCU_SIDE_BRUSH, bytes([0])),
                (CMD_MCU_BLOWER_PWM, bytes([0])),
                (CMD_MCU_BLOWER_SPEED, struct.pack("<h", 0)),
                (CMD_MCU_BLOWER_CONTROL_MODE, struct.pack("<I", 0)),
                (CMD_MCU_WATER_PUMP, bytes([0])),
                (CMD_MCU_SONIC_MOP, bytes([0, 0, 0])),
                (CMD_MCU_LIDAR_POWER, bytes([0])),
                (CMD_MCU_MOTOR_SPEED, struct.pack("<ii", 0, 0)),
                (CMD_MCU_MOTOR_CONTROL_TYPE, bytes([0])),
                (CMD_MCU_CLIFF_IR_VALID, bytes([1])),
                (CMD_MCU_BUTTON_LED, bytes([0])),
                (CMD_MCU_WIFI_LED, bytes([0])),
                (CMD_MCU_RESET_ERROR, b""),
            ):
                sent = self.send_mcu_packet(build_inner_mcu_packet(cmd, payload), timeout=0.3)
                if not sent:
                    break
            return {"success": True, "action": "stop_all"}
        else:
            return {"success": False, "error": f"Unknown component: {component}"}

        ok = self.send_mcu_packet(pkt_start)
        if not ok:
            return {"success": False, "error": "Failed to send actuation packet over port 6001"}

        def _auto_stop() -> None:
            time.sleep(duration_s)
            self.send_mcu_packet(pkt_stop)
            if is_wheel:
                # Restore normal navigation control mode and cliff safety
                self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_MOTOR_CONTROL_TYPE, bytes([0])))
                self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_CLIFF_IR_VALID, bytes([1])))
            elif is_blower:
                self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_BLOWER_SPEED, struct.pack("<h", 0)))
                self.send_mcu_packet(build_inner_mcu_packet(CMD_MCU_BLOWER_CONTROL_MODE, struct.pack("<I", 0)))
            elif component == "lidar":
                self.send_raw_command(CMD_LIDAR_CONTROL, bytes([0]))

        threading.Thread(target=_auto_stop, daemon=True).start()

        return {
            "success": True,
            "component": component,
            "value": value,
            "duration_s": duration_s,
            "packet_hex": pkt_start.hex(),
            "auto_stop_scheduled": True,
        }


