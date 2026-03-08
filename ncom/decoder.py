"""Core NCOM packet decoder."""

from __future__ import annotations

import struct
from dataclasses import dataclass

from .constants import ACC2MPS2, ANG2RAD, NAV_STATUS, NCOM_PACKET_LENGTH, NCOM_SYNC, RATE2RPS, VEL2MPS
from .status_channels import StatusChannelDecoder


def decode_int24_le(data: bytes) -> int:
    """Decode signed 24-bit little-endian integer."""
    value = int.from_bytes(data[:3], byteorder="little", signed=False)
    if value >= 0x800000:
        value -= 0x1000000
    return value


def decode_uint24_le(data: bytes) -> int:
    """Decode unsigned 24-bit little-endian integer."""
    return int.from_bytes(data[:3], byteorder="little", signed=False)


def verify_checksum(packet: bytes, batch: int) -> bool:
    """Verify cumulative NCOM checksum for batch 1..3."""
    ranges = {1: (1, 22), 2: (1, 61), 3: (1, 71)}
    positions = {1: 22, 2: 61, 3: 71}
    start, end = ranges[batch]
    expected = sum(packet[start:end]) & 0xFF
    return expected == packet[positions[batch]]


@dataclass(slots=True)
class NcomPacket:
    timestamp_ms: int
    nav_status: int
    nav_status_name: str

    acc_x: float
    acc_y: float
    acc_z: float
    ang_rate_x: float
    ang_rate_y: float
    ang_rate_z: float

    latitude: float
    longitude: float
    altitude: float
    vel_north: float
    vel_east: float
    vel_down: float
    heading: float
    pitch: float
    roll: float

    status_channel: int
    status_data: bytes
    checksum_valid: tuple[bool, bool, bool]


class NcomDecoder:
    """Decode fixed-length NCOM structure-A packets."""

    def __init__(self) -> None:
        self.status_decoder = StatusChannelDecoder()
        self._packet_count = 0
        self._error_count = 0

    @property
    def packet_count(self) -> int:
        return self._packet_count

    @property
    def error_count(self) -> int:
        return self._error_count

    def decode(self, data: bytes) -> NcomPacket | None:
        if len(data) != NCOM_PACKET_LENGTH:
            self._error_count += 1
            return None
        if data[0] != NCOM_SYNC:
            self._error_count += 1
            return None

        nav_status = data[21]
        if nav_status == 11:
            return None

        cs_valid = (
            verify_checksum(data, 1),
            verify_checksum(data, 2),
            verify_checksum(data, 3),
        )

        timestamp_ms = struct.unpack_from("<H", data, 1)[0]
        acc_x = decode_int24_le(data[3:6]) * ACC2MPS2
        acc_y = decode_int24_le(data[6:9]) * ACC2MPS2
        acc_z = decode_int24_le(data[9:12]) * ACC2MPS2
        ang_x = decode_int24_le(data[12:15]) * RATE2RPS
        ang_y = decode_int24_le(data[15:18]) * RATE2RPS
        ang_z = decode_int24_le(data[18:21]) * RATE2RPS

        lat = struct.unpack_from("<d", data, 23)[0]
        lon = struct.unpack_from("<d", data, 31)[0]
        alt = struct.unpack_from("<f", data, 39)[0]
        vel_n = decode_int24_le(data[43:46]) * VEL2MPS
        vel_e = decode_int24_le(data[46:49]) * VEL2MPS
        vel_d = decode_int24_le(data[49:52]) * VEL2MPS
        heading = decode_int24_le(data[52:55]) * ANG2RAD
        pitch = decode_int24_le(data[55:58]) * ANG2RAD
        roll = decode_int24_le(data[58:61]) * ANG2RAD

        status_ch = data[62]
        status_data = data[63:71]
        self.status_decoder.decode(status_ch, status_data)

        self._packet_count += 1
        return NcomPacket(
            timestamp_ms=timestamp_ms,
            nav_status=nav_status,
            nav_status_name=NAV_STATUS.get(nav_status, f"Unknown({nav_status})"),
            acc_x=acc_x,
            acc_y=acc_y,
            acc_z=acc_z,
            ang_rate_x=ang_x,
            ang_rate_y=ang_y,
            ang_rate_z=ang_z,
            latitude=lat,
            longitude=lon,
            altitude=alt,
            vel_north=vel_n,
            vel_east=vel_e,
            vel_down=vel_d,
            heading=heading,
            pitch=pitch,
            roll=roll,
            status_channel=status_ch,
            status_data=status_data,
            checksum_valid=cs_valid,
        )
