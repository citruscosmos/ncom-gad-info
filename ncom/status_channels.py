"""Status-channel decoders for NCOM batch S."""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass

from .constants import ACCURACY_AGE_INVALID, ORI_ACC_SCALE, POS_ACC_SCALE, VEL_ACC_SCALE


def decode_innovation(raw_byte: int) -> tuple[bool, float]:
    """Decode channel-95 innovation byte into (is_valid, sigma)."""
    is_valid = bool(raw_byte & 0x01)
    raw_value = raw_byte >> 1
    if raw_value >= 64:  # 7-bit two's complement
        raw_value -= 128
    return is_valid, raw_value * 0.1


@dataclass(slots=True)
class GadStreamInfo:
    """Last reported state for one GAD stream."""

    stream_id: int
    last_seen: float
    rejection_count: int
    innovations: list[float | None]
    timestamp_ms: int

    @property
    def status(self) -> str:
        age = time.monotonic() - self.last_seen
        if age > 5.0:
            return "Timeout"
        if self.rejection_count > 10:
            return "Rejected"
        if self.rejection_count > 0:
            return "Partially Rejected"
        return "Active"

    @property
    def innovation_quality(self) -> str:
        valid_values = [abs(v) for v in self.innovations if v is not None]
        if not valid_values:
            return "N/A"
        max_inn = max(valid_values)
        if max_inn <= 1.0:
            return "Good"
        if max_inn <= 3.5:
            return "Fair"
        return "Poor"


class StatusChannelDecoder:
    """Dispatcher/holder for decoded status channels."""

    def __init__(self) -> None:
        self.velocity_accuracy = {"north": None, "east": None, "down": None}
        self.velocity_accuracy_age = 255

        self.position_accuracy = {"north": None, "east": None, "down": None}
        self.position_accuracy_age = 255

        self.orientation_accuracy = {"heading": None, "pitch": None, "roll": None}
        self.orientation_accuracy_age = 255

        self.gnss_pos_mode: int | None = None
        self.gnss_vel_mode: int | None = None
        self.gnss_att_mode: int | None = None
        self.num_satellites: int | None = None

        self.can_status = {
            "tx_count": 0,
            "rx_count": 0,
            "tx_ok_pct": None,
            "rx_ok_pct": None,
            "error_count": 0,
            "last_error_code": 0,
        }

        self.gad_streams: dict[int, GadStreamInfo] = {}

    def decode(self, channel: int, data: bytes) -> None:
        handler = self._handlers.get(channel)
        if handler is not None:
            handler(self, data)

    def is_known_channel(self, channel: int) -> bool:
        return channel in self._handlers

    def summarize_channel(self, channel: int, data: bytes) -> dict[str, object]:
        summary: dict[str, object] = {
            "known_channel": self.is_known_channel(channel),
            "status_channel": channel,
            "status_data_hex": data.hex(),
        }
        if channel == 0:
            summary.update(
                {
                    "num_satellites": self.num_satellites,
                    "gnss_pos_mode": self.gnss_pos_mode,
                    "gnss_vel_mode": self.gnss_vel_mode,
                    "gnss_att_mode": self.gnss_att_mode,
                }
            )
        elif channel == 3:
            summary.update(
                {
                    "position_accuracy_age": self.position_accuracy_age,
                    "position_accuracy": dict(self.position_accuracy),
                }
            )
        elif channel == 4:
            summary.update(
                {
                    "velocity_accuracy_age": self.velocity_accuracy_age,
                    "velocity_accuracy": dict(self.velocity_accuracy),
                }
            )
        elif channel == 5:
            summary.update(
                {
                    "orientation_accuracy_age": self.orientation_accuracy_age,
                    "orientation_accuracy": dict(self.orientation_accuracy),
                }
            )
        elif channel == 78:
            summary.update({"can_status": dict(self.can_status)})
        elif channel == 95:
            stream_id = data[0]
            stream = self.gad_streams.get(stream_id)
            summary.update(
                {
                    "stream_id": stream_id,
                    "rejection_count": None if stream is None else stream.rejection_count,
                    "innovations": None if stream is None else list(stream.innovations),
                    "gad_timestamp_ms": None if stream is None else stream.timestamp_ms,
                }
            )
        return summary

    def _decode_ch0_gnss_mode(self, data: bytes) -> None:
        num_sats = data[4]
        pos_mode = data[5]
        vel_mode = data[6]
        att_mode = data[7]
        self.num_satellites = num_sats if num_sats != 255 else None
        self.gnss_pos_mode = pos_mode if pos_mode != 255 else None
        self.gnss_vel_mode = vel_mode if vel_mode != 255 else None
        self.gnss_att_mode = att_mode if att_mode != 255 else None

    def _decode_ch3_position_accuracy(self, data: bytes) -> None:
        age = data[6]
        self.position_accuracy_age = age
        if age >= ACCURACY_AGE_INVALID:
            return
        self.position_accuracy["north"] = struct.unpack_from("<H", data, 0)[0] * POS_ACC_SCALE
        self.position_accuracy["east"] = struct.unpack_from("<H", data, 2)[0] * POS_ACC_SCALE
        self.position_accuracy["down"] = struct.unpack_from("<H", data, 4)[0] * POS_ACC_SCALE

    def _decode_ch4_velocity_accuracy(self, data: bytes) -> None:
        age = data[6]
        self.velocity_accuracy_age = age
        if age >= ACCURACY_AGE_INVALID:
            return
        self.velocity_accuracy["north"] = struct.unpack_from("<H", data, 0)[0] * VEL_ACC_SCALE
        self.velocity_accuracy["east"] = struct.unpack_from("<H", data, 2)[0] * VEL_ACC_SCALE
        self.velocity_accuracy["down"] = struct.unpack_from("<H", data, 4)[0] * VEL_ACC_SCALE

    def _decode_ch5_orientation_accuracy(self, data: bytes) -> None:
        age = data[6]
        self.orientation_accuracy_age = age
        if age >= ACCURACY_AGE_INVALID:
            return
        self.orientation_accuracy["heading"] = struct.unpack_from("<H", data, 0)[0] * ORI_ACC_SCALE
        self.orientation_accuracy["pitch"] = struct.unpack_from("<H", data, 2)[0] * ORI_ACC_SCALE
        self.orientation_accuracy["roll"] = struct.unpack_from("<H", data, 4)[0] * ORI_ACC_SCALE

    def _decode_ch78_can_status(self, data: bytes) -> None:
        self.can_status["tx_count"] = struct.unpack_from("<H", data, 0)[0]
        self.can_status["rx_count"] = struct.unpack_from("<H", data, 2)[0]
        self.can_status["tx_ok_pct"] = data[4] if data[4] != 0xFF else None
        self.can_status["rx_ok_pct"] = data[5] if data[5] != 0xFF else None
        self.can_status["error_count"] = data[6]
        self.can_status["last_error_code"] = data[7]

    def _decode_ch95_gad_info(self, data: bytes) -> None:
        stream_id = data[0]
        if stream_id == 0:
            return

        rejection_count = data[1] if data[1] != 0xFF else 0
        i1_ok, i1 = decode_innovation(data[2])
        i2_ok, i2 = decode_innovation(data[3])
        i3_ok, i3 = decode_innovation(data[4])
        timestamp_ms = struct.unpack_from("<H", data, 5)[0]
        self.gad_streams[stream_id] = GadStreamInfo(
            stream_id=stream_id,
            last_seen=time.monotonic(),
            rejection_count=rejection_count,
            innovations=[i1 if i1_ok else None, i2 if i2_ok else None, i3 if i3_ok else None],
            timestamp_ms=timestamp_ms if timestamp_ms < 60000 else 0,
        )

    _handlers = {
        0: _decode_ch0_gnss_mode,
        3: _decode_ch3_position_accuracy,
        4: _decode_ch4_velocity_accuracy,
        5: _decode_ch5_orientation_accuracy,
        78: _decode_ch78_can_status,
        95: _decode_ch95_gad_info,
    }
