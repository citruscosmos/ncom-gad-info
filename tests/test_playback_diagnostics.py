"""Tests for playback binary diagnostics collection."""

from __future__ import annotations

import math
import struct
import tempfile
from pathlib import Path

from receiver.file_replay import NcomFileReplay
from receiver.udp_receiver import ReceiverState


def _encode_int24(value: int) -> bytes:
    if value < 0:
        value += 1 << 24
    return value.to_bytes(3, byteorder="little", signed=False)


def _set_checksums(packet: bytearray) -> None:
    packet[22] = sum(packet[1:22]) & 0xFF
    packet[61] = sum(packet[1:61]) & 0xFF
    packet[71] = sum(packet[1:71]) & 0xFF


def _make_packet(status_channel: int, status_data: bytes, nav_status: int = 4) -> bytes:
    packet = bytearray(72)
    packet[0] = 0xE7
    struct.pack_into("<H", packet, 1, 12345)
    packet[3:6] = _encode_int24(1000)
    packet[6:9] = _encode_int24(-2000)
    packet[9:12] = _encode_int24(3000)
    packet[12:15] = _encode_int24(100)
    packet[15:18] = _encode_int24(-200)
    packet[18:21] = _encode_int24(300)
    packet[21] = nav_status
    struct.pack_into("<d", packet, 23, math.radians(35.0))
    struct.pack_into("<d", packet, 31, math.radians(139.0))
    struct.pack_into("<f", packet, 39, 42.5)
    packet[43:46] = _encode_int24(4000)
    packet[46:49] = _encode_int24(-5000)
    packet[49:52] = _encode_int24(6000)
    packet[52:55] = _encode_int24(700)
    packet[55:58] = _encode_int24(-800)
    packet[58:61] = _encode_int24(900)
    packet[62] = status_channel
    packet[63:71] = status_data
    _set_checksums(packet)
    return bytes(packet)


def test_playback_collects_diagnostics_and_keeps_running() -> None:
    stale_ch4 = struct.pack("<HHHBB", 12, 34, 56, 200, 0)
    good_ch4 = struct.pack("<HHHBB", 12, 34, 56, 10, 0)
    raw = _make_packet(4, stale_ch4) + _make_packet(4, good_ch4)

    with tempfile.NamedTemporaryFile(prefix="diag_", suffix=".ncom", delete=False) as tmp:
        tmp.write(raw)
        tmp_path = Path(tmp.name)

    state = ReceiverState()
    state.diagnostics_enabled = True
    replay = NcomFileReplay(
        state=state,
        file_path=str(tmp_path),
        replay_hz=2000.0,
        speed=10.0,
        loop=False,
        start_index=0,
    )
    replay.start()
    replay.join(timeout=2.0)

    assert not replay.is_alive()
    with state.lock:
        records = list(state.diagnostics_buffer)
        warning_count = state.diagnostics_warning_count
        packet_count = state.packet_count
        error_count = state.error_count

    assert len(records) == 2
    assert packet_count == 2
    assert error_count == 0
    assert warning_count >= 1
    assert any("stale_accuracy" in r.get("warnings", []) for r in records)

    tmp_path.unlink(missing_ok=True)
