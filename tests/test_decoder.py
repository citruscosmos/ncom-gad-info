"""Unit tests for NCOM decoder and status channels."""

from __future__ import annotations

import math
import struct

from ncom.decoder import NcomDecoder, decode_int24_le, verify_checksum
from ncom.status_channels import decode_innovation


def _encode_int24(value: int) -> bytes:
    if value < 0:
        value += 1 << 24
    return value.to_bytes(3, byteorder="little", signed=False)


def _set_checksums(packet: bytearray) -> None:
    packet[22] = sum(packet[1:22]) & 0xFF
    packet[61] = sum(packet[1:61]) & 0xFF
    packet[71] = sum(packet[1:71]) & 0xFF


def _make_packet(status_channel: int = 4, status_data: bytes | None = None, nav_status: int = 4) -> bytes:
    if status_data is None:
        status_data = b"\x00" * 8
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


def test_decode_int24_le() -> None:
    assert decode_int24_le(bytes.fromhex("000000")) == 0
    assert decode_int24_le(bytes.fromhex("ffff7f")) == 0x7FFFFF
    assert decode_int24_le(bytes.fromhex("010080")) == -0x7FFFFF
    assert decode_int24_le(bytes.fromhex("ffffff")) == -1


def test_verify_checksum() -> None:
    packet = _make_packet()
    assert verify_checksum(packet, 1)
    assert verify_checksum(packet, 2)
    assert verify_checksum(packet, 3)

    broken = bytearray(packet)
    broken[40] ^= 0xFF
    assert not verify_checksum(bytes(broken), 2)
    assert not verify_checksum(bytes(broken), 3)


def test_decode_innovation() -> None:
    valid, sigma = decode_innovation(0b00010101)  # valid, +1.0 sigma
    assert valid
    assert sigma == 1.0

    valid, sigma = decode_innovation(0b11110011)  # valid, negative value
    assert valid
    assert sigma < 0

    valid, sigma = decode_innovation(0b00010100)  # invalid bit
    assert not valid
    assert sigma == 1.0


def test_ncom_decoder_decode_full_packet() -> None:
    status_data = struct.pack("<HHHBB", 10, 20, 30, 2, 0)
    decoder = NcomDecoder()
    packet = decoder.decode(_make_packet(status_channel=4, status_data=status_data))
    assert packet is not None
    assert packet.nav_status == 4
    assert packet.nav_status_name == "Real-Time"
    assert packet.timestamp_ms == 12345
    assert packet.checksum_valid == (True, True, True)
    assert math.isclose(packet.vel_north, 0.4, rel_tol=0, abs_tol=1e-9)
    assert math.isclose(packet.vel_east, -0.5, rel_tol=0, abs_tol=1e-9)
    assert math.isclose(packet.vel_down, 0.6, rel_tol=0, abs_tol=1e-9)


def test_decode_ch4_velocity_accuracy() -> None:
    status_data = struct.pack("<HHHBB", 12, 34, 56, 10, 1)
    decoder = NcomDecoder()
    packet = decoder.decode(_make_packet(status_channel=4, status_data=status_data))
    assert packet is not None

    va = decoder.status_decoder.velocity_accuracy
    assert math.isclose(va["north"], 0.012, rel_tol=0, abs_tol=1e-12)
    assert math.isclose(va["east"], 0.034, rel_tol=0, abs_tol=1e-12)
    assert math.isclose(va["down"], 0.056, rel_tol=0, abs_tol=1e-12)
    assert decoder.status_decoder.velocity_accuracy_age == 10


def test_decode_ch95_gad_info() -> None:
    # stream_id=1, rejection=2, innovation bytes valid, timestamp=3456
    status_data = bytes([1, 2, 0b00000111, 0b00001101, 0b00000011, 0x80, 0x0D, 0])
    decoder = NcomDecoder()
    packet = decoder.decode(_make_packet(status_channel=95, status_data=status_data))
    assert packet is not None

    gad = decoder.status_decoder.gad_streams
    assert 1 in gad
    info = gad[1]
    assert info.rejection_count == 2
    assert info.timestamp_ms == 3456
    assert info.innovations[0] is not None


def test_decode_ch78_can_status() -> None:
    status_data = struct.pack("<HHBBBB", 100, 200, 98, 97, 3, 42)
    decoder = NcomDecoder()
    packet = decoder.decode(_make_packet(status_channel=78, status_data=status_data))
    assert packet is not None

    can = decoder.status_decoder.can_status
    assert can["tx_count"] == 100
    assert can["rx_count"] == 200
    assert can["tx_ok_pct"] == 98
    assert can["rx_ok_pct"] == 97
    assert can["error_count"] == 3
    assert can["last_error_code"] == 42


def test_navstatus_11_is_ignored() -> None:
    decoder = NcomDecoder()
    packet = decoder.decode(_make_packet(nav_status=11))
    assert packet is None
