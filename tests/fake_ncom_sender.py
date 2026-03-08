"""Send synthetic NCOM packets over UDP for dashboard testing."""

from __future__ import annotations

import argparse
import math
import socket
import struct
import time


def _encode_int24(value: int) -> bytes:
    if value < 0:
        value += 1 << 24
    return value.to_bytes(3, byteorder="little", signed=False)


def _set_checksums(packet: bytearray) -> None:
    packet[22] = sum(packet[1:22]) & 0xFF
    packet[61] = sum(packet[1:61]) & 0xFF
    packet[71] = sum(packet[1:71]) & 0xFF


def _build_packet(seq: int, status_channel: int, status_data: bytes) -> bytes:
    packet = bytearray(72)
    packet[0] = 0xE7
    struct.pack_into("<H", packet, 1, seq % 60000)

    packet[3:6] = _encode_int24(200)
    packet[6:9] = _encode_int24(-150)
    packet[9:12] = _encode_int24(1000)
    packet[12:15] = _encode_int24(20)
    packet[15:18] = _encode_int24(-10)
    packet[18:21] = _encode_int24(5)
    packet[21] = 4

    struct.pack_into("<d", packet, 23, math.radians(35.0))
    struct.pack_into("<d", packet, 31, math.radians(139.0))
    struct.pack_into("<f", packet, 39, 20.0)

    packet[43:46] = _encode_int24(1200)
    packet[46:49] = _encode_int24(10)
    packet[49:52] = _encode_int24(-30)
    packet[52:55] = _encode_int24(0)
    packet[55:58] = _encode_int24(0)
    packet[58:61] = _encode_int24(0)

    packet[62] = status_channel
    packet[63:71] = status_data
    _set_checksums(packet)
    return bytes(packet)


def _make_ch0_data(seq: int) -> bytes:
    gps_min = 1000 + (seq // 100)
    num_sats = 12
    pos_mode = 3 if (seq // 300) % 2 == 0 else 5
    vel_mode = 3 if (seq // 300) % 2 == 0 else 5
    att_mode = 255
    return struct.pack("<IBBBB", gps_min, num_sats, pos_mode, vel_mode, att_mode)


def _make_ch4_data(seq: int) -> bytes:
    base = 30 + 20 * math.sin(seq / 80.0)
    north_mmps = int(max(10, min(50, base)))
    east_mmps = int(max(10, min(50, base * 0.8)))
    down_mmps = int(max(10, min(50, base * 1.2)))
    age = 5
    blended = 1
    return struct.pack("<HHHBB", north_mmps, east_mmps, down_mmps, age, blended)


def _make_ch78_data(seq: int) -> bytes:
    tx = seq % 65536
    rx = (seq - 1) % 65536
    tx_ok = 100
    rx_ok = 100 if (seq // 500) % 2 == 0 else 98
    errors = 0 if (seq // 500) % 2 == 0 else 3
    last_error = 0 if errors == 0 else 42
    return struct.pack("<HHBBBB", tx, rx, tx_ok, rx_ok, errors, last_error)


def _make_ch95_data(seq: int) -> bytes:
    stream_id = 1
    # 20s active, 10s timeout by sending invalid stream id
    if (seq // 1000) % 3 == 2:
        stream_id = 0
    rejection = 0 if (seq // 300) % 2 == 0 else 2

    def inn_byte(sigma_tenths: int, valid: bool = True) -> int:
        val = sigma_tenths & 0x7F
        return (val << 1) | (1 if valid else 0)

    inn1 = inn_byte(3, True)
    inn2 = inn_byte(10 if rejection == 0 else 20, True)
    inn3 = inn_byte((-5) & 0x7F, True)
    ms = seq % 60000
    return bytes([stream_id, rejection, inn1, inn2, inn3]) + struct.pack("<H", ms) + bytes([0])


def main() -> None:
    parser = argparse.ArgumentParser(description="Fake NCOM UDP sender")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=3000)
    parser.add_argument("--hz", type=float, default=100.0)
    args = parser.parse_args()

    channels = [0, 4, 78, 95]
    interval = 1.0 / args.hz
    seq = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Sending fake NCOM to {args.host}:{args.port} at {args.hz:.1f} Hz")
    try:
        while True:
            ch = channels[seq % len(channels)]
            if ch == 0:
                data = _make_ch0_data(seq)
            elif ch == 4:
                data = _make_ch4_data(seq)
            elif ch == 78:
                data = _make_ch78_data(seq)
            else:
                data = _make_ch95_data(seq)

            packet = _build_packet(seq, ch, data)
            sock.sendto(packet, (args.host, args.port))
            seq += 1
            time.sleep(interval)
    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
