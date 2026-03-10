"""NCOM binary file replay thread."""

from __future__ import annotations

import threading
import time
from pathlib import Path

from ncom.constants import NCOM_PACKET_LENGTH, NCOM_SYNC
from ncom.decoder import NcomDecoder
from receiver.udp_receiver import ReceiverState, append_decode_diagnostics, apply_decoder_snapshot


def extract_ncom_packets(raw_bytes: bytes) -> list[bytes]:
    """Extract candidate 72-byte NCOM packets with sync recovery."""
    packets: list[bytes] = []
    idx = 0
    size = len(raw_bytes)
    while idx + NCOM_PACKET_LENGTH <= size:
        if raw_bytes[idx] == NCOM_SYNC:
            packets.append(raw_bytes[idx : idx + NCOM_PACKET_LENGTH])
            idx += NCOM_PACKET_LENGTH
        else:
            idx += 1
    return packets


class NcomFileReplay(threading.Thread):
    """Replay NCOM packets from recorded binary files."""

    def __init__(
        self,
        state: ReceiverState,
        file_path: str,
        replay_hz: float = 100.0,
        speed: float = 1.0,
        loop: bool = False,
        start_index: int = 0,
    ) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.file_path = file_path
        self.replay_hz = replay_hz
        self.speed = speed
        self.loop = loop
        self.start_index = start_index
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._decoder = NcomDecoder()

    def run(self) -> None:
        path = Path(self.file_path)
        try:
            raw_bytes = path.read_bytes()
        except OSError as exc:
            with self.state.lock:
                self.state.last_error = f"Playback file read failed: {exc}"
                self.state.is_running = False
            return

        packets = extract_ncom_packets(raw_bytes)
        if not packets:
            with self.state.lock:
                self.state.last_error = "Playback file does not contain valid NCOM packets."
                self.state.is_running = False
            return

        first_index = max(0, min(self.start_index, len(packets) - 1))
        sleep_sec = 1.0 / max(self.replay_hz * self.speed, 1e-6)
        with self.state.lock:
            self.state.is_running = True
            self.state.is_paused = False
            self.state.last_error = ""
            self.state.source_mode = "playback"
            self.state.playback_total_packets = len(packets)
            self.state.playback_current_packet = first_index

        try:
            while not self._stop_event.is_set():
                for packet_index in range(first_index, len(packets)):
                    if self._stop_event.is_set():
                        break
                    if self._pause_event.is_set():
                        with self.state.lock:
                            self.state.is_paused = True
                        time.sleep(0.05)
                        continue
                    with self.state.lock:
                        self.state.is_paused = False
                    raw = packets[packet_index]
                    packet, diag = self._decoder.decode_with_diagnostics(raw)
                    with self.state.lock:
                        append_decode_diagnostics(self.state, diag, replay_packet_index=packet_index)
                    if packet is None:
                        with self.state.lock:
                            self.state.error_count += 1
                        continue

                    now = time.monotonic()
                    with self.state.lock:
                        apply_decoder_snapshot(self.state, packet, self._decoder.status_decoder, now)
                        self.state.playback_current_packet = packet_index
                    time.sleep(sleep_sec)

                if not self.loop:
                    break
                first_index = 0
        finally:
            with self.state.lock:
                self.state.is_running = False
                self.state.is_paused = False

    def stop(self) -> None:
        self._stop_event.set()

    def pause(self) -> None:
        self._pause_event.set()

    def resume(self) -> None:
        self._pause_event.clear()
