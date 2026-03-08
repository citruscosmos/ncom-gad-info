"""UDP receiver thread for NCOM stream ingestion."""

from __future__ import annotations

import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field

from ncom.constants import GNSS_MODE
from ncom.decoder import NcomDecoder
from ncom.status_channels import GadStreamInfo


@dataclass
class ReceiverState:
    """Thread-shared state, guarded by lock."""

    lock: threading.Lock = field(default_factory=threading.Lock)

    vel_acc_north: float | None = None
    vel_acc_east: float | None = None
    vel_acc_down: float | None = None
    vel_acc_age: int = 255
    accuracy_history: deque = field(default_factory=lambda: deque(maxlen=60))

    gnss_pos_mode: int | None = None
    gnss_vel_mode: int | None = None
    gnss_pos_mode_name: str = "Unknown"
    gnss_vel_mode_name: str = "Unknown"
    num_satellites: int | None = None

    gad_streams: dict[int, GadStreamInfo] = field(default_factory=dict)
    can_status: dict = field(default_factory=dict)

    nav_status: str = "Unknown"
    nav_status_code: int = 0

    packet_count: int = 0
    error_count: int = 0
    last_packet_time: float = 0.0
    last_error: str = ""

    is_running: bool = False


class NcomUdpReceiver(threading.Thread):
    """Receive/decode NCOM packets in a background daemon."""

    def __init__(self, state: ReceiverState, host: str = "", port: int = 3000) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.host = host
        self.port = port
        self._stop_event = threading.Event()
        self._decoder = NcomDecoder()

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            sock.bind((self.host, self.port))
            sock.settimeout(1.0)
        except OSError as exc:
            with self.state.lock:
                self.state.is_running = False
                self.state.last_error = f"UDP bind failed on port {self.port}: {exc}"
            sock.close()
            return

        with self.state.lock:
            self.state.is_running = True
            self.state.last_error = ""

        try:
            while not self._stop_event.is_set():
                try:
                    data, _ = sock.recvfrom(256)
                except socket.timeout:
                    continue
                except OSError as exc:
                    with self.state.lock:
                        self.state.last_error = f"Socket receive error: {exc}"
                    continue

                packet = self._decoder.decode(data)
                if packet is None:
                    with self.state.lock:
                        self.state.error_count += 1
                    continue

                now = time.monotonic()
                sd = self._decoder.status_decoder
                with self.state.lock:
                    self.state.nav_status = packet.nav_status_name
                    self.state.nav_status_code = packet.nav_status
                    self.state.packet_count += 1
                    self.state.last_packet_time = now

                    va = sd.velocity_accuracy
                    if va["north"] is not None:
                        prev_north = self.state.vel_acc_north
                        self.state.vel_acc_north = va["north"]
                        self.state.vel_acc_east = va["east"]
                        self.state.vel_acc_down = va["down"]
                        self.state.vel_acc_age = sd.velocity_accuracy_age
                        if prev_north != va["north"]:
                            self.state.accuracy_history.append((now, va["north"], va["east"], va["down"]))

                    if sd.gnss_pos_mode is not None:
                        self.state.gnss_pos_mode = sd.gnss_pos_mode
                        self.state.gnss_vel_mode = sd.gnss_vel_mode
                        self.state.gnss_pos_mode_name = GNSS_MODE.get(sd.gnss_pos_mode, f"Unknown({sd.gnss_pos_mode})")
                        self.state.gnss_vel_mode_name = GNSS_MODE.get(sd.gnss_vel_mode, f"Unknown({sd.gnss_vel_mode})")
                        self.state.num_satellites = sd.num_satellites

                    if sd.gad_streams:
                        self.state.gad_streams = dict(sd.gad_streams)
                    if sd.can_status.get("tx_ok_pct") is not None:
                        self.state.can_status = dict(sd.can_status)
        finally:
            sock.close()
            with self.state.lock:
                self.state.is_running = False

    def stop(self) -> None:
        self._stop_event.set()
