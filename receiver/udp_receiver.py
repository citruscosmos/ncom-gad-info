"""UDP receiver thread for NCOM stream ingestion."""

from __future__ import annotations

import math
import socket
import threading
import time
from collections import deque
from dataclasses import asdict
from dataclasses import dataclass, field

from ncom.constants import GNSS_MODE
from ncom.decoder import DecodeDiagnostics, NcomDecoder
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

    pos_acc_north: float | None = None
    pos_acc_east: float | None = None
    pos_acc_down: float | None = None
    pos_acc_age: int = 255

    ori_acc_heading: float | None = None
    ori_acc_pitch: float | None = None
    ori_acc_roll: float | None = None
    ori_acc_age: int = 255
    pos_acc_history: deque = field(default_factory=lambda: deque(maxlen=300))
    ori_acc_history: deque = field(default_factory=lambda: deque(maxlen=300))

    vel_north: float | None = None
    vel_east: float | None = None
    vel_down: float | None = None
    heading: float | None = None
    pitch: float | None = None
    roll: float | None = None
    velocity_history: deque = field(default_factory=lambda: deque(maxlen=600))
    attitude_history: deque = field(default_factory=lambda: deque(maxlen=600))

    inn_pos_x: float | None = None
    inn_pos_y: float | None = None
    inn_pos_z: float | None = None
    inn_vel_x: float | None = None
    inn_vel_y: float | None = None
    inn_vel_z: float | None = None
    inn_pitch: float | None = None
    inn_heading: float | None = None
    inn_zero_vel_x: float | None = None
    inn_zero_vel_y: float | None = None
    inn_zero_vel_z: float | None = None
    inn_no_slip_h: float | None = None
    inn_heading_h: float | None = None
    inn_wspeed: float | None = None
    inn_set3: dict[str, float | None] = field(
        default_factory=lambda: {"vertical_advanced_slip": None, **{f"reserved_{i}": None for i in range(1, 8)}}
    )
    inn_pos_history: deque = field(default_factory=lambda: deque(maxlen=600))
    inn_vel_history: deque = field(default_factory=lambda: deque(maxlen=600))
    inn_att_history: deque = field(default_factory=lambda: deque(maxlen=600))

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
    source_mode: str = "udp"

    latitude_deg: float | None = None
    longitude_deg: float | None = None
    altitude_m: float | None = None
    position_history: deque = field(default_factory=lambda: deque(maxlen=300))
    position_value_history: deque = field(default_factory=lambda: deque(maxlen=600))
    playback_total_packets: int = 0
    playback_current_packet: int = 0
    diagnostics_enabled: bool = False
    diagnostics_buffer: deque = field(default_factory=lambda: deque(maxlen=200))
    diagnostics_warning_count: int = 0
    is_paused: bool = False

    is_running: bool = False


def reset_receiver_state(state: ReceiverState) -> None:
    """Reset decoded telemetry values while keeping lock/source metadata."""
    state.vel_acc_north = None
    state.vel_acc_east = None
    state.vel_acc_down = None
    state.vel_acc_age = 255
    state.accuracy_history.clear()

    state.pos_acc_north = None
    state.pos_acc_east = None
    state.pos_acc_down = None
    state.pos_acc_age = 255
    state.pos_acc_history.clear()

    state.ori_acc_heading = None
    state.ori_acc_pitch = None
    state.ori_acc_roll = None
    state.ori_acc_age = 255
    state.ori_acc_history.clear()

    state.vel_north = None
    state.vel_east = None
    state.vel_down = None
    state.heading = None
    state.pitch = None
    state.roll = None
    state.velocity_history.clear()
    state.attitude_history.clear()

    state.inn_pos_x = None
    state.inn_pos_y = None
    state.inn_pos_z = None
    state.inn_vel_x = None
    state.inn_vel_y = None
    state.inn_vel_z = None
    state.inn_pitch = None
    state.inn_heading = None
    state.inn_zero_vel_x = None
    state.inn_zero_vel_y = None
    state.inn_zero_vel_z = None
    state.inn_no_slip_h = None
    state.inn_heading_h = None
    state.inn_wspeed = None
    for key in state.inn_set3:
        state.inn_set3[key] = None
    state.inn_pos_history.clear()
    state.inn_vel_history.clear()
    state.inn_att_history.clear()

    state.gnss_pos_mode = None
    state.gnss_vel_mode = None
    state.gnss_pos_mode_name = "Unknown"
    state.gnss_vel_mode_name = "Unknown"
    state.num_satellites = None

    state.gad_streams.clear()
    state.can_status.clear()
    state.nav_status = "Unknown"
    state.nav_status_code = 0
    state.packet_count = 0
    state.error_count = 0
    state.last_packet_time = 0.0
    state.latitude_deg = None
    state.longitude_deg = None
    state.altitude_m = None
    state.position_history.clear()
    state.position_value_history.clear()
    state.playback_current_packet = 0
    state.playback_total_packets = 0
    state.diagnostics_buffer.clear()
    state.diagnostics_warning_count = 0
    state.is_paused = False


def append_decode_diagnostics(
    state: ReceiverState,
    diag: DecodeDiagnostics,
    replay_packet_index: int | None = None,
) -> None:
    """Record one decode diagnostics entry into the shared ring buffer."""
    if not state.diagnostics_enabled:
        return

    record = asdict(diag)
    if replay_packet_index is not None:
        record["replay_packet_index"] = replay_packet_index

    status_ch = record.get("status_channel")
    status_summary = record.get("status_summary", {})
    if status_ch in (3, 4, 5):
        age_key = {
            3: "position_accuracy_age",
            4: "velocity_accuracy_age",
            5: "orientation_accuracy_age",
        }[status_ch]
        age = status_summary.get(age_key)
        if isinstance(age, int) and age >= 150:
            record["warnings"].append("stale_accuracy")

    state.diagnostics_buffer.append(record)
    if record["warnings"]:
        state.diagnostics_warning_count += 1


def apply_decoder_snapshot(
    state: ReceiverState,
    packet,
    status_decoder,
    now: float,
) -> None:
    """Copy decoder output into shared receiver state."""
    state.nav_status = packet.nav_status_name
    state.nav_status_code = packet.nav_status
    state.packet_count += 1
    state.last_packet_time = now
    state.vel_north = packet.vel_north
    state.vel_east = packet.vel_east
    state.vel_down = packet.vel_down
    state.heading = packet.heading
    state.pitch = packet.pitch
    state.roll = packet.roll
    state.altitude_m = packet.altitude
    state.velocity_history.append((now, packet.vel_north, packet.vel_east, packet.vel_down))
    state.attitude_history.append((now, packet.heading, packet.pitch, packet.roll))

    va = status_decoder.velocity_accuracy
    if va["north"] is not None:
        prev_north = state.vel_acc_north
        state.vel_acc_north = va["north"]
        state.vel_acc_east = va["east"]
        state.vel_acc_down = va["down"]
        state.vel_acc_age = status_decoder.velocity_accuracy_age
        if prev_north != va["north"]:
            state.accuracy_history.append((now, va["north"], va["east"], va["down"]))

    pa = status_decoder.position_accuracy
    if pa["north"] is not None:
        state.pos_acc_north = pa["north"]
        state.pos_acc_east = pa["east"]
        state.pos_acc_down = pa["down"]
        state.pos_acc_age = status_decoder.position_accuracy_age
        state.pos_acc_history.append((now, pa["north"], pa["east"], pa["down"]))

    oa = status_decoder.orientation_accuracy
    if oa["heading"] is not None:
        state.ori_acc_heading = oa["heading"]
        state.ori_acc_pitch = oa["pitch"]
        state.ori_acc_roll = oa["roll"]
        state.ori_acc_age = status_decoder.orientation_accuracy_age
        state.ori_acc_history.append((now, oa["heading"], oa["pitch"], oa["roll"]))

    inns1 = status_decoder.kf_innovations_set1
    state.inn_pos_x = inns1["pos_x"]
    state.inn_pos_y = inns1["pos_y"]
    state.inn_pos_z = inns1["pos_z"]
    state.inn_vel_x = inns1["vel_x"]
    state.inn_vel_y = inns1["vel_y"]
    state.inn_vel_z = inns1["vel_z"]
    state.inn_pitch = inns1["pitch"]
    state.inn_heading = inns1["heading"]
    if any(v is not None for v in (inns1["pos_x"], inns1["pos_y"], inns1["pos_z"])):
        state.inn_pos_history.append((now, inns1["pos_x"], inns1["pos_y"], inns1["pos_z"]))
    if any(v is not None for v in (inns1["vel_x"], inns1["vel_y"], inns1["vel_z"])):
        state.inn_vel_history.append((now, inns1["vel_x"], inns1["vel_y"], inns1["vel_z"]))
    if any(v is not None for v in (inns1["pitch"], inns1["heading"])):
        state.inn_att_history.append((now, inns1["pitch"], inns1["heading"]))

    inns2 = status_decoder.kf_innovations_set2
    state.inn_zero_vel_x = inns2["zero_vel_x"]
    state.inn_zero_vel_y = inns2["zero_vel_y"]
    state.inn_zero_vel_z = inns2["zero_vel_z"]
    state.inn_no_slip_h = inns2["no_slip_h"]
    state.inn_heading_h = inns2["heading_lock"]
    state.inn_wspeed = inns2["wheel_speed"]

    state.inn_set3 = dict(status_decoder.kf_innovations_set3)

    lat_deg = math.degrees(packet.latitude)
    lon_deg = math.degrees(packet.longitude)
    if (
        math.isfinite(lat_deg)
        and math.isfinite(lon_deg)
        and -90.0 <= lat_deg <= 90.0
        and -180.0 <= lon_deg <= 180.0
    ):
        state.latitude_deg = lat_deg
        state.longitude_deg = lon_deg
        if not state.position_history or state.position_history[-1] != (lat_deg, lon_deg):
            state.position_history.append((lat_deg, lon_deg))
        state.position_value_history.append((now, lat_deg, lon_deg, packet.altitude))

    if status_decoder.gnss_pos_mode is not None:
        state.gnss_pos_mode = status_decoder.gnss_pos_mode
        state.gnss_vel_mode = status_decoder.gnss_vel_mode
        state.gnss_pos_mode_name = GNSS_MODE.get(
            status_decoder.gnss_pos_mode,
            f"Unknown({status_decoder.gnss_pos_mode})",
        )
        state.gnss_vel_mode_name = GNSS_MODE.get(
            status_decoder.gnss_vel_mode,
            f"Unknown({status_decoder.gnss_vel_mode})",
        )
        state.num_satellites = status_decoder.num_satellites

    if status_decoder.gad_streams:
        state.gad_streams = dict(status_decoder.gad_streams)
    if status_decoder.can_status.get("tx_ok_pct") is not None:
        state.can_status = dict(status_decoder.can_status)


class NcomUdpReceiver(threading.Thread):
    """Receive/decode NCOM packets in a background daemon."""

    def __init__(self, state: ReceiverState, host: str = "", port: int = 3000) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.host = host
        self.port = port
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
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
            self.state.source_mode = "udp"

        try:
            while not self._stop_event.is_set():
                if self._pause_event.is_set():
                    with self.state.lock:
                        self.state.is_paused = True
                    time.sleep(0.05)
                    continue
                with self.state.lock:
                    self.state.is_paused = False
                try:
                    data, _ = sock.recvfrom(256)
                except socket.timeout:
                    continue
                except OSError as exc:
                    with self.state.lock:
                        self.state.last_error = f"Socket receive error: {exc}"
                    continue

                packet, diag = self._decoder.decode_with_diagnostics(data)
                with self.state.lock:
                    append_decode_diagnostics(self.state, diag)
                if packet is None:
                    with self.state.lock:
                        self.state.error_count += 1
                    continue

                now = time.monotonic()
                sd = self._decoder.status_decoder
                with self.state.lock:
                    apply_decoder_snapshot(self.state, packet, sd, now)
        finally:
            sock.close()
            with self.state.lock:
                self.state.is_running = False
                self.state.is_paused = False

    def stop(self) -> None:
        self._stop_event.set()

    def pause(self) -> None:
        self._pause_event.set()

    def resume(self) -> None:
        self._pause_event.clear()
