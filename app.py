"""Streamlit dashboard for OxTS GAD monitoring."""

from __future__ import annotations

import math
import tempfile
import time
from pathlib import Path

import pandas as pd
import pydeck as pdk
import streamlit as st

from ncom.constants import ACCURACY_AGE_INVALID
from receiver import NcomFileReplay, NcomUdpReceiver, ReceiverState, reset_receiver_state
from receiver.file_replay import extract_ncom_packets


def init_session() -> None:
    if "receiver_state" not in st.session_state:
        st.session_state.receiver_state = ReceiverState()
    if "receiver_thread" not in st.session_state:
        st.session_state.receiver_thread = None
    if "prev_vel_acc" not in st.session_state:
        st.session_state.prev_vel_acc = (None, None, None)
    if "playback_file_path" not in st.session_state:
        st.session_state.playback_file_path = ""
    if "playback_file_name" not in st.session_state:
        st.session_state.playback_file_name = ""
    if "playback_file_token" not in st.session_state:
        st.session_state.playback_file_token = ""
    if "playback_packet_count" not in st.session_state:
        st.session_state.playback_packet_count = 0
    if "playback_seek_percent" not in st.session_state:
        st.session_state.playback_seek_percent = 0
    if "binary_diagnostics_enabled" not in st.session_state:
        st.session_state.binary_diagnostics_enabled = False


def _fmt_mms(value: float | None) -> str:
    return f"{value * 1000:.1f} mm/s" if value is not None else "N/A"


def _delta_mms(current: float | None, previous: float | None) -> str | None:
    if current is None or previous is None:
        return None
    return f"{(current - previous) * 1000:+.1f}"


def _fmt_meter(value: float | None) -> str:
    return f"{value:.3f} m" if value is not None else "N/A"


def _fmt_mrad(value: float | None) -> str:
    return f"{value * 1000:.3f} mrad" if value is not None else "N/A"


def _fmt_sigma(value: float | None) -> str:
    return f"{value:.1f} sigma" if value is not None else "N/A"


def _series_from_history(history: list[tuple], columns: list[str], value_columns: list[str]) -> pd.DataFrame:
    if not history:
        return pd.DataFrame()
    df = pd.DataFrame(history, columns=columns)
    now = time.monotonic()
    df["Seconds Ago"] = now - df["time"]
    df = df[df["Seconds Ago"] <= 60]
    if df.empty:
        return pd.DataFrame()
    return df[["Seconds Ago", *value_columns]].set_index("Seconds Ago")


def _save_uploaded_file(uploaded_file) -> str:
    suffix = Path(uploaded_file.name).suffix or ".ncom"
    with tempfile.NamedTemporaryFile(prefix="ncom_playback_", suffix=suffix, delete=False) as tmp:
        tmp.write(uploaded_file.getbuffer())
        return tmp.name


def _build_dashed_circle_paths(
    lat_deg: float,
    lon_deg: float,
    radius_m: float,
    points: int = 96,
    dash_points: int = 3,
    gap_points: int = 2,
) -> list[dict[str, list[list[float]]]]:
    """Build short path segments to render a dashed circle."""
    if radius_m <= 0:
        return []
    lat_rad = math.radians(lat_deg)
    dlat = radius_m / 111_320.0
    cos_lat = max(abs(math.cos(lat_rad)), 1e-6)
    dlon = radius_m / (111_320.0 * cos_lat)

    ring: list[list[float]] = []
    for i in range(points):
        theta = (2.0 * math.pi * i) / points
        ring.append([lon_deg + dlon * math.cos(theta), lat_deg + dlat * math.sin(theta)])
    ring.append(ring[0])

    paths: list[dict[str, list[list[float]]]] = []
    idx = 0
    while idx < points:
        end = min(idx + dash_points, points)
        segment = ring[idx : end + 1]
        if len(segment) >= 2:
            paths.append({"path": segment})
        idx = end + gap_points
    return paths


@st.fragment(run_every=1.5)
def realtime_metrics() -> None:
    state: ReceiverState = st.session_state.receiver_state
    with state.lock:
        vn = state.vel_north
        ve = state.vel_east
        vd = state.vel_down
        vel_acc = (state.vel_acc_north, state.vel_acc_east, state.vel_acc_down)
        vel_age = state.vel_acc_age
        vel_acc_history = list(state.accuracy_history)
        velocity_history = list(state.velocity_history)
        pos_acc = (state.pos_acc_north, state.pos_acc_east, state.pos_acc_down)
        pos_age = state.pos_acc_age
        pos_acc_history = list(state.pos_acc_history)
        ori_acc = (state.ori_acc_heading, state.ori_acc_pitch, state.ori_acc_roll)
        ori_age = state.ori_acc_age
        ori_acc_history = list(state.ori_acc_history)
        heading = state.heading
        pitch = state.pitch
        roll = state.roll
        gad = dict(state.gad_streams)
        can = dict(state.can_status)
        kf_set2 = {
            "zero_vel_x": state.inn_zero_vel_x,
            "zero_vel_y": state.inn_zero_vel_y,
            "zero_vel_z": state.inn_zero_vel_z,
            "no_slip_h": state.inn_no_slip_h,
            "heading_lock": state.inn_heading_h,
            "wheel_speed": state.inn_wspeed,
        }
        kf_set3 = dict(state.inn_set3)
        inn_pos = (state.inn_pos_x, state.inn_pos_y, state.inn_pos_z)
        inn_vel = (state.inn_vel_x, state.inn_vel_y, state.inn_vel_z)
        inn_att = (state.inn_pitch, state.inn_heading)
        inn_pos_history = list(state.inn_pos_history)
        inn_vel_history = list(state.inn_vel_history)
        inn_att_history = list(state.inn_att_history)
        nav = state.nav_status
        nav_code = state.nav_status_code
        gnss_pos = state.gnss_pos_mode_name
        gnss_vel = state.gnss_vel_mode_name
        sats = state.num_satellites
        packets = state.packet_count
        errors = state.error_count
        last_time = state.last_packet_time
        last_error = state.last_error
        source_mode = state.source_mode
        lat = state.latitude_deg
        lon = state.longitude_deg
        alt = state.altitude_m
        position_history = list(state.position_history)
        position_value_history = list(state.position_value_history)
        playback_total = state.playback_total_packets
        playback_index = state.playback_current_packet
        diagnostics_enabled = state.diagnostics_enabled
        diagnostics_warning_count = state.diagnostics_warning_count
        diagnostics_rows = list(state.diagnostics_buffer)
        is_paused = state.is_paused

    if last_error:
        st.error(last_error)

    if source_mode == "playback" and playback_total > 0:
        progress = min(max((playback_index + 1) / playback_total, 0.0), 1.0)
        st.caption(f"Playback progress: {playback_index + 1}/{playback_total} packets")
        st.progress(progress)

    if last_time > 0 and (time.monotonic() - last_time) < 3.0:
        status_text = (
            f"Nav: **{nav}** | GNSS Vel: **{gnss_vel}** | GNSS Pos: **{gnss_pos}** "
            f"| Sats: **{sats or 'N/A'}** | Source: **{source_mode}** "
            f"| Packets: {packets:,} | Errors: {errors:,}"
        )
        if is_paused:
            status_text += " | State: **Paused**"
        if nav_code == 4:
            st.success(status_text)
        else:
            st.warning(status_text)
    elif last_time > 0:
        st.warning(f"No data for {time.monotonic() - last_time:.0f}s | Last nav: {nav}")
    else:
        st.info("Waiting for NCOM data on UDP...")

    st.subheader("Position")
    pos_current_cols = st.columns(3)
    pos_current_cols[0].metric("Latitude", f"{lat:.6f}" if lat is not None else "N/A")
    pos_current_cols[1].metric("Longitude", f"{lon:.6f}" if lon is not None else "N/A")
    pos_current_cols[2].metric("Altitude", f"{alt:.2f} m" if alt is not None else "N/A")
    pos_acc_cols = st.columns(3)
    pos_acc_cols[0].metric("North sigma", _fmt_meter(pos_acc[0]))
    pos_acc_cols[1].metric("East sigma", _fmt_meter(pos_acc[1]))
    pos_acc_cols[2].metric("Down sigma", _fmt_meter(pos_acc[2]))
    pos_inn_cols = st.columns(3)
    pos_inn_cols[0].metric("Innovation X", _fmt_sigma(inn_pos[0]))
    pos_inn_cols[1].metric("Innovation Y", _fmt_sigma(inn_pos[1]))
    pos_inn_cols[2].metric("Innovation Z", _fmt_sigma(inn_pos[2]))
    if pos_age >= ACCURACY_AGE_INVALID:
        st.caption("Position accuracy is stale (age >= 150).")

    st.subheader("Attitude")
    att_current_cols = st.columns(2)
    att_current_cols[0].metric("Pitch", f"{math.degrees(pitch):.3f} deg" if pitch is not None else "N/A")
    att_current_cols[1].metric("Roll", f"{math.degrees(roll):.3f} deg" if roll is not None else "N/A")
    att_acc_cols = st.columns(2)
    att_acc_cols[0].metric("Pitch sigma", _fmt_mrad(ori_acc[1]))
    att_acc_cols[1].metric("Roll sigma", _fmt_mrad(ori_acc[2]))
    att_inn_cols = st.columns(2)
    att_inn_cols[0].metric("Pitch innovation", _fmt_sigma(inn_att[0]))
    att_inn_cols[1].metric("Roll innovation", "N/A")

    st.subheader("Heading")
    head_cols = st.columns(3)
    head_cols[0].metric("Current", f"{math.degrees(heading):.3f} deg" if heading is not None else "N/A")
    head_cols[1].metric("Heading sigma", _fmt_mrad(ori_acc[0]))
    head_cols[2].metric("Heading innovation", _fmt_sigma(inn_att[1]))

    st.subheader("Speed")
    speed_current_cols = st.columns(3)
    speed_current_cols[0].metric("North", f"{vn:.3f} m/s" if vn is not None else "N/A")
    speed_current_cols[1].metric("East", f"{ve:.3f} m/s" if ve is not None else "N/A")
    speed_current_cols[2].metric("Down", f"{vd:.3f} m/s" if vd is not None else "N/A")
    speed_acc_cols = st.columns(3)
    prev_n, prev_e, prev_d = st.session_state.prev_vel_acc
    speed_acc_cols[0].metric("North sigma", _fmt_mms(vel_acc[0]), _delta_mms(vel_acc[0], prev_n))
    speed_acc_cols[1].metric("East sigma", _fmt_mms(vel_acc[1]), _delta_mms(vel_acc[1], prev_e))
    speed_acc_cols[2].metric("Down sigma", _fmt_mms(vel_acc[2]), _delta_mms(vel_acc[2], prev_d))
    st.session_state.prev_vel_acc = vel_acc
    speed_inn_cols = st.columns(3)
    speed_inn_cols[0].metric("Innovation North", _fmt_sigma(inn_vel[0]))
    speed_inn_cols[1].metric("Innovation East", _fmt_sigma(inn_vel[1]))
    speed_inn_cols[2].metric("Innovation Down", _fmt_sigma(inn_vel[2]))
    if vel_age >= ACCURACY_AGE_INVALID:
        st.caption("Velocity accuracy is stale (age >= 150).")
    if ori_age >= ACCURACY_AGE_INVALID:
        st.caption("Orientation accuracy is stale (age >= 150).")

    st.subheader("Map")
    if lat is not None and lon is not None:
        history_df = pd.DataFrame(
            [{"lat": p[0], "lon": p[1]} for p in position_history],
            columns=["lat", "lon"],
        )
        current_df = pd.DataFrame([{"lat": lat, "lon": lon}])

        layers: list[pdk.Layer] = []
        if not history_df.empty:
            layers.append(
                pdk.Layer(
                    "ScatterplotLayer",
                    data=history_df,
                    get_position="[lon, lat]",
                    stroked=True,
                    filled=False,
                    get_line_color=[120, 120, 120, 160],
                    line_width_min_pixels=1,
                    radius_min_pixels=2,
                )
            )

        if pos_acc[0] is not None and pos_acc[1] is not None:
            horizontal_accuracy_m = max(pos_acc[0], pos_acc[1])
            dashed_paths = _build_dashed_circle_paths(lat, lon, horizontal_accuracy_m)
            if dashed_paths:
                layers.append(
                    pdk.Layer(
                        "PathLayer",
                        data=dashed_paths,
                        get_path="path",
                        get_color=[0, 102, 204, 220],
                        width_min_pixels=2,
                    )
                )

        # 現在位置は「ポイント（輪郭）」として描画し、精度円は表示しない。
        layers.append(
            pdk.Layer(
                "ScatterplotLayer",
                data=current_df,
                get_position="[lon, lat]",
                stroked=True,
                filled=False,
                get_line_color=[255, 80, 80, 255],
                line_width_min_pixels=2,
                radius_min_pixels=5,
            )
        )
        view_state = pdk.ViewState(latitude=lat, longitude=lon, zoom=16, pitch=0)
        st.pydeck_chart(
            pdk.Deck(
                map_style=None,
                initial_view_state=view_state,
                layers=[
                    pdk.Layer(
                        "TileLayer",
                        data="https://tile.openstreetmap.org/{z}/{x}/{y}.png",
                        min_zoom=0,
                        max_zoom=19,
                        tile_size=256,
                    ),
                    *layers,
                ],
                tooltip={"text": "lat: {lat}\nlon: {lon}"},
            ),
            use_container_width=True,
        )
        if pos_acc[0] is not None and pos_acc[1] is not None:
            st.caption(
                "Position Accuracy (horizontal): "
                f"{max(pos_acc[0], pos_acc[1]):.3f} m (dashed circle)"
            )
        st.caption(f"Latest position: lat={lat:.6f}, lon={lon:.6f}")
    else:
        st.caption("Waiting for navigation position data.")

    st.subheader("Trend Graph")
    chart_category = st.selectbox("Category", options=["Position", "Attitude", "Heading", "Speed"], key="chart_category")
    chart_type = st.selectbox("Data Type", options=["Current", "Accuracy", "Innovation"], key="chart_type")
    chart_df = pd.DataFrame()
    y_label = ""
    if chart_category == "Position":
        if chart_type == "Current":
            chart_df = _series_from_history(
                position_value_history,
                ["time", "Latitude", "Longitude", "Altitude"],
                ["Latitude", "Longitude", "Altitude"],
            )
            y_label = "deg / m"
        elif chart_type == "Accuracy":
            chart_df = _series_from_history(
                pos_acc_history,
                ["time", "North", "East", "Down"],
                ["North", "East", "Down"],
            )
            y_label = "m"
        else:
            chart_df = _series_from_history(
                inn_pos_history,
                ["time", "X", "Y", "Z"],
                ["X", "Y", "Z"],
            )
            y_label = "sigma"
    elif chart_category == "Attitude":
        if chart_type == "Current":
            chart_df = _series_from_history(
                attitude_history,
                ["time", "Heading", "Pitch", "Roll"],
                ["Pitch", "Roll"],
            )
            if not chart_df.empty:
                chart_df["Pitch"] = chart_df["Pitch"].map(math.degrees)
                chart_df["Roll"] = chart_df["Roll"].map(math.degrees)
            y_label = "deg"
        elif chart_type == "Accuracy":
            chart_df = _series_from_history(
                ori_acc_history,
                ["time", "HeadingSigma", "PitchSigma", "RollSigma"],
                ["PitchSigma", "RollSigma"],
            )
            if not chart_df.empty:
                chart_df["PitchSigma"] = chart_df["PitchSigma"] * 1000.0
                chart_df["RollSigma"] = chart_df["RollSigma"] * 1000.0
            y_label = "mrad"
        else:
            chart_df = _series_from_history(
                inn_att_history,
                ["time", "PitchInnovation", "HeadingInnovation"],
                ["PitchInnovation"],
            )
            y_label = "sigma"
    elif chart_category == "Heading":
        if chart_type == "Current":
            chart_df = _series_from_history(
                attitude_history,
                ["time", "Heading", "Pitch", "Roll"],
                ["Heading"],
            )
            if not chart_df.empty:
                chart_df["Heading"] = chart_df["Heading"].map(math.degrees)
            y_label = "deg"
        elif chart_type == "Accuracy":
            chart_df = _series_from_history(
                ori_acc_history,
                ["time", "HeadingSigma", "PitchSigma", "RollSigma"],
                ["HeadingSigma"],
            )
            if not chart_df.empty:
                chart_df["HeadingSigma"] = chart_df["HeadingSigma"] * 1000.0
            y_label = "mrad"
        else:
            chart_df = _series_from_history(
                inn_att_history,
                ["time", "PitchInnovation", "HeadingInnovation"],
                ["HeadingInnovation"],
            )
            y_label = "sigma"
    else:
        if chart_type == "Current":
            chart_df = _series_from_history(
                velocity_history,
                ["time", "North", "East", "Down"],
                ["North", "East", "Down"],
            )
            y_label = "m/s"
        elif chart_type == "Accuracy":
            chart_df = _series_from_history(
                vel_acc_history,
                ["time", "North", "East", "Down"],
                ["North", "East", "Down"],
            )
            if not chart_df.empty:
                for col_name in ["North", "East", "Down"]:
                    chart_df[col_name] = chart_df[col_name] * 1000.0
            y_label = "mm/s"
        else:
            chart_df = _series_from_history(
                inn_vel_history,
                ["time", "North", "East", "Down"],
                ["North", "East", "Down"],
            )
            y_label = "sigma"

    if chart_df.empty:
        st.caption("No data available for selected graph.")
    else:
        st.line_chart(chart_df, use_container_width=True, y_label=y_label)

    with st.expander("Detailed Information", expanded=False):
        st.subheader("GAD Stream Monitor")
        if gad:
            active_streams = [info for info in gad.values() if info.status != "Timeout"]
            if active_streams:
                st.success(f"Channel 95 reports {len(active_streams)} active GAD stream(s).")
            else:
                st.warning("Only timeout GAD streams are currently visible on Channel 95.")

            rows = []
            for sid, info in sorted(gad.items()):
                if info.status == "Active" and info.innovation_quality == "Good":
                    indicator = "OK"
                elif info.status == "Active":
                    indicator = "WARN"
                elif info.status == "Timeout":
                    indicator = "ERR"
                else:
                    indicator = "INFO"
                valid_inns = [f"{v:.1f}sigma" for v in info.innovations if v is not None]
                rows.append(
                    {
                        "Stream ID": sid,
                        "Status": info.status,
                        "Rejections": info.rejection_count,
                        "Innovations": ", ".join(valid_inns) if valid_inns else "N/A",
                        "Quality": info.innovation_quality,
                        "Indicator": indicator,
                    }
                )
            st.dataframe(pd.DataFrame(rows), use_container_width=True, hide_index=True)
        else:
            st.caption("No GAD streams detected yet (Channel 95).")

        st.subheader("KF Innovations Set 2 (Channel 32)")
        set2_rows = [{"Name": key, "Value (sigma)": value} for key, value in kf_set2.items()]
        st.dataframe(pd.DataFrame(set2_rows), use_container_width=True, hide_index=True)

        st.subheader("KF Innovations Set 3 (Channel 88)")
        set3_order = ["vertical_advanced_slip", *[f"reserved_{i}" for i in range(1, 8)]]
        set3_label = {
            "vertical_advanced_slip": "Vertical advanced slip innovation (Byte0)",
            **{f"reserved_{i}": f"Reserved (Byte{i})" for i in range(1, 8)},
        }
        set3_rows = [
            {"Name": set3_label[key], "Value (sigma)": kf_set3.get(key)}
            for key in set3_order
        ]
        st.dataframe(pd.DataFrame(set3_rows), use_container_width=True, hide_index=True)

        st.subheader("Binary Diagnostics")
        diag_col1, diag_col2, diag_col3 = st.columns(3)
        diag_col1.metric("Diagnostics", "Enabled" if diagnostics_enabled else "Disabled")
        diag_col2.metric("Records (buffer)", str(len(diagnostics_rows)))
        diag_col3.metric("Warnings", str(diagnostics_warning_count))
        if diagnostics_enabled and diagnostics_rows:
            rows = []
            for row in diagnostics_rows:
                rows.append(
                    {
                        "PacketIdx": row.get("replay_packet_index", row.get("packet_index")),
                        "StatusCh": row.get("status_channel"),
                        "KnownCh": row.get("known_channel"),
                        "Checksums": row.get("checksum_valid"),
                        "Warnings": ", ".join(row.get("warnings", [])) or "none",
                        "StatusDataHex": row.get("status_data_hex"),
                    }
                )
            st.dataframe(pd.DataFrame(rows[-30:]), use_container_width=True, hide_index=True)
        elif diagnostics_enabled:
            st.caption("No diagnostics captured yet.")
        else:
            st.caption("Diagnostics is disabled.")

        st.subheader("CAN Bus Status")
        if can:
            tx_ok = can.get("tx_ok_pct")
            rx_ok = can.get("rx_ok_pct")
            errors_count = can.get("error_count", 0)
            cols = st.columns(4)
            cols[0].metric("TX OK", f"{tx_ok}%" if tx_ok is not None else "N/A")
            cols[1].metric("RX OK", f"{rx_ok}%" if rx_ok is not None else "N/A")
            cols[2].metric("Errors", str(errors_count))
            cols[3].metric("Last Error Code", str(can.get("last_error_code", 0)))

            if tx_ok is not None and tx_ok < 100:
                st.warning(f"CAN TX quality degraded: {tx_ok}% OK")
            if rx_ok is not None and rx_ok < 100:
                st.warning(f"CAN RX quality degraded: {rx_ok}% OK")
            if errors_count > 0:
                st.error(f"CAN bus errors detected: {errors_count}, last code: {can.get('last_error_code', 0)}")
        else:
            st.caption("Waiting for CAN bus data (Channel 78).")


def main() -> None:
    st.set_page_config(page_title="OxTS GAD Monitor", layout="wide")
    st.title("OxTS GAD Monitor")
    init_session()
    state: ReceiverState = st.session_state.receiver_state

    thread = st.session_state.receiver_thread
    if thread is not None and not thread.is_alive():
        # Clear stale thread references so UI can recover automatically.
        st.session_state.receiver_thread = None
        thread = None

    with state.lock:
        last_error = state.last_error
    is_running = bool(thread is not None and thread.is_alive())

    with st.sidebar:
        st.header("Settings")
        source = st.radio("Input Source", options=["UDP", "Playback"], disabled=is_running)
        port = int(st.number_input("UDP Port", value=3000, min_value=1, max_value=65535, disabled=source != "UDP"))
        playback_upload = st.file_uploader(
            "NCOM file (.ncom / .bin)",
            type=["ncom", "bin", "dat"],
            disabled=source != "Playback" or is_running,
        )
        replay_hz = float(
            st.number_input(
                "Playback Base Rate (Hz)",
                min_value=1.0,
                max_value=1000.0,
                value=100.0,
                step=1.0,
                disabled=source != "Playback",
            )
        )
        replay_speed = st.selectbox(
            "Playback Speed",
            options=[0.5, 1.0, 2.0, 5.0],
            index=1,
            disabled=source != "Playback",
        )
        replay_loop = st.checkbox("Loop playback", value=False, disabled=source != "Playback")
        binary_diagnostics = st.checkbox(
            "Binary Diagnostics",
            value=bool(st.session_state.binary_diagnostics_enabled),
            disabled=is_running,
            help="Capture per-packet binary diagnostics for status-channel verification.",
        )
        st.session_state.binary_diagnostics_enabled = binary_diagnostics
        playback_packet_count = st.session_state.playback_packet_count
        if source == "Playback":
            if playback_upload is not None:
                token = f"{playback_upload.name}:{playback_upload.size}"
                if token != st.session_state.playback_file_token:
                    raw = playback_upload.getvalue()
                    st.session_state.playback_file_path = _save_uploaded_file(playback_upload)
                    st.session_state.playback_file_name = playback_upload.name
                    st.session_state.playback_file_token = token
                    st.session_state.playback_packet_count = len(extract_ncom_packets(raw))
                    st.session_state.playback_seek_percent = 0
            playback_packet_count = st.session_state.playback_packet_count
            seek_pct = st.slider(
                "Seek position (%)",
                min_value=0,
                max_value=100,
                value=int(st.session_state.playback_seek_percent),
                step=1,
                disabled=is_running or playback_packet_count == 0,
            )
            st.session_state.playback_seek_percent = seek_pct
            if playback_packet_count > 0:
                seek_index = int((seek_pct / 100.0) * max(playback_packet_count - 1, 0))
                st.caption(f"Start packet index: {seek_index:,} / {playback_packet_count - 1:,}")
            else:
                st.caption("Upload a playback file to enable seek.")

        if is_running:
            st.success("Receiver: Running")
        elif last_error:
            st.error("Receiver: Error / Stopped")
            st.caption(last_error)
        else:
            st.info("Receiver: Stopped")

        col_start, col_pause, col_stop = st.columns(3)
        start = col_start.button("Start", use_container_width=True, disabled=is_running)
        pause_label = "Resume" if (thread is not None and getattr(state, "is_paused", False)) else "Pause"
        pause_resume = col_pause.button(pause_label, use_container_width=True, disabled=not is_running)
        stop = col_stop.button("Stop", use_container_width=True, disabled=not is_running)

    if start and st.session_state.receiver_thread is None:
        with state.lock:
            reset_receiver_state(state)
            state.last_error = ""
            state.diagnostics_enabled = bool(st.session_state.binary_diagnostics_enabled)
        if source == "Playback":
            if not st.session_state.playback_file_path:
                st.error("Playback mode requires a recorded NCOM file.")
                st.stop()
            start_index = int(
                (st.session_state.playback_seek_percent / 100.0)
                * max(st.session_state.playback_packet_count - 1, 0)
            )
            thread = NcomFileReplay(
                state=state,
                file_path=st.session_state.playback_file_path,
                replay_hz=replay_hz,
                speed=float(replay_speed),
                loop=replay_loop,
                start_index=start_index,
            )
        else:
            thread = NcomUdpReceiver(state=state, port=port)
        thread.start()
        st.session_state.receiver_thread = thread
        st.toast("Streaming started")
        st.rerun()

    if stop and st.session_state.receiver_thread is not None:
        thread = st.session_state.receiver_thread
        thread.stop()
        thread.join(timeout=1.5)
        st.session_state.receiver_thread = None
        st.toast("Streaming stopped")
        st.rerun()

    if pause_resume and st.session_state.receiver_thread is not None:
        thread = st.session_state.receiver_thread
        with state.lock:
            paused = state.is_paused
        if paused and hasattr(thread, "resume"):
            thread.resume()
            with state.lock:
                state.is_paused = False
            st.toast("Streaming resumed")
        elif (not paused) and hasattr(thread, "pause"):
            thread.pause()
            with state.lock:
                state.is_paused = True
            st.toast("Streaming paused")
        st.rerun()

    realtime_metrics()


if __name__ == "__main__":
    main()
