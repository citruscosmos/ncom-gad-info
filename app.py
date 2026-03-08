"""Streamlit dashboard for OxTS GAD monitoring."""

from __future__ import annotations

import time

import pandas as pd
import streamlit as st

from ncom.constants import ACCURACY_AGE_INVALID
from receiver.udp_receiver import NcomUdpReceiver, ReceiverState


def init_session() -> None:
    if "receiver_state" not in st.session_state:
        st.session_state.receiver_state = ReceiverState()
    if "receiver_thread" not in st.session_state:
        st.session_state.receiver_thread = None
    if "prev_vel_acc" not in st.session_state:
        st.session_state.prev_vel_acc = (None, None, None)


def _fmt_mms(value: float | None) -> str:
    return f"{value * 1000:.1f} mm/s" if value is not None else "N/A"


def _delta_mms(current: float | None, previous: float | None) -> str | None:
    if current is None or previous is None:
        return None
    return f"{(current - previous) * 1000:+.1f}"


@st.fragment(run_every=1.5)
def realtime_metrics() -> None:
    state: ReceiverState = st.session_state.receiver_state
    with state.lock:
        vn = state.vel_acc_north
        ve = state.vel_acc_east
        vd = state.vel_acc_down
        vel_age = state.vel_acc_age
        history = list(state.accuracy_history)
        gad = dict(state.gad_streams)
        can = dict(state.can_status)
        nav = state.nav_status
        nav_code = state.nav_status_code
        gnss_pos = state.gnss_pos_mode_name
        gnss_vel = state.gnss_vel_mode_name
        sats = state.num_satellites
        packets = state.packet_count
        errors = state.error_count
        last_time = state.last_packet_time
        last_error = state.last_error

    if last_error:
        st.error(last_error)

    if last_time > 0 and (time.monotonic() - last_time) < 3.0:
        status_text = (
            f"Nav: **{nav}** | GNSS Vel: **{gnss_vel}** | GNSS Pos: **{gnss_pos}** "
            f"| Sats: **{sats or 'N/A'}** | Packets: {packets:,} | Errors: {errors:,}"
        )
        if nav_code == 4:
            st.success(status_text)
        else:
            st.warning(status_text)
    elif last_time > 0:
        st.warning(f"No data for {time.monotonic() - last_time:.0f}s | Last nav: {nav}")
    else:
        st.info("Waiting for NCOM data on UDP...")

    st.subheader("Velocity Accuracy")
    col1, col2, col3 = st.columns(3)
    prev_n, prev_e, prev_d = st.session_state.prev_vel_acc
    col1.metric("North sigma", _fmt_mms(vn), _delta_mms(vn, prev_n))
    col2.metric("East sigma", _fmt_mms(ve), _delta_mms(ve, prev_e))
    col3.metric("Down sigma", _fmt_mms(vd), _delta_mms(vd, prev_d))
    st.session_state.prev_vel_acc = (vn, ve, vd)

    if vel_age >= ACCURACY_AGE_INVALID:
        st.caption("Velocity accuracy is stale (age >= 150).")

    if history:
        df = pd.DataFrame(history, columns=["time", "North", "East", "Down"])
        now = time.monotonic()
        df["Seconds Ago"] = now - df["time"]
        df = df[df["Seconds Ago"] <= 60]
        for col_name in ["North", "East", "Down"]:
            df[col_name] = df[col_name] * 1000.0
        chart_df = df[["Seconds Ago", "North", "East", "Down"]].set_index("Seconds Ago")
        st.line_chart(chart_df, use_container_width=True, y_label="mm/s")
    else:
        st.caption("Waiting for velocity accuracy data (Channel 4).")

    st.subheader("GAD Stream Monitor")
    if gad:
        active_streams = [info for info in gad.values() if info.status != "Timeout"]
        if active_streams:
            st.success(
                f"Channel 95 reports {len(active_streams)} active GAD stream(s)."
            )
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

    thread: NcomUdpReceiver | None = st.session_state.receiver_thread
    if thread is not None and not thread.is_alive():
        # Clear stale thread references so UI can recover automatically.
        st.session_state.receiver_thread = None
        thread = None

    with state.lock:
        last_error = state.last_error
    is_running = bool(thread is not None and thread.is_alive())

    with st.sidebar:
        st.header("Settings")
        port = int(st.number_input("UDP Port", value=3000, min_value=1, max_value=65535))
        if is_running:
            st.success("Receiver: Running")
        elif last_error:
            st.error("Receiver: Error / Stopped")
            st.caption(last_error)
        else:
            st.info("Receiver: Stopped")

        col_start, col_stop = st.columns(2)
        start = col_start.button("Start", use_container_width=True, disabled=is_running)
        stop = col_stop.button("Stop", use_container_width=True, disabled=not is_running)

    if start and st.session_state.receiver_thread is None:
        thread = NcomUdpReceiver(state=state, port=port)
        thread.start()
        st.session_state.receiver_thread = thread
        st.toast("Streaming started")
        st.rerun()

    if stop and st.session_state.receiver_thread is not None:
        thread: NcomUdpReceiver = st.session_state.receiver_thread
        thread.stop()
        thread.join(timeout=1.5)
        st.session_state.receiver_thread = None
        st.toast("Streaming stopped")
        st.rerun()

    realtime_metrics()


if __name__ == "__main__":
    main()
