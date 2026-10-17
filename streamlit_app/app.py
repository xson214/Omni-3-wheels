"""Streamlit UI for Omni 3-wheel ESP32-S3 (idea.json)."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import pandas as pd
import requests
import streamlit as st
import websocket


DEFAULT_HOST = "192.168.4.1"
HTTP_PORT = 80
WS_PORT = 81


@dataclass
class TelemetryStore:
    lock: threading.Lock = field(default_factory=threading.Lock)
    latest: Dict[str, Any] = field(default_factory=dict)
    connected: bool = False
    last_rx_ms: float = 0.0
    error: str = ""


class EspBridge:
    def __init__(self, host: str) -> None:
        self.host = host
        self.http_base = f"http://{host}:{HTTP_PORT}"
        self.ws_url = f"ws://{host}:{WS_PORT}/"
        self.store = TelemetryStore()
        self._ws: Optional[websocket.WebSocketApp] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()

        def on_message(_ws, message: str) -> None:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                return
            with self.store.lock:
                if data.get("type") == "telemetry":
                    self.store.latest = data
                elif data.get("type") == "status":
                    self.store.latest = {**self.store.latest, **data}
                self.store.last_rx_ms = time.time() * 1000.0
                self.store.connected = True
                self.store.error = ""

        def on_error(_ws, err) -> None:
            with self.store.lock:
                self.store.error = str(err)
                self.store.connected = False

        def on_close(_ws, *_args) -> None:
            with self.store.lock:
                self.store.connected = False

        def on_open(_ws) -> None:
            with self.store.lock:
                self.store.connected = True
                self.store.error = ""

        self._ws = websocket.WebSocketApp(
            self.ws_url,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close,
            on_open=on_open,
        )

        def run() -> None:
            while not self._stop.is_set():
                try:
                    self._ws.run_forever(ping_interval=20, ping_timeout=10)
                except Exception as exc:  # noqa: BLE001
                    with self.store.lock:
                        self.store.error = str(exc)
                        self.store.connected = False
                if self._stop.is_set():
                    break
                time.sleep(1.0)

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._ws:
            try:
                self._ws.close()
            except Exception:  # noqa: BLE001
                pass
        if self._thread:
            self._thread.join(timeout=2.0)

    def send_json(self, payload: Dict[str, Any]) -> bool:
        if not self._ws:
            return False
        try:
            self._ws.send(json.dumps(payload))
            return True
        except Exception:  # noqa: BLE001
            return False

    def set_mode(self, mode: str) -> bool:
        return self.send_json({"type": "mode", "value": mode})

    def command(self, cmd: str) -> bool:
        return self.send_json({"type": "command", "value": cmd})

    def joystick(self, x: float, y: float) -> bool:
        return self.send_json({"type": "joystick", "x": float(x), "y": float(y)})

    def upload_trajectory(self, points: List[Dict[str, float]]) -> Dict[str, Any]:
        r = requests.post(
            f"{self.http_base}/trajectory",
            json={"type": "trajectory", "points": points},
            timeout=3.0,
        )
        r.raise_for_status()
        return r.json()

    def get_trajectory(self) -> Dict[str, Any]:
        r = requests.get(f"{self.http_base}/trajectory", timeout=3.0)
        r.raise_for_status()
        return r.json()

    def ping_http(self) -> bool:
        try:
            r = requests.get(f"{self.http_base}/", timeout=2.0)
            return r.status_code == 200
        except Exception:  # noqa: BLE001
            return False


def get_bridge(host: str) -> EspBridge:
    if "bridge" not in st.session_state or st.session_state.get("bridge_host") != host:
        old = st.session_state.get("bridge")
        if isinstance(old, EspBridge):
            old.stop()
        bridge = EspBridge(host)
        bridge.start()
        st.session_state.bridge = bridge
        st.session_state.bridge_host = host
    return st.session_state.bridge


def main() -> None:
    st.set_page_config(page_title="OmniCar", layout="wide")
    st.title("OmniCar — ESP32-S3")
    st.caption("Manual joystick · trajectory upload · live telemetry")

    with st.sidebar:
        host = st.text_input("ESP32 IP", value=DEFAULT_HOST)
        st.write("SoftAP SSID: `OmniCar` / pass: `omnicar123`")
        if st.button("Reconnect WebSocket"):
            old = st.session_state.get("bridge")
            if isinstance(old, EspBridge):
                old.stop()
            st.session_state.pop("bridge", None)
            st.session_state.pop("bridge_host", None)

    bridge = get_bridge(host)

    http_ok = bridge.ping_http()
    with bridge.store.lock:
        ws_ok = bridge.store.connected
        telem = dict(bridge.store.latest)
        err = bridge.store.error
        last_rx = bridge.store.last_rx_ms

    c1, c2, c3 = st.columns(3)
    c1.metric("HTTP", "OK" if http_ok else "DOWN")
    c2.metric("WebSocket", "OK" if ws_ok else "DOWN")
    age_ms = (time.time() * 1000.0 - last_rx) if last_rx else None
    c3.metric("Telemetry age (ms)", f"{age_ms:.0f}" if age_ms is not None else "—")
    if err:
        st.warning(f"WS error: {err}")

    mode = st.radio("Mode", ["MANUAL", "AUTO", "STOP"], horizontal=True)
    if st.button("Apply mode"):
        bridge.set_mode(mode)
        st.success(f"Mode → {mode}")

    b1, b2, b3, b4 = st.columns(4)
    if b1.button("START"):
        bridge.command("START")
    if b2.button("STOP", type="primary"):
        bridge.command("STOP")
    if b3.button("PAUSE"):
        bridge.command("PAUSE")
    if b4.button("RESUME"):
        bridge.command("RESUME")

    tab_manual, tab_auto, tab_telem = st.tabs(["Manual", "Autonomous", "Telemetry"])

    with tab_manual:
        st.write("Joystick body-frame: **x → vx**, **y → vy**, wz = 0")
        jx = st.slider("Joystick X (vx)", -1.0, 1.0, 0.0, 0.01)
        jy = st.slider("Joystick Y (vy)", -1.0, 1.0, 0.0, 0.01)
        send_joy = st.checkbox("Stream joystick while MANUAL", value=True)
        if send_joy and mode == "MANUAL":
            ok = bridge.joystick(jx, jy)
            if not ok:
                st.error("Failed to send joystick (WS not connected)")
            else:
                st.caption(f"Streaming joystick x={jx:.2f} y={jy:.2f}")
        if st.button("Send joystick once"):
            bridge.joystick(jx, jy)

    with tab_auto:
        st.write("Upload trajectory points (meters). Firmware follows with pose gain λ.")
        default_csv = "0.0,0.0\n0.5,0.0\n1.0,0.5\n"
        text = st.text_area("Points CSV (x,y per line)", value=default_csv, height=160)
        points: List[Dict[str, float]] = []
        for line in text.strip().splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) >= 2:
                points.append({"x": float(parts[0]), "y": float(parts[1])})

        st.write(f"{len(points)} points parsed")
        if points:
            st.dataframe(pd.DataFrame(points), use_container_width=True)

        u1, u2 = st.columns(2)
        if u1.button("Upload trajectory"):
            try:
                resp = bridge.upload_trajectory(points)
                st.success(f"Uploaded: {resp}")
                bridge.set_mode("AUTO")
            except Exception as exc:  # noqa: BLE001
                st.error(str(exc))
        if u2.button("Fetch stored trajectory"):
            try:
                st.json(bridge.get_trajectory())
            except Exception as exc:  # noqa: BLE001
                st.error(str(exc))

    with tab_telem:
        if telem:
            st.json(telem)
            m1, m2, m3, m4 = st.columns(4)
            m1.metric("x (m)", f"{telem.get('x', 0):.3f}")
            m2.metric("y (m)", f"{telem.get('y', 0):.3f}")
            m3.metric("theta (rad)", f"{telem.get('theta', 0):.3f}")
            m4.metric("speed", f"{telem.get('speed', 0):.3f}")
        else:
            st.info("Waiting for telemetry…")

    # Safety: if we were receiving telemetry and then lost link while streaming manual
    if "was_ws_ok" not in st.session_state:
        st.session_state.was_ws_ok = False
    if mode == "MANUAL" and send_joy and st.session_state.was_ws_ok and not ws_ok:
        bridge.command("STOP")
    st.session_state.was_ws_ok = ws_ok

    time.sleep(0.2)
    st.rerun()


if __name__ == "__main__":
    main()
