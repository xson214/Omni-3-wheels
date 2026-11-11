"""Streamlit UI for Omni 3-wheel ESP32-S3."""

from __future__ import annotations

import json
import math
import secrets
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import pandas as pd
import requests
import streamlit as st
import streamlit.components.v1 as components
import websocket
from websockets.exceptions import ConnectionClosed
from websockets.sync.server import ServerConnection, serve


DEFAULT_HOST = "192.168.4.1"
HTTP_PORT = 80
WS_PORT = 81
TRAJ_MAX_POINTS = 64
TRAJ_MAX_COORD_M = 1000


@dataclass
class TelemetryStore:
    lock: threading.Lock = field(default_factory=threading.Lock)
    latest: dict[str, Any] = field(default_factory=dict)
    status: dict[str, Any] = field(default_factory=dict)
    connected: bool = False
    last_telemetry: float = 0.0
    link_error: str = ""
    command_error: str = ""


class EspBridge:
    def __init__(self, host: str) -> None:
        self.host = host
        self.http_base = f"http://{host}:{HTTP_PORT}"
        self.ws_url = f"ws://{host}:{WS_PORT}/"
        self.store = TelemetryStore()
        self._ws: websocket.WebSocket | None = None
        self._stop = threading.Event()
        self._last_ui_activity = time.monotonic()
        self._send_lock = threading.Lock()
        self._pad: ServerConnection | None = None
        self._holding = False
        self._relay_path = "/" + secrets.token_urlsafe(24)
        self._relay = serve(
            self._handle_pad, "127.0.0.1", 0, max_size=512, max_queue=4, close_timeout=0.2
        )
        port = self._relay.socket.getsockname()[1]
        self.pad_url = f"ws://127.0.0.1:{port}{self._relay_path}"
        self._thread = threading.Thread(target=self._run, daemon=True)
        threading.Thread(target=self._relay.serve_forever, daemon=True).start()
        self._thread.start()

    @property
    def closed(self) -> bool:
        return self._stop.is_set()

    def touch(self) -> None:
        self._last_ui_activity = time.monotonic()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.5)

    def _active(self) -> bool:
        # A closed browser stops the Streamlit fragment and releases both sockets.
        return not self.closed and time.monotonic() - self._last_ui_activity < 30

    def _run(self) -> None:
        try:
            while self._active():
                try:
                    self._receive()
                except (OSError, websocket.WebSocketException) as exc:
                    with self.store.lock:
                        self.store.link_error = str(exc)
                self._stop.wait(0.5)
        finally:
            self._stop.set()
            self._relay.shutdown()

    def _receive(self) -> None:
        ws = websocket.create_connection(self.ws_url, timeout=0.2)
        with self._send_lock:
            self._ws = ws
        try:
            last_rx = time.monotonic()
            while self._active():
                try:
                    message = ws.recv()
                except websocket.WebSocketTimeoutException:
                    if time.monotonic() - last_rx > 2:
                        raise websocket.WebSocketTimeoutException("No firmware messages for 2 s")
                    continue
                if not message:
                    raise websocket.WebSocketConnectionClosedException("Firmware closed WebSocket")
                try:
                    data = json.loads(message)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
                if not isinstance(data, dict) or data.get("type") not in ("status", "telemetry"):
                    continue
                last_rx = time.monotonic()
                with self.store.lock:
                    self.store.connected = True
                    self.store.link_error = ""
                    self.store.status = {"mode": data.get("mode"), "state": data.get("state")}
                    if data["type"] == "telemetry":
                        self.store.latest = data
                        self.store.last_telemetry = last_rx
                    else:
                        self.store.command_error = data.get("error", "")
                if data.get("mode") != "MANUAL" or data.get("state") != "RUNNING":
                    with self._send_lock:
                        pad = self._clear_hold_locked()
                    self._notify_release(pad)
        finally:
            with self.store.lock:
                self.store.connected = False
            with self._send_lock:
                self._ws = None
                pad = self._clear_hold_locked()
            self._notify_release(pad)
            ws.close(timeout=0.2)

    def _clear_hold_locked(self) -> ServerConnection | None:
        pad = self._pad if self._holding else None
        self._holding = False
        return pad

    @staticmethod
    def _notify_release(pad: ServerConnection | None) -> None:
        if pad is not None:
            try:
                pad.send('{"type":"release"}')
            except ConnectionClosed:
                pass

    def _send_locked(self, payload: dict[str, Any]) -> bool:
        if self._ws is None or self.closed:
            return False
        try:
            self._ws.send(json.dumps(payload, allow_nan=False))
            return True
        except (OSError, websocket.WebSocketException):
            return False

    def send_action(self, message_type: str, value: str) -> bool:
        with self.store.lock:
            self.store.command_error = ""
        with self._send_lock:
            pad = self._clear_hold_locked()
            sent = self._send_locked({"type": message_type, "value": value})
        self._notify_release(pad)
        return sent

    def _handle_pad(self, connection: ServerConnection) -> None:
        # A random path grants access only to this Streamlit session's pad.
        if connection.request.path != self._relay_path:
            connection.close(1008)
            return
        with self._send_lock:
            accepted = self._pad is None and not self.closed
            if accepted:
                self._pad = connection
        if not accepted:
            connection.close(1008)
            return

        try:
            last_input = time.monotonic()
            while self._active():
                try:
                    message = connection.recv(timeout=0.25)
                except TimeoutError:
                    # No synthetic zero here: the firmware watchdog disarms on link loss.
                    with self._send_lock:
                        pad = self._clear_hold_locked()
                    self._notify_release(pad)
                    continue
                try:
                    data = json.loads(message)
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
                if not isinstance(data, dict):
                    continue

                with self.store.lock:
                    armed = (
                        self.store.connected
                        and self.store.status.get("mode") == "MANUAL"
                        and self.store.status.get("state") == "RUNNING"
                    )
                now = time.monotonic()
                kind = data.get("type")
                with self._send_lock:
                    if kind == "release":
                        if self._holding:
                            self._send_locked({"type": "joystick", "x": 0, "y": 0})
                            self._clear_hold_locked()
                        continue

                    x, y, sent_at = data.get("x"), data.get("y"), data.get("sentAt")
                    fresh = type(sent_at) in (int, float) and 0 <= time.time() * 1000 - sent_at < 200
                    valid_axes = all(type(axis) in (int, float) and -1 <= axis <= 1 for axis in (x, y))
                    if kind not in ("press", "move") or not fresh or not valid_axes:
                        continue
                    if now - last_input > 0.25:
                        self._holding = False
                    if kind == "press":
                        self._holding = armed
                    if self._holding and armed:
                        self._holding = self._send_locked({"type": "joystick", "x": x, "y": y})
                        last_input = now
                    release = not self._holding
                if release:
                    self._notify_release(connection)
        except ConnectionClosed:
            pass
        finally:
            with self._send_lock:
                self._pad = None
                self._holding = False

    def upload_trajectory(self, points: list[dict[str, float]]) -> dict[str, Any]:
        response = requests.post(
            f"{self.http_base}/trajectory", json={"type": "trajectory", "points": points}, timeout=3
        )
        if not response.ok:
            try:
                reason = response.json().get("err")
            except (requests.RequestException, AttributeError):
                reason = None
            if reason:
                raise ValueError(f"ESP32 rejected trajectory ({response.status_code}): {reason}")
        response.raise_for_status()
        return response.json()

    def get_trajectory(self) -> dict[str, Any]:
        response = requests.get(f"{self.http_base}/trajectory", timeout=3)
        response.raise_for_status()
        return response.json()

    def ping_http(self) -> bool:
        try:
            return requests.get(f"{self.http_base}/", timeout=2).status_code == 200
        except requests.RequestException:
            return False


def parse_points(text: str) -> list[dict[str, float]]:
    points = []
    for number, line in enumerate(text.splitlines(), start=1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(",")
        if len(parts) != 2:
            raise ValueError(f"Line {number}: expected exactly two values (x,y).")
        try:
            x, y = map(float, parts)
        except ValueError:
            raise ValueError(f"Line {number}: x and y must be numbers.") from None
        if not all(math.isfinite(value) and abs(value) <= TRAJ_MAX_COORD_M for value in (x, y)):
            raise ValueError(f"Line {number}: x and y must be finite and within ±{TRAJ_MAX_COORD_M} m.")
        if len(points) == TRAJ_MAX_POINTS:
            raise ValueError(f"Line {number}: at most {TRAJ_MAX_POINTS} points are allowed.")
        points.append({"x": x, "y": y})
    if not points:
        raise ValueError("Enter at least one point.")
    return points


def get_bridge(host: str, reconnect: bool) -> EspBridge:
    old = st.session_state.get("bridge")
    if old is None or old.host != host or old.closed or reconnect:
        if old is not None:
            old.stop()
        st.session_state.bridge = EspBridge(host)
    return st.session_state.bridge


@st.fragment(run_every=0.3)
def render_controls(bridge: EspBridge) -> None:
    if bridge.closed:
        st.rerun()
    bridge.touch()
    with bridge.store.lock:
        connected = bridge.store.connected
        status = dict(bridge.store.status) if connected else {}
        telemetry = dict(bridge.store.latest)
        last_telemetry = bridge.store.last_telemetry
        link_error = bridge.store.link_error
        command_error = bridge.store.command_error

    mode = status.get("mode") or "—"
    state = status.get("state") or "—"
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("WebSocket", "OK" if connected else "DOWN")
    c2.metric("Firmware mode", mode)
    c3.metric("Firmware state", state)
    age_ms = (time.monotonic() - last_telemetry) * 1000 if last_telemetry else None
    c4.metric("Telemetry age (ms)", f"{age_ms:.0f}" if age_ms is not None else "—")
    if link_error:
        st.warning(f"WebSocket: {link_error}")
    if command_error:
        st.error(f"ESP32 rejected the request: {command_error}")

    selected_mode = st.radio("Select mode", ["MANUAL", "AUTO", "STOP"], horizontal=True)
    action = ("mode", selected_mode) if st.button("Apply mode", disabled=not connected) else None
    b1, b2, b3, b4 = st.columns(4)
    if b1.button("START", disabled=mode not in ("MANUAL", "AUTO") or state != "IDLE"):
        action = ("command", "START")
    if b2.button("STOP", type="primary"):
        action = ("command", "STOP")
    if b3.button("PAUSE", disabled=mode not in ("MANUAL", "AUTO") or state != "RUNNING"):
        action = ("command", "PAUSE")
    if b4.button("RESUME", disabled=state != "PAUSED"):
        action = ("command", "RESUME")
    if action and not bridge.send_action(*action):
        st.error(f"Could not send {action[1]} to ESP32.")

    with st.expander("Telemetry"):
        if telemetry:
            metrics = st.columns(4)
            for column, key, label in zip(
                metrics, ("x", "y", "theta", "speed"), ("x (m)", "y (m)", "Yaw (rad)", "Speed (m/s)")
            ):
                value = telemetry.get(key)
                column.metric(label, f"{value:.3f}" if isinstance(value, (int, float)) else "—")
            st.json(telemetry)
        else:
            st.info("Waiting for telemetry…")


# Run pointer events and motion heartbeats in the browser independently of Streamlit reruns.
PAD_HTML = """
<style>
  body { margin: 0; font: 14px sans-serif; }
  #pad { display: grid; place-items: center; position: relative; box-sizing: border-box;
         width: 176px; height: 176px; border: 1px solid #568582; border-radius: 50%;
         background: #eff8f6; touch-action: none; user-select: none; cursor: grab; }
  #pad[data-held] { cursor: grabbing; }
  #pad::before, #pad::after { content: ""; position: absolute; background: #bdd5d2; }
  #pad::before { height: 1px; width: 100%; }
  #pad::after { width: 1px; height: 100%; }
  #knob { z-index: 1; height: 34px; width: 34px; border-radius: 50%; background: #176965;
          pointer-events: none; }
  #status { margin: 8px 0 0; color: #376b68; }
</style>
<div id="pad" aria-label="Hold and drag joystick"><div id="knob"></div></div>
<p id="status" role="status">Connecting to local control…</p>
<script>
  const pad = document.getElementById("pad");
  const knob = document.getElementById("knob");
  const status = document.getElementById("status");
  let socket, pointer = null, x = 0, y = 0;
  function send(type) {
    if (socket?.readyState !== WebSocket.OPEN || socket.bufferedAmount !== 0) return false;
    socket.send(JSON.stringify({type, x, y, sentAt: Date.now()}));
    return true;
  }
  function release(notify = true) {
    if (pointer === null) return;
    const id = pointer;
    pointer = null;
    x = y = 0;
    knob.style.transform = "";
    delete pad.dataset.held;
    if (notify) send("release");
    if (pad.hasPointerCapture(id)) pad.releasePointerCapture(id);
  }
  function connect() {
    socket = new WebSocket(__PAD_URL__);
    socket.onopen = () => status.textContent = "Local control connected";
    socket.onclose = () => {
      release(false);
      status.textContent = "Local control disconnected";
      setTimeout(connect, 750);
    };
    socket.onmessage = event => {
      if (JSON.parse(event.data).type === "release") release(false);
    };
  }
  function update(event) {
    const box = pad.getBoundingClientRect();
    const radius = (box.width - knob.offsetWidth) / 2;
    const dx = (event.clientX - box.left - box.width / 2) / radius;
    const dy = (event.clientY - box.top - box.height / 2) / radius;
    const scale = Math.max(1, Math.hypot(dx, dy));
    x = dx / scale;
    y = -dy / scale;
    knob.style.transform = "translate(" + x * radius + "px," + -y * radius + "px)";
  }
  pad.addEventListener("pointerdown", event => {
    if (pointer !== null || event.button !== 0 || socket?.readyState !== WebSocket.OPEN) return;
    pointer = event.pointerId;
    pad.setPointerCapture(pointer);
    pad.dataset.held = "";
    update(event);
    if (!send("press")) release(false);
  });
  pad.addEventListener("pointermove", event => {
    if (event.pointerId === pointer) update(event);
  });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
    pad.addEventListener(type, event => { if (event.pointerId === pointer) release(); });
  }
  addEventListener("blur", () => release());
  addEventListener("pagehide", () => release());
  document.addEventListener("visibilitychange", () => { if (document.hidden) release(); });
  setInterval(() => { if (pointer !== null && !send("move")) release(false); }, 80);
  connect();
</script>
"""


def main() -> None:
    st.set_page_config(page_title="OmniCar", layout="wide")
    st.title("OmniCar — ESP32-S3")

    with st.sidebar:
        host = st.text_input("ESP32 IP", value=DEFAULT_HOST).strip()
        st.write("SoftAP **OmniCar** · password **omnicar123**")
        reconnect = st.button("Reconnect WebSocket")
    bridge = get_bridge(host, reconnect)
    with st.sidebar:
        if st.button("Check HTTP"):
            st.metric("HTTP", "OK" if bridge.ping_http() else "DOWN")

    render_controls(bridge)
    tab_manual, tab_auto = st.tabs(["Manual", "Autonomous"])
    with tab_manual:
        st.write("Apply MANUAL, press START, then hold and drag. Release to stop.")
        st.caption("Body frame: right → +vx, up → +vy. Open this app on the PC connected to the ESP32.")
        components.html(PAD_HTML.replace("__PAD_URL__", json.dumps(bridge.pad_url)), height=210)

    with tab_auto:
        st.write("Upload points in meters, then apply AUTO and press START.")
        default_csv = "0.0,0.0\n0.5,0.0\n1.0,0.5\n"
        text = st.text_area("Points CSV (x,y per line)", value=default_csv, height=160)
        try:
            points = parse_points(text)
        except ValueError as exc:
            points = []
            st.error(str(exc))
        if points:
            st.dataframe(pd.DataFrame(points), hide_index=True)

        u1, u2 = st.columns(2)
        if u1.button("Upload trajectory", disabled=not points):
            try:
                response = bridge.upload_trajectory(points)
                if response.get("ok"):
                    st.success(f"Uploaded {len(points)} points.")
                else:
                    st.error(f"ESP32 rejected trajectory: {response.get('err', 'unknown error')}")
            except (requests.RequestException, ValueError) as exc:
                st.error(str(exc))
        if u2.button("Fetch stored trajectory"):
            try:
                st.json(bridge.get_trajectory())
            except requests.RequestException as exc:
                st.error(str(exc))


if __name__ == "__main__":
    main()
