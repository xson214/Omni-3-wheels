"""Kinematics calculators and simulated path comparison for the Streamlit UI."""

from collections.abc import Callable
from math import ceil, radians

import altair as alt
import pandas as pd
import streamlit as st

from omni_model import (
    OMEGA_MAX_RAD_S,
    TICKS_PER_WHEEL_REV,
    WORLD_COORD_LIMIT_M,
    SimulationResult,
    forward_kinematics,
    inverse_kinematics,
    limit_wheel_speeds,
    pose_velocity,
    simulate_paths,
)


def render_forward_tab() -> None:
    st.write("Nhập tốc độ góc có dấu của ba bánh và góc hướng hiện tại của xe.")
    c1, c2, c3, c4 = st.columns(4)
    w1 = c1.number_input("ω₁ (rad/s)", value=0.0, step=0.1, format="%.4f")
    w2 = c2.number_input("ω₂ (rad/s)", value=-3.0, step=0.1, format="%.4f")
    w3 = c3.number_input("ω₃ (rad/s)", value=3.0, step=0.1, format="%.4f")
    heading_deg = c4.number_input("Góc hướng φ (°)", value=0.0, step=5.0, key="forward_heading")

    vx, vy, yaw_rate = forward_kinematics((w1, w2, w3), radians(heading_deg))
    c1, c2, c3 = st.columns(3)
    c1.metric("ẋ toàn cục (m/s)", f"{vx:.4f}")
    c2.metric("ẏ toàn cục (m/s)", f"{vy:.4f}")
    c3.metric("φ̇ (rad/s)", f"{yaw_rate:.4f}")

    with st.expander("Công thức trong README"):
        st.latex(r"""\begin{aligned}
        \dot x &= \frac{2r}{3}\sum_{i=1}^3 -\sin(\phi+\alpha_i)\,\omega_i \\
        \dot y &= \frac{2r}{3}\sum_{i=1}^3 \cos(\phi+\alpha_i)\,\omega_i \\
        \dot\phi &= \frac{r}{3L}\sum_{i=1}^3\omega_i
        \end{aligned}""")
        st.caption("r = 0,029 m · L = 0,12 m · α₁ = 0°, α₂ = +120°, α₃ = −120°")


def render_inverse_tab() -> None:
    st.write("Nhập điểm đích theo mét và tư thế hiện tại trong cùng hệ tọa độ.")
    c1, c2 = st.columns(2)
    coord = dict(min_value=-WORLD_COORD_LIMIT_M, max_value=WORLD_COORD_LIMIT_M, step=0.01, format="%.3f")
    target_x = c1.number_input("x đích (m)", value=0.5, **coord)
    target_y = c2.number_input("y đích (m)", value=0.0, **coord)
    c1, c2, c3 = st.columns(3)
    x = c1.number_input("x hiện tại (m)", value=0.0, **coord)
    y = c2.number_input("y hiện tại (m)", value=0.0, **coord)
    heading_deg = c3.number_input("Góc hướng φ (°)", value=0.0, step=5.0, key="inverse_heading")

    vx, vy, yaw_rate = pose_velocity(x, y, target_x, target_y)
    raw = inverse_kinematics(vx, vy, yaw_rate, radians(heading_deg))
    limited = limit_wheel_speeds(raw)
    st.caption(f"Vận tốc đặt toàn cục trước giới hạn: ẋ = {vx:.4f} m/s · ẏ = {vy:.4f} m/s · φ̇ = 0 rad/s.")
    st.dataframe(
        pd.DataFrame(
            {
                "Bánh": ["1 (0°)", "2 (+120°)", "3 (−120°)"],
                "ω trước giới hạn (rad/s)": raw,
                "ω sau giới hạn (rad/s)": limited,
            }
        ),
        hide_index=True,
    )
    st.caption(f"Khi vượt {OMEGA_MAX_RAD_S:.3f} rad/s, cả ba tốc độ bánh được nhân cùng một hệ số.")

    with st.expander("Công thức trong README"):
        st.latex(r"""\begin{aligned}
        \dot x &= 0.6(x_{\text{đích}}-x), \quad \dot y = 0.6(y_{\text{đích}}-y), \quad \dot\phi=0\\
        \omega_i &= \frac{-\sin(\phi+\alpha_i)\dot x+\cos(\phi+\alpha_i)\dot y+L\dot\phi}{r}
        \end{aligned}""")
        st.caption("AUTO dùng sai số vị trí để tính vận tốc. Trong phạm vi 0,01 m quanh đích, tốc độ đặt bằng 0.")


def _path_chart(result: SimulationResult) -> alt.LayerChart:
    paths = {"Lý thuyết": result.theory, "Encoder": result.encoder, "IMU": result.imu}
    rows = []
    for name, path in paths.items():
        last = len(path) - 1
        stride = max(1, ceil(last / 750))
        indices = [*range(0, last, stride), last]
        for index in indices:
            x, y = path[index]
            rows.append(
                {
                    "source": name,
                    "x": x,
                    "y": y,
                    "time": result.duration_s * index / max(last, 1),
                    "end": index == last,
                }
            )

    data = pd.DataFrame(rows)
    x_min, x_max = data["x"].min(), data["x"].max()
    y_min, y_max = data["y"].min(), data["y"].max()
    # A square plot with the same domain length on both axes keeps meters at equal scale.
    radius = max(x_max - x_min, y_max - y_min, 0.1) * 0.55
    x_mid = (x_min + x_max) / 2
    y_mid = (y_min + y_max) / 2
    names = list(paths)
    base = alt.Chart(data).encode(
        x=alt.X("x:Q", title="x (m)", scale=alt.Scale(domain=[x_mid - radius, x_mid + radius], nice=False, zero=False)),
        y=alt.Y("y:Q", title="y (m)", scale=alt.Scale(domain=[y_mid - radius, y_mid + radius], nice=False, zero=False)),
        color=alt.Color(
            "source:N",
            title=None,
            scale=alt.Scale(domain=names, range=["#2563eb", "#b45309", "#0f766e"]),
            legend=alt.Legend(orient="bottom"),
        ),
        tooltip=[
            alt.Tooltip("source:N", title="Đường đi"),
            alt.Tooltip("time:Q", title="Thời gian (s)", format=".2f"),
            alt.Tooltip("x:Q", title="x (m)", format=".3f"),
            alt.Tooltip("y:Q", title="y (m)", format=".3f"),
        ],
    )
    # Draw a narrower, dashed encoder line above theory so both are visible when they overlap.
    line_styles = [(3.5, [1, 0]), (1.8, [6, 3]), (2.5, [2, 3])]
    lines = [
        base.transform_filter(alt.datum.source == name)
        .mark_line(strokeWidth=width, strokeDash=dash)
        .encode(order="time:Q")
        for name, (width, dash) in zip(names, line_styles)
    ]
    endpoints = base.transform_filter("datum.end").mark_point(filled=True, size=60).encode(
        shape=alt.Shape(
            "source:N", scale=alt.Scale(domain=names, range=["circle", "square", "triangle-up"]), legend=None
        )
    )
    return alt.layer(*lines, endpoints).properties(width=340, height=340).interactive()


def render_simulation_tab(
    trajectory_text: str, parse_points: Callable[[str], list[dict[str, float]]]
) -> None:
    st.write("So sánh đường đi lý thuyết với ước lượng từ encoder và IMU trên cùng mặt phẳng x–y.")
    st.caption("Cả ba đường đều là mô phỏng. Tab này hoạt động độc lập với xe.")
    copy_csv = st.button("Sao chép điểm từ Autonomous")
    if "simulation_csv" not in st.session_state or copy_csv:
        st.session_state["simulation_csv"] = trajectory_text
    draft = st.text_area("Điểm mô phỏng (x,y theo mét, mỗi dòng một điểm)", key="simulation_csv", height=120)

    with st.form("simulation_settings"):
        c1, c2, c3, c4 = st.columns(4)
        ticks = c1.number_input("Encoder (tick/vòng bánh)", min_value=1, value=TICKS_PER_WHEEL_REV)
        slip = c2.number_input("Trượt bánh (%)", min_value=0.0, max_value=50.0, value=5.0, step=1.0)
        accel_bias = c3.number_input(
            "Lệch gia tốc IMU (m/s²)", min_value=-1.0, max_value=1.0, value=0.002, step=0.001, format="%.3f"
        )
        gyro_bias = c4.number_input(
            "Lệch gyro IMU (°/s)", min_value=-90.0, max_value=90.0, value=0.05, step=0.01, format="%.3f"
        )
        submitted = st.form_submit_button("Chạy mô phỏng", type="primary")

    inputs = (draft, ticks, slip, accel_bias, gyro_bias)
    if submitted:
        st.session_state.pop("simulation_result", None)
        try:
            points = [(point["x"], point["y"]) for point in parse_points(draft)]
            result = simulate_paths(
                points,
                ticks_per_rev=ticks,
                slip_percent=slip,
                accel_bias=accel_bias,
                gyro_bias_deg_s=gyro_bias,
            )
        except ValueError as exc:
            st.error(str(exc))
            return
        st.session_state["simulation_result"] = (inputs, result)

    previous = st.session_state.get("simulation_result")
    if previous is None or previous[0] != inputs:
        return

    result = previous[1]
    if result.completed:
        st.caption(f"Mô hình lý thuyết đã tới điểm cuối sau {result.duration_s:.2f} s.")
    else:
        st.warning(f"Mô phỏng dừng sau {result.duration_s:.2f} s trước khi tới điểm cuối.")
    st.altair_chart(_path_chart(result), use_container_width=False)
    st.caption("Ký hiệu cuối mỗi đường là vị trí tại thời điểm mô phỏng kết thúc.")
