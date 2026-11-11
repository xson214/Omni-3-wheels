"""Kinematics and offline path simulation for the three-wheel OmniCar."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, isfinite, pi, radians, sin, tau
from typing import Sequence


# Match README_FIRMWARE.md and firmware/include/config.h.
WHEEL_RADIUS_M = 0.029
BASE_RADIUS_M = 0.12
WHEEL_ANGLES_RAD = (0.0, 2.0 * pi / 3.0, -2.0 * pi / 3.0)
OMEGA_MAX_RAD_S = 5.236
LAMBDA_POSE = 0.6
POSE_XY_TOL_M = 0.01
TICKS_PER_WHEEL_REV = 4192
TRAJ_MAX_POINTS = 64
WORLD_COORD_LIMIT_M = 1000.0
SIM_DT_S = 0.01
SIM_MAX_DURATION_S = 60.0

WheelSpeeds = tuple[float, float, float]
Point = tuple[float, float]


def forward_kinematics(wheel_rad_s: WheelSpeeds, heading_rad: float) -> tuple[float, float, float]:
    """Convert signed wheel angular speeds into global (vx, vy, yaw rate)."""
    angles = [heading_rad + angle for angle in WHEEL_ANGLES_RAD]
    factor = 2.0 * WHEEL_RADIUS_M / 3.0
    vx = factor * sum(-sin(angle) * speed for angle, speed in zip(angles, wheel_rad_s))
    vy = factor * sum(cos(angle) * speed for angle, speed in zip(angles, wheel_rad_s))
    wz = WHEEL_RADIUS_M / (3.0 * BASE_RADIUS_M) * sum(wheel_rad_s)
    return vx, vy, wz


def inverse_kinematics(vx: float, vy: float, wz: float, heading_rad: float) -> WheelSpeeds:
    """Convert global (vx, vy, yaw rate) into signed wheel speeds in rad/s."""
    w1, w2, w3 = (
        (-sin(heading_rad + angle) * vx + cos(heading_rad + angle) * vy + BASE_RADIUS_M * wz)
        / WHEEL_RADIUS_M
        for angle in WHEEL_ANGLES_RAD
    )
    return w1, w2, w3


def limit_wheel_speeds(wheel_rad_s: WheelSpeeds) -> WheelSpeeds:
    """Apply the same scale to every wheel, as the firmware does."""
    peak = max(map(abs, wheel_rad_s))
    if peak <= OMEGA_MAX_RAD_S:
        return wheel_rad_s
    scale = OMEGA_MAX_RAD_S / peak
    w1, w2, w3 = wheel_rad_s
    return w1 * scale, w2 * scale, w3 * scale


def pose_velocity(x_m: float, y_m: float, target_x_m: float, target_y_m: float) -> tuple[float, float, float]:
    """Convert an AUTO target in meters into a global velocity command."""
    ex, ey = target_x_m - x_m, target_y_m - y_m
    if hypot(ex, ey) <= POSE_XY_TOL_M:
        return 0.0, 0.0, 0.0
    return LAMBDA_POSE * ex, LAMBDA_POSE * ey, 0.0


@dataclass
class SimulationResult:
    theory: list[Point]
    encoder: list[Point]
    imu: list[Point]
    duration_s: float
    completed: bool


def simulate_paths(
    points: Sequence[Point],
    *,
    ticks_per_rev: int = TICKS_PER_WHEEL_REV,
    slip_percent: float = 5.0,
    accel_bias: float = 0.002,
    gyro_bias_deg_s: float = 0.05,
) -> SimulationResult:
    """Compare ideal motion, encoder odometry and synthetic planar IMU dead reckoning.

    Ideal wheels track their commands instantly. Uniform slip reduces actual
    translation without changing their rotation. A simulated IMU measures planar
    body acceleration with constant x-axis bias and yaw rate with gyro bias.
    The estimator integrates yaw and rotates acceleration into the global frame
    before integrating velocity and position. It starts from rest and applies no
    noise, gravity, tilt or sensor fusion.
    """
    if not 1 <= len(points) <= TRAJ_MAX_POINTS:
        raise ValueError(f"Enter 1–{TRAJ_MAX_POINTS} points.")
    if any(not -WORLD_COORD_LIMIT_M <= value <= WORLD_COORD_LIMIT_M for point in points for value in point):
        raise ValueError(f"x and y must be within ±{WORLD_COORD_LIMIT_M:g} m.")
    if not isinstance(ticks_per_rev, int) or ticks_per_rev < 1:
        raise ValueError("Encoder ticks per wheel revolution must be a positive integer.")
    if not 0.0 <= slip_percent <= 100.0 or not all(map(isfinite, (accel_bias, gyro_bias_deg_s))):
        raise ValueError("Slip must be in [0, 100]% and IMU biases must be finite.")

    theory = [(0.0, 0.0)]
    encoder = [(0.0, 0.0)]
    imu = [(0.0, 0.0)]
    theory_heading = encoder_heading = imu_heading = 0.0
    wheel_angle = [0.0, 0.0, 0.0]
    previous_ticks = [0, 0, 0]
    actual_vx = actual_vy = imu_vx = imu_vy = 0.0
    translation_scale = 1.0 - slip_percent / 100.0
    gyro_bias = radians(gyro_bias_deg_s)
    max_steps = round(SIM_MAX_DURATION_S / SIM_DT_S)
    index = 0

    for step in range(max_steps + 1):
        x, y = theory[-1]
        while index < len(points) and hypot(points[index][0] - x, points[index][1] - y) <= POSE_XY_TOL_M:
            index += 1
        if index == len(points) or step == max_steps:
            break

        vx_cmd, vy_cmd, wz_cmd = pose_velocity(x, y, *points[index])
        wheels = limit_wheel_speeds(inverse_kinematics(vx_cmd, vy_cmd, wz_cmd, theory_heading))
        vx, vy, wz = forward_kinematics(wheels, theory_heading)
        theory.append((x + vx * SIM_DT_S, y + vy * SIM_DT_S))

        # Quantize cumulative angle so fractional ticks carry over between samples.
        wheel_measured = []
        for i, speed in enumerate(wheels):
            wheel_angle[i] += speed * SIM_DT_S
            ticks = round(wheel_angle[i] * ticks_per_rev / tau)
            wheel_measured.append((ticks - previous_ticks[i]) * tau / (ticks_per_rev * SIM_DT_S))
            previous_ticks[i] = ticks
        w1, w2, w3 = wheel_measured
        enc_vx, enc_vy, enc_wz = forward_kinematics((w1, w2, w3), encoder_heading)
        enc_x, enc_y = encoder[-1]
        encoder.append((enc_x + enc_vx * SIM_DT_S, enc_y + enc_vy * SIM_DT_S))
        encoder_heading += enc_wz * SIM_DT_S

        # Uniform translation slip leaves the true heading equal to the ideal heading.
        next_actual_vx, next_actual_vy = vx * translation_scale, vy * translation_scale
        ax = (next_actual_vx - actual_vx) / SIM_DT_S
        ay = (next_actual_vy - actual_vy) / SIM_DT_S
        actual_vx, actual_vy = next_actual_vx, next_actual_vy
        true_c, true_s = cos(theory_heading), sin(theory_heading)
        ax_body = true_c * ax + true_s * ay + accel_bias
        ay_body = -true_s * ax + true_c * ay
        est_c, est_s = cos(imu_heading), sin(imu_heading)
        imu_vx += (est_c * ax_body - est_s * ay_body) * SIM_DT_S
        imu_vy += (est_s * ax_body + est_c * ay_body) * SIM_DT_S
        imu_x, imu_y = imu[-1]
        imu.append((imu_x + imu_vx * SIM_DT_S, imu_y + imu_vy * SIM_DT_S))
        imu_heading += (wz + gyro_bias) * SIM_DT_S
        theory_heading += wz * SIM_DT_S

    return SimulationResult(theory, encoder, imu, step * SIM_DT_S, index == len(points))
