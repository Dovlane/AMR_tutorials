from __future__ import annotations

import math


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def clamp_abs(value: float, limit: float) -> float:
    return clamp(value, -abs(limit), abs(limit))


def compute_polar_errors(robot_yaw: float, dx: float, dy: float) -> tuple[float, float]:
    heading_to_goal = math.atan2(dy, dx)
    alpha = normalize_angle(heading_to_goal - robot_yaw)
    beta = normalize_angle(-robot_yaw - alpha)
    return alpha, beta


def compute_rho_alpha_beta_control(
    rho: float,
    alpha: float,
    beta: float,
    k_rho: float,
    k_alpha: float,
    k_beta: float,
    direction: float = 1.0,
) -> tuple[float, float]:
    linear_velocity = direction * k_rho * rho
    angular_velocity = k_alpha * alpha + k_beta * beta
    return linear_velocity, angular_velocity
