from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, Sequence

import numpy
import yaml


PoseVector = numpy.ndarray
ControlVector = numpy.ndarray
LineArray = numpy.ndarray


@dataclass(frozen=True)
class AssociationMatch:
    observation_index: int
    map_index: int
    distance: float


@dataclass(frozen=True)
class AssociationResult:
    innovation: numpy.ndarray
    jacobian: numpy.ndarray
    covariance: numpy.ndarray
    matches: tuple[AssociationMatch, ...]

    @property
    def count(self) -> int:
        return len(self.matches)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_line(alpha: float, radius: float) -> tuple[float, float]:
    alpha = normalize_angle(alpha)
    radius = float(radius)
    if radius < 0.0:
        radius = -radius
        alpha = normalize_angle(alpha + math.pi)
    return alpha, radius


def as_pose_vector(pose: Sequence[float]) -> PoseVector:
    pose_vector = numpy.asarray(pose, dtype=float).reshape(3)
    return numpy.array(
        [
            float(pose_vector[0]),
            float(pose_vector[1]),
            normalize_angle(float(pose_vector[2])),
        ],
        dtype=float,
    )


def as_covariance_matrix(covariance: Sequence[float] | numpy.ndarray) -> numpy.ndarray:
    matrix = numpy.asarray(covariance, dtype=float)
    if matrix.shape == (3,):
        return numpy.diag(matrix)
    return matrix.reshape(3, 3)


def transition_function(
    previous_pose: Sequence[float],
    control: Sequence[float],
    wheel_separation: float,
) -> tuple[PoseVector, numpy.ndarray, numpy.ndarray]:
    """Predict pose and Jacobians for a ROS-frame differential-drive robot.

    The control vector is [left_wheel_distance, right_wheel_distance].
    Positive yaw follows the ROS convention: counter-clockwise in the map frame.
    """

    pose = as_pose_vector(previous_pose)
    left_delta, right_delta = numpy.asarray(control, dtype=float).reshape(2)
    wheel_separation = float(wheel_separation)
    if wheel_separation <= 0.0:
        raise ValueError("wheel_separation must be positive.")

    distance = 0.5 * (left_delta + right_delta)
    heading_delta = (right_delta - left_delta) / wheel_separation
    mid_heading = pose[2] + 0.5 * heading_delta

    predicted_pose = numpy.array(
        [
            pose[0] + distance * math.cos(mid_heading),
            pose[1] + distance * math.sin(mid_heading),
            normalize_angle(pose[2] + heading_delta),
        ],
        dtype=float,
    )

    fx = numpy.eye(3, dtype=float)
    fx[0, 2] = -distance * math.sin(mid_heading)
    fx[1, 2] = distance * math.cos(mid_heading)

    half_over_base = 0.5 / wheel_separation
    fu = numpy.array(
        [
            [
                0.5 * math.cos(mid_heading)
                + distance * half_over_base * math.sin(mid_heading),
                0.5 * math.cos(mid_heading)
                - distance * half_over_base * math.sin(mid_heading),
            ],
            [
                0.5 * math.sin(mid_heading)
                - distance * half_over_base * math.cos(mid_heading),
                0.5 * math.sin(mid_heading)
                + distance * half_over_base * math.cos(mid_heading),
            ],
            [-1.0 / wheel_separation, 1.0 / wheel_separation],
        ],
        dtype=float,
    )

    return predicted_pose, fx, fu


def wheel_control_covariance(
    control: Sequence[float],
    motion_noise_gain: float,
    minimum_variance: float = 1e-9,
) -> numpy.ndarray:
    left_delta, right_delta = numpy.asarray(control, dtype=float).reshape(2)
    gain = max(0.0, float(motion_noise_gain))
    return numpy.diag(
        [
            max(minimum_variance, gain * abs(left_delta)),
            max(minimum_variance, gain * abs(right_delta)),
        ]
    )


def predict_covariance(
    previous_covariance: Sequence[float] | numpy.ndarray,
    fx: numpy.ndarray,
    fu: numpy.ndarray,
    control: Sequence[float],
    motion_noise_gain: float,
) -> numpy.ndarray:
    covariance = as_covariance_matrix(previous_covariance)
    q = wheel_control_covariance(control, motion_noise_gain)
    predicted = fx @ covariance @ fx.T + fu @ q @ fu.T
    return symmetrize(predicted)


def measurement_function(
    pose: Sequence[float],
    map_line: Sequence[float],
) -> tuple[numpy.ndarray, numpy.ndarray]:
    """Predict one global map line as a local robot-frame line measurement."""

    pose_vector = as_pose_vector(pose)
    map_alpha, map_radius = normalize_line(float(map_line[0]), float(map_line[1]))
    normal_x = math.cos(map_alpha)
    normal_y = math.sin(map_alpha)

    robot_alpha = normalize_angle(map_alpha - pose_vector[2])
    robot_radius = map_radius - pose_vector[0] * normal_x - pose_vector[1] * normal_y

    hx = numpy.array(
        [
            [0.0, 0.0, -1.0],
            [-normal_x, -normal_y, 0.0],
        ],
        dtype=float,
    )

    return numpy.array([robot_alpha, robot_radius], dtype=float), hx


def load_line_map(map_file: str | Path) -> LineArray:
    with Path(map_file).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    raw_lines = data.get("walls", data.get("lines", data))
    lines = []
    if isinstance(raw_lines, dict):
        raw_line_iterable = [
            raw_lines[key]
            for key in sorted(raw_lines, key=lambda value: int(value))
        ]
    else:
        raw_line_iterable = raw_lines

    for raw_line in raw_line_iterable:
        if isinstance(raw_line, dict):
            alpha = raw_line["alpha"]
            radius = raw_line.get("r", raw_line.get("radius"))
        else:
            alpha, radius = raw_line
        lines.append(normalize_line(float(alpha), float(radius)))

    if not lines:
        raise ValueError(f"No line features found in map file: {map_file}")
    return numpy.asarray(lines, dtype=float)


def associate_measurements(
    pose: Sequence[float],
    covariance: Sequence[float] | numpy.ndarray,
    observations: Sequence[Sequence[float]] | numpy.ndarray,
    measurement_covariances: numpy.ndarray | Sequence[numpy.ndarray],
    map_lines: Sequence[Sequence[float]] | numpy.ndarray,
    validation_gate: float,
) -> AssociationResult:
    pose_vector = as_pose_vector(pose)
    pose_covariance = as_covariance_matrix(covariance)
    observation_array = normalize_observations(observations)
    map_array = normalize_observations(map_lines)

    if observation_array.size == 0 or map_array.size == 0:
        return empty_association()

    measurement_covariance_array = expand_measurement_covariances(
        measurement_covariances,
        len(observation_array),
    )

    predicted = []
    for map_line in map_array:
        predicted.append(measurement_function(pose_vector, map_line))

    candidates = []
    gate_squared = float(validation_gate) ** 2
    for observation_index, observation in enumerate(observation_array):
        observation_covariance = measurement_covariance_array[observation_index]
        for map_index, (predicted_measurement, hx) in enumerate(predicted):
            innovation = measurement_innovation(observation, predicted_measurement)
            innovation_covariance = hx @ pose_covariance @ hx.T + observation_covariance
            distance = mahalanobis_distance(innovation, innovation_covariance)
            if distance < gate_squared:
                candidates.append(
                    (
                        distance,
                        observation_index,
                        map_index,
                        innovation,
                        hx,
                        observation_covariance,
                    )
                )

    if not candidates:
        return empty_association()

    candidates.sort(key=lambda candidate: candidate[0])
    used_observations = set()
    used_map_lines = set()
    innovations = []
    jacobians = []
    covariances = []
    matches = []

    for distance, observation_index, map_index, innovation, hx, r in candidates:
        if observation_index in used_observations or map_index in used_map_lines:
            continue

        used_observations.add(observation_index)
        used_map_lines.add(map_index)
        innovations.append(innovation)
        jacobians.append(hx)
        covariances.append(r)
        matches.append(
            AssociationMatch(
                observation_index=observation_index,
                map_index=map_index,
                distance=float(distance),
            )
        )

    if not matches:
        return empty_association()

    return AssociationResult(
        innovation=numpy.concatenate(innovations),
        jacobian=numpy.vstack(jacobians),
        covariance=block_diag(covariances),
        matches=tuple(matches),
    )


def filter_step(
    predicted_pose: Sequence[float],
    predicted_covariance: Sequence[float] | numpy.ndarray,
    innovation: Sequence[float] | numpy.ndarray,
    jacobian: numpy.ndarray,
    measurement_covariance: numpy.ndarray,
) -> tuple[PoseVector, numpy.ndarray]:
    pose = as_pose_vector(predicted_pose)
    covariance = as_covariance_matrix(predicted_covariance)
    innovation_vector = numpy.asarray(innovation, dtype=float).reshape(-1)

    if innovation_vector.size == 0 or jacobian.size == 0:
        return pose.copy(), covariance.copy()

    h = numpy.asarray(jacobian, dtype=float)
    r = numpy.asarray(measurement_covariance, dtype=float)
    s = h @ covariance @ h.T + r
    kalman_gain = solve_kalman_gain(covariance, h, s)

    corrected_pose = pose + kalman_gain @ innovation_vector
    corrected_pose[2] = normalize_angle(float(corrected_pose[2]))

    identity = numpy.eye(3, dtype=float)
    corrected_covariance = (identity - kalman_gain @ h) @ covariance
    return corrected_pose, symmetrize(corrected_covariance)


def normalize_observations(
    observations: Sequence[Sequence[float]] | numpy.ndarray,
) -> numpy.ndarray:
    array = numpy.asarray(observations, dtype=float)
    if array.size == 0:
        return numpy.zeros((0, 2), dtype=float)

    array = array.reshape((-1, 2))
    return numpy.asarray([normalize_line(alpha, radius) for alpha, radius in array])


def measurement_innovation(
    observation: numpy.ndarray,
    predicted_measurement: numpy.ndarray,
) -> numpy.ndarray:
    return numpy.array(
        [
            normalize_angle(float(observation[0] - predicted_measurement[0])),
            float(observation[1] - predicted_measurement[1]),
        ],
        dtype=float,
    )


def mahalanobis_distance(vector: numpy.ndarray, covariance: numpy.ndarray) -> float:
    try:
        solved = numpy.linalg.solve(covariance, vector)
    except numpy.linalg.LinAlgError:
        solved = numpy.linalg.solve(
            covariance + numpy.eye(covariance.shape[0]) * 1e-9,
            vector,
        )
    return float(vector.T @ solved)


def solve_kalman_gain(
    covariance: numpy.ndarray,
    jacobian: numpy.ndarray,
    innovation_covariance: numpy.ndarray,
) -> numpy.ndarray:
    right_hand_side = jacobian @ covariance
    try:
        return numpy.linalg.solve(innovation_covariance, right_hand_side).T
    except numpy.linalg.LinAlgError:
        jitter = numpy.eye(innovation_covariance.shape[0], dtype=float) * 1e-9
        return numpy.linalg.solve(innovation_covariance + jitter, right_hand_side).T


def expand_measurement_covariances(
    measurement_covariances: numpy.ndarray | Sequence[numpy.ndarray],
    observation_count: int,
) -> numpy.ndarray:
    array = numpy.asarray(measurement_covariances, dtype=float)
    if array.shape == (2, 2):
        return numpy.repeat(array[numpy.newaxis, :, :], observation_count, axis=0)
    return array.reshape((observation_count, 2, 2))


def block_diag(blocks: Iterable[numpy.ndarray]) -> numpy.ndarray:
    block_list = [numpy.asarray(block, dtype=float).reshape(2, 2) for block in blocks]
    result = numpy.zeros((2 * len(block_list), 2 * len(block_list)), dtype=float)
    for index, block in enumerate(block_list):
        start = 2 * index
        result[start : start + 2, start : start + 2] = block
    return result


def symmetrize(matrix: numpy.ndarray) -> numpy.ndarray:
    return 0.5 * (matrix + matrix.T)


def empty_association() -> AssociationResult:
    return AssociationResult(
        innovation=numpy.zeros((0,), dtype=float),
        jacobian=numpy.zeros((0, 3), dtype=float),
        covariance=numpy.zeros((0, 0), dtype=float),
        matches=(),
    )
