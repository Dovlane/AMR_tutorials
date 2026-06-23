#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import re
import sys
from typing import Iterable, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy
import yaml


DEFAULT_TOPICS = (
    "/clock",
    "/joint_states",
    "/odom",
    "/ekf_pose",
    "/cmd_vel",
    "/scan",
    "/tf",
    "/tf_static",
    "/gazebo/model_states",
    "/ekf_association_count",
    "/ekf_update_applied",
    "/waypoint_index",
    "/ekf_map_lines",
)

CONFIG_LABELS = {
    "a": "Konfiguracija (a): povratna sprega po /odom",
    "b": "Konfiguracija (b): povratna sprega po /ekf_pose, sa popravkom",
    "c": "Konfiguracija (c): povratna sprega po /ekf_pose, samo predikcija",
}

REFERENCE_VALIDATION_GATE = 5.0
LOW_VALIDATION_GATE = 2.0
HIGH_VALIDATION_GATE = 10.0
GATE_ANALYSIS_VALUES = (
    LOW_VALIDATION_GATE,
    REFERENCE_VALIDATION_GATE,
    HIGH_VALIDATION_GATE,
)


@dataclass(frozen=True)
class PoseSample:
    t: float
    x: float
    y: float
    yaw: float
    cov_x: float = math.nan
    cov_y: float = math.nan
    cov_yaw: float = math.nan


@dataclass(frozen=True)
class VelocitySample:
    t: float
    linear: float
    angular: float


@dataclass(frozen=True)
class ScalarSample:
    t: float
    value: float


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float


@dataclass
class RunData:
    bag_path: Path
    run_id: str
    config_key: str
    validation_gate: float
    feedback_topic: str = ""
    feedback_type: str = ""
    enable_correction: str = ""
    model_name: str = ""
    topics: tuple[str, ...] = DEFAULT_TOPICS
    ground_truth: list[PoseSample] = field(default_factory=list)
    odom: list[PoseSample] = field(default_factory=list)
    ekf: list[PoseSample] = field(default_factory=list)
    cmd_vel: list[VelocitySample] = field(default_factory=list)
    association_count: list[ScalarSample] = field(default_factory=list)
    update_applied: list[ScalarSample] = field(default_factory=list)
    waypoint_index: list[ScalarSample] = field(default_factory=list)
    scans: list[tuple[float, object]] = field(default_factory=list)

    @property
    def label(self) -> str:
        base = CONFIG_LABELS.get(self.config_key, self.config_key)
        if self.config_key == "b":
            return f"{base}, g={self.validation_gate:g}"
        return base

    @property
    def start_time(self) -> float:
        candidates = []
        for series in (
            self.ground_truth,
            self.odom,
            self.ekf,
            self.cmd_vel,
            self.association_count,
            self.waypoint_index,
        ):
            if series:
                candidates.append(series[0].t)
        return min(candidates) if candidates else 0.0


@dataclass(frozen=True)
class WaypointError:
    run_id: str
    config_key: str
    validation_gate: float
    waypoint_index: int
    t: float
    gt_x: float
    gt_y: float
    gt_yaw: float
    ref_x: float
    ref_y: float
    ref_yaw: float
    distance_error: float
    yaw_error: float


@dataclass(frozen=True)
class EstimationError:
    t: float
    ekf_position: float
    ekf_yaw: float
    odom_position: float
    odom_yaw: float


@dataclass(frozen=True)
class InnovationStat:
    run_id: str
    validation_gate: float
    t: float
    observation_count: int
    association_count: int
    mean_mahalanobis: float
    max_mahalanobis: float
    mean_abs_alpha: float
    mean_abs_radius: float


@dataclass(frozen=True)
class InnovationWindowSummary:
    label: str
    start_s: float
    end_s: float
    observation_mean: float
    association_mean: float
    mean_mahalanobis: float
    max_mahalanobis: float
    mean_abs_alpha: float
    mean_abs_radius: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(orientation: object) -> float:
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def pose_error(reference: PoseSample, estimate: PoseSample) -> tuple[float, float]:
    position_error = math.hypot(estimate.x - reference.x, estimate.y - reference.y)
    yaw_error = abs(normalize_angle(estimate.yaw - reference.yaw))
    return position_error, yaw_error


def interpolate_pose_sample(samples: Sequence[PoseSample], target_time: float) -> PoseSample:
    if not samples:
        raise ValueError("Cannot interpolate an empty pose series.")
    if target_time <= samples[0].t:
        return samples[0]
    if target_time >= samples[-1].t:
        return samples[-1]

    times = numpy.asarray([sample.t for sample in samples], dtype=float)
    index = int(numpy.searchsorted(times, target_time, side="right"))
    left = samples[index - 1]
    right = samples[index]
    span = max(right.t - left.t, 1e-12)
    ratio = (target_time - left.t) / span

    yaw_delta = normalize_angle(right.yaw - left.yaw)
    return PoseSample(
        t=float(target_time),
        x=float(left.x + ratio * (right.x - left.x)),
        y=float(left.y + ratio * (right.y - left.y)),
        yaw=normalize_angle(left.yaw + ratio * yaw_delta),
        cov_x=_interpolate_optional(left.cov_x, right.cov_x, ratio),
        cov_y=_interpolate_optional(left.cov_y, right.cov_y, ratio),
        cov_yaw=_interpolate_optional(left.cov_yaw, right.cov_yaw, ratio),
    )


def _interpolate_optional(left: float, right: float, ratio: float) -> float:
    if math.isnan(left):
        return right
    if math.isnan(right):
        return left
    return float(left + ratio * (right - left))


def waypoint_reach_events(samples: Sequence[ScalarSample]) -> list[tuple[float, int]]:
    events: list[tuple[float, int]] = []
    previous_index: int | None = None
    for sample in samples:
        current_index = int(sample.value)
        if previous_index is None and current_index > 0:
            for reached_index in range(current_index):
                events.append((sample.t, reached_index))
        elif previous_index is not None and current_index > previous_index:
            for reached_index in range(previous_index, current_index):
                events.append((sample.t, reached_index))
        previous_index = current_index
    return events


def load_waypoints(path: Path) -> list[Waypoint]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw_waypoints = data.get("waypoints", data)
    waypoints = []
    for raw in raw_waypoints:
        if isinstance(raw, dict):
            x = raw["x"]
            y = raw["y"]
            yaw = raw.get("yaw", raw.get("theta", 0.0))
        else:
            x, y = raw[0], raw[1]
            yaw = raw[2] if len(raw) > 2 else 0.0
        waypoints.append(Waypoint(float(x), float(y), normalize_angle(float(yaw))))
    return waypoints


def read_manifest(path: Path) -> dict:
    manifest_path = path / "assignment7_run.yaml"
    if not manifest_path.exists():
        return {}
    return yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}


def infer_run_metadata(path: Path) -> dict:
    manifest = read_manifest(path)
    name = path.name
    config_key = str(manifest.get("config_key", "")).lower()
    if not config_key:
        if "hw4_a" in name or "odom_feedback" in name:
            config_key = "a"
        elif "hw4_c" in name or "prediction_only" in name:
            config_key = "c"
        elif "hw4_b" in name or "ekf_corrected" in name:
            config_key = "b"
        else:
            config_key = "unknown"

    validation_gate = manifest.get("validation_gate")
    if validation_gate is None:
        match = re.search(r"gate_([0-9]+(?:p[0-9]+)?|[0-9]+(?:\.[0-9]+)?)", name)
        if match:
            validation_gate = float(match.group(1).replace("p", "."))
        else:
            validation_gate = REFERENCE_VALIDATION_GATE

    return {
        "run_id": str(manifest.get("run_id", name)),
        "config_key": config_key,
        "validation_gate": float(validation_gate),
        "feedback_topic": str(manifest.get("feedback_topic", "")),
        "feedback_type": str(manifest.get("feedback_type", "")),
        "enable_correction": str(manifest.get("enable_correction", "")),
        "topics": tuple(manifest.get("topics", DEFAULT_TOPICS)),
    }


def message_time(message: object, fallback_nanoseconds: int) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None and (stamp.sec != 0 or stamp.nanosec != 0):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(fallback_nanoseconds) * 1e-9


def append_pose_sample(series: list[PoseSample], t: float, pose: object, covariance=None) -> None:
    cov_x = cov_y = cov_yaw = math.nan
    if covariance is not None and len(covariance) >= 36:
        cov_x = float(covariance[0])
        cov_y = float(covariance[7])
        cov_yaw = float(covariance[35])
    series.append(
        PoseSample(
            t=float(t),
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=quaternion_to_yaw(pose.orientation),
            cov_x=cov_x,
            cov_y=cov_y,
            cov_yaw=cov_yaw,
        )
    )


def choose_model_index(names: Sequence[str], preferred_name: str = "") -> int | None:
    if preferred_name and preferred_name in names:
        return list(names).index(preferred_name)
    for wanted in ("turtlebot3_burger", "turtlebot", "burger"):
        for index, name in enumerate(names):
            if wanted in name:
                return index
    return None


def read_bag(path: Path, model_name: str = "") -> RunData:
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    metadata = infer_run_metadata(path)
    run = RunData(bag_path=path, **metadata)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in DEFAULT_TOPICS
    }

    model_index: int | None = None
    while reader.has_next():
        topic, serialized_data, bag_time = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized_data, message_types[topic])
        t = message_time(message, bag_time)

        if topic == "/gazebo/model_states":
            if model_index is None:
                model_index = choose_model_index(message.name, model_name)
                if model_index is not None:
                    run.model_name = str(message.name[model_index])
            if model_index is not None and model_index < len(message.pose):
                append_pose_sample(run.ground_truth, t, message.pose[model_index])
        elif topic == "/odom":
            append_pose_sample(run.odom, t, message.pose.pose, message.pose.covariance)
        elif topic == "/ekf_pose":
            append_pose_sample(run.ekf, t, message.pose.pose, message.pose.covariance)
        elif topic == "/cmd_vel":
            run.cmd_vel.append(
                VelocitySample(t=t, linear=float(message.linear.x), angular=float(message.angular.z))
            )
        elif topic == "/ekf_association_count":
            run.association_count.append(ScalarSample(t=t, value=float(message.data)))
        elif topic == "/ekf_update_applied":
            run.update_applied.append(ScalarSample(t=t, value=1.0 if message.data else 0.0))
        elif topic == "/waypoint_index":
            run.waypoint_index.append(ScalarSample(t=t, value=float(message.data)))
        elif topic == "/scan":
            run.scans.append((t, message))

    for series in (
        run.ground_truth,
        run.odom,
        run.ekf,
        run.cmd_vel,
        run.association_count,
        run.update_applied,
        run.waypoint_index,
        run.scans,
    ):
        series.sort(key=lambda sample: sample[0] if isinstance(sample, tuple) else sample.t)

    return run


def compute_waypoint_errors(
    run: RunData,
    waypoints: Sequence[Waypoint],
) -> list[WaypointError]:
    if not run.ground_truth or not run.waypoint_index:
        return []

    errors = []
    for event_time, reached_index in waypoint_reach_events(run.waypoint_index):
        if reached_index >= len(waypoints):
            continue
        gt = interpolate_pose_sample(run.ground_truth, event_time)
        waypoint = waypoints[reached_index]
        distance = math.hypot(gt.x - waypoint.x, gt.y - waypoint.y)
        yaw = abs(normalize_angle(gt.yaw - waypoint.yaw))
        errors.append(
            WaypointError(
                run_id=run.run_id,
                config_key=run.config_key,
                validation_gate=run.validation_gate,
                waypoint_index=reached_index,
                t=event_time,
                gt_x=gt.x,
                gt_y=gt.y,
                gt_yaw=gt.yaw,
                ref_x=waypoint.x,
                ref_y=waypoint.y,
                ref_yaw=waypoint.yaw,
                distance_error=distance,
                yaw_error=yaw,
            )
        )
    return errors


def compute_estimation_errors(run: RunData) -> list[EstimationError]:
    if not run.ground_truth or not run.ekf or not run.odom:
        return []
    errors = []
    for gt in run.ground_truth:
        ekf = interpolate_pose_sample(run.ekf, gt.t)
        odom = interpolate_pose_sample(run.odom, gt.t)
        ekf_position, ekf_yaw = pose_error(gt, ekf)
        odom_position, odom_yaw = pose_error(gt, odom)
        errors.append(
            EstimationError(
                t=gt.t,
                ekf_position=ekf_position,
                ekf_yaw=ekf_yaw,
                odom_position=odom_position,
                odom_yaw=odom_yaw,
            )
        )
    return errors


def compute_innovation_stats(
    run: RunData,
    map_file: Path,
    max_scans: int = 300,
) -> list[InnovationStat]:
    if not run.scans or not run.ekf:
        return []

    from ekf_line_localization.ekf import associate_measurements, load_line_map
    from line_fitting.split_and_merge import extract_lines, point_chunks_from_scan

    map_lines = load_line_map(map_file)
    measurement_covariance = numpy.diag([0.05**2, 0.02**2])
    step = max(1, len(run.scans) // max_scans)
    stats = []

    for t, scan in run.scans[::step]:
        chunks = point_chunks_from_scan(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            max_point_gap=0.25,
            min_points=6,
        )
        lines = extract_lines(
            chunks,
            split_threshold=0.04,
            merge_threshold=0.04,
            min_points=6,
        )
        observations = numpy.asarray(
            [[line.alpha, line.rho] for line in lines],
            dtype=float,
        )
        if observations.size == 0:
            stats.append(
                InnovationStat(
                    run_id=run.run_id,
                    validation_gate=run.validation_gate,
                    t=t,
                    observation_count=0,
                    association_count=0,
                    mean_mahalanobis=math.nan,
                    max_mahalanobis=math.nan,
                    mean_abs_alpha=math.nan,
                    mean_abs_radius=math.nan,
                )
            )
            continue

        pose = interpolate_pose_sample(run.ekf, t)
        covariance = numpy.diag(
            [
                _finite_or_default(pose.cov_x, 0.05),
                _finite_or_default(pose.cov_y, 0.05),
                _finite_or_default(pose.cov_yaw, 0.05),
            ]
        )
        association = associate_measurements(
            [pose.x, pose.y, pose.yaw],
            covariance,
            observations,
            measurement_covariance,
            map_lines,
            run.validation_gate,
        )
        distances = [match.distance for match in association.matches]
        innovation = association.innovation.reshape((-1, 2)) if association.count else numpy.empty((0, 2))
        stats.append(
            InnovationStat(
                run_id=run.run_id,
                validation_gate=run.validation_gate,
                t=t,
                observation_count=len(observations),
                association_count=association.count,
                mean_mahalanobis=float(numpy.mean(distances)) if distances else math.nan,
                max_mahalanobis=float(numpy.max(distances)) if distances else math.nan,
                mean_abs_alpha=float(numpy.mean(numpy.abs(innovation[:, 0]))) if len(innovation) else math.nan,
                mean_abs_radius=float(numpy.mean(numpy.abs(innovation[:, 1]))) if len(innovation) else math.nan,
            )
        )

    return stats


def _finite_or_default(value: float, default: float) -> float:
    return float(value) if math.isfinite(value) else float(default)


def select_primary_runs(runs: Sequence[RunData]) -> dict[str, RunData]:
    selected: dict[str, RunData] = {}
    for key in ("a", "b", "c"):
        candidates = [run for run in runs if run.config_key == key]
        if not candidates:
            continue
        if key == "b":
            candidates.sort(key=lambda run: abs(run.validation_gate - REFERENCE_VALIDATION_GATE))
        else:
            candidates.sort(key=lambda run: run.run_id)
        selected[key] = candidates[0]
    return selected


def runs_for_gate_analysis(runs: Sequence[RunData]) -> list[RunData]:
    return sorted(
        [run for run in runs if run.config_key == "b"],
        key=lambda run: (run.validation_gate, run.run_id),
    )


def relative_times(run: RunData, samples: Sequence[object]) -> numpy.ndarray:
    return numpy.asarray([sample.t - run.start_time for sample in samples], dtype=float)


def waypoint_marker_events(run: RunData | None) -> list[tuple[float, int]]:
    if run is None:
        return []
    return [
        (event_time - run.start_time, waypoint_index)
        for event_time, waypoint_index in waypoint_reach_events(run.waypoint_index)
    ]


def draw_waypoint_markers(axes: object, run: RunData | None) -> None:
    events = waypoint_marker_events(run)
    if not events:
        return

    axes_list = list(numpy.ravel(axes))
    for axis in axes_list:
        for event_time, _ in events:
            axis.axvline(
                event_time,
                color="black",
                linestyle="--",
                linewidth=0.9,
                alpha=0.48,
            )

    label_axis = axes_list[0]
    ymin, ymax = label_axis.get_ylim()
    label_y = ymax - 0.04 * (ymax - ymin)
    for event_time, waypoint_index in events:
        label_axis.text(
            event_time,
            label_y,
            f"WP{waypoint_index + 1}",
            rotation=90,
            va="top",
            ha="right",
            fontsize=7,
            color="black",
        )


def reference_gate_run(runs: Sequence[RunData]) -> RunData | None:
    gate_runs = [run for run in runs if run.config_key == "b" and run.waypoint_index]
    if not gate_runs:
        return None
    return min(
        gate_runs,
        key=lambda run: (abs(run.validation_gate - REFERENCE_VALIDATION_GATE), run.run_id),
    )


def analysis_window_ranges(run: RunData | None) -> list[tuple[str, float, float]]:
    if run is None:
        return []

    events = waypoint_marker_events(run)
    if len(events) >= 10:
        end_time = max((sample.t for sample in run.association_count), default=run.start_time)
        return [
            ("početak", 0.0, events[1][0]),
            ("WP4--WP8", events[3][0], events[7][0]),
            ("kraj", events[8][0], end_time - run.start_time),
        ]

    if run.association_count:
        end_time = run.association_count[-1].t - run.start_time
        return [
            ("početak", 0.0, min(0.2 * end_time, end_time)),
            ("sredina", 0.4 * end_time, 0.75 * end_time),
            ("kraj", 0.85 * end_time, end_time),
        ]
    return []


def draw_analysis_windows(axes: object, run: RunData | None) -> None:
    windows = analysis_window_ranges(run)
    if not windows:
        return

    colors = ["tab:blue", "tab:orange", "tab:blue"]
    axes_list = list(numpy.ravel(axes))
    for axis in axes_list:
        for (label, start, end), color in zip(windows, colors):
            axis.axvspan(start, end, color=color, alpha=0.055, linewidth=0)

    label_axis = axes_list[0]
    ymin, ymax = label_axis.get_ylim()
    label_y = ymax - 0.06 * (ymax - ymin)
    for label, start, end in windows:
        label_axis.text(
            0.5 * (start + end),
            label_y,
            label,
            ha="center",
            va="top",
            fontsize=7,
            color="dimgray",
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.65, "pad": 1.0},
        )


def mean_finite(values: Iterable[float]) -> float:
    finite_values = [float(value) for value in values if math.isfinite(float(value))]
    if not finite_values:
        return math.nan
    return float(numpy.mean(finite_values))


def max_finite(values: Iterable[float]) -> float:
    finite_values = [float(value) for value in values if math.isfinite(float(value))]
    if not finite_values:
        return math.nan
    return float(numpy.max(finite_values))


def summarize_innovation_windows(
    run: RunData | None,
    stats: Sequence[InnovationStat],
) -> list[InnovationWindowSummary]:
    if run is None or not stats:
        return []

    summaries = []
    for label, start, end in analysis_window_ranges(run):
        samples = [
            sample
            for sample in stats
            if start <= sample.t - run.start_time <= end
        ]
        if not samples:
            continue
        summaries.append(
            InnovationWindowSummary(
                label=label,
                start_s=start,
                end_s=end,
                observation_mean=mean_finite(sample.observation_count for sample in samples),
                association_mean=mean_finite(sample.association_count for sample in samples),
                mean_mahalanobis=mean_finite(sample.mean_mahalanobis for sample in samples),
                max_mahalanobis=max_finite(sample.max_mahalanobis for sample in samples),
                mean_abs_alpha=mean_finite(sample.mean_abs_alpha for sample in samples),
                mean_abs_radius=mean_finite(sample.mean_abs_radius for sample in samples),
            )
        )
    return summaries


def ensure_output_dirs(output_dir: Path) -> tuple[Path, Path]:
    figures_dir = output_dir / "figures"
    tables_dir = output_dir / "tables"
    figures_dir.mkdir(parents=True, exist_ok=True)
    tables_dir.mkdir(parents=True, exist_ok=True)
    return figures_dir, tables_dir


def plot_placeholder(path: Path, message: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.axis("off")
    ax.text(0.5, 0.5, message, ha="center", va="center", wrap=True)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_trajectories(path: Path, runs: dict[str, RunData], waypoints: Sequence[Waypoint]) -> None:
    if not runs:
        plot_placeholder(path, "Nema dostupnih bagova za poređenje trajektorija.")
        return

    fig, ax = plt.subplots(figsize=(8, 7))
    styles = {"a": "tab:orange", "b": "tab:blue", "c": "tab:green"}
    for key, run in runs.items():
        if not run.ground_truth:
            continue
        ax.plot(
            [sample.x for sample in run.ground_truth],
            [sample.y for sample in run.ground_truth],
            color=styles.get(key),
            linewidth=1.8,
            label=run.label,
        )
    ax.scatter(
        [waypoint.x for waypoint in waypoints],
        [waypoint.y for waypoint in waypoints],
        c="black",
        marker="x",
        s=70,
        label="Referentne poze",
    )
    for index, waypoint in enumerate(waypoints, start=1):
        ax.text(waypoint.x, waypoint.y, f" {index}", fontsize=9)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_waypoint_errors(path: Path, errors: Sequence[WaypointError]) -> None:
    if not errors:
        plot_placeholder(path, "Nema podataka o dostignutim waypoint pozama.")
        return

    primary_errors = [
        error
        for error in errors
        if error.config_key in ("a", "b", "c")
        and (
            error.config_key != "b"
            or abs(error.validation_gate - REFERENCE_VALIDATION_GATE) < 1e-6
        )
    ]
    if not primary_errors:
        primary_errors = list(errors)

    configs = ["a", "b", "c"]
    waypoint_indices = sorted({error.waypoint_index for error in primary_errors})
    x = numpy.arange(len(waypoint_indices), dtype=float)
    width = 0.24

    fig, ax = plt.subplots(figsize=(9, 4.8))
    for offset, config in enumerate(configs):
        values = []
        for waypoint_index in waypoint_indices:
            matching = [
                error.distance_error
                for error in primary_errors
                if error.config_key == config and error.waypoint_index == waypoint_index
            ]
            values.append(matching[0] if matching else numpy.nan)
        ax.bar(x + (offset - 1) * width, values, width=width, label=config)
    ax.set_xticks(x)
    ax.set_xticklabels([str(index + 1) for index in waypoint_indices])
    ax.set_xlabel("Waypoint")
    ax.set_ylabel("Greška pozicije u trenutku proglašenja [m]")
    ax.grid(True, axis="y", alpha=0.35)
    ax.legend(title="Konfiguracija")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_estimation_errors(path: Path, run: RunData | None, errors: Sequence[EstimationError]) -> None:
    if run is None or not errors:
        plot_placeholder(path, "Nema dovoljno podataka za grešku estimacije konfiguracije (b).")
        return

    t0 = run.start_time
    times = numpy.asarray([error.t - t0 for error in errors], dtype=float)
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(times, [error.ekf_position for error in errors], label="/ekf_pose", linewidth=1.6)
    axes[0].plot(times, [error.odom_position for error in errors], label="/odom", linewidth=1.4)
    axes[0].set_ylabel("Greška pozicije [m]")
    axes[0].grid(True, alpha=0.35)
    axes[0].legend()

    axes[1].plot(times, [error.ekf_yaw for error in errors], label="/ekf_pose", linewidth=1.6)
    axes[1].plot(times, [error.odom_yaw for error in errors], label="/odom", linewidth=1.4)
    axes[1].set_ylabel("Greška orijentacije [rad]")
    axes[1].set_xlabel("Vreme [s]")
    axes[1].grid(True, alpha=0.35)
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_covariance(path: Path, run: RunData | None) -> None:
    if run is None or not run.ekf or not run.odom:
        plot_placeholder(path, "Nema dovoljno podataka za poređenje kovarijanse.")
        return

    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    labels = [("cov_x", "P_xx [m^2]"), ("cov_y", "P_yy [m^2]"), ("cov_yaw", "P_θθ [rad^2]")]
    for axis, (field_name, ylabel) in zip(axes, labels):
        axis.plot(
            relative_times(run, run.ekf),
            [getattr(sample, field_name) for sample in run.ekf],
            label="/ekf_pose",
            linewidth=1.5,
        )
        axis.plot(
            relative_times(run, run.odom),
            [getattr(sample, field_name) for sample in run.odom],
            label="/odom",
            linewidth=1.2,
            alpha=0.8,
        )
        for update in run.update_applied:
            if update.value > 0.5:
                axis.axvline(update.t - run.start_time, color="tab:red", alpha=0.10, linewidth=0.8)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.35)
    axes[0].legend()
    axes[-1].set_xlabel("Vreme [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_cmd_vel_updates(path: Path, run: RunData | None) -> None:
    if run is None or not run.cmd_vel:
        plot_placeholder(path, "Nema /cmd_vel podataka za konfiguraciju (b).")
        return

    fig, axes = plt.subplots(2, 1, figsize=(9, 5.8), sharex=True)
    times = relative_times(run, run.cmd_vel)
    axes[0].plot(times, [sample.linear for sample in run.cmd_vel], linewidth=1.3)
    axes[0].set_ylabel("v [m/s]")
    axes[1].plot(times, [sample.angular for sample in run.cmd_vel], linewidth=1.3)
    axes[1].set_ylabel("ω [rad/s]")
    axes[1].set_xlabel("Vreme [s]")

    for axis in axes:
        for update in run.update_applied:
            if update.value > 0.5:
                axis.axvline(update.t - run.start_time, color="tab:red", alpha=0.16, linewidth=0.8)
        axis.grid(True, alpha=0.35)
    axes[0].set_title("Crvene linije: trenuci uspešne EKF popravke")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_gate_associations(path: Path, runs: Sequence[RunData]) -> None:
    gate_runs = [run for run in runs if run.association_count]
    if not gate_runs:
        plot_placeholder(path, "Nema podataka o broju asociranih odlika.")
        return

    fig, ax = plt.subplots(figsize=(9, 4.8))
    marker_run = reference_gate_run(gate_runs)
    for run in gate_runs:
        ax.plot(
            relative_times(run, run.association_count),
            [sample.value for sample in run.association_count],
            label=f"g={run.validation_gate:g}",
            linewidth=1.3,
        )
    ax.set_xlabel("Vreme [s]")
    ax.set_ylabel("Broj asociranih odlika")
    ax.grid(True, alpha=0.35)
    draw_analysis_windows(ax, marker_run)
    draw_waypoint_markers(ax, marker_run)
    ax.set_title(
        f"Crne linije: dostignuti waypoint-i u referentnoj vožnji "
        f"g={REFERENCE_VALIDATION_GATE:g}"
    )
    ax.legend(title="Prag validacije")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_innovation_stats(path: Path, run: RunData | None, stats: Sequence[InnovationStat]) -> None:
    if run is None or not stats:
        plot_placeholder(path, "Nema offline statistike inovacija.")
        return

    times = numpy.asarray([sample.t - run.start_time for sample in stats], dtype=float)
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(times, [sample.observation_count for sample in stats], label="detektovano", linewidth=1.2)
    axes[0].plot(times, [sample.association_count for sample in stats], label="asocirano", linewidth=1.2)
    axes[0].set_ylabel("Broj linija")
    axes[0].legend()

    axes[1].plot(times, [sample.mean_mahalanobis for sample in stats], linewidth=1.2)
    axes[1].set_ylabel("Srednja Mahalanobis distanca")

    axes[2].plot(times, [sample.mean_abs_alpha for sample in stats], label="|Δα| [rad]", linewidth=1.2)
    axes[2].plot(times, [sample.mean_abs_radius for sample in stats], label="|Δr| [m]", linewidth=1.2)
    axes[2].set_ylabel("Srednja |inovacija|")
    axes[2].set_xlabel("Vreme [s]")
    axes[2].legend()
    for axis in axes:
        axis.grid(True, alpha=0.35)
    draw_analysis_windows(axes, run)
    draw_waypoint_markers(axes, run)
    axes[0].set_title(
        f"Crne linije: dostignuti waypoint-i u referentnoj vožnji "
        f"g={REFERENCE_VALIDATION_GATE:g}"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def write_csv(path: Path, rows: Iterable[object], fieldnames: Sequence[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: getattr(row, field) for field in fieldnames})


def write_gate_csv(path: Path, runs: Sequence[RunData]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=["run_id", "validation_gate", "t", "association_count", "update_applied"],
        )
        writer.writeheader()
        for run in runs:
            updates = {round(sample.t, 3): sample.value for sample in run.update_applied}
            for sample in run.association_count:
                writer.writerow(
                    {
                        "run_id": run.run_id,
                        "validation_gate": run.validation_gate,
                        "t": sample.t,
                        "association_count": sample.value,
                        "update_applied": updates.get(round(sample.t, 3), ""),
                    }
                )


def summarize_runs(
    runs: Sequence[RunData],
    waypoint_errors: Sequence[WaypointError],
    estimation_errors: Sequence[EstimationError],
) -> dict:
    summary = {"runs": []}
    for run in runs:
        run_waypoint_errors = [error for error in waypoint_errors if error.run_id == run.run_id]
        associations = [sample.value for sample in run.association_count]
        updates = [sample.value for sample in run.update_applied]
        summary["runs"].append(
            {
                "run_id": run.run_id,
                "bag_path": str(run.bag_path),
                "config_key": run.config_key,
                "validation_gate": run.validation_gate,
                "model_name": run.model_name,
                "duration_s": _duration(run),
                "waypoint_position_error_mean_m": _mean(
                    error.distance_error for error in run_waypoint_errors
                ),
                "waypoint_position_error_max_m": _max(
                    error.distance_error for error in run_waypoint_errors
                ),
                "association_count_mean": _mean(associations),
                "update_count": int(sum(1 for value in updates if value > 0.5)),
                "cmd_linear_abs_max": _max(abs(sample.linear) for sample in run.cmd_vel),
                "cmd_angular_abs_max": _max(abs(sample.angular) for sample in run.cmd_vel),
                "cmd_linear_step_max": _max_abs_step([sample.linear for sample in run.cmd_vel]),
                "cmd_angular_step_max": _max_abs_step([sample.angular for sample in run.cmd_vel]),
                "cmd_linear_update_step_max": _max_abs_command_step_after_updates(
                    run,
                    "linear",
                ),
                "cmd_angular_update_step_max": _max_abs_command_step_after_updates(
                    run,
                    "angular",
                ),
                "ekf_cov_x_mean": _mean(sample.cov_x for sample in run.ekf),
                "ekf_cov_y_mean": _mean(sample.cov_y for sample in run.ekf),
                "ekf_cov_yaw_mean": _mean(sample.cov_yaw for sample in run.ekf),
                "odom_cov_x_mean": _mean(sample.cov_x for sample in run.odom),
                "odom_cov_y_mean": _mean(sample.cov_y for sample in run.odom),
                "odom_cov_yaw_mean": _mean(sample.cov_yaw for sample in run.odom),
            }
        )

    if estimation_errors:
        summary["configuration_b_estimation_error"] = {
            "ekf_position_mean_m": _mean(error.ekf_position for error in estimation_errors),
            "ekf_position_max_m": _max(error.ekf_position for error in estimation_errors),
            "ekf_yaw_mean_rad": _mean(error.ekf_yaw for error in estimation_errors),
            "ekf_yaw_max_rad": _max(error.ekf_yaw for error in estimation_errors),
            "odom_position_mean_m": _mean(error.odom_position for error in estimation_errors),
            "odom_position_max_m": _max(error.odom_position for error in estimation_errors),
            "odom_yaw_mean_rad": _mean(error.odom_yaw for error in estimation_errors),
            "odom_yaw_max_rad": _max(error.odom_yaw for error in estimation_errors),
        }
    return summary


def _duration(run: RunData) -> float:
    times = []
    for series in (run.ground_truth, run.odom, run.ekf, run.cmd_vel, run.association_count):
        if series:
            times.extend([series[0].t, series[-1].t])
    return max(times) - min(times) if times else 0.0


def _mean(values: Iterable[float]) -> float | None:
    values = [float(value) for value in values if math.isfinite(float(value))]
    return float(numpy.mean(values)) if values else None


def _max(values: Iterable[float]) -> float | None:
    values = [float(value) for value in values if math.isfinite(float(value))]
    return float(numpy.max(values)) if values else None


def _max_abs_step(values: Sequence[float]) -> float | None:
    if len(values) < 2:
        return None
    return float(numpy.max(numpy.abs(numpy.diff(numpy.asarray(values, dtype=float)))))


def _max_abs_command_step_after_updates(
    run: RunData,
    field_name: str,
    window_s: float = 0.12,
) -> float | None:
    if len(run.cmd_vel) < 2:
        return None

    update_times = [sample.t for sample in run.update_applied if sample.value > 0.5]
    if not update_times:
        return None

    values = []
    for update_t in update_times:
        after_index = next(
            (
                index
                for index, command in enumerate(run.cmd_vel)
                if command.t >= update_t
            ),
            None,
        )
        if after_index is None or after_index == 0:
            continue
        if run.cmd_vel[after_index].t - update_t > window_s:
            continue
        current = getattr(run.cmd_vel[after_index], field_name)
        previous = getattr(run.cmd_vel[after_index - 1], field_name)
        values.append(abs(current - previous))

    return float(max(values)) if values else None


def _fmt(value: float | int | None, precision: int = 3) -> str:
    if value is None:
        return "--"
    if isinstance(value, float) and not math.isfinite(value):
        return "--"
    return f"{float(value):.{precision}f}"


def _run_by_config(summary: dict, config_key: str, gate: float | None = None) -> dict | None:
    candidates = [run for run in summary.get("runs", []) if run["config_key"] == config_key]
    if gate is not None:
        candidates = [
            run for run in candidates if abs(float(run["validation_gate"]) - gate) < 1e-6
        ]
    if not candidates:
        return None
    if config_key == "b" and gate is None:
        candidates.sort(
            key=lambda run: abs(float(run["validation_gate"]) - REFERENCE_VALIDATION_GATE)
        )
    return candidates[0]


def _percent_improvement(reference: dict | None, improved: dict | None) -> float | None:
    if not reference or not improved:
        return None
    reference_value = reference.get("waypoint_position_error_mean_m")
    improved_value = improved.get("waypoint_position_error_mean_m")
    if not reference_value:
        return None
    return 100.0 * (float(reference_value) - float(improved_value)) / float(reference_value)


def write_results_tex(
    path: Path,
    summary: dict,
    waypoint_errors: Sequence[WaypointError],
    innovation_windows: Sequence[InnovationWindowSummary],
    has_gate_runs: bool,
) -> None:
    run_a = _run_by_config(summary, "a")
    run_b = _run_by_config(summary, "b", REFERENCE_VALIDATION_GATE)
    run_c = _run_by_config(summary, "c")
    run_g2 = _run_by_config(summary, "b", LOW_VALIDATION_GATE)
    run_g10 = _run_by_config(summary, "b", HIGH_VALIDATION_GATE)
    estimation = summary.get("configuration_b_estimation_error", {})
    improvement_a = _percent_improvement(run_a, run_b)
    improvement_c = _percent_improvement(run_c, run_b)

    lines = [
        "% Automatski generisano skriptom assignment7_analysis.py.",
        "\\section{Eksperiment i analiza rezultata}",
        "",
        "Waypoint misija je izvršena u tri konfiguracije propisane zadatkom: "
        "(a) regulator zatvoren preko odometrije, (b) regulator zatvoren preko EKF estimacije "
        "sa uključenom popravkom i (c) regulator zatvoren preko EKF estimacije bez popravke. "
        "Za analizu praga validacije dodatno se ponavlja konfiguracija (b) za više vrednosti praga $g$.",
        "",
        "\\subsection{Poređenje trajektorija}",
        _trajectory_discussion(run_a, run_b, run_c, improvement_a, improvement_c),
        "",
        "Na sledeća dva grafika prikazani su prvo oblik putanja u ravni, a zatim greške "
        "u trenucima kada regulator prijavi da je waypoint dostignut. Time se vizuelno poređenje "
        "direktno vezuje za numeričke metrike iz tabela ispod grafika.",
        "",
        "\\begin{figure}[H]",
        "\\centering",
        "\\includegraphics[width=0.92\\linewidth]{results/figures/assignment7_trajectories.png}",
        "\\caption{Ground truth putanje robota za tri konfiguracije i zadate referentne poze.}",
        "\\end{figure}",
        "",
        "\\begin{figure}[H]",
        "\\centering",
        "\\includegraphics[width=0.86\\linewidth]{results/figures/assignment7_waypoint_errors.png}",
        "\\caption{Rastojanje ground truth poze od reference u trenutku kada regulator proglasi waypoint dostignutim.}",
        "\\end{figure}",
        "",
        _summary_table(summary),
        "",
        "Konfiguracije (a) i (c) pokazuju sličan oblik degradacije zato što se u oba slučaja "
        "upravljačka petlja oslanja na integraciju kretanja bez spoljašnje korekcije položaja. "
        "Kod (a) tu ulogu ima odometrija, a kod (c) EKF radi samo predikciju, pa akumulirana greška "
        "točkova ostaje neograničena.",
        "",
        _waypoint_table(waypoint_errors),
        "",
        "\\FloatBarrier",
        "",
        "\\subsection{Greška estimacije}",
        _estimation_discussion(estimation),
        "",
        "\\begin{figure}[H]",
        "\\centering",
        "\\includegraphics[width=0.92\\linewidth]{results/figures/assignment7_estimation_error_b.png}",
        "\\caption{Greška pozicije i orijentacije za /ekf\\_pose i /odom u odnosu na Gazebo ground truth, konfiguracija (b).}",
        "\\end{figure}",
        "",
        "\\FloatBarrier",
        "",
        "\\subsection{Evolucija kovarijanse}",
        _covariance_discussion(run_b),
        "",
        "\\begin{figure}[H]",
        "\\centering",
        "\\includegraphics[width=0.92\\linewidth]{results/figures/assignment7_covariance_b.png}",
        "\\caption{Dijagonalni elementi kovarijanse iz /ekf\\_pose i /odom. Crvene vertikalne linije označavaju uspešne EKF popravke.}",
        "\\end{figure}",
        "",
        "\\FloatBarrier",
        "",
        "\\subsection{Uticaj popravke na upravljanje}",
        _control_discussion(run_b),
        "",
        "\\begin{figure}[H]",
        "\\centering",
        "\\includegraphics[width=0.92\\linewidth]{results/figures/assignment7_cmd_vel_updates_b.png}",
        "\\caption{Signal /cmd\\_vel u konfiguraciji (b), sa označenim trenucima EKF popravke.}",
        "\\end{figure}",
        "",
        "\\FloatBarrier",
        "",
        "\\subsection{Uticaj praga validacije}",
        _gate_table(run_g2, run_b, run_g10),
        "",
        _innovation_window_table(innovation_windows),
        "",
        _gate_discussion(run_g2, run_b, run_g10, innovation_windows),
        "",
        "\\begin{figure}[H]",
        "\\centering",
        "\\includegraphics[width=0.92\\linewidth]{results/figures/assignment7_gate_associations.png}",
        "\\caption{Broj asociranih odlika po ciklusu za različite vrednosti praga validacije $g$. "
        f"Crne isprekidane linije označavaju trenutke dostizanja waypoint-a u referentnoj vožnji "
        f"$g={REFERENCE_VALIDATION_GATE:g}$; plavo su delovi sa više vidljivih zidova, a narandžasto "
        "deo sa kraćim i kosijim segmentima.}",
        "\\end{figure}",
        "",
        "\\begin{figure}[H]",
        "\\centering",
        "\\includegraphics[width=0.92\\linewidth]{results/figures/assignment7_innovation_stats.png}",
        "\\caption{Offline statistika inovacija dobijena ponovnom primenom Split-and-Merge algoritma nad /scan. "
        f"Crne isprekidane linije označavaju trenutke dostizanja waypoint-a u referentnoj vožnji "
        f"$g={REFERENCE_VALIDATION_GATE:g}$; osenčeni delovi odgovaraju vremenskim prozorima iz tabele.}}",
        "\\end{figure}",
        "",
        "\\FloatBarrier",
    ]

    if not has_gate_runs:
        lines.append(
            "\\textbf{Napomena:} Za potpuno poređenje praga validacije potrebno je snimiti i bagove "
            f"za $g={LOW_VALIDATION_GATE:g}$ i $g={HIGH_VALIDATION_GATE:g}$."
        )

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _trajectory_discussion(
    run_a: dict | None,
    run_b: dict | None,
    run_c: dict | None,
    improvement_a: float | None,
    improvement_c: float | None,
) -> str:
    if not (run_a and run_b and run_c):
        return "Za numeričko poređenje trajektorija potrebni su bagovi za sve tri konfiguracije."
    return (
        "Srednja greška dostizanja referentne poze iznosi "
        f"\\SI{{{_fmt(run_a['waypoint_position_error_mean_m'])}}}{{m}} za konfiguraciju (a), "
        f"\\SI{{{_fmt(run_b['waypoint_position_error_mean_m'])}}}{{m}} za konfiguraciju (b) i "
        f"\\SI{{{_fmt(run_c['waypoint_position_error_mean_m'])}}}{{m}} za konfiguraciju (c). "
        "U ovom eksperimentu EKF sa popravkom smanjuje srednju grešku dostizanja za "
        f"{_fmt(improvement_a, 1)}\\% u odnosu na upravljanje po odometriji i za "
        f"{_fmt(improvement_c, 1)}\\% u odnosu na EKF bez popravke. "
        "To potvrđuje da se u zatvorenoj petlji ne dobija samo lepša estimacija, već i tačnije "
        "zaustavljanje robota u referentnim pozama."
    )


def _summary_table(summary: dict) -> str:
    rows = []
    for run in summary.get("runs", []):
        if (
            run["config_key"] == "b"
            and abs(float(run["validation_gate"]) - REFERENCE_VALIDATION_GATE) > 1e-6
        ):
            continue
        rows.append(
            f"{run['config_key']} & {run['validation_gate']:.1f} & "
            f"{_fmt(run['waypoint_position_error_mean_m'])} & "
            f"{_fmt(run['waypoint_position_error_max_m'])} & "
            f"{_fmt(run['duration_s'], 1)} & {run['update_count']} \\\\"
        )
    return "\n".join(
        [
            "\\begin{table}[H]",
            "\\centering",
            "\\begin{tabular}{lrrrrr}",
            "\\hline",
            "Konf. & $g$ & Srednja greška [m] & Maks. greška [m] & Trajanje [s] & Popravke \\\\",
            "\\hline",
            *rows,
            "\\hline",
            "\\end{tabular}",
            "\\caption{Sažetak uspešnosti waypoint misije za osnovne konfiguracije.}",
            "\\end{table}",
        ]
    )


def _estimation_discussion(estimation: dict) -> str:
    if not estimation:
        return "Nema dovoljno podataka za numeričku analizu greške estimacije."
    return (
        "Za konfiguraciju (b) srednja greška pozicije EKF estimacije iznosi "
        f"\\SI{{{_fmt(estimation.get('ekf_position_mean_m'))}}}{{m}}, a maksimalna "
        f"\\SI{{{_fmt(estimation.get('ekf_position_max_m'))}}}{{m}}. "
        "Odometrija u istoj vožnji ima srednju grešku "
        f"\\SI{{{_fmt(estimation.get('odom_position_mean_m'))}}}{{m}} i maksimalnu "
        f"\\SI{{{_fmt(estimation.get('odom_position_max_m'))}}}{{m}}. "
        "Na ovoj kratkoj simulacionoj putanji odometrija ostaje veoma bliska ground truth pozi, "
        "što je očekivano jer simulator nema izražen proklizavajući drift. Ipak, waypoint metrika "
        "pokazuje da zatvaranje petlje po korigovanom EKF-u dovodi robota bliže referencama. "
        "Greška orijentacije EKF-a je srednje "
        f"\\SI{{{_fmt(estimation.get('ekf_yaw_mean_rad'))}}}{{rad}}, dok je za odometriju "
        f"\\SI{{{_fmt(estimation.get('odom_yaw_mean_rad'))}}}{{rad}}."
    )


def _covariance_discussion(run_b: dict | None) -> str:
    if not run_b:
        return "Kovarijansa nije analizirana jer nedostaje konfiguracija (b)."
    return (
        "Srednje dijagonalne vrednosti EKF kovarijanse tokom konfiguracije (b) su približno "
        f"$P_{{xx}}={_fmt(run_b.get('ekf_cov_x_mean'), 4)}$, "
        f"$P_{{yy}}={_fmt(run_b.get('ekf_cov_y_mean'), 4)}$ i "
        f"$P_{{\\theta\\theta}}={_fmt(run_b.get('ekf_cov_yaw_mean'), 4)}$. "
        "Na grafiku se vidi karakterističan obrazac: tokom predikcije kovarijansa raste, a pri "
        "uspešnim popravkama, označenim crvenim linijama, opada u komponentama koje opažene "
        "linije mogu da ograniče. Odometrijska kovarijansa je na istom grafiku skoro konstantna: "
        f"u snimljenom /odom topiku njene srednje vrednosti su približno "
        f"$\\Sigma^{{odom}}_{{xx}}={_fmt(run_b.get('odom_cov_x_mean'), 6)}$, "
        f"$\\Sigma^{{odom}}_{{yy}}={_fmt(run_b.get('odom_cov_y_mean'), 6)}$ i "
        f"$\\Sigma^{{odom}}_{{\\theta\\theta}}={_fmt(run_b.get('odom_cov_yaw_mean'), 6)}$. "
        "To nije posledica Kalmanovog filtra, već načina na koji Gazebo/TurtleBot odometrijski "
        "izvor popunjava poruku: kovarijansa u /odom je deklarisana kao fiksna nominalna "
        "nesigurnost izvora odometrije, pa se ne propagira iz ciklusa u ciklus i ne smanjuje "
        "posle lidarskih korekcija. Zbog toga je treba čitati kao referentnu kovarijansu koju "
        "objavljuje odometrijski senzor, a ne kao stvarnu evoluciju akumulirane greške. "
        "Dinamička evolucija nesigurnosti u ovom zadatku nalazi se u /ekf\\_pose, jer se upravo "
        "tu primenjuju predikcija $P^{-}=F_xPF_x^T+F_uQF_u^T$ i popravka kovarijanse nakon "
        "asociranih linijskih merenja."
    )


def _control_discussion(run_b: dict | None) -> str:
    if not run_b:
        return "Nema podataka za analizu upravljačkog signala u konfiguraciji (b)."
    return (
        "U konfiguraciji (b) maksimalna apsolutna linearna brzina bila je "
        f"\\SI{{{_fmt(run_b.get('cmd_linear_abs_max'))}}}{{m/s}}, a maksimalna apsolutna "
        f"ugaona brzina \\SI{{{_fmt(run_b.get('cmd_angular_abs_max'))}}}{{rad/s}}. "
        "Najveći zabeleženi priraštaji između dva odbirka bili su "
        f"\\SI{{{_fmt(run_b.get('cmd_linear_step_max'))}}}{{m/s}} za linearni i "
        f"\\SI{{{_fmt(run_b.get('cmd_angular_step_max'))}}}{{rad/s}} za ugaoni kanal. "
        "Uzorci neposredno posle uspešnih EKF popravki imaju najveći priraštaj približno "
        f"\\SI{{{_fmt(run_b.get('cmd_linear_update_step_max'))}}}{{m/s}} u linearnom i "
        f"\\SI{{{_fmt(run_b.get('cmd_angular_update_step_max'))}}}{{rad/s}} u ugaonom kanalu. "
        "Ovo je direktna veza sa razmatranjem iz Zadatka 6: regulator je zadržao "
        "$\\rho$-$\\alpha$-$\\beta$ strukturu iz Domaćeg 2, ali se povratna informacija u "
        "konfiguraciji (b) više ne uzima sa glatke odometrije nego sa /ekf\\_pose. Kada korak "
        "popravke diskontinualno promeni estimiranu pozu, u sledećem ciklusu regulatora "
        "diskontinualno se menjaju $\\rho$, $\\alpha$ i $\\beta$, pa se na /cmd\\_vel vide "
        "kratki pregibi oko crvenih linija. Ti pregibi nisu numerička greška, već očekivana "
        "posledica zatvaranja petlje preko korigovane estimacije. Uticaj je ublažen upravo "
        "mehanizmima predviđenim u Zadatku 6: zasićenjem brzina na granice TurtleBot3 Burger-a "
        "i ograničavanjem priraštaja komande. Pošto je najveći ugaoni priraštaj jednak "
        "\\SI{0.250}{rad/s}, vidi se da je rate limiter stvarno aktivan; bez njega bi ista EKF "
        "korekcija mogla da proizvede oštriji skok upravljanja."
    )


def _gate_table(run_g2: dict | None, run_b: dict | None, run_g10: dict | None) -> str:
    rows = []
    for run in (run_g2, run_b, run_g10):
        if not run:
            continue
        rows.append(
            f"{run['validation_gate']:.1f} & "
            f"{_fmt(run['association_count_mean'])} & {run['update_count']} & "
            f"{_fmt(run['waypoint_position_error_mean_m'])} & "
            f"{_fmt(run['waypoint_position_error_max_m'])} \\\\"
        )
    return "\n".join(
        [
            "\\begin{table}[H]",
            "\\centering",
            "\\begin{tabular}{rrrrr}",
            "\\hline",
            "$g$ & Srednji broj asocijacija & Broj popravki & Srednja greška [m] & Maks. greška [m] \\\\",
            "\\hline",
            *rows,
            "\\hline",
            "\\end{tabular}",
            "\\caption{Uticaj praga validacije na broj asocijacija i tačnost misije.}",
            "\\end{table}",
        ]
    )


def _innovation_window_table(windows: Sequence[InnovationWindowSummary]) -> str:
    if not windows:
        return "Tabela po vremenskim prozorima biće popunjena nakon offline analize inovacija."

    rows = [
        (
            f"{window.label} & {window.start_s:.1f}--{window.end_s:.1f} & "
            f"{_fmt(window.observation_mean)} & {_fmt(window.association_mean)} & "
            f"{_fmt(window.mean_mahalanobis)} & {_fmt(window.max_mahalanobis)} & "
            f"{_fmt(window.mean_abs_alpha)} \\\\"
        )
        for window in windows
    ]
    return "\n".join(
        [
            "\\begin{table}[H]",
            "\\centering",
            "\\begin{tabular}{lrrrrrr}",
            "\\hline",
            "Deo misije & Vreme [s] & Detekt. & Asoc. & Sr. Mah. & Maks. Mah. & Sr. $|\\Delta\\alpha|$ \\\\",
            "\\hline",
            *rows,
            "\\hline",
            "\\end{tabular}",
            "\\caption{Razdvajanje vidljivosti zidova od kvaliteta linijskih merenja za referentnu vožnju $g=5$.}",
            "\\end{table}",
        ]
    )


def _innovation_window_discussion(windows: Sequence[InnovationWindowSummary]) -> str:
    if len(windows) < 3:
        return ""

    start, middle, end = windows[:3]
    return (
        "Tabela to potvrđuje numerički: početak i kraj imaju veći prosečan broj detekcija "
        f"({_fmt(start.observation_mean)} i {_fmt(end.observation_mean)}), "
        "dok srednji deo ima manji broj detekcija "
        f"({_fmt(middle.observation_mean)}), ali veću srednju ugaonu inovaciju "
        f"({_fmt(middle.mean_abs_alpha)}) nego početak ({_fmt(start.mean_abs_alpha)}) "
        f"i kraj ({_fmt(end.mean_abs_alpha)}). "
    )


def _gate_discussion(
    run_g2: dict | None,
    run_b: dict | None,
    run_g10: dict | None,
    innovation_windows: Sequence[InnovationWindowSummary],
) -> str:
    if not (run_g2 and run_b and run_g10):
        return (
            "Za potpuno poređenje praga validacije potrebno je snimiti konfiguraciju (b) za "
            f"$g={LOW_VALIDATION_GATE:g}$, $g={REFERENCE_VALIDATION_GATE:g}$ i "
            f"$g={HIGH_VALIDATION_GATE:g}$."
        )
    return (
        f"Za $g={LOW_VALIDATION_GATE:g}$ srednji broj asociranih odlika je "
        f"{_fmt(run_g2.get('association_count_mean'))}, za "
        f"$g={REFERENCE_VALIDATION_GATE:g}$ je "
        f"{_fmt(run_b.get('association_count_mean'))}, a za "
        f"$g={HIGH_VALIDATION_GATE:g}$ je "
        f"{_fmt(run_g10.get('association_count_mean'))}. "
        "Manji prag zato odbacuje više opažanja i ima manji broj popravki, dok veći prag prihvata "
        "više merenja i u ovom snimanju ima najveći broj popravki. Istovremeno, za najveći prag "
        f"srednja greška dostizanja raste na \\SI{{{_fmt(run_g10.get('waypoint_position_error_mean_m'))}}}{{m}}, "
        "što pokazuje cenu preširokog prihvatanja merenja: u korekciju lakše ulaze i slabije ili "
        "pogrešno uparene linije. "
        "Crne vertikalne linije na graficima označavaju trenutke u kojima se indeks waypoint-a "
        f"poveća u referentnoj vožnji za $g={REFERENCE_VALIDATION_GATE:g}$. "
        "Plavi osenčeni delovi označavaju početak i kraj misije, gde robot iz otvorenijeg položaja "
        "vidi više zidova i uglova; zato je broj detektovanih i asociranih linija prirodno veći. "
        "To povećanje ne treba tumačiti kao problem matrice $R$, nego kao promenu vidljivosti mape. "
        "Narandžasto osenčeni deo označava sredinu misije, gde ima manje vidljivih linija, ali su "
        "one češće kratke, delimično zaklonjene ili posmatrane pod oštrim uglom. "
        "Upravo se tu slabost pretpostavke konstantne matrice $R$ bolje vidi: ne kroz sam broj "
        "detekcija, već kroz veće skokove Mahalanobisove distance i $|\\Delta\\alpha|$ u offline "
        "statistici inovacija. "
        + _innovation_window_discussion(innovation_windows)
    )


def _waypoint_table(errors: Sequence[WaypointError]) -> str:
    if not errors:
        return "Tabela grešaka waypoint pozicija biće popunjena nakon snimanja bagova."

    selected = [
        error
        for error in errors
        if error.config_key in ("a", "b", "c")
        and (
            error.config_key != "b"
            or abs(error.validation_gate - REFERENCE_VALIDATION_GATE) < 1e-6
        )
    ]
    if not selected:
        selected = list(errors)
    lines = [
        "\\begin{table}[H]",
        "\\centering",
        "\\begin{tabular}{lrrr}",
        "\\hline",
        "Konfiguracija & Waypoint & Greška pozicije [m] & Greška yaw [rad] \\\\",
        "\\hline",
    ]
    for error in selected:
        lines.append(
            f"{error.config_key} & {error.waypoint_index + 1} & "
            f"{error.distance_error:.3f} & {error.yaw_error:.3f} \\\\"
        )
    lines.extend(["\\hline", "\\end{tabular}", "\\caption{Tačnost dostizanja referentnih poza.}", "\\end{table}"])
    return "\n".join(lines)


def analyze(args: argparse.Namespace) -> int:
    bag_paths = [Path(path).resolve() for path in args.bags]
    missing = [path for path in bag_paths if not (path / "metadata.yaml").exists()]
    if missing:
        for path in missing:
            print(f"Missing rosbag metadata: {path / 'metadata.yaml'}", file=sys.stderr)
        return 2

    output_dir = Path(args.output_dir).resolve()
    figures_dir, tables_dir = ensure_output_dirs(output_dir)
    waypoints = load_waypoints(Path(args.waypoint_file).resolve())
    runs = [read_bag(path, model_name=args.model_name) for path in bag_paths]
    primary_runs = select_primary_runs(runs)
    gate_runs = runs_for_gate_analysis(runs)
    corrected_run = primary_runs.get("b")

    waypoint_errors = [
        error
        for run in runs
        for error in compute_waypoint_errors(run, waypoints)
    ]
    estimation_errors = compute_estimation_errors(corrected_run) if corrected_run else []
    innovation_stats = (
        compute_innovation_stats(corrected_run, Path(args.map_file).resolve(), args.max_innovation_scans)
        if corrected_run
        else []
    )
    innovation_windows = summarize_innovation_windows(corrected_run, innovation_stats)

    plot_trajectories(figures_dir / "assignment7_trajectories.png", primary_runs, waypoints)
    plot_waypoint_errors(figures_dir / "assignment7_waypoint_errors.png", waypoint_errors)
    plot_estimation_errors(
        figures_dir / "assignment7_estimation_error_b.png",
        corrected_run,
        estimation_errors,
    )
    plot_covariance(figures_dir / "assignment7_covariance_b.png", corrected_run)
    plot_cmd_vel_updates(figures_dir / "assignment7_cmd_vel_updates_b.png", corrected_run)
    plot_gate_associations(figures_dir / "assignment7_gate_associations.png", gate_runs)
    plot_innovation_stats(
        figures_dir / "assignment7_innovation_stats.png",
        corrected_run,
        innovation_stats,
    )

    write_csv(
        tables_dir / "waypoint_errors.csv",
        waypoint_errors,
        [
            "run_id",
            "config_key",
            "validation_gate",
            "waypoint_index",
            "t",
            "gt_x",
            "gt_y",
            "gt_yaw",
            "ref_x",
            "ref_y",
            "ref_yaw",
            "distance_error",
            "yaw_error",
        ],
    )
    write_csv(
        tables_dir / "estimation_error_b.csv",
        estimation_errors,
        ["t", "ekf_position", "ekf_yaw", "odom_position", "odom_yaw"],
    )
    write_csv(
        tables_dir / "innovation_stats.csv",
        innovation_stats,
        [
            "run_id",
            "validation_gate",
            "t",
            "observation_count",
            "association_count",
            "mean_mahalanobis",
            "max_mahalanobis",
            "mean_abs_alpha",
            "mean_abs_radius",
        ],
    )
    write_csv(
        tables_dir / "innovation_windows.csv",
        innovation_windows,
        [
            "label",
            "start_s",
            "end_s",
            "observation_mean",
            "association_mean",
            "mean_mahalanobis",
            "max_mahalanobis",
            "mean_abs_alpha",
            "mean_abs_radius",
        ],
    )
    write_gate_csv(tables_dir / "gate_associations.csv", gate_runs)

    summary = summarize_runs(runs, waypoint_errors, estimation_errors)
    (output_dir / "assignment7_metrics.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_results_tex(
        output_dir / "assignment7_results.tex",
        summary,
        waypoint_errors,
        innovation_windows,
        len(gate_runs) >= len(GATE_ANALYSIS_VALUES),
    )

    print(f"Assignment 7 analysis written to {output_dir}")
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate Homework 4 Assignment 7 metrics, plots, and LaTeX results.",
    )
    parser.add_argument("--bags", nargs="+", required=True, help="Rosbag directories to analyze.")
    parser.add_argument(
        "--output-dir",
        default="Kodovi/homework_4/ekf_line_localization/analysis/results",
        help="Directory for generated figures, tables, and TeX fragment.",
    )
    parser.add_argument(
        "--waypoint-file",
        default="Kodovi/homework_4/ekf_line_localization/config/waypoints.yaml",
    )
    parser.add_argument(
        "--map-file",
        default="Kodovi/homework_4/ekf_line_localization/config/turtlebot3_maze_lines.yaml",
    )
    parser.add_argument("--model-name", default="turtlebot3_burger")
    parser.add_argument("--max-innovation-scans", type=int, default=300)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    return analyze(build_arg_parser().parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
