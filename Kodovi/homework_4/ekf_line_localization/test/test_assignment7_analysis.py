import math
from pathlib import Path
import sys
from types import SimpleNamespace


ANALYSIS_DIR = Path(__file__).resolve().parents[1] / "analysis"
sys.path.insert(0, str(ANALYSIS_DIR))

from assignment7_analysis import (  # noqa: E402
    PoseSample,
    RunData,
    ScalarSample,
    VelocitySample,
    _max_abs_command_step_after_updates,
    interpolate_pose_sample,
    normalize_angle,
    pose_error,
    quaternion_to_yaw,
    waypoint_reach_events,
)


def test_quaternion_to_yaw():
    yaw = math.pi / 2.0
    orientation = SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )

    assert math.isclose(quaternion_to_yaw(orientation), yaw, abs_tol=1e-9)


def test_interpolate_pose_sample_wraps_yaw_short_way():
    samples = [
        PoseSample(0.0, 0.0, 0.0, math.radians(170.0)),
        PoseSample(2.0, 2.0, 4.0, math.radians(-170.0)),
    ]

    midpoint = interpolate_pose_sample(samples, 1.0)

    assert math.isclose(midpoint.x, 1.0, abs_tol=1e-9)
    assert math.isclose(midpoint.y, 2.0, abs_tol=1e-9)
    assert math.isclose(abs(midpoint.yaw), math.pi, abs_tol=1e-9)


def test_waypoint_reach_events_detects_index_increases():
    samples = [
        ScalarSample(0.0, 0.0),
        ScalarSample(1.0, 0.0),
        ScalarSample(2.0, 1.0),
        ScalarSample(3.0, 3.0),
    ]

    assert waypoint_reach_events(samples) == [(2.0, 0), (3.0, 1), (3.0, 2)]


def test_waypoint_reach_events_handles_bag_started_after_first_reach():
    samples = [
        ScalarSample(2.0, 1.0),
        ScalarSample(3.0, 2.0),
    ]

    assert waypoint_reach_events(samples) == [(2.0, 0), (3.0, 1)]


def test_pose_error_uses_euclidean_position_and_wrapped_yaw():
    reference = PoseSample(0.0, 1.0, 2.0, math.radians(179.0))
    estimate = PoseSample(0.0, 4.0, 6.0, math.radians(-179.0))

    position_error, yaw_error = pose_error(reference, estimate)

    assert math.isclose(position_error, 5.0, abs_tol=1e-9)
    assert math.isclose(yaw_error, math.radians(2.0), abs_tol=1e-9)
    assert -math.pi <= normalize_angle(10.0) <= math.pi


def test_max_abs_command_step_after_updates_uses_first_command_after_correction():
    run = RunData(Path("/tmp/example_bag"), "run", "b", 5.0)
    run.cmd_vel = [
        VelocitySample(0.00, 0.00, 0.00),
        VelocitySample(0.02, 0.02, 0.10),
        VelocitySample(0.04, 0.05, 0.35),
        VelocitySample(0.20, 0.06, 0.37),
    ]
    run.update_applied = [
        ScalarSample(0.031, 1.0),
        ScalarSample(0.50, 1.0),
    ]

    assert math.isclose(
        _max_abs_command_step_after_updates(run, "linear"),
        0.03,
        abs_tol=1e-12,
    )
    assert math.isclose(
        _max_abs_command_step_after_updates(run, "angular"),
        0.25,
        abs_tol=1e-12,
    )
