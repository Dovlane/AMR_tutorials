import math

import numpy

from ekf_line_localization.ekf import (
    associate_measurements,
    filter_step,
    load_line_map,
    load_line_segments,
    measurement_function,
    transition_function,
)


def test_transition_function_straight_motion():
    pose, fx, fu = transition_function([0.0, 0.0, 0.0], [0.1, 0.1], 0.16)

    numpy.testing.assert_allclose(pose, [0.1, 0.0, 0.0], atol=1e-9)
    numpy.testing.assert_allclose(fx[0:2, 0:2], numpy.eye(2), atol=1e-9)
    assert fu.shape == (3, 2)


def test_measurement_function_global_line_in_robot_frame():
    measurement, hx = measurement_function([0.2, 0.0, 0.0], [0.0, 1.0])

    numpy.testing.assert_allclose(measurement, [0.0, 0.8], atol=1e-9)
    numpy.testing.assert_allclose(hx, [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]])


def test_association_and_filter_step_reduce_line_radius_error():
    pose = numpy.array([0.0, 0.0, 0.0])
    covariance = numpy.diag([0.1, 0.1, 0.05])
    observations = numpy.array([[0.0, 0.9]])
    measurement_covariance = numpy.diag([0.05**2, 0.02**2])
    map_lines = numpy.array([[0.0, 1.0]])

    association = associate_measurements(
        pose,
        covariance,
        observations,
        measurement_covariance,
        map_lines,
        validation_gate=5.0,
    )
    corrected_pose, corrected_covariance = filter_step(
        pose,
        covariance,
        association.innovation,
        association.jacobian,
        association.covariance,
    )

    assert association.count == 1
    assert corrected_pose[0] > 0.0
    assert corrected_covariance[0, 0] < covariance[0, 0]
    assert math.isfinite(corrected_covariance[2, 2])


def test_map_loader_keeps_segments_separate_from_line_features(tmp_path):
    map_file = tmp_path / "lines.yaml"
    map_file.write_text(
        """
walls:
  1: [0.0, 1.0]
  2: [1.5708, 2.0]
segments:
  1: [[1.0, -0.5], [1.0, 0.5]]
  2: [[-0.5, 2.0], [0.5, 2.0]]
""",
        encoding="utf-8",
    )

    lines = load_line_map(map_file)
    segments = load_line_segments(map_file)

    numpy.testing.assert_allclose(lines, [[0.0, 1.0], [1.5708, 2.0]])
    numpy.testing.assert_allclose(
        segments,
        [
            [[1.0, -0.5], [1.0, 0.5]],
            [[-0.5, 2.0], [0.5, 2.0]],
        ],
    )
