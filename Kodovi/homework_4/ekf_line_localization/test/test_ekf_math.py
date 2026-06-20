import math

import numpy

from ekf_line_localization.ekf import (
    associate_measurements,
    filter_step,
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
