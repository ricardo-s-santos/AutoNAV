"""This file contains the GTRS.py tests."""

import re

import pytest
from numpy import array
from numpy.testing import assert_allclose, assert_array_equal

from autonav.GTRS import _calc_eigen, gtrs


@pytest.mark.critical()
def test_gtrs_no_noise(default_values, expected_trajectories_gtrs_sigma_0):
    """Tests if the algorithm is correctly implemented by setting the noise to zero."""
    # This test does not use seeds because the noise is set to zero.
    # Values used in test
    sigma = 0  # Noise STD in meters
    trajectories = gtrs(
        default_values.a_i,
        default_values.n,
        default_values.k,
        sigma,
        default_values.destinations,
        default_values.initial_uav_position,
        default_values.v_max,
        default_values.tau,
        default_values.gamma,
    )
    gtrs_estimated_trajectory = trajectories[0]
    gtrs_true_trajectory = trajectories[1]
    # With sigma zero the trajectories should be the following ones if one performs the math
    assert_allclose(expected_trajectories_gtrs_sigma_0[0], gtrs_estimated_trajectory)
    assert_allclose(expected_trajectories_gtrs_sigma_0[1], gtrs_true_trajectory)


@pytest.mark.critical()
def test_gtrs_reproducibility(default_values, seeds):
    """Tests if the algorithm is reproducible."""
    # Values used in test
    sigma = 1  # Noise STD in meters
    trajectories = gtrs(
        default_values.a_i,
        default_values.n,
        default_values.k,
        sigma,
        default_values.destinations,
        default_values.initial_uav_position,
        default_values.v_max,
        default_values.tau,
        default_values.gamma,
        seeds,
    )
    gtrs_estimated_trajectory_1 = trajectories[0]
    gtrs_true_trajectory_1 = trajectories[1]
    # Call GTRS again
    trajectories = gtrs(
        default_values.a_i,
        default_values.n,
        default_values.k,
        sigma,
        default_values.destinations,
        default_values.initial_uav_position,
        default_values.v_max,
        default_values.tau,
        default_values.gamma,
        seeds,
    )
    gtrs_estimated_trajectory_2 = trajectories[0]
    gtrs_true_trajectory_2 = trajectories[1]
    # Check to see if both results are equal
    assert_allclose(gtrs_estimated_trajectory_1, gtrs_estimated_trajectory_2)
    assert_allclose(gtrs_true_trajectory_1, gtrs_true_trajectory_2)


@pytest.mark.critical()
def test_gtrs_exceptions(default_values):
    """Tests if the expected exceptions are raised with wrong arguments."""
    sigma = 0
    # Case n != size(a_i, axis=1)
    with pytest.raises(ValueError, match=re.escape("The length of a_i must be equal to N.")):
        gtrs(
            default_values.a_i,
            10,
            default_values.k,
            sigma,
            default_values.destinations,
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
        )
    # Case k < 0
    with pytest.raises(ValueError, match=re.escape("K must be positive.")):
        gtrs(
            default_values.a_i,
            default_values.n,
            -1,
            sigma,
            default_values.destinations,
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
        )
    # Case sigma < 0
    with pytest.raises(ValueError, match=re.escape("Sigma must be between 0 and 5.")):
        gtrs(
            default_values.a_i,
            default_values.n,
            default_values.k,
            -1,
            default_values.destinations,
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
        )
    # Case destinations with wrong coordinates
    with pytest.raises(ValueError, match=re.escape("Waypoints must contain the 3 coordinates (x, y, z).")):
        gtrs(
            default_values.a_i,
            default_values.n,
            default_values.k,
            sigma,
            array([[1, 2]]),
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
        )
    # Case empty destinations
    with pytest.raises(ValueError, match=re.escape("Waypoints cannot be empty.")):
        gtrs(
            default_values.a_i,
            default_values.n,
            default_values.k,
            sigma,
            array([]),
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
        )
    # Case initial_uav_position with wrong coordinates
    with pytest.raises(ValueError, match=re.escape("Initial UAV position must contain the 3 coordinates (x, y, z).")):
        gtrs(
            default_values.a_i,
            default_values.n,
            default_values.k,
            sigma,
            default_values.destinations,
            [1, 2],
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
        )
    # Validate optional parameters
    # Case tol < 0
    with pytest.raises(ValueError, match=re.escape("Tolerance must be positive.")):
        gtrs(
            default_values.a_i,
            default_values.n,
            default_values.k,
            sigma,
            default_values.destinations,
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
            0,
            -1,
        )
    # Case n_iter < 0
    with pytest.raises(ValueError, match=re.escape("Number of Bisection iterations must be positive.")):
        gtrs(
            default_values.a_i,
            default_values.n,
            default_values.k,
            sigma,
            default_values.destinations,
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
            0,
            0.001,
            -1,
        )
    # Case max_lim < 0
    with pytest.raises(
        ValueError, match=re.escape("The maximum value for the interval in the bisection function must be positive.")
    ):
        gtrs(
            default_values.a_i,
            default_values.n,
            default_values.k,
            sigma,
            default_values.destinations,
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
            0,
            0.001,
            30,
            -1,
        )


def test_gtrs_optional_parameters(default_values, expected_trajectories_gtrs_sigma_0, seeds):
    """Tests if the algorithm correctly accepts optional parameters."""
    # Values used in test
    sigma = 1  # Noise STD in meters
    tol = 0.0015
    n_iter = 35
    max_lim = 1000005.0
    trajectories = gtrs(
        default_values.a_i,
        default_values.n,
        default_values.k,
        sigma,
        default_values.destinations,
        default_values.initial_uav_position,
        default_values.v_max,
        default_values.tau,
        default_values.gamma,
        seeds,
        tol,
        n_iter,
        max_lim,
    )
    gtrs_estimated_trajectory = trajectories[0]
    gtrs_true_trajectory = trajectories[1]
    # Check if the algorithm reach the final position with a tolerance 1 (m) due to noise
    assert_allclose(expected_trajectories_gtrs_sigma_0[0][-1], gtrs_estimated_trajectory[-1], rtol=1)
    assert_allclose(expected_trajectories_gtrs_sigma_0[1][-1], gtrs_true_trajectory[-1], rtol=1)
    # Check to see if the algorithm solved the problem in the expected number of steps
    # Tolerance of ten steps due to noise
    expected_steps_necessary_estimated_trajectory = len(expected_trajectories_gtrs_sigma_0[0])
    expected_steps_necessary_true_trajectory = len(expected_trajectories_gtrs_sigma_0[1])
    steps_necessary_estimated_trajectory = len(trajectories[0])
    steps_necessary_true_trajectory = len(trajectories[1])
    assert expected_steps_necessary_estimated_trajectory <= steps_necessary_estimated_trajectory + 10
    assert expected_steps_necessary_true_trajectory <= steps_necessary_true_trajectory + 10


def test_calc_eigen_incorrect_parameters():
    """Tests the _calc_eigen function with incorrect parameters."""
    eigen = _calc_eigen(array([]), array([]))
    assert_array_equal([0], eigen)
