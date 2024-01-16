"""This file contains the GTRS.py tests."""

import re

import pytest
from autonav.GTRS import _calc_eigen, gtrs
from numpy import array
from numpy.testing import assert_allclose, assert_array_equal


@pytest.mark.critical()
def test_gtrs_no_noise(default_values, expected_trajectories_gtrs):
    """This test pretends to see if the algorithm is correctly implemented by setting the noise to zero."""
    # Values used in test
    sigma = 0  # Noise STD in meters
    trajectories = gtrs(
        default_values[0], default_values[1], default_values[2], sigma, default_values[3], default_values[4]
    )
    gtrs_estimated_trajectory = trajectories[0]
    gtrs_true_trajectory = trajectories[1]
    # With sigma zero the trajectories should be the following ones if one performs the math
    assert_allclose(expected_trajectories_gtrs[0], gtrs_estimated_trajectory)
    assert_allclose(expected_trajectories_gtrs[1], gtrs_true_trajectory)


@pytest.mark.critical()
def test_gtrs_reproducibility(default_values):
    """This test pretends to see if the algorithm is reproducible."""
    # Values used in test
    sigma = 0  # Noise STD in meters
    trajectories = gtrs(
        default_values[0], default_values[1], default_values[2], sigma, default_values[3], default_values[4]
    )
    gtrs_estimated_trajectory_1 = trajectories[0]
    gtrs_true_trajectory_1 = trajectories[1]
    # Call GTRS again
    trajectories = gtrs(
        default_values[0], default_values[1], default_values[2], sigma, default_values[3], default_values[4]
    )
    gtrs_estimated_trajectory_2 = trajectories[0]
    gtrs_true_trajectory_2 = trajectories[1]
    # Check to see if both results are equal
    assert_allclose(gtrs_estimated_trajectory_1, gtrs_estimated_trajectory_2)
    assert_allclose(gtrs_true_trajectory_1, gtrs_true_trajectory_2)


@pytest.mark.critical()
def test_GTRS_exceptions(default_values):
    """This test tests the expected exceptions with wrong args."""
    sigma = 0
    # Case n != size(a_i, axis=1)
    with pytest.raises(ValueError, match=re.escape("The length of a_i must be equal to N.")):
        gtrs(default_values[0], 10, default_values[2], sigma, default_values[3], default_values[4])
    # Case k < 0
    with pytest.raises(ValueError, match=re.escape("K must be positive.")):
        gtrs(default_values[0], default_values[1], -1, sigma, default_values[3], default_values[4])
    # Case sigma < 0
    with pytest.raises(ValueError, match=re.escape("Sigma must be positive.")):
        gtrs(default_values[0], default_values[1], default_values[2], -1, default_values[3], default_values[4])
    # Case destinations with wrong coordinates
    with pytest.raises(ValueError, match=re.escape("Waypoints must contain the 3 coordinates (x, y, z).")):
        gtrs(default_values[0], default_values[1], default_values[2], sigma, array([[1, 2]]), default_values[4])
    # Case empty destinations
    with pytest.raises(ValueError, match=re.escape("Waypoints cannot be empty.")):
        gtrs(default_values[0], default_values[1], default_values[2], sigma, array([]), default_values[4])
    # Case initial_uav_position with wrong coordinates
    with pytest.raises(ValueError, match=re.escape("Initial UAV position must contain the 3 coordinates (x, y, z).")):
        gtrs(default_values[0], default_values[1], default_values[2], sigma, default_values[3], [1, 2])
    # Validate optional parameters
    # Case tol < 0
    with pytest.raises(ValueError, match=re.escape("Tolerance must be positive.")):
        gtrs(
            default_values[0], default_values[1], default_values[2], sigma, default_values[3], default_values[4], 0, -1
        )
    # Case n_iter < 0
    with pytest.raises(ValueError, match=re.escape("Number of Bisection iterations must be positive.")):
        gtrs(
            default_values[0],
            default_values[1],
            default_values[2],
            sigma,
            default_values[3],
            default_values[4],
            0,
            0.001,
            -1,
        )
    # Case max_lim < 0
    with pytest.raises(
        ValueError, match=re.escape("The maximum value for the interval in the bisection function must be positive.")
    ):
        gtrs(
            default_values[0],
            default_values[1],
            default_values[2],
            sigma,
            default_values[3],
            default_values[4],
            0,
            0.001,
            30,
            -1,
        )


def test_gtrs_optional_parameters(default_values, expected_trajectories_gtrs):
    """This test pretends to see if the algorithm correctly accepts optional parameters."""
    # Values used in test
    sigma = 0  # Noise STD in meters
    noise_seed = 1
    tol = 0.0015
    n_iter = 35
    max_lim = 1000005.0
    trajectories = gtrs(
        default_values[0],
        default_values[1],
        default_values[2],
        sigma,
        default_values[3],
        default_values[4],
        noise_seed,
        tol,
        n_iter,
        max_lim,
    )
    gtrs_estimated_trajectory = trajectories[0]
    gtrs_true_trajectory = trajectories[1]
    # With sigma zero the trajectories should be the following ones if one performs the math
    assert_allclose(expected_trajectories_gtrs[2], gtrs_estimated_trajectory)
    assert_allclose(expected_trajectories_gtrs[3], gtrs_true_trajectory)


def test_calc_eigen_incorrect_parameters():
    """This test tests the _calc_eigen function with incorrect parameters."""
    eigen = _calc_eigen([], [])
    assert_array_equal([0], eigen)
