"""This file contains the WLS.py tests."""

import re

import pytest
from numpy import array
from numpy.testing import assert_allclose

from autonav.WLS import wls


@pytest.mark.critical
def test_wls_no_noise(default_values, expected_trajectories_wls_standard_normal):
    """Tests if the algorithm is correctly implemented by setting the noise to zero."""
    # This test does not use seeds because the noise is set to zero.
    # Values used in test
    trajectories = wls(
        default_values.a_i,
        default_values.n,
        default_values.k,
        default_values.destinations,
        default_values.initial_uav_position,
        default_values.v_max,
        default_values.tau,
        default_values.gamma,
    )
    wls_estimated_trajectory = trajectories[0]
    wls_true_trajectory = trajectories[1]
    # With sigma zero the trajectories should be the following ones if one performs the math
    assert_allclose(expected_trajectories_wls_standard_normal[0], wls_estimated_trajectory, rtol=1e-03)
    assert_allclose(expected_trajectories_wls_standard_normal[1], wls_true_trajectory, rtol=1e-03)


@pytest.mark.critical
def test_wls_reproducibility(default_values, seeds):
    """Tests if the algorithm is reproducible."""
    # Values used in test
    trajectories = wls(
        default_values.a_i,
        default_values.n,
        default_values.k,
        default_values.destinations,
        default_values.initial_uav_position,
        default_values.v_max,
        default_values.tau,
        default_values.gamma,
        seeds,
    )
    wls_estimated_trajectory_1 = trajectories[0]
    wls_true_trajectory_1 = trajectories[1]
    # Call WLS again
    trajectories = wls(
        default_values.a_i,
        default_values.n,
        default_values.k,
        default_values.destinations,
        default_values.initial_uav_position,
        default_values.v_max,
        default_values.tau,
        default_values.gamma,
        seeds,
    )
    wls_estimated_trajectory_2 = trajectories[0]
    wls_true_trajectory_2 = trajectories[1]
    # Check to see if both results are equal
    assert_allclose(wls_estimated_trajectory_1, wls_estimated_trajectory_2)
    assert_allclose(wls_true_trajectory_1, wls_true_trajectory_2)


@pytest.mark.critical
def test_wls_exceptions(default_values):
    """Tests if the expected exceptions are raised with wrong arguments."""
    # Case n != size(a_i, axis=1)
    with pytest.raises(ValueError, match=re.escape("The length of a_i must be equal to N.")):
        wls(
            default_values.a_i,
            10,
            default_values.k,
            default_values.destinations,
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
        )
    # Case k < 0
    with pytest.raises(ValueError, match=re.escape("K must be positive.")):
        wls(
            default_values.a_i,
            default_values.n,
            -1,
            default_values.destinations,
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
        )
    # Case destinations with wrong coordinates
    with pytest.raises(ValueError, match=re.escape("Waypoints must contain the 3 coordinates (x, y, z).")):
        wls(
            default_values.a_i,
            default_values.n,
            default_values.k,
            array([[1, 2]]),
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
        )
    # Case empty destinations
    with pytest.raises(ValueError, match=re.escape("Waypoints cannot be empty.")):
        wls(
            default_values.a_i,
            default_values.n,
            default_values.k,
            array([]),
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
        )
    # Case initial_uav_position with wrong coordinates
    with pytest.raises(ValueError, match=re.escape("Initial UAV position must contain the 3 coordinates (x, y, z).")):
        wls(
            default_values.a_i,
            default_values.n,
            default_values.k,
            default_values.destinations,
            [1, 2],
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
        )
    # Case wrong distribution
    with pytest.raises(ValueError, match=re.escape("Noise distribution must be normal, standard_normal, exponential.")):
        wls(
            default_values.a_i,
            default_values.n,
            default_values.k,
            default_values.destinations,
            default_values.initial_uav_position,
            default_values.v_max,
            default_values.tau,
            default_values.gamma,
            0,
            "exp",
        )


def test_wls_optional_parameters(
    default_values, expected_trajectories_wls_standard_normal, seeds, noise_distributions, distribution_parameters
):
    """Tests if the algorithm correctly accepts optional parameters."""
    # Values used in test
    if noise_distributions == "normal" and distribution_parameters == [10**-3]:
        distribution_parameters = [0, 1]
    trajectories = wls(
        default_values.a_i,
        default_values.n,
        default_values.k,
        default_values.destinations,
        default_values.initial_uav_position,
        default_values.v_max,
        default_values.tau,
        default_values.gamma,
        seeds,
        noise_distributions,
        distribution_parameters,
    )
    wls_estimated_trajectory = trajectories[0]
    wls_true_trajectory = trajectories[1]
    # Check if the algorithm reach the final position with a tolerance 1 (m) due to noise
    assert_allclose(expected_trajectories_wls_standard_normal[0][-1], wls_estimated_trajectory[-1], rtol=1)
    assert_allclose(expected_trajectories_wls_standard_normal[1][-1], wls_true_trajectory[-1], rtol=1)
    # Check to see if the algorithm solved the problem in the expected number of steps
    # Tolerance of ten steps due to noise
    expected_steps_necessary_estimated_trajectory = len(expected_trajectories_wls_standard_normal[0])
    expected_steps_necessary_true_trajectory = len(expected_trajectories_wls_standard_normal[1])
    steps_necessary_estimated_trajectory = len(trajectories[0])
    steps_necessary_true_trajectory = len(trajectories[1])
    assert expected_steps_necessary_estimated_trajectory <= steps_necessary_estimated_trajectory + 10
    assert expected_steps_necessary_true_trajectory <= steps_necessary_true_trajectory + 10
