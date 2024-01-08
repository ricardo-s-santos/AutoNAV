"""This file contains the WLS.py tests."""

import pytest
from autonav.WLS import wls
from numpy.testing import assert_array_almost_equal


@pytest.mark.critical()
def test_wls_no_noise(default_values, expected_trajectories_wls):
    """This test pretends to see if the algorithm is correctly implemented by setting the noise to zero."""
    # Values used in test
    sigma = 0  # Noise STD in meters
    trajectories = wls(
        default_values[0], default_values[1], default_values[2], sigma, default_values[3], default_values[4]
    )
    wls_estimated_trajectory = trajectories[0]
    wls_true_trajectory = trajectories[1]
    # With sigma zero the trajectories should be the following ones if one performs the math
    assert_array_almost_equal(
        expected_trajectories_wls[0].round(decimals=3), wls_estimated_trajectory.round(decimals=3)
    )
    assert_array_almost_equal(expected_trajectories_wls[1].round(decimals=3), wls_true_trajectory.round(decimals=3))
