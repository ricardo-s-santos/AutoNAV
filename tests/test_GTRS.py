"""This file contains the GTRS.py tests."""

import pytest
from autonav.GTRS import gtrs
from numpy.testing import assert_allclose


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
    assert_allclose(expected_trajectories_gtrs[0].round(decimals=5), gtrs_estimated_trajectory.round(decimals=5))
    assert_allclose(expected_trajectories_gtrs[1].round(decimals=5), gtrs_true_trajectory.round(decimals=5))
