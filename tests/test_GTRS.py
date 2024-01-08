"""This file contains the GTRS.py tests."""

import pytest
from autonav.GTRS import _calc_eigen, gtrs
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


def test_calc_eigen_incorrect_parameters():
    """This test tests the _calc_eigen function with incorrect parameters."""
    eigen = _calc_eigen([], [])
    assert_array_equal([0], eigen)
