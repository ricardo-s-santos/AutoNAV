"""This file contains the tests for the metrics.py."""

import pytest
from numpy.testing import assert_allclose

from autonav.metrics import compute_armse, compute_rmse


def test_compute_rmse(metrics_trajectories):
    """Tests the compute_rmse function in normal conditions."""
    rmse = compute_rmse(metrics_trajectories[0], metrics_trajectories[1])
    # The RMSE can be computed by hand, such that, with these trajectories one gets:
    expected_rmse = [
        0.0015778142924356634,
        0.002651871961695091,
        0.0010248112356221553,
        0.0024024521133049303,
        0.001353914632877439,
        0.002750205461669971,
        0.0025692484886090596,
        0.0019259680800039801,
        0.002337631694009033,
        0.005003020062700637,
        0.0037045672771045274,
    ]
    assert_allclose(expected_rmse, rmse)


def test_compute_rmse_different_lengths(metrics_trajectories):
    """Tests the compute_rmse function with trajectories with different lengths."""
    rmse = compute_rmse(metrics_trajectories[0], metrics_trajectories[1][-1])
    expected_rmse = []
    assert_allclose(expected_rmse, rmse)


def test_compute_armse(metrics_trajectories):
    """Tests the compute_armse function in normal conditions."""
    armse = compute_armse(metrics_trajectories[0], metrics_trajectories[1])
    armse = float("{:.10f}".format(armse))
    # The ARMSE can be computed by hand, such that, with these trajectories one gets:
    expected_armse = 0.0024819550
    assert expected_armse == armse


def test_compute_armse_different_lengths(metrics_trajectories):
    """Tests the compute_armse function with trajectories with different lengths."""
    with pytest.raises(ZeroDivisionError):
        compute_armse(metrics_trajectories[0], metrics_trajectories[1][-1])
