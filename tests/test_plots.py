"""This file contains the plot.py tests."""

from typing import List

from autonav.plots import plot_rmse, plot_trajectories
from numpy import ndarray


def test_plot_trajectories(default_values, expected_trajectories_gtrs, expected_trajectories_wls, ideal_trajectory):
    """This tests if the function returns the correct type."""
    ax = plot_trajectories(
        ideal_trajectory, [expected_trajectories_gtrs[0], expected_trajectories_wls[0]], default_values[0]
    )
    assert type(ax) is List


def test_plot_rmse(expected_trajectories_gtrs, expected_trajectories_wls):
    """This tests if the function returns the correct type."""
    ax = plot_rmse(
        [expected_trajectories_gtrs[1], expected_trajectories_gtrs[0]],
        [expected_trajectories_wls[1], expected_trajectories_wls[0]],
    )
    assert type(ax) is ndarray
