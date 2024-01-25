"""This file contains the plot.py tests."""

import re

import pytest
from autonav.plots import plot_rmse, plot_trajectories
from numpy import ndarray


def test_plot_trajectories(default_values, expected_trajectories_gtrs, expected_trajectories_wls, ideal_trajectory):
    """This tests if the function returns the correct type."""
    ax = plot_trajectories(
        ideal_trajectory, [expected_trajectories_gtrs[0], expected_trajectories_wls[0]], default_values[0]
    )
    assert type(ax) is list


def test_plot_rmse(expected_trajectories_gtrs, expected_trajectories_wls):
    """This tests if the function returns the correct type."""
    ax = plot_rmse(
        [expected_trajectories_gtrs[1], expected_trajectories_gtrs[0]],
        [expected_trajectories_wls[1], expected_trajectories_wls[0]],
    )
    assert type(ax) is ndarray


def test_plot_trajectories_exceptions(
    default_values, expected_trajectories_gtrs, expected_trajectories_wls, ideal_trajectory
):
    """This tests if the function returns the correct type."""
    with pytest.raises(
        ValueError,
        match=re.escape(
            "The number of algorithms must be the same in estimated_trajectories and names_of_the_algorithms."
        ),
    ):
        plot_trajectories(
            ideal_trajectory, [expected_trajectories_gtrs[0], expected_trajectories_wls[0]], default_values[0], ["GTRS"]
        )


def test_plot_rmse_exceptions(expected_trajectories_gtrs, metrics_trajectories):
    """This tests if the function catches the exceptions."""
    with pytest.raises(
        ValueError,
        match=re.escape("The number of algorithms must be the same in estimated_trajectories and true_trajectories."),
    ):
        plot_rmse([expected_trajectories_gtrs[1], metrics_trajectories[0]], [expected_trajectories_gtrs[1]])
