"""This file contains the plot.py tests."""

import re

import pytest
from numpy import ndarray

from autonav.plots import plot_rmse, plot_trajectories


def test_plot_trajectories(
    default_values, expected_trajectories_gtrs_sigma_0, expected_trajectories_wls_sigma_0, ideal_trajectory
):
    """Tests if the plot_trajectories function returns the correct type."""
    ax = plot_trajectories(
        ideal_trajectory,
        [expected_trajectories_gtrs_sigma_0[0], expected_trajectories_wls_sigma_0[0]],
        default_values[0],
    )
    assert type(ax) is list


def test_plot_rmse(expected_trajectories_gtrs_sigma_0, expected_trajectories_wls_sigma_0):
    """Tests if the plot_rmse function returns the correct type."""
    ax = plot_rmse(
        [expected_trajectories_gtrs_sigma_0[0], expected_trajectories_gtrs_sigma_0[1]],
        [expected_trajectories_wls_sigma_0[0], expected_trajectories_wls_sigma_0[1]],
    )
    assert type(ax) is ndarray


def test_plot_trajectories_exceptions(
    default_values, expected_trajectories_gtrs_sigma_0, expected_trajectories_wls_sigma_0, ideal_trajectory
):
    """Tests if the expected exceptions are raised."""
    with pytest.raises(
        ValueError,
        match=re.escape(
            "The number of algorithms must be the same in estimated_trajectories and names_of_the_algorithms."
        ),
    ):
        plot_trajectories(
            ideal_trajectory,
            [expected_trajectories_gtrs_sigma_0[0], expected_trajectories_wls_sigma_0[0]],
            default_values[0],
            ["GTRS"],
        )


def test_plot_rmse_exceptions(
    expected_trajectories_gtrs_sigma_0, expected_trajectories_wls_sigma_0, metrics_trajectories
):
    """Tests if the expected exceptions are raised."""
    with pytest.raises(
        ValueError,
        match=re.escape("The number of algorithms must be the same in estimated_trajectories and true_trajectories."),
    ):
        plot_rmse(
            [expected_trajectories_gtrs_sigma_0[1], expected_trajectories_wls_sigma_0[0]],
            [expected_trajectories_gtrs_sigma_0[1]],
        )
