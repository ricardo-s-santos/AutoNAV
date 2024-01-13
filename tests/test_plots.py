"""This file contains the plot.py tests."""

import matplotlib.pyplot as plt
from autonav.plots import (
    plot_comparison_between_algorithms,
    plot_rmse,
    plot_rmse_comparison_between_algorithms,
    plot_trajectories,
)


def test_plot_rmse(monkeypatch, metrics_trajectories):
    """This tests if the function shows a plot to the user."""
    monkeypatch.setattr(plt, "show", lambda: None)
    plot_rmse(metrics_trajectories[1], metrics_trajectories[0])
    assert True


def test_plot_rmse_comparison_between_algorithms(monkeypatch, expected_trajectories_gtrs, expected_trajectories_wls):
    """This tests if the function shows a plot to the user."""
    monkeypatch.setattr(plt, "show", lambda: None)
    plot_rmse_comparison_between_algorithms(
        expected_trajectories_gtrs[1],
        expected_trajectories_gtrs[0],
        expected_trajectories_wls[1],
        expected_trajectories_wls[0],
    )
    assert True


def test_plot_trajectories(monkeypatch, default_values, metrics_trajectories, ideal_trajectory):
    """This tests if the function shows a plot to the user."""
    monkeypatch.setattr(plt, "show", lambda: None)
    plot_trajectories(ideal_trajectory, metrics_trajectories[0], default_values[0])
    assert True


def test_plot_comparison_between_algorithms(monkeypatch, default_values, metrics_trajectories, ideal_trajectory):
    """This tests if the function shows a plot to the user."""
    monkeypatch.setattr(plt, "show", lambda: None)
    plot_comparison_between_algorithms(
        ideal_trajectory, metrics_trajectories[0], metrics_trajectories[0], default_values[0]
    )
    assert True
