"""This module contains the plotting functions."""

import matplotlib.pyplot as plt
from numpy.typing import NDArray


def plot_trajectories(ideal_trajectory: NDArray, estimated_trajectory: NDArray):
    """This function plots the ideal and estimated trajectory.

    Args:
        ideal_trajectory: The ideal trajectory that the UAV is supposed to follow.
        estimated_trajectory: The estimated trajectory that the UAV followed.

    Returns:
        Nothing.
    """
    plt.plot(
        ideal_trajectory[:, 0],
        ideal_trajectory[:, 1],
        color="green",
        label="Ideal Trajectory",
        linewidth=3.0,
        alpha=0.7,
    )
    plt.plot(
        estimated_trajectory[:, 0],
        estimated_trajectory[:, 1],
        color="red",
        label="Estimated Trajectory",
        alpha=1.0,
    )
    plt.xlabel("Width")
    plt.ylabel("Length")
    plt.legend()
    plt.show()
