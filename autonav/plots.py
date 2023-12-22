"""This module contains the plotting functions."""

import matplotlib.pyplot as plt
from numpy.typing import NDArray


def plot_trajectories(ideal_trajectory: NDArray, estimated_trajectory: NDArray, a_i: NDArray):
    """This function plots the ideal and estimated trajectory.

    Args:
        ideal_trajectory: The ideal trajectory that the UAV is supposed to follow.
        estimated_trajectory: The estimated trajectory that the UAV followed.
        a_i: The position of the anchors.

    Returns:
        Nothing.
    """
    ax = plt.axes(projection="3d")
    a_i_label = "a_i"
    for i in range(0, a_i.shape[1]):
        ax.plot3D(
            a_i[0][i],
            a_i[1][i],
            a_i[2][i],
            marker="s",
            markersize=10,
            markeredgecolor="black",
            markerfacecolor="black",
            label=a_i_label,
        )
        a_i_label = "_nolegend_"  # Legend only in the first iteration
    ax.plot3D(
        ideal_trajectory[:, 0],
        ideal_trajectory[:, 1],
        ideal_trajectory[:, 2],
        color="green",
        label="Ideal Trajectory",
        linewidth=3.0,
        alpha=0.7,
    )
    ax.plot3D(
        estimated_trajectory[:, 0],
        estimated_trajectory[:, 1],
        estimated_trajectory[:, 2],
        color="red",
        label="Estimated Trajectory",
        alpha=1.0,
    )
    plt.xlabel("Width")
    plt.ylabel("Length")
    plt.legend()
    plt.show()
