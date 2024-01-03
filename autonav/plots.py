"""This module contains the plotting functions."""

import matplotlib.pyplot as plt
from numpy.typing import NDArray

from .metrics import compute_rmse


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


def plot_comparison_between_algorithms(
    ideal_trajectory: NDArray, estimated_trajectory_GTRS: NDArray, estimated_trajectory_WLS: NDArray, a_i: NDArray
):
    """This function plots the ideal and estimated trajectory for both algorithms.

        Args:
        ideal_trajectory: The ideal trajectory that the UAV is supposed to follow.
        estimated_trajectory_GTRS: The estimated trajectory GTRS that the UAV followed.
        estimated_trajectory_WLS: The estimated trajectory WLS that the UAV followed.
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
        estimated_trajectory_GTRS[:, 0],
        estimated_trajectory_GTRS[:, 1],
        estimated_trajectory_GTRS[:, 2],
        color="red",
        label="Estimated Trajectory GTRS",
        alpha=1.0,
    )
    ax.plot3D(
        estimated_trajectory_WLS[:, 0],
        estimated_trajectory_WLS[:, 1],
        estimated_trajectory_WLS[:, 2],
        color="blue",
        label="Estimated Trajectory WLS",
        alpha=1.0,
    )
    plt.xlabel("Width")
    plt.ylabel("Length")
    plt.legend()
    plt.show()


def plot_rmse_comparison_between_algorithms(
    true_trajectory_GTRS: NDArray,
    estimated_trajectory_GTRS: NDArray,
    true_trajectory_WLS: NDArray,
    estimated_trajectory_WLS: NDArray,
):
    """This function plots the root mean squared error along the trajectory.

    Args:
        ideal_trajectory: The ideal trajectory that the UAV is supposed to follow.
        estimated_trajectory: The estimated trajectory that the UAV followed.

    Returns:
        Nothing.
    """
    rmse_GTRS = compute_rmse(estimated_trajectory_GTRS, true_trajectory_GTRS)
    rmse_WLS = compute_rmse(estimated_trajectory_WLS, true_trajectory_WLS)
    plt.plot(rmse_GTRS, label="RMSE GTRS")
    plt.plot(rmse_WLS, label="RMSE WLS")
    plt.xlabel("Iteration")
    plt.ylabel("RMSE (m)")
    plt.legend()
    plt.show()


def plot_rmse(true_trajectory: NDArray, estimated_trajectory: NDArray):
    """This function plots the root mean squared error along the trajectory.

    Args:
        ideal_trajectory: The ideal trajectory that the UAV is supposed to follow.
        estimated_trajectory: The estimated trajectory that the UAV followed.

    Returns:
        Nothing.
    """
    rmse = compute_rmse(estimated_trajectory, true_trajectory)
    plt.plot(rmse, label="RMSE")
    plt.xlabel("Iteration")
    plt.ylabel("RMSE (m)")
    plt.legend()
    plt.show()
