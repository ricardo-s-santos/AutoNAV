"""This module contains the plotting functions."""

from typing import List, Optional

import matplotlib.pyplot as plt
from numpy import asarray
from numpy.typing import ArrayLike, NDArray

from .metrics import compute_rmse


def plot_trajectories(
    ideal_trajectory: ArrayLike,
    estimated_trajectories: List[ArrayLike],
    a_i: ArrayLike,
    names_of_the_algorithms: Optional[List[str]] = None,
) -> list:
    """Plots the ideal and estimated trajectory for one or more algorithms.

    Args:
        ideal_trajectory: The ideal trajectory that the UAV is supposed to follow.
        estimated_trajectories: The estimated trajectory that the UAV followed using the GTRS algorithm.
        names_of_the_algorithms: The names of the algorithms in the same order as in estimated_trajectories.
        a_i: The position of the anchors.

    Returns:
        A list of Matplotlib Axes object containing the ideal and estimated trajectory comparison.
    """
    # Transform inputs in NDArray
    arr_ideal_trajectory: NDArray = asarray(ideal_trajectory, dtype=float)
    arr_a_i: NDArray = asarray(a_i, dtype=float)
    # User didn't input names_of_the_algorithms
    if names_of_the_algorithms is None:
        names_of_the_algorithms = ["GTRS", "WLS"]
    if len(estimated_trajectories) == len(names_of_the_algorithms):
        axes = []
        for j in range(len(estimated_trajectories)):
            # Transform inputs in NDArray
            arr_estimated_trajectories: NDArray = asarray(estimated_trajectories[j], dtype=float)
            plt.figure(j)  # New figure foreach algorithm
            ax = plt.axes(projection="3d")
            a_i_label = "a_i"
            for i in range(0, arr_a_i.shape[1]):
                ax.plot(
                    arr_a_i[0][i],
                    arr_a_i[1][i],
                    arr_a_i[2][i],
                    marker="s",
                    markersize=10,
                    markeredgecolor="black",
                    markerfacecolor="black",
                    label=a_i_label,
                )
                a_i_label = "_nolegend_"  # Legend only in the first iteration
            ax.plot(
                arr_ideal_trajectory[:, 0],
                arr_ideal_trajectory[:, 1],
                arr_ideal_trajectory[:, 2],
                color="green",
                label="Ideal Trajectory",
                linewidth=3.0,
                alpha=0.7,
            )
            ax.plot(
                arr_estimated_trajectories[:, 0],
                arr_estimated_trajectories[:, 1],
                arr_estimated_trajectories[:, 2],
                label="Estimated Trajectory " + names_of_the_algorithms[j],
                color="red",
                alpha=1.0,
            )
            ax.set_title("Estimated Trajectory " + names_of_the_algorithms[j])
            ax.set_xlabel("Width (m)")
            ax.set_ylabel("Length (m)")
            ax.set(zlabel="Heigth (m)")
            ax.legend()
            axes.append(ax)
        return axes
    else:
        raise ValueError(
            "The number of algorithms must be the same in estimated_trajectories and names_of_the_algorithms."
        )


def plot_rmse(
    estimated_trajectories: List[ArrayLike],
    true_trajectories: List[ArrayLike],
    names_of_the_algorithms: Optional[List[str]] = None,
) -> NDArray:
    """Plots the root mean squared error along the trajectory for one or more algorithms.

    Args:
       estimated_trajectories: The estimated trajectory that the UAV followed.
       true_trajectories: The true trajectory that the UAV followed.
       names_of_the_algorithms: The names of the algorithms in the same order as in estimated_trajectories.

    Returns:
        An NDArray object containing the RMSE comparison.
    """
    # User didn't input names_of_the_algorithms
    if names_of_the_algorithms is None:
        names_of_the_algorithms = ["GTRS", "WLS"]
    if len(estimated_trajectories) == len(true_trajectories):
        fig, axs = plt.subplots(len(estimated_trajectories), sharey=True)
        # Space between subplots
        fig.tight_layout(pad=5.0)
        for i in range(len(estimated_trajectories)):
            # Transform inputs in NDArray
            arr_estimated_trajectories: NDArray = asarray(estimated_trajectories[i], dtype=float)
            arr_true_trajectories: NDArray = asarray(true_trajectories[i], dtype=float)
            rmse = compute_rmse(arr_estimated_trajectories[:, :], arr_true_trajectories[:, :])
            axs[i].plot(rmse)
            axs[i].set_title("RMSE " + names_of_the_algorithms[i])
        for ax in axs.flat:
            ax.set(xlabel="Iteration", ylabel="RMSE")
        return axs
    else:
        raise ValueError("The number of algorithms must be the same in estimated_trajectories and true_trajectories.")
