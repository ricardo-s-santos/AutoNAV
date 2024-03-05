"""This module contains the metrics functions."""

from math import sqrt

from numpy import asarray
from numpy.typing import ArrayLike


def compute_rmse(estimated_trajectory: ArrayLike, true_trajectory: ArrayLike) -> list:
    """Computes the root mean squared error between the true and estimated trajectory of the UAV.

    Args:
        estimated_trajectory: The estimated trajectory by the algorithm.
        true_trajectory: The true trajectory that the UAV followed.

    Returns:
        The average root mean squared error between the true and the estimated trajectories.
    """
    # Transform inputs in NDArray
    estimated_trajectory = asarray(estimated_trajectory, dtype=float)
    true_trajectory = asarray(true_trajectory, dtype=float)
    rmse = []
    # Trajectories must have the same length for comparison
    if len(estimated_trajectory) == len(true_trajectory):
        for i in range(len(estimated_trajectory)):
            norm = sqrt(
                (true_trajectory[i][0] - estimated_trajectory[i][0]) ** 2
                + (true_trajectory[i][1] - estimated_trajectory[i][1]) ** 2
                + (true_trajectory[i][2] - estimated_trajectory[i][2]) ** 2
            )
            rmse.append(sqrt(norm))
    return rmse


def compute_armse(estimated_trajectory: ArrayLike, true_trajectory: ArrayLike) -> float:
    """This function computes the average root mean squared error between the true and estimated trajectory of the UAV.

    Args:
        estimated_trajectory: The estimated trajectory by the algorithm.
        true_trajectory: The true trajectory that the UAV followed.

    Returns:
        The average root mean squared error between the true and the estimated trajectories.
    """
    # Transform inputs in NDArray
    estimated_trajectory = asarray(estimated_trajectory, dtype=float)
    true_trajectory = asarray(true_trajectory, dtype=float)
    rmse = compute_rmse(estimated_trajectory, true_trajectory)
    if len(rmse) != 0:
        armse = sum(rmse) / len(rmse)
    else:
        raise ZeroDivisionError("RMSE is empty!")
    return armse
