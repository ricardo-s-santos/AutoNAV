"""This module contains the metrics functions."""

from math import sqrt

from numpy import asarray, size
from numpy.typing import ArrayLike, NDArray


def compute_rmse(estimated_trajectory: ArrayLike, true_trajectory: ArrayLike) -> list:
    """Computes the root mean squared error between the true and estimated trajectory of the UAV.

    Args:
        estimated_trajectory: The estimated trajectory by the algorithm.
        true_trajectory: The true trajectory that the UAV followed.

    Returns:
        The average root mean squared error between the true and the estimated trajectories.
    """
    # Transform inputs in NDArray
    arr_estimated_trajectory: NDArray = asarray(estimated_trajectory, dtype=float)
    arr_true_trajectory: NDArray = asarray(true_trajectory, dtype=float)
    rmse = []
    # Trajectories must have the same length for comparison
    if size(arr_estimated_trajectory) == size(arr_true_trajectory):
        for i in range(len(arr_estimated_trajectory)):
            norm = sqrt(
                (arr_true_trajectory[i][0] - arr_estimated_trajectory[i][0]) ** 2
                + (arr_true_trajectory[i][1] - arr_estimated_trajectory[i][1]) ** 2
                + (arr_true_trajectory[i][2] - arr_estimated_trajectory[i][2]) ** 2
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
    arr_estimated_trajectory: NDArray = asarray(estimated_trajectory, dtype=float)
    arr_true_trajectory: NDArray = asarray(true_trajectory, dtype=float)
    rmse = compute_rmse(arr_estimated_trajectory, arr_true_trajectory)
    if len(rmse) != 0:
        armse = sum(rmse) / len(rmse)
    else:
        raise ZeroDivisionError("RMSE is empty!")
    return armse
