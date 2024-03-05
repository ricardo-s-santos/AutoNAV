"""This module contains the velocity functions."""

from math import sqrt

import numpy
from numpy import dot, zeros
from numpy.typing import NDArray


def _velocity(
    current_position: NDArray,
    destination_position: NDArray,
    v_max: float,
    tau: float,
    gamma: float,
) -> NDArray:
    """Computes the max speed allowed to the UAV according to the distance to the destination.

    Args:
        current_position: The current position of the UAV.
        destination_position: The destination the UAV desires to reach.
        v_max: The maximum velocity that the UAV can fly.
        tau: The threshold to reach the destination.
        gamma: The smoothing factor.

    Returns:
        The maximum velocity allowed to the UAV in the three axis (x,y,z).
    """
    velocity_allowed = zeros(3)
    """Check if parameters contain only numbers."""
    if destination_position.dtype == float or destination_position.dtype == int:
        if current_position.dtype == float or current_position.dtype == int:
            if len(destination_position) == 3 and len(current_position) == 3:
                error_position = numpy.subtract(destination_position, current_position)
                error_norm = sqrt(numpy.sum(numpy.square(error_position)))
                if error_norm > 1.0:  # Longer Distance = More Speed
                    scale = v_max / error_norm
                    velocity_allowed = dot(error_position, scale)
                    if error_norm < tau:  # Lower Distance = Less Speed
                        velocity_allowed = velocity_allowed * ((error_norm / tau) ** gamma)
        else:
            raise TypeError("'current_position' must contain only numeric values")
    else:
        raise TypeError("'destination_position' must contain only numeric values")
    return velocity_allowed
