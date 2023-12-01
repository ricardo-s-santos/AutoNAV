import numpy
from math import sqrt
from numpy import dot
from numpy.typing import NDArray


def _velocity(current_position: NDArray, destination_position: NDArray):
    """
    This function computes the max speed allowed to the UAV according to the distance to the destination.

    """
    param_max_velocity = 2
    param_reach_distance = 4
    param_smooth_factor = 2
    velocity_allowed = [0, 0, 0]
    error_position = numpy.subtract(destination_position, current_position)
    error_norm = sqrt(numpy.sum(numpy.square(error_position)))
    if error_norm > 1.0:  # Longer Distance = More Speed
        scale = param_max_velocity / error_norm
        velocity_allowed = dot(error_position, scale)
        if error_norm < param_reach_distance:  # Lower Distance = Less Speed
            velocity_allowed = velocity_allowed * ((error_norm / param_reach_distance) ** param_smooth_factor)
    return velocity_allowed
