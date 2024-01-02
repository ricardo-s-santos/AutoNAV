"""This file contains the GTRS.py tests."""
import os

from autonav.file_handlers import _readpathfile
from autonav.GTRS import gtrs
from autonav.metrics import armse
from autonav.plots import plot_trajectories
from numpy import array, insert

ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
filename = ROOT_DIR + "/Path.txt"
destinations = _readpathfile(filename)
N = 8  # Number of anchors
K = 10  # Number of measurement samples
sigma = 3  # Noise STD in meters
B = 200  # Area border in meters
a_i = array(
    [
        [0, 0, 0],
        [0, B, 0],
        [B / 2, 0, 0],
        [B / 2, B, 0],
        [0, 0, B / 8],
        [0, B, B / 8],
        [B / 2, 0, B / 8],
        [B / 2, B, B / 8],
    ]
).T
initial_uav_position = [10, 10, 5]
trajectories = gtrs(a_i, N, K, sigma, destinations, initial_uav_position)
estimated_trajectory = trajectories[0]
true_trajectory = trajectories[1]
# Add initial position of the UAV to the plot
destinations = insert(destinations, 0, initial_uav_position, axis=0)
# Compute metrics
print(f"Average RMSE: {armse(estimated_trajectory, true_trajectory):0,.2f} (m)")
# Plot trajectories
plot_trajectories(destinations, estimated_trajectory)
