"""This file contains the WLS.py tests."""
import os

from autonav.file_handlers import _readpathfile
from autonav.metrics import compute_armse
from autonav.plots import plot_rmse, plot_trajectories
from autonav.WLS import wls
from numpy import array, insert

ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
filename = ROOT_DIR + "/Path.txt"
destinations = _readpathfile(filename)
N = 8  # Number of anchors
M = 1  # Number of target
K = 10  # Number of measurement samples
Mc = 1  # Number of Monte Carlo runs
sigma = 1  # Noise STD in meters
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
trajectories = wls(a_i, N, K, sigma, destinations, initial_uav_position)
estimated_trajectory = trajectories[0]
true_trajectory = trajectories[1]
# Compute metrics
print(f"Average RMSE: {compute_armse(estimated_trajectory, true_trajectory):0,.2f} (m)")
# Add initial position of the UAV to the plot
destinations = insert(destinations, 0, initial_uav_position, axis=0)
# Plot trajectories
plot_trajectories(destinations, estimated_trajectory, a_i)
# Plot metrics
plot_rmse(true_trajectory, estimated_trajectory)
