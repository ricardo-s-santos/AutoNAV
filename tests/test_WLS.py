"""This file contains the WLS tests."""
import os

from autonav.file_handlers import _readpathfile
from autonav.plots import plot_trajectories
from autonav.WLS import wls
from numpy import array, insert

ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
filename = ROOT_DIR + "/Path.txt"
destinations = _readpathfile(filename)
N = 8  # Number of anchors
M = 1  # Number of target
K = 10  # Number of measurement samples
Mc = 1  # Number of Monte Carlo runs
sigma = 0  # Noise STD in meters
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
estimated_trajectory = wls(a_i, N, K, sigma, destinations, initial_uav_position)
destinations = insert(destinations, 0, initial_uav_position, axis=0)
plot_trajectories(destinations, estimated_trajectory)
