from autonav.GTRS import gtrs
from autonav.fileHandlers import _readpathfile
from autonav.plots import plot_trajectories
from numpy import array
import os

ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
filename = ROOT_DIR + "/Path.txt"
destinations = _readpathfile(filename)
N = 8  # Number of anchors
K = 10  # Number of measurement samples
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
    ]).T

estimated_trajectory = gtrs(a_i, N, K, sigma, destinations, [10, 10, 5])
plot_trajectories(destinations, estimated_trajectory)
