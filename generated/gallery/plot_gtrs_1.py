"""# GTRS example in 3D

"""

from autonav import gtrs
from autonav.file_handlers import readpathfile
from autonav.plots import plot_trajectories
from numpy import array

b = 200
n = 8
a_i = array(
    [
    [0, 0, 0],
    [0, b, 0],
    [b / 2, 0, 0],
    [b / 2, b, 0],
    [0, 0, b / 8],
    [0, b, b / 8],
    [b / 2, 0, b / 8],
    [b / 2, b, b / 8],]
    ).T
k = 50
sigma = 1
initial_uav_position = [10, 10, 5]
destinations = readpathfile("Path.csv")

[estimated_trajectory, true_trajectory] = gtrs(a_i, n, k, sigma, destinations, initial_uav_position)
plot_trajectories(destinations, [estimated_trajectory], a_i, ['GTRS'])
