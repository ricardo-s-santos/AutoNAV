"""# WLS example in 3D

"""

# %%
# First, the required dependencies are imported:
from autonav import wls
from autonav.file_handlers import readpathfile
from autonav.plots import plot_trajectories
from numpy import array

# %%
# The next step is to define the simulation parameters:

# Area border
b = 200
# Number of anchors
n = 8
# Position of the anchors
a_i = array(
    [
        [0, 0, 0],
        [0, b, 0],
        [b / 2, 0, 0],
        [b / 2, b, 0],
        [0, 0, b / 8],
        [0, b, b / 8],
        [b / 2, 0, b / 8],
        [b / 2, b, b / 8],
    ]
).T
# Number of measurement samples
k = 50
# Maximum velocity allowed to the UAV
v_max = b / 100
# Distance threshold
tau = b / 50
# Smoothing factor
gamma = b / 100
# Initial position of the UAV
initial_uav_position = [10, 10, 5]
# File containing the waypoints
destinations = readpathfile("Path.csv")
# Noise seed
noise_seed = 1
# Noise Distribution
noise_distribution = "normal"
# Noise distribution parameters
mean = 0
std = 1
noise_distribution_parameters = [mean, std]

# %%
# Finally, one can invoke the [`wls`] and function and plot the estimated trajectory:

[estimated_trajectory, true_trajectory] = wls(
    a_i,
    n,
    k,
    destinations,
    initial_uav_position,
    v_max,
    tau,
    gamma,
    noise_seed,
    noise_distribution,
    noise_distribution_parameters,
)
plt_obj = plot_trajectories(destinations, [estimated_trajectory], a_i, ["WLS"])
