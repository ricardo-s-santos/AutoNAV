"""# Extract RMSE and ARMSE metrics

"""

# %%
# First, the required dependencies are imported:
from autonav import gtrs, wls
from autonav.file_handlers import readpathfile
from autonav.metrics import compute_armse
from autonav.plots import plot_rmse
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

# %%
# Invoke the [`gtrs`] and [`wls`] functions and afterwards the [`compute_armse`] and [`plot_rmse`] as follows to compute the ARMSE and plot the RMSE, respectively:

[estimated_trajectory_gtrs, true_trajectory_gtrs] = gtrs(
    a_i, n, k, destinations, initial_uav_position, v_max, tau, gamma
)
[estimated_trajectory_wls, true_trajectory_wls] = wls(a_i, n, k, destinations, initial_uav_position, v_max, tau, gamma)
armsegtrs = print(f"GTRS ARMSE: {compute_armse(estimated_trajectory_gtrs, true_trajectory_gtrs)}")
armsewls = print(f"WLS ARMSE: {compute_armse(estimated_trajectory_wls, true_trajectory_wls)}")
plt_obj = plot_rmse([estimated_trajectory_gtrs, estimated_trajectory_wls], [true_trajectory_gtrs, true_trajectory_wls])
