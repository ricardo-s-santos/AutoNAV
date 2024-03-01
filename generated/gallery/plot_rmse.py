"""# Extract RMSE and ARMSE metrics

"""

#%%
# First, the required dependencies are imported:
from autonav import gtrs, wls
from autonav.file_handlers import readpathfile
from autonav.metrics import compute_armse
from autonav.plots import plot_rmse
from numpy import array

#%%
# The next step is to define the simulation parameters:

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

#%%
# Invoke the [`gtrs`] and [`wls`]functions and afterwards the [`compute_armse`] and [`plot_rmse`] as follows to compute the ARMSE and plot the RMSE, respectively:

[estimated_trajectory_gtrs, true_trajectory_gtrs] = gtrs(a_i, n, k, sigma, destinations, initial_uav_position)
[estimated_trajectory_wls, true_trajectory_wls] = wls(a_i, n, k, sigma, destinations, initial_uav_position)
print(f'GTRS ARMSE: {compute_armse(estimated_trajectory_gtrs, true_trajectory_gtrs)}')
print(f'WLS ARMSE: {compute_armse(estimated_trajectory_wls, true_trajectory_wls)}')
plot_rmse([estimated_trajectory_gtrs, estimated_trajectory_wls],[true_trajectory_gtrs, true_trajectory_wls])
