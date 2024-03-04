![example workflow](https://github.com/ricardo-s-santos/AutoNAV/actions/workflows/test.yml/badge.svg)
[![codecov](https://codecov.io/gh/ricardo-s-santos/AutoNAV/graph/badge.svg?token=LCR7KDRK3E)](https://codecov.io/gh/ricardo-s-santos/AutoNAV)
[![docs](https://img.shields.io/badge/docs-click_here-blue.svg)](https://ricardo-s-santos.github.io/AutoNAV/)
[![PyPI](https://img.shields.io/pypi/v/autonav)](https://pypi.org/project/autonav/)

<p align="center">
  <img src="https://github.com/ricardo-s-santos/AutoNAV/blob/main/docs/docs/figures/icon.png?raw=true" alt="image" width="200" height="auto">
</p>

A Python package for simulating UAV Navigation in Satellite-Less Environments. The package contains two algorithms the GTRS <a href="https://ieeexplore.ieee.org/document/9456863">[1]</a> and WLS <a href="https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/wss2.12041">[2]</a>  whose goal is to estimate and navigate a UAV.

## Installation

Install from PyPI:

```sh
pip install --upgrade pip
pip install autonav
```

## First Steps

After installing the package one can import the algorithms and necessary dependencies as follows:

```python
import matplotlib.pyplot as plt

from autonav import gtrs, wls
from autonav.file_handlers import readpathfile
from autonav.plots import plot_trajectories
from numpy import array
```

Afterwards, one can create the necessary values to run the algorithms:

```python
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
    [b / 2, b, b / 8],]
    ).T
# Number of measurement samples
k = 50
# Noise standard deviation
sigma = 1
# Maximum velocity allowed to the UAV
v_max = b / 100
# Distance threshold
tau = b / 50
# Smoothing factor
gamma = b / 100
# Initial position of the UAV
initial_uav_position = [10, 10, 5]
# File containing the waypoints
destinations = readpathfile("docs/docs/examples/Path.csv")
```

Finally, run the GTRS or WLS algorithm and plot the trajectories:

```python
# Estimate the trajectory using the GTRS algorithm
[estimated_trajectory, true_trajectory] = gtrs(a_i, n, k, sigma, destinations, initial_uav_position, v_max, tau, gamma)
# Plot the estimated trajectory
plot_trajectories(destinations, [estimated_trajectory], a_i, ['GTRS'])
plt.show()
```

<p align="center">
  <img src="https://github.com/ricardo-s-santos/AutoNAV/blob/main/docs/docs/figures/trajectories_plot.png?raw=true" alt="image" width="auto" height="auto">
</p>

## References

[1] J. P. Matos-Carvalho, R. Santos, S. Tomic and M. Beko, "GTRS-Based Algorithm for UAV Navigation in Indoor Environments Employing Range Measurements and Odometry," in IEEE Access, vol. 9, pp. 89120-89132, 2021, doi: 10.1109/ACCESS.2021.3089900. https://ieeexplore.ieee.org/document/9456863

[2] R. Santos, J. P. Matos-Carvalho, S. Tomic and M. Beko, "WLS algorithm for UAV navigation in satellite‐less environments," in IET Wireless Sensor Systems, 2022, 12, (3-4), p. 93-102, DOI: 10.1049/wss2.12041
IET Digital Library, https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/wss2.12041

## License

[MIT License](LICENSE.txt)
