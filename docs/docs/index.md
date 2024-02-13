# AutoNAV

<div style="text-align:center">
  <img src="https://github.com/ricardo-s-santos/AutoNAV/blob/main/docs/docs/figures/icon.png?raw=true" alt="image" width="200" height="auto">
</div>

A Python package for simulating UAV Navigation in Satellite-Less Environments. The package contains two algorithms the GTRS and WLS whose goal is to estimate and navigate a UAV. Apart from these algorithms a hand-full of other functions and features are also provided, such as, measure the performance using metrics and plot the trajectories.

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
destinations = readpathfile("tests/path_files/Path.txt")
```

Finally, call the GTRS or WLS algorithm and plot the trajectories:

```python
[estimated_trajectory, true_trajectory] = gtrs(a_i, n, k, sigma, destinations, initial_uav_position)
plot_trajectories(destinations, [estimated_trajectory], a_i, ['GTRS'])
plt.show()
```

<p align="center">
  <img src="https://github.com/ricardo-s-santos/AutoNAV/blob/main/docs/docs/figures/trajectories_plot.png?raw=true" alt="image" width="auto" height="auto">
</p>
