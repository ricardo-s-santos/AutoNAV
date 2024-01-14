![example workflow](https://github.com/Ricardo-Santos-21904332/AutoNAV/actions/workflows/test.yml/badge.svg)
[![codecov](https://codecov.io/gh/Ricardo-Santos-21904332/AutoNAV/graph/badge.svg?token=LCR7KDRK3E)](https://codecov.io/gh/Ricardo-Santos-21904332/AutoNAV)
[![docs](https://img.shields.io/badge/docs-click_here-blue.svg)](https://ricardo-santos-21904332.github.io/AutoNAV/)
[![PyPI](https://img.shields.io/pypi/v/autonav)](https://pypi.org/project/autonav/)

# AutoNAV

A Python package for simulating UAV Navigation in Satellite-Less Environments. The package contains two algorithms the GTRS and WLS whose goal is to estimate and navigate a UAV.

<p align="center">
  <img src="https://github.com/Ricardo-Santos-21904332/AutoNAV/blob/main/docs/docs/figures/icon.png?raw=true" alt="image" width="100" height="auto">
</p>

## Installation

Install from PyPI:

```sh
pip install --upgrade pip
pip install autonav
```

## First Steps

After installing the package one can import the algorithms and necessary dependencies as follows:

```python
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
destinations = _readpathfile("tests/path_files/Path.txt")
```

Finally, call the GTRS or WLS algorithm and plot the trajectories:

```python
[estimated_trajectory, true_trajectory] = gtrs(a_i, n, k, sigma, destinations, initial_uav_position)
plot_trajectories(destinations, estimated_trajectory, a_i)
```

![Trajectories](https://github.com/Ricardo-Santos-21904332/AutoNAV/blob/main/docs/docs/figures/trajectories.png?raw=true)

## Reference

## License
