# Examples

In this page a few examples of the algorithms in action are provided.

# Run and plot the trajctories using the GTRS algorithm

```python
from autonav import gtrs
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

Finally, call the GTRS algorithm and plot the trajectories:

```python
[estimated_trajectory, true_trajectory] = gtrs(a_i, n, k, sigma, destinations, initial_uav_position)
plot_trajectories(destinations, estimated_trajectory, a_i)
```

![Trajectories](https://github.com/Ricardo-Santos-21904332/AutoNAV/blob/main/docs/docs/figures/gtrs.png?raw=true)

# Run and plot the trajctories using the WLS algorithm

```python
from autonav import wls
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

Finally, call the WLS and plot the trajectories:

```python
[estimated_trajectory, true_trajectory] = wls(a_i, n, k, sigma, destinations, initial_uav_position)
plot_trajectories(destinations, estimated_trajectory, a_i)
```

![Trajectories](https://github.com/Ricardo-Santos-21904332/AutoNAV/blob/main/docs/docs/figures/wls.png?raw=true)

# Extract the RMSE and ARMSE from the algorithms
