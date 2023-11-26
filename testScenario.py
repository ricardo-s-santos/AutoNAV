from numpy import dot

from WLS import wls
from readPathFile import readpathfile
import numpy as np

filename = "Path.txt"
destinations = readpathfile(filename)
N = 8  # Number of anchors
M = 1  # Number of target
K = 10  # Number of measurement samples
Mc = 1  # Number of Monte Carlo runs
sigma = 3  # Noise STD in meters
B = 200  # Area border in meters
Ts = 1  # Time sample in seconds
T = 10000
S = np.eye(6)  # State transition matrix
S[0, 3] = Ts
S[1, 4] = Ts
S[2, 5] = Ts
sigma_w = 0.05  # State process noise intensity  # State process noise covariance

a_i = np.array(
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

wls(a_i, N, K, sigma, destinations, [10, 10, 5])
