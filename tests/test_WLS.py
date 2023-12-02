from autonav.WLS import wls
from autonav.fileHandlers import _readpathfile
from numpy import array
import os

ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
filename = ROOT_DIR + "/Path.txt"
destinations = _readpathfile(filename)
N = 8  # Number of anchors
M = 1  # Number of target
K = 10  # Number of measurement samples
Mc = 1  # Number of Monte Carlo runs
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

wls(a_i, N, K, sigma, destinations, [10, 10, 5])
