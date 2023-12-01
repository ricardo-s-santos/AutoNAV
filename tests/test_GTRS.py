from autonav.GTRS import gtrs
import numpy as np
import os

ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
filename = ROOT_DIR + "/Path.txt"
N = 8  # Number of anchors
K = 10  # Number of measurement samples
sigma = 0  # Noise STD in meters
B = 200  # Area border in meters
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

gtrs(a_i, N, K, sigma, filename, [10, 10, 5])
