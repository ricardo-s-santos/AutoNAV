import numpy as np


def readpathfile(filename):
    positions = np.genfromtxt(filename, delimiter=',')
    return positions
