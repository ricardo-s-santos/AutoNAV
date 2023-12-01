import numpy as np


def readpathfile(filename):
    positions = []
    try:
        positions = np.genfromtxt(filename, delimiter=',')
    except FileNotFoundError:
        print("File not found, please check path.")
        exit()
    return positions
