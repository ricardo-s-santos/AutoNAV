import numpy as np


def _readpathfile(filename):
    try:
        positions = np.genfromtxt(filename, delimiter=',')
    except:
        print("File not found, please check path.")
        exit()
    return positions
