import numpy as np


def readpathfile(filename):
    try:
        positions = np.genfromtxt(filename, delimiter=',')
    except:
        print("File not found, please check path.")
        exit()
    return positions
