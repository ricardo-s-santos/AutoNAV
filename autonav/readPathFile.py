import csv
import numpy as np


def _readpathfile(filename):
    positions = []
    try:
        with open(filename, 'r') as file:
            csv_file = csv.reader(file)
            for line in csv_file:
                positions.append([float(x) for x in line])
    except:
        print("File not found, please check path.")
        exit()
    return np.array(positions)
