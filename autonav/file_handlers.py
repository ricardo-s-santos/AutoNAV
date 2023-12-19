"""This module contains the functions to read the Path.txt file."""
import csv

from numpy import array
from numpy.typing import NDArray


def _readpathfile(filename: str) -> NDArray:
    positions = []
    try:
        with open(filename, "r") as file:
            csv_file = csv.reader(file)
            for line in csv_file:
                positions.append([float(x) for x in line])
    except FileNotFoundError:
        print("File not found, please check path.")
        exit()
    return array(positions)
