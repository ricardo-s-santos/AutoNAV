"""This module contains the functions to read the path file."""

import csv
import os

from numpy import array
from numpy.typing import NDArray


def readpathfile(filename: str) -> NDArray:
    """Reads the path file.

    Args:
        filename: The name of the file to read.

    Returns:
        The waypoints needed to guide the drone.
    """
    positions = []
    if os.path.isfile(filename):  # See if file exists
        with open(filename, "r") as file:
            csv_file = csv.reader(file)
            for line in csv_file:
                positions.append([float(x) for x in line])
    else:
        raise FileNotFoundError("File not found, please check path.") from None
    return array(positions)
