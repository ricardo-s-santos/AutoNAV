"""This module contains the functions to read the Path.txt file."""
import csv
import os

from numpy import array
from numpy.typing import NDArray


def _readpathfile(filename: str) -> NDArray:
    """This function reads the Path file.

    Args:
        filename: The name of the file to read.

    Returns:
        The waypoints needed to guide the drone.
    """
    positions = []
    try:  # See if file exists
        os.path.isfile(filename)
    except FileNotFoundError:
        raise FileNotFoundError("File not found, please check path.") from None
    else:
        with open(filename, "r") as file:
            csv_file = csv.reader(file)
            for line in csv_file:
                positions.append([float(x) for x in line])
    return array(positions)
