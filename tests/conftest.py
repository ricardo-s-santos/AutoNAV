"""This file contains the fixtures needed for the tests."""

import os

import pytest
from autonav.file_handlers import _readpathfile
from numpy import array


@pytest.fixture()
def default_values():
    """This fixture defines the default values to be used in the tests."""
    ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
    filename = ROOT_DIR + "/Path_small.txt"
    destinations = _readpathfile(filename)
    N = 8  # Number of anchors
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
        ]
    ).T
    initial_uav_position = [10, 10, 5]
    return [a_i, N, destinations, initial_uav_position]
