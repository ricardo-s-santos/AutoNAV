"""This file contains the file_handlers.py tests."""

import os

import pytest
from autonav.file_handlers import readpathfile
from numpy import array
from numpy.testing import assert_array_equal


def test_read_file():
    """This test the reading of input file."""
    ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
    filename = ROOT_DIR + "/path_files/Path_small.txt"
    destinations = readpathfile(filename)
    expected_destinations = array(
        [
            [75.0, 5.0, 5.0],
            [75.0, 10.0, 5.0],
            [75.0, 20.0, 5.0],
            [75.0, 23.0, 5.0],
            [55.0, 23.0, 5.0],
            [35.0, 23.0, 5.0],
            [25.0, 23.0, 5.0],
            [15.0, 23.0, 5.0],
            [10.0, 23.0, 5.0],
            [10.0, 50.0, 5.0],
        ]
    )
    assert_array_equal(expected_destinations, destinations)


def test_file_not_found():
    """Test for non-existing file."""
    ROOT_DIR = os.path.dirname(os.path.realpath(__file__))
    filename = ROOT_DIR + "/path_files/Path_not_found.txt"
    with pytest.raises(FileNotFoundError):
        readpathfile(filename)
