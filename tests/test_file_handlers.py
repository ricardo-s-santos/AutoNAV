"""This file contains the file_handlers.py tests."""

import os

import pytest
from numpy import array
from numpy.testing import assert_array_equal

from autonav.file_handlers import readpathfile


def test_read_file():
    """Tests the reading of an existing and correctly formatted path file."""
    root_dir = os.path.dirname(os.path.realpath(__file__))
    filename = root_dir + "/path_files/Path_small.csv"
    destinations = readpathfile(filename)
    expected_destinations = array(
        [
            [35.0, 5.0, 5.0],
            [35.0, 20.0, 5.0],
            [35.0, 23.0, 5.0],
            [25.0, 30.0, 5.0],
        ]
    )
    assert_array_equal(expected_destinations, destinations)


def test_file_not_found():
    """Tests if a FileNotFoundError exception is raised when reading a non-existing path file."""
    root_dir = os.path.dirname(os.path.realpath(__file__))
    filename = root_dir + "/path_files/Path_not_found.csv"
    with pytest.raises(FileNotFoundError):
        readpathfile(filename)
