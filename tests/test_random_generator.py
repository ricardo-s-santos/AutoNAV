"""This file contains the random_generator.py tests."""

from autonav.random_generator import randomGenerator
from numpy import array
from numpy.testing import assert_allclose


def test_random_generator():
    """This test verifies the random generator in normal conditions."""
    random_number = randomGenerator(1, 1, 0)
    assert_allclose(array([[0.12573022]]), random_number)
