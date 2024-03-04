"""This file contains the random_generator.py tests."""

from numpy import array
from numpy.testing import assert_allclose

from autonav.random_generator import random_generator


def test_random_generator():
    """Verifies the correct operation of the random generator in normal conditions."""
    gen = random_generator(0)
    random_number = gen.standard_normal(size=(1, 1))
    assert_allclose(array([[0.12573022]]), random_number)
