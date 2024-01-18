"""This module contains the random generator needed to compute the noise."""
from numpy.random import PCG64, Generator


def randomGenerator(seed):
    """This function creates a generator to be used by the algorithms."""
    gen = Generator(PCG64(seed))
    return gen
