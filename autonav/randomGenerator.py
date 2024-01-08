"""This module contains the random generator need to compute the noise."""
from numpy.random import PCG64, Generator


def randomGenerator(n, k, seed):
    """This function generates random numbers from the standart distribution."""
    gen = Generator(PCG64(seed))
    return gen.standard_normal(size=(n, k))
