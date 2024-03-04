"""This module contains the random generator needed to compute the noise."""

from numpy.random import PCG64, Generator


def random_generator(seed):
    """Creates a generator to be used by the algorithms.

    Args:
        seed: The seed to allow reproducibility.

    Returns:
        A radom number generator based on the PCG64.
    """
    gen = Generator(PCG64(seed))
    return gen
