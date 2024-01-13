"""This file contains the velocity.py tests."""

import pytest
from autonav.velocity import _velocity
from numpy import array
from numpy.testing import assert_allclose, assert_array_equal


@pytest.mark.critical()
def test_velocity_higher_distance():
    """Test velocity function in normal conditions when the error_norm is higher than 1."""
    current_position = array([0, 0, 5])
    destination_position = array([10, 10, 5])
    estimated_velocity = _velocity(current_position, destination_position)
    assert_allclose(estimated_velocity, array([1.41421356, 1.41421356, 0]))


@pytest.mark.critical()
def test_velocity_lower_distance():
    """Test velocity function in normal conditions when the error_norm is lower than the param_reach_distance."""
    current_position = array([8, 10, 5])
    destination_position = array([10, 10, 5])
    estimated_velocity = _velocity(current_position, destination_position)
    assert_allclose(estimated_velocity, array([0.5, 0, 0]))


@pytest.mark.critical()
def test_velocity_reached_destination():
    """Test velocity function when reaching the destination."""
    current_position = array([10, 10, 5])
    destination_position = array([10, 10, 5])
    estimated_velocity = _velocity(current_position, destination_position)
    assert_array_equal(estimated_velocity, array([0, 0, 0]))


@pytest.mark.parametrize(
    ("current_position", "destination_position", "expected_error"),
    [
        (array(["a", "a", "a"]), array([10, 10, 5]), TypeError),
        (array([10, 10, 5]), array(["a", "a", "a"]), TypeError),
    ],
)
def test_velocity_invalid_parameters(current_position, destination_position, expected_error):
    """Test velocity function when the parameters are incorrect."""
    with pytest.raises(expected_error):
        _velocity(current_position, destination_position)
