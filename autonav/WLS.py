"""This module contains the WLS algorithm."""

import cmath
import itertools
import math

from numpy import array, asarray, cos, dot, eye, median, sin, size, sqrt, zeros
from numpy.lib import scimath
from numpy.linalg import norm, solve
from numpy.typing import ArrayLike, NDArray

from autonav.random_generator import random_generator
from autonav.velocity import _velocity


def wls(
    a_i: ArrayLike,
    n: int,
    k: int,
    sigma: float,
    destinations: ArrayLike,
    initial_uav_position: ArrayLike,
    v_max: int,
    tau: int,
    gamma: int,
    noise_seed: int = 1,
) -> NDArray:
    """Executes the WLS algorithm.

    [See here more details about the WLS algorithm.]
    (https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/wss2.12041)

    Args:
        a_i: The true position of the anchors in 3D.
        n: The number of anchors.
        k: The number of measurements.
        sigma: The noise level in meters.
        destinations: The intermediate points need for navigation in 3D.
        initial_uav_position: The initial UAV position in 3D.
        v_max: The maximum velocity that the UAV can fly.
        tau: The threshold to reach the destination.
        gamma: The smoothing factor.
        noise_seed: The seed to generate the noise.

    Returns:
        The estimated trajectory computed using the WLS algorithm for the given input scenario
          and the true trajectory that the UAV followed.
    """
    # Transform inputs in NDArray
    arr_a_i: NDArray = asarray(a_i, dtype=float)
    arr_destinations: NDArray = asarray(destinations, dtype=float)
    arr_initial_uav_position: NDArray = asarray(initial_uav_position, dtype=float)
    # Validate inputs
    if size(arr_a_i, axis=1) != n:
        raise ValueError("The length of a_i must be equal to N.")
    if k <= 0:
        raise ValueError("K must be positive.")
    if sigma < 0 or sigma > 5:
        raise ValueError("Sigma must be between 0 and 5.")
    if size(arr_destinations) == 0:
        raise ValueError("Waypoints cannot be empty.")
    if size(arr_destinations, axis=1) != 3:
        raise ValueError("Waypoints must contain the 3 coordinates (x, y, z).")
    if size(initial_uav_position) != 3:
        raise ValueError("Initial UAV position must contain the 3 coordinates (x, y, z).")
    x_true = arr_initial_uav_position[:]
    ww = 0
    n_dest = len(arr_destinations) - 1
    estimated_trajectory = []
    true_trajectory = []
    # Generator to create random numbers (see line 65)
    gen = random_generator(noise_seed)
    while ww <= n_dest:
        distance = math.sqrt(
            (x_true[0] - arr_destinations[ww][0]) ** 2
            + (x_true[1] - arr_destinations[ww][1]) ** 2
            + (x_true[2] - arr_destinations[ww][2]) ** 2
        )
        while distance > 1:
            x = x_true[0:3]
            # ---------------------------------------------------------------------
            # Simulation part
            # ---------------------------------------------------------------------
            di_k = sqrt(((x[0] - arr_a_i[0, :]) ** 2) + ((x[1] - arr_a_i[1, :]) ** 2) + ((x[2] - arr_a_i[2, :]) ** 2))
            di_k = array([di_k]).T
            di_k = di_k + (sigma * gen.standard_normal(size=(n, k)))
            d_i = median(di_k, axis=1)
            d_i = array([d_i]).T
            # ---------------------------------------------------------------------
            # Estimation part
            # ---------------------------------------------------------------------
            xi_est = zeros(shape=(3, n))
            phi_i = zeros(shape=(n, 1))
            alpha_i = zeros(shape=(n, 1))
            for ii in range(0, n):
                kk = [ii + 1]
                for jj in range(0, n):
                    total = 0
                    if arr_a_i[0, ii] == arr_a_i[0, jj]:
                        total += 1
                    if arr_a_i[1, ii] == arr_a_i[1, jj]:
                        total += 1
                    if ii != jj and total > 0:
                        kk.append(jj + 1)
                a2 = zeros(shape=(len(array(list(itertools.combinations(kk, 2)))), 3))
                b2 = zeros(shape=(len(array(list(itertools.combinations(kk, 2)))), 1))
                for uu in range(0, len(array(list(itertools.combinations(kk, 2))))):
                    combinations = array(list(itertools.combinations(kk, 2)))
                    gg = combinations[uu, 0]
                    hh = combinations[uu, 1]
                    a2[uu, :] = 2 * (arr_a_i[0:3, gg - 1] - arr_a_i[0:3, hh - 1]).T
                    b2[uu] = (
                        d_i[hh - 1] ** 2
                        - d_i[gg - 1] ** 2
                        - norm(arr_a_i[0:3, hh - 1]) ** 2
                        + norm(arr_a_i[0:3, gg - 1]) ** 2
                    )
                xi_est[:, [ii]] = solve(dot(a2.T, a2) + (1 * 10 ** (-6)) * eye(3), dot(a2.T, b2))
                di_xy = norm(xi_est[0:2, 0])
                xi_est[2][ii] = (cmath.sqrt((d_i[0] ** 2)[0] - (di_xy**2)).real) + (
                    cmath.sqrt((d_i[0] ** 2)[0] - (di_xy**2)).imag
                )
                phi_i[ii] = (
                    math.atan2((xi_est[1][ii] - arr_a_i[1, ii]), (xi_est[0][ii] - arr_a_i[0, ii])) * 180 / math.pi
                )
                alpha_i[ii] = (
                    math.acos((xi_est[2][ii] - arr_a_i[2, ii]) / (norm(xi_est[:, ii] - arr_a_i[:, ii]))) * 180 / math.pi
                )
            u_i_1 = cos((phi_i * math.pi) / 180).T
            u_i_2 = sin((alpha_i * math.pi) / 180).T
            u_i_3 = sin((phi_i * math.pi) / 180).T
            u_i_4 = cos((alpha_i * math.pi) / 180).T
            u_i = zeros(shape=(3, n))
            u_i[0, :] = u_i_1 * u_i_2
            u_i[1, :] = u_i_3 * u_i_2
            u_i[2, :] = u_i_4
            a_1 = u_i.T
            b_1 = d_i + (sum(u_i * arr_a_i).T.reshape(n, 1))
            w_i = asarray((1 / d_i) / (sum(1 / d_i)))
            w = asarray(eye(n) * scimath.sqrt(w_i))
            a_loc = dot(w, a_1)
            b_loc = dot(w, b_1)
            x_est = asarray(solve(dot(a_loc.T, a_loc) + (1 * 10 ** (-6)) * eye(3), dot(a_loc.T, b_loc)))
            estimated_trajectory.append(x_est[:, 0])
            true_trajectory.append(x_true.copy())
            uav_velocity = _velocity(x_est[:, 0], arr_destinations[ww, :], v_max, tau, gamma)
            x_true[0] = x_true[0] + uav_velocity[0]
            x_true[1] = x_true[1] + uav_velocity[1]
            x_true[2] = x_true[2] + uav_velocity[2]
            distance = math.sqrt(
                (x_true[0] - arr_destinations[ww][0]) ** 2
                + (x_true[1] - arr_destinations[ww][1]) ** 2
                + (x_true[2] - arr_destinations[ww][2]) ** 2
            )
        ww += 1
    return array([array(estimated_trajectory), array(true_trajectory)])
