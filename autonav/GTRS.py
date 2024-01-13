"""This module contains the GTRS algorithm functions."""
import math

from autonav.file_handlers import readpathfile
from autonav.random_generator import randomGenerator
from autonav.velocity import _velocity
from numpy import (
    append,
    arange,
    array,
    concatenate,
    diag,
    dot,
    eye,
    insert,
    matmul,
    median,
    ones,
    real,
    size,
    sqrt,
    subtract,
    zeros,
)
from numpy.linalg import eigvals, inv, norm, pinv, solve
from numpy.typing import NDArray
from scipy.linalg import fractional_matrix_power, sqrtm


def gtrs(
    a_i: NDArray,
    n: int,
    k: int,
    sigma: float,
    destinations: NDArray,
    initial_uav_position: list,
    noise_seed=0,
    tol: float = 0.001,
    n_iter: int = 30,
    max_lim: float = 1000000.0,
) -> NDArray:
    """This function executes the GTRS algorithm.

    Args:
        a_i (NDArray): The true position of the anchors in 3D.
        n (int): The number of anchors.
        k (int): The number of measurements.
        sigma (float): The noise level in meters.
        destinations (NDArray): The intermediate points need for navigation in 3D.
        initial_uav_position (list): The initial UAV position in 3D.
        noise_seed (int): The seed to generate the noise.
        tol (float): The tolerance for the bisection function.
        n_iter (int): The max number of iterations for the bisection function.
        max_lim (float): The maximum value for the interval in the bisection function.

    Returns:
        The estimated trajectory computed using the GTRS algorithm for the given input scenario
        and the true trajectory that the UAV followed.
    """
    ts = 1  # Time sample in seconds
    s = eye(6)  # State transition matrix
    s[0, 3] = ts
    s[1, 4] = ts
    s[2, 5] = ts
    sigma_w = 0.05  # State process noise intensity  # State process noise covariance
    q = dot(
        sigma_w**2,
        (
            [
                [ts**3 / 3, 0, 0, ts**2 / 2, 0, 0],
                [0, ts**3 / 3, 0, 0, ts**2 / 2, 0],
                [0, 0, ts**3 / 3, 0, 0, ts**2 / 2],
                [ts**2 / 2, 0, 0, ts, 0, 0],
                [0, ts**2 / 2, 0, 0, ts, 0],
                [0, 0, ts**2 / 2, 0, 0, ts],
            ]
        ),
    )
    x_state = zeros((6, 1))
    x_loc = zeros((3, 1))
    p = None
    qq = 0
    x_true = initial_uav_position[:]
    estimated_trajectory = []
    true_trajectory = []
    ww = 0
    n_dest = len(destinations) - 1
    while ww <= n_dest:
        distance = math.sqrt(
            (x_true[0] - destinations[ww][0]) ** 2
            + (x_true[1] - destinations[ww][1]) ** 2
            + (x_true[2] - destinations[ww][2]) ** 2
        )
        while distance > 1:
            x = x_true[0:3]
            # ---------------------------------------------------------------------
            # Simulation part
            # ---------------------------------------------------------------------
            di_k = sqrt(((x[0] - a_i[0, :]) ** 2) + ((x[1] - a_i[1, :]) ** 2) + ((x[2] - a_i[2, :]) ** 2))
            di_k = array([di_k]).T
            # di_k_bom = di_k + (sigma * randn(n, k))
            noise_seed += 1
            di_k = di_k + (sigma * randomGenerator(n, k, noise_seed))
            d_i = median(di_k, axis=1)
            d_i = array([d_i]).T
            # ---------------------------------------------------------------------
            # Estimation part
            # ---------------------------------------------------------------------
            a1_loc = []
            b1_loc = []
            a_track = []
            d_track = []
            f_track = []
            b_track = []
            w_i_loc = array(sqrt(d_i ** (-1.0) / (sum(d_i ** (-1.0)))))
            for tt in arange(0, n, 1).reshape(-1):
                a1_loc.append(append(dot(2, a_i[0:3, tt].T), -1))
                b1_loc.append(norm(a_i[0:3, tt]) ** 2 - d_i[tt] ** 2)
            a1_loc = array(a1_loc)
            b1_loc = array(b1_loc)
            w_loc = diag(w_i_loc.T[0])
            d_loc = eye(4)
            d_loc[3, 3] = 0
            f_loc = array([0, 0, 0, -1.0 / 2.0]).reshape(4, 1)
            a_loc = dot(w_loc, a1_loc)
            b_loc = dot(w_loc, b1_loc)
            if qq != 0:
                p_pred = dot(dot(s, p), s.T) + q
                x_pred = dot(s, x_state[0:6, qq - 1])
                x_pred = x_pred.reshape(len(x_pred), 1)
                a1_track = []
                b1_track = []
                for tt in arange(0, n, 1).reshape(-1):
                    a1_track.append(concatenate((dot(2, a_i[0:3, tt].T), zeros(size(x, 0)), [-1]), axis=0))
                    b1_track.append(norm(a_i[0:3, tt]) ** 2 - abs(d_i[tt] ** 2))
                a1_track = array(a1_track)
                b1_track = array(b1_track)
                left_matrix = sqrtm(inv(p_pred))
                right_matrix = zeros((size(x_state, 0), 1))
                a1_track = concatenate((a1_track, concatenate((left_matrix, right_matrix), axis=1)), axis=0)
                a1_track[a1_track == math.inf] = 0
                inf_p_pred = array(sqrtm(inv(p_pred)))
                inf_p_pred[inf_p_pred == math.inf] = 0
                b1_track = concatenate((b1_track, dot(inf_p_pred, x_pred)), axis=0)
                a = dot(math.sqrt(1.0 / 2.0), w_i_loc.T)
                b = dot(math.sqrt(1.0 / 8.0), ones((1, size(x_state, 0))))
                w_track = concatenate((a, b), axis=1)
                w_track = eye(size(w_track, 1)) * w_track
                d_track = zeros((7, 7))
                d_track[0, 0] = 1
                d_track[1, 1] = 1
                d_track[2, 2] = 1
                f_track = array([0, 0, 0, 0, 0, 0, -1.0 / 2.0]).reshape(7, 1)
                a_track = dot(w_track, a1_track)
                b_track = dot(w_track, b1_track)
            eigen_values = _calc_eigen(a_loc, d_loc)
            eig_1 = max(eigen_values)
            min_lim = -1.0 / eig_1
            lambda_loc = _bisection_fun(min_lim, max_lim, tol, n_iter, a_loc, d_loc, b_loc, f_loc)
            y_hat_loc = solve(
                (dot(a_loc.T, a_loc) + dot(lambda_loc, d_loc) + dot(1e-06, eye(size(a_loc, 1)))),
                (dot(a_loc.T, b_loc) - dot(lambda_loc, f_loc)),
            )
            if qq == 0:
                x_loc[0:3, qq] = real(y_hat_loc[0:3, 0])
                x_state[0:6, qq] = concatenate((x_loc[0:3, qq], [0], [0], [0]), axis=0)
                p = eye(6)
                estimated_trajectory.append(x_loc[0:3, qq])
            else:
                x_loc = insert(x_loc, qq, real(y_hat_loc[0:3, 0]), axis=1)
                eigen_values = _calc_eigen(a_track, d_track)
                eig_1 = max(eigen_values)
                min_lim = -1.0 / eig_1
                lambda_track = _bisection_fun(min_lim, max_lim, tol, n_iter, a_track, d_track, b_track, f_track)
                y_hat_track = solve(
                    (dot(a_track.T, a_track) + dot(lambda_track, d_track) + dot(1e-06, eye(size(a_track, 1)))),
                    (dot(a_track.T, b_track) - dot(lambda_track, f_track)),
                )
                x_state = concatenate((x_state, zeros((size(x_state, 0), 1))), axis=1)
                x_state[0:6, qq] = concatenate((real(y_hat_track[arange(0, size(x_state, 0))])), axis=0)
                lk1 = subtract(x_state[0:6, qq], x_state[0:6, qq - 1]).reshape((6, 1))
                lk2 = subtract(x_state[0:6, qq], x_state[0:6, qq - 1]).reshape((6, 1)).T
                p = matmul(lk1, lk2)
                estimated_trajectory.append(x_loc[0:3, qq])
            true_trajectory.append(x_true[:])
            uav_velocity = _velocity(x_loc[0:3, qq], destinations[ww, :])
            x_true[0] = x_true[0] + uav_velocity[0]
            x_true[1] = x_true[1] + uav_velocity[1]
            x_true[2] = x_true[2] + uav_velocity[2]
            distance = math.sqrt(
                (x_true[0] - destinations[ww][0]) ** 2
                + (x_true[1] - destinations[ww][1]) ** 2
                + (x_true[2] - destinations[ww][2]) ** 2
            )
            qq += 1
        ww += 1
    return array([array(estimated_trajectory), array(true_trajectory)])


def _bisection_fun(
    min_lim: float,
    max_lim: float,
    tol: float,
    n_iter: int,
    a: NDArray,
    d: NDArray,
    b: NDArray,
    f: NDArray,
) -> float:
    """This function executes the bisection procedure to solve the GTRS problem.

    Note:
        The definitions of the matrices can be seen in the paper.

    Args:
        min_lim (float): Minimum interval value.
        max_lim (float): Maximum interval value.
        tol (float): Tolerance value
        n_iter (int): The max number of interactions for the bisection procedure.
        a (NDArray): Matrix A.
        d (NDArray): Matrix D.
        b (NDArray): Matrix b.
        f (NDArray): Matrix f.

    Returns:
        The solution to the GTRS problem.
    """
    lambda_ = (min_lim + max_lim) / 2
    fun_val = 10**9
    num_iter = 1
    while num_iter <= n_iter and abs(fun_val) > tol and abs(min_lim - max_lim) > 0.0001:
        lambda_ = (min_lim + max_lim) / 2
        fun_val = _fi_fun(lambda_, a, d, b, f)
        if fun_val > 0:
            min_lim = lambda_.copy()
        else:
            max_lim = lambda_.copy()
        num_iter = num_iter + 1
    return lambda_


def _fi_fun(lambda_1: float, a: NDArray, d: NDArray, b: NDArray, f: NDArray) -> NDArray:
    """This function computes the fi value for a given lambda.

    Note:
        The definitions of the matrices can be seen in the paper.

    Args:
        lambda_1 (float): Minimum interval value.
        a (NDArray): Matrix A.
        d (NDArray): Matrix D.
        b (NDArray): Matrix b.
        f (NDArray): Matrix f.

    Returns:
        An NDArray with the fi value for the given input.
    """
    g_ = dot(a.T, a)
    gg_ = dot(lambda_1, d)
    ggg_ = dot(1e-06, eye(size(d, 1)))
    t_ = dot(a.T, b)
    ttt_ = dot(lambda_1, f)
    y = solve((g_ + gg_ + ggg_), (t_ - ttt_))
    fi = dot(dot(y.T, d), y) + dot(dot(2, f.T), y)
    return fi


def _calc_eigen(a: NDArray, d: NDArray) -> NDArray:
    """This function computes the Eigen values of the matrices.

    Note:
        The definitions of the matrices can be seen in the paper.

    Args:
        a (NDArray): Matrix A.
        d (NDArray): Matrix D.

    Returns:
        The Eigen values for the given matrices.

    Raises:
        Exception if the Eigen Values cannot be computed.
    """
    try:
        left = dot(a.conj().transpose(), a)
        left = fractional_matrix_power(left, 0.5)
        right = dot(a.conj().transpose(), a)
        right = fractional_matrix_power(right, 0.5)
        aux = solve(left, d)
        result = dot(aux, pinv(right))
        return eigvals(result)
    except Exception:
        print("An exception occurred")
        return array([0])


if __name__ == "__main__":
    b = 200
    n = 8
    a_i = array(
        [
            [0, 0, 0],
            [0, b, 0],
            [b / 2, 0, 0],
            [b / 2, b, 0],
            [0, 0, b / 8],
            [0, b, b / 8],
            [b / 2, 0, b / 8],
            [b / 2, b, b / 8],
        ]
    ).T
    k = 50
    sigma = 1
    initial_uav_position = [10, 10, 5]
    destinations = readpathfile(
        "/Users/ricardo/Documents/Doutoramento/Software De Investigação/AutoNAV/tests/path_files/Path_small.txt"
    )
    gtrs(a_i, n, k, sigma, destinations, initial_uav_position)
