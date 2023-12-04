import math

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
from numpy.random import randn
from numpy.typing import NDArray
from scipy.linalg import fractional_matrix_power, sqrtm


def gtrs(
    a_i: NDArray, N: int, K: int, sigma: float, destinations: NDArray, initial_uav_position: list
) -> NDArray:
    """
    This function executes the GTRS algorithm
    """
    Ts = 1  # Time sample in seconds
    S = eye(6)  # State transition matrix
    S[0, 3] = Ts
    S[1, 4] = Ts
    S[2, 5] = Ts
    sigma_w = 0.05  # State process noise intensity  # State process noise covariance
    Q = dot(
        sigma_w**2,
        (
            [
                [Ts**3 / 3, 0, 0, Ts**2 / 2, 0, 0],
                [0, Ts**3 / 3, 0, 0, Ts**2 / 2, 0],
                [0, 0, Ts**3 / 3, 0, 0, Ts**2 / 2],
                [Ts**2 / 2, 0, 0, Ts, 0, 0],
                [0, Ts**2 / 2, 0, 0, Ts, 0],
                [0, 0, Ts**2 / 2, 0, 0, Ts],
            ]
        ),
    )
    x_state = zeros((6, 1))
    x_loc = zeros((3, 1))
    P = None
    qq = 0
    x_true = initial_uav_position
    estimated_trajectory = []
    ww = 0
    N_dest = len(destinations) - 1
    while ww <= N_dest:
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
            di_k = sqrt(
                ((x[0] - a_i[0, :]) ** 2) + ((x[1] - a_i[1, :]) ** 2) + ((x[2] - a_i[2, :]) ** 2)
            )
            di_k = array([di_k]).T
            di_k = di_k + (sigma * randn(N, K))
            d_i = median(di_k, axis=1)
            d_i = array([d_i]).T
            # ---------------------------------------------------------------------
            # Estimation part
            # ---------------------------------------------------------------------
            A1_loc = []
            b1_loc = []
            A_track = []
            D_track = []
            f_track = []
            b_track = []
            w_i_loc = array(sqrt(d_i ** (-1.0) / (sum(d_i ** (-1.0)))))
            for tt in arange(0, N, 1).reshape(-1):
                A1_loc.append(append(dot(2, a_i[0:3, tt].T), -1))
                b1_loc.append(norm(a_i[0:3, tt]) ** 2 - d_i[tt] ** 2)
            A1_loc = array(A1_loc)
            b1_loc = array(b1_loc)
            W_loc = diag(w_i_loc.T[0])
            D_loc = eye(4)
            D_loc[3, 3] = 0
            f_loc = array([0, 0, 0, -1.0 / 2.0]).reshape(4, 1)
            A_loc = dot(W_loc, A1_loc)
            b_loc = dot(W_loc, b1_loc)
            if qq != 0:
                P_pred = dot(dot(S, P), S.T) + Q
                x_pred = dot(S, x_state[0:6, qq - 1])
                x_pred = x_pred.reshape(len(x_pred), 1)
                A1_track = []
                b1_track = []
                for tt in arange(0, N, 1).reshape(-1):
                    A1_track.append(
                        concatenate((dot(2, a_i[0:3, tt].T), zeros(size(x, 0)), [-1]), axis=0)
                    )
                    b1_track.append(norm(a_i[0:3, tt]) ** 2 - abs(d_i[tt] ** 2))
                A1_track = array(A1_track)
                b1_track = array(b1_track)
                left_matrix = sqrtm(inv(P_pred))
                right_matrix = zeros((size(x_state, 0), 1))
                A1_track = concatenate(
                    (A1_track, concatenate((left_matrix, right_matrix), axis=1)), axis=0
                )
                A1_track[A1_track == math.inf] = 0
                INF_P_pred = array(sqrtm(inv(P_pred)))
                INF_P_pred[INF_P_pred == math.inf] = 0
                b1_track = concatenate((b1_track, dot(INF_P_pred, x_pred)), axis=0)
                a = dot(math.sqrt(1.0 / 2.0), w_i_loc.T)
                b = dot(math.sqrt(1.0 / 8.0), ones((1, size(x_state, 0))))
                W_track = concatenate((a, b), axis=1)
                W_track = eye(size(W_track, 1)) * W_track
                D_track = zeros((7, 7))
                D_track[0, 0] = 1
                D_track[1, 1] = 1
                D_track[2, 2] = 1
                f_track = array([0, 0, 0, 0, 0, 0, -1.0 / 2.0]).reshape(7, 1)
                A_track = dot(W_track, A1_track)
                b_track = dot(W_track, b1_track)
            eigen_values = _calc_eigen(A_loc, D_loc)
            eig_1 = max(eigen_values)
            min_lim = -1.0 / eig_1
            max_lim = 1000000.0
            tol = 0.001
            N_iter = 30
            lambda_loc = _bisection_fun(min_lim, max_lim, tol, N_iter, A_loc, D_loc, b_loc, f_loc)
            y_hat_loc = solve(
                (dot(A_loc.T, A_loc) + dot(lambda_loc, D_loc) + dot(1e-06, eye(size(A_loc, 1)))),
                (dot(A_loc.T, b_loc) - dot(lambda_loc, f_loc)),
            )
            if qq == 0:
                x_loc[0:3, qq] = real(y_hat_loc[0:3, 0])
                x_state[0:6, qq] = concatenate((x_loc[0:3, qq], [0], [0], [0]), axis=0)
                P = eye(6)
                estimated_trajectory.append(x_loc[0:3, qq])
            else:
                x_loc = insert(x_loc, qq, real(y_hat_loc[0:3, 0]), axis=1)
                eigen_values = _calc_eigen(A_track, D_track)
                eig_1 = max(eigen_values)
                min_lim = -1.0 / eig_1
                max_lim = 1000000.0
                tol = 0.001
                N_iter = 30
                lambda_track = _bisection_fun(
                    min_lim, max_lim, tol, N_iter, A_track, D_track, b_track, f_track
                )
                y_hat_track = solve(
                    (
                        dot(A_track.T, A_track)
                        + dot(lambda_track, D_track)
                        + dot(1e-06, eye(size(A_track, 1)))
                    ),
                    (dot(A_track.T, b_track) - dot(lambda_track, f_track)),
                )
                x_state = concatenate((x_state, zeros((size(x_state, 0), 1))), axis=1)
                x_state[0:6, qq] = concatenate(
                    (real(y_hat_track[arange(0, size(x_state, 0))])), axis=0
                )
                lk1 = subtract(x_state[0:6, qq], x_state[0:6, qq - 1]).reshape((6, 1))
                lk2 = subtract(x_state[0:6, qq], x_state[0:6, qq - 1]).reshape((6, 1)).T
                P = matmul(lk1, lk2)
                estimated_trajectory.append(x_loc[0:3, qq])
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
    return array(estimated_trajectory)


def _bisection_fun(
    min_lim: float,
    max_lim: float,
    tol: float,
    N_iter: int,
    A: NDArray,
    D: NDArray,
    b: NDArray,
    f: NDArray,
) -> float:
    """
    This function executes the bisection procedure to solve the GTRS problem.

    Args:
        min_lim:
        max_lim:
        tol:
        N_iter:
        A:
        D:
        b:
        f:
    Returns:

    """
    lambda_ = (min_lim + max_lim) / 2
    fun_val = 10**9
    num_iter = 1
    while num_iter <= N_iter and abs(fun_val) > tol and abs(min_lim - max_lim) > 0.0001:
        lambda_ = (min_lim + max_lim) / 2
        fun_val = _fi_fun(lambda_, A, D, b, f)
        if fun_val > 0:
            min_lim = lambda_.copy()
        else:
            max_lim = lambda_.copy()
        num_iter = num_iter + 1
    return lambda_


def _fi_fun(lambda_1: float, A: NDArray, D: NDArray, b: NDArray, f: NDArray) -> NDArray:
    """
    This function computes the fi value.
    """
    g_ = dot(A.T, A)
    gg_ = dot(lambda_1, D)
    ggg_ = dot(1e-06, eye(size(D, 1)))
    t_ = dot(A.T, b)
    ttt_ = dot(lambda_1, f)
    y = solve((g_ + gg_ + ggg_), (t_ - ttt_))
    fi = dot(dot(y.T, D), y) + dot(dot(2, f.T), y)
    return fi


def _calc_eigen(A: NDArray, D: NDArray) -> NDArray:
    """This function computes the Eigen values of the matrices."""
    try:
        left = dot(A.conj().transpose(), A)
        left = fractional_matrix_power(left, 0.5)
        right = dot(A.conj().transpose(), A)
        right = fractional_matrix_power(right, 0.5)
        aux = solve(left, D)
        result = dot(aux, pinv(right))
        return eigvals(result)
    except Exception:
        print("An exception occurred")
        return array([0])
