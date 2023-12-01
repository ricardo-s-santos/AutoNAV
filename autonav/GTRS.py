import numpy as np
import scipy.linalg as sp
import math

from autonav.velocity import _velocity


def gtrs(a_i, N: int, K: int, sigma: float, destinations: str, initial_uav_position):
    """
    This function executes the GTRS algorithm
    """
    Ts = 1  # Time sample in seconds
    S = np.eye(6)  # State transition matrix
    S[0, 3] = Ts
    S[1, 4] = Ts
    S[2, 5] = Ts
    sigma_w = 0.05  # State process noise intensity  # State process noise covariance
    Q = np.dot(sigma_w ** 2,
               ([
                   [Ts ** 3 / 3, 0, 0, Ts ** 2 / 2, 0, 0],
                   [0, Ts ** 3 / 3, 0, 0, Ts ** 2 / 2, 0],
                   [0, 0, Ts ** 3 / 3, 0, 0, Ts ** 2 / 2],
                   [Ts ** 2 / 2, 0, 0, Ts, 0, 0],
                   [0, Ts ** 2 / 2, 0, 0, Ts, 0],
                   [0, 0, Ts ** 2 / 2, 0, 0, Ts]
               ]))
    x_state = np.zeros((6, 1))
    x_loc = np.zeros((3, 1))
    P = None
    qq = 0
    x_true = initial_uav_position
    ww = 0
    N_dest = len(destinations) - 1
    while ww <= N_dest:
        RMSE_Goal = []
        distance = math.sqrt((x_true[0] - destinations[ww][0]) ** 2 +
                             (x_true[1] - destinations[ww][1]) ** 2 +
                             (x_true[2] - destinations[ww][2]) ** 2)
        while distance > 1:
            x = x_true[0:3]
            # ---------------------------------------------------------------------
            # Simulation part
            # ---------------------------------------------------------------------
            di_k = np.sqrt(((x[0] - a_i[0, :]) ** 2) + ((x[1] - a_i[1, :]) ** 2) + ((x[2] - a_i[2, :]) ** 2))
            di_k = np.array([di_k]).T
            di_k = di_k + (sigma * np.random.randn(N, K))
            d_i = np.median(di_k, axis=1)
            d_i = np.array([d_i]).T
            # ---------------------------------------------------------------------
            # Estimation part
            # ---------------------------------------------------------------------
            A1_loc = []
            b1_loc = []
            A_track = []
            D_track = []
            f_track = []
            b_track = []
            w_i_loc = np.array(np.sqrt(d_i ** (- 1.0) / (sum(d_i ** (- 1.0)))))
            for tt in np.arange(0, N, 1).reshape(-1):
                A1_loc.append(np.append(np.dot(2, a_i[0:3, tt].T), -1))
                b1_loc.append(np.linalg.norm(a_i[0:3, tt]) ** 2 - d_i[tt] ** 2)
            A1_loc = np.array(A1_loc)
            b1_loc = np.array(b1_loc)
            W_loc = np.diag(w_i_loc.T[0])
            D_loc = np.eye(4)
            D_loc[3, 3] = 0
            f_loc = np.array([0, 0, 0, - 1. / 2.]).reshape(4, 1)
            A_loc = np.dot(W_loc, A1_loc)
            b_loc = np.dot(W_loc, b1_loc)
            if qq != 0:
                P_pred = np.dot(np.dot(S, P), S.T) + Q
                x_pred = np.dot(S, x_state[0:6, qq - 1])
                x_pred = x_pred.reshape(len(x_pred), 1)
                A1_track = []
                b1_track = []
                for tt in np.arange(0, N, 1).reshape(-1):
                    A1_track.append(np.concatenate((np.dot(2, a_i[0:3, tt].T), np.zeros(np.size(x, 0)), [-1]), axis=0))
                    b1_track.append(np.linalg.norm(a_i[0:3, tt]) ** 2 - abs(d_i[tt] ** 2))
                A1_track = np.array(A1_track)
                b1_track = np.array(b1_track)
                left_matrix = sp.sqrtm(np.linalg.inv(P_pred))
                right_matrix = np.zeros((np.size(x_state, 0), 1))
                A1_track = np.concatenate(
                    (A1_track, np.concatenate((left_matrix, right_matrix), axis=1))
                    , axis=0
                )
                A1_track[A1_track == math.inf] = 0
                INF_P_pred = np.array(sp.sqrtm(np.linalg.inv(P_pred)))
                INF_P_pred[INF_P_pred == math.inf] = 0
                b1_track = np.concatenate((b1_track, np.dot(INF_P_pred, x_pred)), axis=0)
                a = np.dot(math.sqrt(1. / 2.), w_i_loc.T)
                b = np.dot(math.sqrt(1. / 8.), np.ones((1, np.size(x_state, 0))))
                W_track = np.concatenate((a, b), axis=1)
                W_track = np.eye(np.size(W_track, 1)) * W_track
                D_track = np.zeros((7, 7))
                D_track[0, 0] = 1
                D_track[1, 1] = 1
                D_track[2, 2] = 1
                f_track = np.array([0, 0, 0, 0, 0, 0, - 1. / 2.]).reshape(7, 1)
                A_track = np.dot(W_track, A1_track)
                b_track = np.dot(W_track, b1_track)
            eigen_values = _calc_eigen(A_loc, D_loc)
            eig_1 = max(eigen_values)
            min_lim = - 1.0 / eig_1
            max_lim = 1000000.0
            tol = 0.001
            N_iter = 30
            lambda_loc = _bisection_fun(min_lim, max_lim, tol, N_iter, A_loc, D_loc, b_loc, f_loc)
            y_hat_loc = np.linalg.solve(
                (np.dot(A_loc.T, A_loc) + np.dot(lambda_loc, D_loc) + np.dot(1e-06, np.eye(np.size(A_loc, 1)))),
                (np.dot(A_loc.T, b_loc) - np.dot(lambda_loc, f_loc)))
            if qq == 0:
                x_loc[0:3, qq] = np.real(y_hat_loc[0:3, 0])
                x_state[0:6, qq] = np.concatenate((x_loc[0:3, qq], [0], [0], [0]), axis=0)
                P = np.eye(6)
            else:
                x_loc = np.insert(x_loc, qq, np.real(y_hat_loc[0:3, 0]), axis=1)
                eigen_values = _calc_eigen(A_track, D_track)
                eig_1 = max(eigen_values)
                min_lim = - 1.0 / eig_1
                max_lim = 1000000.0
                tol = 0.001
                N_iter = 30
                lambda_track = _bisection_fun(min_lim, max_lim, tol, N_iter, A_track, D_track, b_track, f_track)
                y_hat_track = np.linalg.solve(
                    (np.dot(A_track.T, A_track) + np.dot(lambda_track, D_track) + np.dot(1e-06,
                                                                                         np.eye(np.size(A_track, 1)))),
                    (np.dot(A_track.T, b_track) - np.dot(lambda_track, f_track)))
                x_state = np.concatenate((x_state, np.zeros((np.size(x_state, 0), 1))), axis=1)
                x_state[0:6, qq] = np.concatenate((np.real(y_hat_track[np.arange(0, np.size(x_state, 0))])), axis=0)
                lk1 = np.subtract(x_state[0:6, qq], x_state[0:6, qq - 1]).reshape((6, 1))
                lk2 = np.subtract(x_state[0:6, qq], x_state[0:6, qq - 1]).reshape((6, 1)).T
                P = np.matmul(lk1, lk2)
            uav_velocity = _velocity(x_loc[0:3, qq], destinations[ww, :])
            x_true[0] = x_true[0] + uav_velocity[0]
            x_true[1] = x_true[1] + uav_velocity[1]
            x_true[2] = x_true[2] + uav_velocity[2]
            distance = math.sqrt((x_true[0] - destinations[ww][0]) ** 2 +
                                 (x_true[1] - destinations[ww][1]) ** 2 +
                                 (x_true[2] - destinations[ww][2]) ** 2)
            qq += 1
        ww += 1


def _bisection_fun(min_lim, max_lim, tol, N_iter, A, D, b, f):
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
    fun_val = 10 ** 9
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


def _fi_fun(lambda_1, A, D, b, f):
    """
    This function computes the fi value.
    """
    g_ = np.dot(A.T, A)
    gg_ = np.dot(lambda_1, D)
    ggg_ = np.dot(1e-06, np.eye(np.size(D, 1)))
    t_ = np.dot(A.T, b)
    ttt_ = np.dot(lambda_1, f)
    y = np.linalg.solve((g_ + gg_ + ggg_), (t_ - ttt_))
    fi = np.dot(np.dot(y.T, D), y) + np.dot(np.dot(2, f.T), y)
    return fi


def _calc_eigen(A, D):
    """
    This function computes the Eigen values of the matrices.
    """
    try:
        left = (np.dot(A.conj().transpose(), A))
        left = sp.fractional_matrix_power(left, 0.5)
        right = (np.dot(A.conj().transpose(), A))
        right = sp.fractional_matrix_power(right, 0.5)
        aux = np.linalg.solve(left, D)
        result = np.dot(aux, np.linalg.pinv(right))
        return np.linalg.eigvals(result)
    except:
        print("An exception occurred")
        return 0
