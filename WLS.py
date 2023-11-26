import numpy as np
import math
import cmath
import itertools
from numpy import dot
from velocity import velocity


def wls(a_i, N, K, sigma, destinations, initial_uav_position):
    """
    This function executes the WLS algorithm
    """
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
            di_k = di_k + ((sigma + 1) * np.random.randn(N, K))
            d_i = np.median(di_k, axis=1)
            d_i = np.array([d_i]).T
            # ---------------------------------------------------------------------
            # Estimation part
            # ---------------------------------------------------------------------
            xi_est = []
            phi_i = []
            alpha_i = []
            for ii in range(0, N):
                A2 = []
                b2 = []
                kk = [ii + 1]
                for jj in range(0, N):
                    total = 0
                    if a_i[0, ii] == a_i[0, jj]:
                        total += 1
                    if a_i[1, ii] == a_i[1, jj]:
                        total += 1
                    if ii != jj and total > 0:
                        kk.append(jj + 1)
                for uu in range(0, len(np.array(list(itertools.combinations(kk, 2))))):
                    combinations = np.array(list(itertools.combinations(kk, 2)))
                    gg = combinations[uu, 0]
                    hh = combinations[uu, 1]
                    A2.append(2 * (a_i[0:3, gg - 1] - a_i[0:3, hh - 1]).T)
                    b2.append(
                        d_i[hh - 1] ** 2 - d_i[gg - 1] ** 2 - np.linalg.norm(a_i[0:3, hh - 1]) ** 2 + np.linalg.norm(
                            a_i[0:3, gg - 1]) ** 2)
                A2 = np.asarray(A2, dtype=np.float32)
                b2 = np.asarray(b2, dtype=np.float32)
                xi_est.append(np.linalg.solve(dot(A2.T, A2) + (1 * 10 ** (-6)) * np.eye(3), dot(A2.T, b2)))
                di_xy = np.linalg.norm(xi_est[0][0:2])
                xi_est[ii][2] = cmath.sqrt((d_i[0] ** 2) - (di_xy ** 2)).real + cmath.sqrt(
                    (d_i[0] ** 2) - (di_xy ** 2)).imag
                phi_i.append(math.atan2(xi_est[ii][1] - a_i[1, ii], xi_est[ii][0] - a_i[0, ii]) * 180 / math.pi)
                alpha_i.append(math.acos((xi_est[ii][2] - a_i[2, ii]) / (
                    np.linalg.norm(xi_est[:][ii] - a_i[:, ii].reshape(len(a_i), 1)))) * 180 / math.pi)
            phi_i = np.asarray(phi_i, dtype=np.float32)
            alpha_i = np.asarray(alpha_i, dtype=np.float32)
            u_i_1 = np.cos(phi_i * math.pi / 180).T
            u_i_2 = np.sin(alpha_i * math.pi / 180).T
            u_i_3 = np.sin(phi_i * math.pi / 180).T
            u_i_4 = np.cos(alpha_i * math.pi / 180).T
            u_i = np.array([[u_i_1 * u_i_2], [u_i_3 * u_i_2], [u_i_4]], dtype=np.float32).reshape(3, N)
            A = np.asarray(u_i.T, dtype=np.float32)
            b = d_i + sum(u_i * a_i).T.reshape(N, 1)
            w_i = np.asarray((1 / d_i) / (sum(1 / d_i)))
            W = np.asarray(np.eye(N) * np.lib.scimath.sqrt(w_i))
            x_est = np.asarray(
                np.linalg.solve(np.dot(np.dot(A.T, W.T), np.dot(W, A)), np.dot(np.dot(A.T, W.T), np.dot(W, b))).real)
            x_est = x_est[:, 0]
            uav_velocity = velocity(x_est, destinations[ww, :])
            x_true[0] = x_true[0] + uav_velocity[0]
            x_true[1] = x_true[1] + uav_velocity[1]
            x_true[2] = x_true[2] + uav_velocity[2]
            distance = math.sqrt((x_true[0] - destinations[ww][0]) ** 2 +
                                 (x_true[1] - destinations[ww][1]) ** 2 +
                                 (x_true[2] - destinations[ww][2]) ** 2)
        ww += 1
