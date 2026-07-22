import numpy as np
import control
from .lqr import LQRController

class LQGController:
    def __init__(self, Ad, Bd, Cd, Tx, Ty, Tu, f_dynamic_step, calculate_gains=True):
        self.lqr = LQRController(Ad,Bd,Tx,Tu,calculate_gains)
        Qr, Rr = self.lqr.weight_matrices()
        Qe, Re = self.cov_matrices()
        self.A = Ad
        self.B = Bd
        self.C = Cd
        self.C_p = Ty @ Cd @ np.linalg.inv(Tx)
        self.Tx = Tx
        self.Ty = Ty
        self.Tu = Tu
        self.a_priori_estimate = f_dynamic_step
        if calculate_gains:
            self.K = self.lqr.calculate_K(Qr,Rr)
            self.L = self.calculate_L(Qe, Re)

    def cov_matrices():
        """Return the covariance matrices Qe, Re for a Luenberger Observer"""
        Q = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        R = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])
        return Q, R

    def calculate_L(self, Q, R):
        G = np.eye(4)

        L, P, E = control.dlqe(self.A, G, self.C, Q, R)

        return self.Tx @ L @ np.linalg.inv(self.Ty)

    def a_posteriori_estimate(self, x, y):
        """This is the state estimate after incorporating the latest measurement y_k."""
        np.asarray(x + self.L @ (y - self.C_p @ x))

    def update_K(self, req):
        pass

    def update_L(self, req):
        pass