import numpy as np
import control

class LQRController:
    def __init__(self, Ad, Bd, Tx, Tu, calculate_gains=True):
        self.b_calculate_gains = calculate_gains
        Q, R = self.weight_matrices()
        if self.b_calculate_gains:
            self.K = self.calculate_K(Ad, Bd, Tx, Tu, Q, R)

    def weight_matrices(self):
        """Return the weight matrices Q and R."""
        Q = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        R = np.array([[1]])
        return Q, R
    
    def calculate_K(self, Ad, Bd, Tx, Tu, Q, R):
        """Calculate the optimal state feedback gain K using the LQR method.
           The returned K is for physical states"""
        K, P, E = control.dlqr(Ad, Bd, Q, R)

        return Tu @ K @ np.linalg.inv(Tx)
    
    def step(self, x):
        """Compute the control input u based on the current state x."""
        u = -self.K @ x
        return u
    
# support for direct K would require top/bottom distinction
