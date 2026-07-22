import numpy as np
import control

from pendulum_control.srv import SetGainsResponse

class LQRController:
    def __init__(self, Ad, Bd, Tx, Tu, calculate_gains=True):
        b_calculate_gains = calculate_gains
        Q, R = self.weight_matrices()
        self.A = Ad
        self.B = Bd
        self.Tx = Tx
        self.Tu = Tu
        if b_calculate_gains:
            self.K = self.calculate_K(Q, R)

    def weight_matrices(self):
        """Return the weight matrices Q and R."""
        Q = np.array([[1000, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        R = np.array([[1000]])
        return Q, R
    
    def calculate_K(self, Q, R):
        """Calculate the optimal state feedback gain K using the LQR method.
           The returned K is for physical states"""
        K, P, E = control.dlqr(self.A, self.B, Q, R)

        return self.Tu @ K @ np.linalg.inv(self.Tx)
    
    def step(self, x):
        """Compute the control input u based on the current state x."""
        u = -self.K @ x
        return u
    
    def update_gains(self, req):
        """Update the LQR gains K based on new system matrices and weight matrices."""
        Q = np.diag(req.Q)
        R = np.diag(req.R)
        self.K = self.calculate_K(Q, R)
        print("Updated K:")
        print(self.K)
        return SetGainsResponse(success=True)

# support for direct K would require top/bottom distinction
    
class Integrator:
    def __init__(self):
        self.v_prev = 0.0

    def integrate(self, u, dt):
        """Integrate the input u over time to get the new value of v"""
        integral = self.v_prev + u * dt
        self.v_prev = integral
        return integral
    
