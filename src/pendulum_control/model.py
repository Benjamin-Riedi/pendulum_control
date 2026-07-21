import numpy as np
import control

class PendulumModel:
    def __init__(self, m_p=0.01, m_d=0.065, l=0.25, b_bottom=True, Ts=0.01):
        self.b_bottom = b_bottom
        self.Ts = Ts
        self.pendulum_params(m_p, m_d, l)
        self.Tx, self.Tu = self.normalization_matrices()

        ### Continuous-time state-space representation (physical) ###
        # c is for continuous-time, p is for physical states

        Ac_p,Bc_p,Cc_p,Dc_p = self.state_space_cont()
        Ac, Bc = self.normalize_matrices(Ac_p, Bc_p, self.Tx, self.Tu)
        self.A, self.B, self.C, self.D = self.discretize(Ac, Bc, Cc_p, Dc_p, self.Ts)

    def pendulum_params(self, m_p, m_d, l):
        self.M = m_p + m_d
        self.cm = 1/self.M*(m_p*l + m_d*l/2)
        self.I = self.calculate_inertia(m_p, m_d, l, self.cm)

    def state_space_cont(self):
        """Return the state-space representation of the pendulum dynamics."""
        a_term = self.M * self.cm * 9.81 / (self.M * self.cm**2 + self.I[0,0])
        b_term = self.M * self.cm / (self.M * self.cm**2 + self.I[0,0])

        A = np.array([[0, 0, 1, 0],
                      [0, 0, 0, 1],
                      [0, 0, 0, 0],
                      [0, a_term, 0, 0]])
        B = np.array([[0],
                      [0],
                      [1],
                      [b_term if self.b_bottom else -b_term]])
        C = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0]])
        D = np.zeros((3, 1))

        return A, B, C, D

    def normalization_matrices(self):
        """Return the normalization matrices Tx and Tu."""
        Tx = np.diag([0.06, np.deg2rad(20), 0.5, 10*np.deg2rad(20)])
        Tu = np.array([[20]])
        return Tx, Tu
    
    def normalize_matrices(self, A, B, Tx, Tu):
        """Normalize the A and B matrices using Tx and Tu."""
        A_normalized = np.linalg.inv(Tx) @ A @ Tx
        B_normalized = np.linalg.inv(Tx) @ B @ Tu
        return A_normalized, B_normalized
    
    def discretize(self, A, B, C, D, Ts):
        """Discretize the continuous-time system matrices A, B, C, and D."""
        sys_c = control.ss(A, B, C, D)
        sys_d = control.c2d(sys_c, Ts)
        return sys_d.A, sys_d.B, sys_d.C, sys_d.D
    
    def dynamics(self, x, u):
        """Compute the dynamics of the pendulum given state x and input u."""
        # think of how to implement with normalization matrices
        

    
    def calculate_inertia(self, m_p, m_d, l, cm):
        x_p = l - cm
        x_d = abs(l/2 - cm)
        I_p = m_p*l**2 *np.array([[1/3, 0, 0],
                        [0, 1/3, 0],
                        [0, 0, 0]])
        I_d = m_d*l**2 * np.array([[1/4, 0, 0],
                        [0, 1/4, 0],
                        [0, 0, 1/2]])
        
        I_p_cm = I_p + m_p * np.array([[x_p**2, 0, 0],
                        [0, x_p**2, 0],
                        [0, 0, 0]])
        I_d_cm = I_d + m_d * np.array([[x_d**2, 0, 0],
                        [0, x_d**2, 0],
                        [0, 0, 0]])
        I = I_p_cm + I_d_cm
        return I