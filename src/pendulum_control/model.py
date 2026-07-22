import numpy as np
import control
# from .lqr import LQRController, Integrator

class PendulumModel:
    def __init__(self, m_p=0.01, m_d=0.065, l=0.25, b_bottom=True, Ts=0.01):
        self.b_bottom = b_bottom
        self.name = 'bottom' if b_bottom else 'top'
        self.Ts = Ts
        self.pendulum_params(m_p, m_d, l)
        self.Tx, self.Tu = self.normalization_matrices(x=0.01, phi=30, dx=0.05, dphi=30, u=0.05) # normalization like cristiano
        self.Tx, self.Tu = self.normalization_matrices(x=0.06, phi=20, dx=0.5, dphi=20, u=20)

        # Matrix naming: c is for continuous-time, p is for physical states

        Ac_p,Bc_p,Cc_p,Dc_p = self.state_space_cont()
        self.A_p, self.B_p, C_p, D_p = self.discretize(Ac_p, Bc_p, Cc_p, Dc_p, self.Ts)
        self.A, self.B = self.normalize_matrices(self.A_p, self.B_p, self.Tx, self.Tu)

        # self.controller = LQRController(self.A, self.B, self.Tx, self.Tu, calculate_gains=calculate_gains)

        # print(f"Pendulum model initialized for {self.name} pendulum with parameters:")
        # print(f'K {self.name}: {self.controller.K}')
        # print(f'A: {self.A}, B: {self.B}')

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
                      [-b_term if self.b_bottom else b_term]])
        C = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0]])
        D = np.zeros((3, 1))

        return A, B, C, D

    def normalization_matrices(self, x=0.01, phi=30, dx=0.05, dphi=30, u=0.05):
        """Return the normalization matrices Tx and Tu."""
        Tx = np.diag([x, np.deg2rad(phi), dx, 5*np.deg2rad(dphi)])
        Tu = np.array([[u]])
        return Tx, Tu
    
    def normalize_matrices(self, A_p, B_p, Tx, Tu):
        """Normalize the A and B matrices using Tx and Tu."""
        A = np.linalg.inv(Tx) @ A_p @ Tx
        B = np.linalg.inv(Tx) @ B_p @ Tu
        return A, B

    def discretize(self, Ac, Bc, Cc, Dc, Ts):
        """Discretize the continuous-time system matrices A, B, C, and D."""
        sys_c = control.ss(Ac, Bc, Cc, Dc)
        sys_d = control.c2d(sys_c, Ts)
        return sys_d.A, sys_d.B, sys_d.C, sys_d.D
    
    def dynamic_step(self, x, u):
        """Compute the dynamics of the pendulum given state x and input u."""
        return self.A_p @ x + self.B_p @ u
        
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