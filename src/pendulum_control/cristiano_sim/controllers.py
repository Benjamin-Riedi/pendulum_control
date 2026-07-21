from scipy.signal import cont2discrete
from scipy.linalg import solve_discrete_are
import numpy as np

class ProportionalController:

    def __init__(self, dt):
        self._Kp = 50
        self._Ki = 20
        self._dt = dt

    @property
    def Kp(self):
        return self._Kp

    def compute(self, current_err, prev_err, prev_u):
        u = prev_u + self._Kp * (current_err - prev_err) + self._Ki * self._dt * current_err
        return u

class LQR:

    def __init__(self, plant, axis):
        M = plant.M
        m = plant.m
        L = plant.L
        d = plant.d
        dt = plant.dt
        g = 9.81

        A = np.array([[0, 0, 1, 0],
                      [0, 0, 0, 1],
                      [0, 0, 0, 0],
                      [0, g*2/L, 0, -4*d/(M*L**2)]])
        if axis == 'x':
            B = np.array([[0], [0], [1], [-2/L]])
        else:
            B = np.array([[0], [0], [1], [2/L]])
        C = np.eye(4)

        # Normalization  (it assumes max speed 5 time max 
        z_max = 10e-3
        z_dot_max = 5 * z_max
        
        angle_max = np.deg2rad(30)
        angle_dot_max = 5 * angle_max
        
        Tzx = np.diag([z_max, angle_max, z_dot_max, angle_dot_max])
        Tzu = np.diag([z_dot_max])
        Tzy = np.diag([z_max, angle_max, z_dot_max, angle_dot_max])
        
        Az_norm = np.linalg.inv(Tzx) @ A @ Tzx
        Bz_norm = np.linalg.inv(Tzx) @ B @ Tzu
        Cz_norm = np.linalg.inv(Tzy) @ C @ Tzx

        Ad, Bd, _, _, _ = cont2discrete((Az_norm,Bz_norm,Cz_norm,0), dt, method='zoh')

        self._Ad = Ad
        self._Bd = Bd


        Q = np.diag([1000,1,1,1])
        R = 50

        P = solve_discrete_are(Ad, Bd, Q, R)
        K_norm = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)

        # Denormalize
        self._K = np.asarray(Tzu @ K_norm @ np.linalg.inv(Tzx))

    def compute(self, err):
        return float(self._K @ err)
    


    
