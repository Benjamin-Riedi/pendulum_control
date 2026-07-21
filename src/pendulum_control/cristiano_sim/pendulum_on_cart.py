import numpy as np 


class Plant:

    def __init__(self, z0_x, z0_y, Nsim, dt, pend_mass, cart_mass, pend_length, damping) -> None:

        self._dt = dt
        self._N = Nsim
        self._z0_x = z0_x
        self._z0_y = z0_y

        # Model Parameters
        self._M = pend_mass
        self._m = cart_mass
        self._L = pend_length
        self._d = damping

        # ZX plane 
        self.z_x = np.zeros((4, self._N + 1))
        self.u_x = np.zeros((self._N))

        # ZY plane
        self.z_y = np.zeros((4, self._N + 1))
        self.u_y = np.zeros((self._N))

        # Dinamics Inizialitation
        self.z_x[:, 0] = self._z0_x
        self.z_y[:, 0] = self._z0_y

    @property
    def dt(self):
        return self._dt
    @property
    def N(self):
        return self._N

    @property
    def z0_x(self):
        return self._z0_x

    @property
    def z0_y(self):
        return self._z0_y

    @property
    def M(self):
        return self._M

    @property
    def m(self):
        return self._m

    @property
    def L(self):
        return self._L

    @property
    def d(self):
        return self._d

    def _continuos_time_state_zx(self, z_x, u_x):
        M = self._M
        m = self._m
        L = self._L
        d = self._d
        g = 9.81

        # ZX plane state dinamics
        phi = z_x[1]
        x_dot = z_x[2]
        phi_dot = z_x[3]
        '''
        Mx = np.array([[m + M, 
                        0.5*M*L*np.cos(phi)],
                       [0.5*M*L*np.cos(phi),
                        0.25*M*L**2]])
        '''
        Mx = np.array([[m + M, 
                        0],
                       [0.5*M*L*np.cos(phi),
                        0.25*M*L**2]])
        '''
        rx = np.array([0.5*M*L*(phi_dot**2)*np.sin(phi) + u_x,
                       M*L*x_dot*phi_dot*np.sin(phi) + 0.5*g*M*L*np.sin(phi) - d*phi_dot])
        '''
        rx = np.array([u_x,
                       M*L*x_dot*phi_dot*np.sin(phi) + 0.5*g*M*L*np.sin(phi) - d*phi_dot])
        
        z_dot_x = np.zeros((4,))
        z_dot_x[0:2] = z_x[2:4]
        z_dot_x[2:4] = np.linalg.solve(Mx,rx)

        return z_dot_x

    def _continuos_time_state_zy(self, z_y, u_y):
        M = self._M
        m = self._m
        L = self._L
        d = self._d
        g = 9.81

        
        alpha = z_y[1]
        y_dot = z_y[2]
        alpha_dot = z_y[3]
        '''
        My = np.array([[m + M, 
                        -0.5*M*L*np.cos(alpha)],
                       [-0.5*M*L*np.cos(alpha),
                        0.25*M*L**2]])
        '''
        My = np.array([[m + M, 
                        0],
                       [-0.5*M*L*np.cos(alpha),
                        0.25*M*L**2]])
        '''
        ry = np.array([-0.5*M*L*(alpha_dot**2)*np.sin(alpha) + u_y,
                       -M*L*y_dot*alpha_dot*np.sin(alpha) + 0.5*g*M*L*np.sin(alpha) - d*alpha_dot])
        '''
        ry = np.array([ u_y,
                       -M*L*y_dot*alpha_dot*np.sin(alpha) + 0.5*g*M*L*np.sin(alpha) - d*alpha_dot])

        z_dot_y = np.zeros((4,))
        z_dot_y[0:2] = z_y[2:4]
        z_dot_y[2:4] = np.linalg.solve(My,ry)

        return z_dot_y


    def update(self, u_x, u_y, t):
        z_x_k = self.z_x[:,t]
        z_y_k = self.z_y[:,t]
        next_state_x = self._rk4_update('x', z_x_k, u_x, self._dt)
        next_state_y = self._rk4_update('y', z_y_k, u_y, self._dt)
        self.z_x[:, t + 1] = next_state_x
        self.u_x[t] = u_x
        self.z_y[:, t + 1] = next_state_y
        self.u_y[t] = u_y

        return next_state_x, next_state_y


    def _rk4_update(self, axis, z_k, u_k, dt):
        """Runge-Kutta 4th order."""
        if axis == 'x':
            k1 = self._continuos_time_state_zx(z_k, u_k)
            k2 = self._continuos_time_state_zx(z_k + 0.5 * dt * k1, u_k)
            k3 = self._continuos_time_state_zx(z_k + 0.5 * dt * k2, u_k)
            k4 = self._continuos_time_state_zx(z_k + dt * k3, u_k)

            return z_k + (dt / 6) * k1 + (dt / 3) * k2 + (dt / 3) * k3 + (dt / 6) * k4

        if axis == 'y':
            k1 = self._continuos_time_state_zy(z_k, u_k)
            k2 = self._continuos_time_state_zy(z_k + 0.5 * dt * k1, u_k)
            k3 = self._continuos_time_state_zy(z_k + 0.5 * dt * k2, u_k)
            k4 = self._continuos_time_state_zy(z_k + dt * k3, u_k)

            return z_k + (dt / 6) * k1 + (dt / 3) * k2 + (dt / 3) * k3 + (dt / 6) * k4

        else:
            raise ValueError("The axis value can only be either 'x' or 'y'")



