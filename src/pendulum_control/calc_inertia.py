import numpy as np
import rospy

m_p = 0.01
m_d = 0.065
M = m_p + m_d
l = 0.25
cm = 1/M*(m_p*l + m_d*l/2)

def calculate_inertia():
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