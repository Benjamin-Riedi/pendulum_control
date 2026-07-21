import rospy
import os
import rospkg
import numpy as np
import control
from calc_inertia import calculate_inertia, M, cm

rospack = rospkg.RosPack()
package_path = rospack.get_path('pendulum_control')
matrices_rel = 'data/matrices/'
B_file_path = 'Bct.csv'
K_file_path = 'Kt.csv'
Tx_file_path = 'Tx.csv'
Tu_file_path = 'Tu.csv'
Ts = 0.01

if os.path.isabs(matrices_rel):
    matrices_path = matrices_rel
else:
    matrices_path = os.path.join(package_path, matrices_rel)
    
Tx = np.atleast_2d(np.genfromtxt(matrices_path + Tx_file_path, delimiter=","))
Tu = np.atleast_2d(np.genfromtxt(matrices_path + Tu_file_path, delimiter=","))
A = np.atleast_2d(np.genfromtxt(matrices_path + "Ac.csv", delimiter=","))
B = np.atleast_2d(np.genfromtxt(matrices_path + B_file_path, delimiter=",")).reshape(-1,1)

I = calculate_inertia()

A[3,1] = M * cm * 9.81 / (M * cm**2 + I[0,0])
B[3,0] *= M * cm / (M * cm**2 + I[0,0])

# normalize A and B matrices
A = np.linalg.inv(Tx) @ A @ Tx
B = np.linalg.inv(Tx) @ B @ Tu

Q = np.atleast_2d(np.genfromtxt(matrices_path + "Qr.csv", delimiter=","))
R = np.atleast_2d(np.genfromtxt(matrices_path + "Rr.csv", delimiter=","))

C = np.zeros((3,4))
D = np.zeros((3,1))

sys_c = control.ss(A,B,C,D)
sys_d = control.c2d(sys_c, Ts)

Ad = sys_d.A
Bd = sys_d.B

# here i'd add the normalization (Tx, Tu) but i'll leave it out for now
# if normalize: ...

K, P, E = control.dlqr(Ad, Bd, Q, R)

# maybe add support to write K to file, but for now i'll just print it
print("Calculated K:")
print(K)
print(Tu @ K @ np.linalg.inv(Tx))