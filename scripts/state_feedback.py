import rospy
import rospkg
import os
import numpy as np
import control

from pendulum_control.common import * # this imports the common messages
from pendulum_control import pubArray, subArray, integrate, integrate_trapezoidal
from pendulum_control import calculate_inertia, M, cm

class StateFeedbackNode:
    def __init__(self):
        rospy.init_node(name='state_feedback', anonymous=True)
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('pendulum_control')
        self.read_params()
        self.init_variables()
        self.read_matrices()
        self.init_topics()
        self.init_publishers()

    def read_params(self):
        self.b_calculate_K = rospy.get_param("~calculate_gains", False)  # if true, provide A,B,Q,R to solve ARE and get K, else provide K
        self.matrices_rel = rospy.get_param('/matrices_path')
        self.B_file_path = rospy.get_param("B_matrix")
        self.K_file_path = rospy.get_param("K_matrix")
        self.Tx_file_path = rospy.get_param("/Tx_matrix")
        self.Tu_file_path = rospy.get_param("/Tu_matrix")
        self.Ts = rospy.get_param('/Ts')

        if os.path.isabs(self.matrices_rel):
            self.matrices_path = self.matrices_rel
        else:
            self.matrices_path = os.path.join(self.package_path, self.matrices_rel)

    def read_matrices(self):
        self.Tx = np.atleast_2d(np.genfromtxt(self.matrices_path + self.Tx_file_path, delimiter=","))
        self.Tu = np.atleast_2d(np.genfromtxt(self.matrices_path + self.Tu_file_path, delimiter=","))
        self.A = np.atleast_2d(np.genfromtxt(self.matrices_path + "Ac.csv", delimiter=","))
        self.B = np.atleast_2d(np.genfromtxt(self.matrices_path + self.B_file_path, delimiter=",")).reshape(-1,1)

        self.A[3,1] = M * cm * 9.81 / (M * cm**2 + self.I[0,0])
        self.B[3,0] *= M * cm / (M * cm**2 + self.I[0,0])

        # normalize A and B matrices
        self.A = np.linalg.inv(self.Tx) @ self.A @ self.Tx
        self.B = np.linalg.inv(self.Tx) @ self.B @ self.Tu

        self.Q = np.atleast_2d(np.genfromtxt(self.matrices_path + "Qr.csv", delimiter=","))
        self.R = np.atleast_2d(np.genfromtxt(self.matrices_path + "Rr.csv", delimiter=","))

        if not self.b_calculate_K:
            self.K = np.atleast_2d(np.genfromtxt(self.matrices_path + self.K_file_path, delimiter=","))
        else:
            self.K = self.calculate_K()
        print('K matrix:')
        print(self.K)
        self.K_full = self.K

    def init_topics(self):
        self.state_topic = 'state'
        self.v_topic = 'v_sp'
        self.u_topic = 'u'
        
    def init_publishers(self):
        self.v_pub = rospy.Publisher(self.v_topic, ScalarStamped, queue_size=1)
        self.u_pub = rospy.Publisher(self.u_topic, ScalarStamped, queue_size=1)
        self.v_sp_msg = ScalarStamped()
        self.u_msg = ScalarStamped()
        # publish u for introspection?

    def init_variables(self):
        self.u = 0.0
        self.x = np.zeros((4, 1))  # state vector
        self.time = rospy.Time.now()
        self.v_prev = 0.0
        self.I = calculate_inertia()
        self.ramp_counter = 0
    
    def calculate_K(self):
        # i don't care about output
        C = np.zeros((3,4))
        D = np.zeros((3,1))

        sys_c = control.ss(self.A,self.B,C,D)
        sys_d = control.c2d(sys_c, self.Ts)

        Ad = sys_d.A
        Bd = sys_d.B

        # here i'd add the normalization (Tx, Tu) but i'll leave it out for now
        # if normalize: ...

        K, P, E = control.dlqr(Ad, Bd, self.Q, self.R)

        # maybe add support to write K to file, but for now i'll just print it

        return self.Tu @ K @ np.linalg.inv(self.Tx)
    
    def callback(self, msg):
        while self.ramp_counter < 400:
            self.K = self.K_full * (self.ramp_counter / 400.0)
            self.ramp_counter += 1
        self.time = msg.header.stamp

        self.x = subArray(msg)
        self.u = -self.K @ self.x

        self.v_sp_msg.scalar = integrate(self.v_prev, self.u, self.Ts)
        self.v_pub.publish(self.v_sp_msg)
        self.v_prev = self.v_sp_msg.scalar
        
        self.u_prev = self.u
        self.u_msg.scalar = self.u
        self.u_pub.publish(self.u_msg)
    
    def run(self):
        rospy.Subscriber(self.state_topic, ArrayStamped, self.callback)
        rospy.spin()

if __name__ == "__main__":
    node = StateFeedbackNode()
    node.run()