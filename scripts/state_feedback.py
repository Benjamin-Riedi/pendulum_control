import rospy
import numpy as np
import control

from pendulum_control.common import * # this imports the common messages
from pendulum_control import pubArray, subArray

class StateFeedbackNode:
    def __init__(self):
        rospy.init_node(name='state_feedback', anonymous=True)
        self.read_params()
        self.read_matrices()
        self.init_publishers()
        self.init_variables()

    def read_params(self):
        self.b_calculate_K = rospy.get_param("~calculate_K", False)  # if true, provide A,B,Q,R to solve ARE and get K, else provide K
        self.matrices_path = rospy.get_param('~matrices_path')
        self.B_file_path = rospy.get_param("~B_path")
        self.state_topic = rospy.get_param('~subsystem_topic')
        self.pub_topic = rospy.get_param('~pub_topic')


    def read_matrices(self):
        self.A = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Ac.csv", delimiter=","))
        self.B = np.atleast_2d(np.genfromtxt(self.matrices_path + self.B_file_path, delimiter=","))

        self.Q = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Q.csv", delimiter=","))
        self.R = np.atleast_2d(np.genfromtxt(self.matrices_path + "/R.csv", delimiter=","))

        if not self.b_calculate_K:
            self.K = np.atleast_2d(np.genfromtxt(self.matrices_path + "/K.csv", delimiter=","))
        else:
            self.K = self.calculate_K()
        
    def init_publishers(self):
        self.pub_v = rospy.Publisher(self.pub_topic, ScalarStamped, queue_size=1)
        self.v_sp_msg = ScalarStamped()
        # publish u for introspection?

    def init_variables(self):
        self.u = 0.0
        self.time = 0
    
    def calculate_K(self):
        # i don't care about output
        C = np.zeros((1,8))
        D = np.zeros((1,2))

        sys_c = control.ss(self.A,self.B,C,D)
        sys_d = control.c2d(sys_c, self.Ts)

        Ad = sys_d.A
        Bd = sys_d.B

        # here i'd add the normalization (Tx, Tu) but i'll leave it out for now
        # if normalize: ...

        K, P, E = control.dlqr(Ad, Bd, self.Q, self.R)

        # maybe add support to write K to file, but for now i'll just print it
        print("Calculated K:")
        print(K)

        return K
    
    def callback(self, msg):
        self.time = msg.header.stamp
        # msg.vector.reshape(-1,1)
        self.u = -self.K @ msg.vector

        self.v_sp_msg.scalar = self.integrate()
        self.pub_v.publish(self.v_sp_msg)

        self.u_prev.scalar = self.u
        self.u_prev.header.stamp = self.time

    def integrate(self):
        if not hasattr(self, 'u_prev'):
            self.u_prev = ScalarStamped(scalar=0.0)
            self.u_prev.header.stamp = self.time - 0.01
        dt = (self.time - self.u_prev.header.stamp).to_sec()
        return 0.5 * dt * (self.u + self.u_prev.scalar)
    
    def run(self):
        rospy.Subscriber(self.state_topic, ArrayStamped, self.callback)
        rospy.spin()

if __name__ == "__main__":
    node = StateFeedbackNode()
    node.run()