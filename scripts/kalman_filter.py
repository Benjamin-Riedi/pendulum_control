import rospy
import rospkg
import os
import numpy as np
import control

from pendulum_control.common import * # this imports the common messages
from pendulum_control import pubArray, subArray

class LQGNode:
    def __init__(self):
        rospy.init_node(name='lqg', anonymous=True)
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('pendulum_control')
        self.read_params()
        self.read_matrices()
        self.init_publishers()
        self.init_variables()


    def read_params(self):
        """Read parameters from launch file"""
        self.b_calculate_gains = rospy.get_param("~calculate_gains", False)  # if true, provide A,B,Q,R to solve ARE and get K, else provide K
        self.matrices_param = rospy.get_param('~matrices_path')
        self.B_file_path = rospy.get_param("~B_path")
        self.input_topic = rospy.get_param('~input_topic')
        self.pub_v_topic = rospy.get_param('~pub_v_topic')
        self.pub_u_topic = rospy.get_param('~pub_u_topic')
        self.set_state_topic = rospy.get_param('~set_state_topic')
        self.Ts = 0.01

        if os.path.isabs(self.matrices_param):
            self.matrices_path = self.matrices_param
        else:
            self.matrices_path = os.path.join(self.package_path, self.matrices_param)


    def read_matrices(self):
        """Read matrices from CSV files"""
        self.A = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Ac.csv", delimiter=","))
        self.B = np.atleast_2d(np.genfromtxt(self.matrices_path + self.B_file_path, delimiter=",")).reshape(-1,1)
        self.C = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Cc.csv", delimiter=","))

        self.Qr = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Qr.csv", delimiter=","))
        self.Rr = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Rr.csv", delimiter=","))

        self.Qe = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Qe.csv", delimiter=","))
        self.Re = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Re.csv", delimiter=","))

        self.discretize()

        if not self.b_calculate_gains:
            self.K = np.atleast_2d(np.genfromtxt(self.matrices_path + "/K.csv", delimiter=","))
            self.L = np.atleast_2d(np.genfromtxt(self.matrices_path + "/L.csv", delimiter=","))
        else:
            self.K = self.calculate_K()
            self.L = self.calculate_L()

    def init_publishers(self):
        self.pub_v = rospy.Publisher(self.pub_v_topic, ScalarStamped, queue_size=1)
        self.pub_u = rospy.Publisher(self.pub_u_topic, ScalarStamped, queue_size=1)

        self.v_sp_msg = ScalarStamped()
        # add state estimation error

    def init_variables(self):
        self.x = np.zeros((self.A.shape[0], 1))  # state estimate
        self.y = np.zeros((self.Cd.shape[0], 1))  # measurement
        self.u = 0.0  # control input
        self.time = rospy.Time.now()
    
    def discretize(self):
        """Converts continuous-time system matrices to discrete-time"""
        # no input feedthrough
        D = np.zeros((3,1))

        sys_c = control.ss(self.A,self.B,self.C,D)
        sys_d = control.c2d(sys_c, self.Ts)

        self.Ad = sys_d.A
        self.Bd = sys_d.B
        self.Cd = sys_d.C

    def calculate_K(self):
        """If self.b_calculate_gains is True, calculate the optimal state feedback gain K using the discrete-time LQR method."""
        # here i'd add the normalization (Tx, Tu) but i'll leave it out for now
        # if normalize: ...

        K, P, E = control.dlqr(self.Ad, self.Bd, self.Qr, self.Rr)

        # maybe add support to write K to file, but for now i'll just print it
        print("Calculated K:")
        print(K)
        return K

    def calculate_L(self):
        """If self.b_calculate_gains is True, calculate the optimal observer gain L using the discrete-time LQE method."""
        G = np.eye(4)

        # here i'd add the normalization (Tx, Tu) but i'll leave it out for now
        # if normalize: ...

        L, P, E = control.dlqe(self.Ad, G, self.Cd, self.Qe, self.Re)

        # maybe add support to write L to file, but for now i'll just print it
        print("Calculated L:")
        print(L)
        return L

    def set_state_callback(self, msg):
        """For testing purposes, set the state estimate x to a specific value."""
        self.x = subArray(msg)
    
    def callback(self, msg):
        """
        msg is measurement y_k (x,xD,phi), we use this to estimate the a posteriori state x_k. This goes into the lqr and gets u_k.
        u_k goes into the system and also calculates the a priori state x_k+1. The a priori estimate x_k+1 is updated with the next measurement y_k+1.
        """
        self.y = subArray(msg)
        self.time = msg.header.stamp
        self.x = self.a_posteriori_estimate()
        self.u = self.state_feedback_step()

        u_msg = ScalarStamped()
        u_msg.scalar = self.u.item() # convert 1x1 array to float
        u_msg.header.stamp = self.time
        self.pub_u.publish(u_msg)

        self.v_sp_msg.scalar = self.integrate()
        self.pub_v.publish(self.v_sp_msg)

        self.v_prev.scalar = self.v_sp_msg.scalar
        self.v_prev.header.stamp = self.time

        # self.u_prev.scalar = self.u # cast to float?
        # self.u_prev.header.stamp = self.time
        #
        self.x = self.a_priori_estimate()
    
    def a_posteriori_estimate(self):
        """This is the state estimate after incorporating the latest measurement y_k."""
        return np.asarray(self.x + self.L @ (self.y - self.Cd @ self.x))
    
    def a_priori_estimate(self):
        """This is the state estimate considering only the system dynamics and the previous control input u_k."""
        return np.asarray(self.Ad @ self.x + self.Bd @ self.u)
    
    def state_feedback_step(self):
        """Calculate the control input u_k based on the current state estimate x_k."""
        return np.asarray(-self.K @ self.x)
    
    # def integrate(self):
    #     """Integrate the control input u_k over time to get the desired velocity setpoint v_sp. First-order integration is used."""
    #     if not hasattr(self, 'u_prev'):
    #         self.u_prev = ScalarStamped(scalar=0.0)
    #         self.u_prev.header.stamp = self.time - rospy.Duration(0,10000000) # maybe use variable self.Ts
    #     dt = (self.time - self.u_prev.header.stamp).to_sec()
    #     return 0.5 * dt * (self.u + self.u_prev.scalar)
    
    def integrate(self):
        """Integrate the control input u_k over time to get the desired velocity setpoint v_sp. First-order integration is used."""
        if not hasattr(self, 'v_prev'):
            self.v_prev = ScalarStamped(scalar=0.0)
            self.v_prev.header.stamp = self.time - rospy.Duration(0,10000000) # maybe use variable self.Ts
        dt = (self.time - self.v_prev.header.stamp).to_sec()
        print("dt:", dt, "u:", self.u, "v_prev:", self.v_prev.scalar)
        return self.u * dt + self.v_prev.scalar
    
    def run(self):
        rospy.Subscriber(self.input_topic, ArrayStamped, self.callback)
        rospy.Subscriber(self.set_state_topic, ArrayStamped, self.set_state_callback)
        rospy.spin()

if __name__ == "__main__":
    node = LQGNode()
    node.run()