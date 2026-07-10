import rospy
import rospkg
import os
import numpy as np
import control

from pendulum_control.common import * # this imports the common messages
from pendulum_control import pubArray

class DynamicsSimulatorNode:
    def __init__(self):
        rospy.init_node('dynamics_simulator', anonymous=True)
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('pendulum_control')
        self.read_params()
        self.read_matrices()
        self.pub_state = rospy.Publisher('/bottom/y', ArrayStamped, queue_size=1)
        self.x = np.zeros((self.A.shape[0], 1))  # state vector

        # print(self.A, self.B, self.C)


    def read_params(self):
        """Read parameters from launch file"""
        self.matrices_param = rospy.get_param('~matrices_path')

        if os.path.isabs(self.matrices_param):
            self.matrices_path = self.matrices_param
        else:
            self.matrices_path = os.path.join(self.package_path, self.matrices_param)

        self.Ts = rospy.get_param('~Ts', 0.01)  # Sampling time
    
    def read_matrices(self):
        """Read matrices from CSV files"""
        self.A = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Ac.csv", delimiter=","))
        self.B = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Bcb.csv", delimiter=",")).reshape(-1,1)
        self.C = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Cc.csv", delimiter=","))

        self.Qr = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Qr.csv", delimiter=","))
        self.Rr = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Rr.csv", delimiter=","))

        self.Qe = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Qe.csv", delimiter=","))
        self.Re = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Re.csv", delimiter=","))

        self.discretize()

    def discretize(self):
        """Converts continuous-time system matrices to discrete-time"""
        # no input feedthrough
        D = np.zeros((3,1))

        sys_c = control.ss(self.A,self.B,self.C,D)
        sys_d = control.c2d(sys_c, self.Ts)

        self.Ad = sys_d.A
        self.Bd = sys_d.B
        self.Cd = sys_d.C

    def input_callback(self, msg):
        """Callback function to receive control input"""
        self.u = np.array(msg.scalar).reshape(1,1)
        # print("Input:", self.u)
        # print("State:", self.x)
        self.output = self.dynamics_step()
        # wait to simulate sensor delay
        rospy.sleep(self.Ts)
        pubArray(self.pub_state, self.output, rospy.Time.now())
        # self.pub_state.publish(ArrayStamped(vector=self.output))


    def dynamics_step(self):
        """Perform one step of the dynamics simulation"""
        self.x = self.A @ self.x + self.B @ self.u
        y = self.C @ self.x
        return y
        

    def run(self):
        """Main loop to simulate dynamics"""
        rospy.Subscriber('/bottom/u', ScalarStamped, self.input_callback)
        rospy.spin()

if __name__ == '__main__':
    node = DynamicsSimulatorNode()
    node.run()
