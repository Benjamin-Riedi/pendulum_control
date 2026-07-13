import rospy
import rospkg
import os
import numpy as np
import control

from pendulum_control.common import * # this imports the common messages
from pendulum_control import pubArray, subArray

class DynamicsSimulatorNode:
    def __init__(self):
        rospy.init_node('dynamics_simulator', anonymous=True)
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('pendulum_control')
        self.read_params()
        self.read_matrices()
        self.pub_output = rospy.Publisher('/top/y', ArrayStamped, queue_size=1)
        self.pub_state = rospy.Publisher('/top/state', ArrayStamped, queue_size=1)
        self.x = np.zeros((self.A.shape[0], 1))  # state vector


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
        self.B = np.atleast_2d(np.genfromtxt(self.matrices_path + "/Bct.csv", delimiter=",")).reshape(-1,1)
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

        print("Ad:", self.Ad, "Bd:", self.Bd, "Cd:", self.Cd)

    def set_state_callback(self, msg):
        """For testing purposes, set the state estimate x to a specific value."""
        self.x = subArray(msg)
        self.y = np.asarray(self.Cd @ self.x)  # update measurement based on new state

        pubArray(self.pub_state, self.x, rospy.Time.now())
        pubArray(self.pub_output, self.y, rospy.Time.now())

    def input_callback(self, msg):
        """Callback function to receive control input"""
        self.u = np.array(msg.scalar).reshape(1,1)
        self.output = self.dynamics_step()
        print("Output:", self.output)
        # wait to simulate sensor delay
        rospy.sleep(self.Ts)
        pubArray(self.pub_output, self.output, rospy.Time.now())
        pubArray(self.pub_state, self.x, rospy.Time.now())


    def dynamics_step(self):
        """Perform one step of the dynamics simulation"""
        self.x = np.asarray(self.Ad @ self.x + self.Bd @ self.u)
        y = self.Cd @ self.x
        return np.asarray(y)
        

    def run(self):
        """Main loop to simulate dynamics"""
        rospy.Subscriber('/top/u', ScalarStamped, self.input_callback)
        # rospy.Subscriber('/Maxon_Motor_top/state', ArrayStamped, self.motor_callback)
        rospy.Subscriber('/top/set_state', ArrayStamped, self.set_state_callback) # for testing, this should be removed in the final version
        rospy.spin()

if __name__ == '__main__':
    node = DynamicsSimulatorNode()
    node.run()