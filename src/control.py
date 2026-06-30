from unicodedata import normalize

import control
import rospy
import numpy as np

class InvertedPendulumControlNode:
    def __init__(self):
        rospy.init_node('inv_pend_node', anonymous=True)

    def read_ROS_params(self):
        # is it worth implementing params? these would go in launch file, like test_generator
        self.calculate_K = rospy.get_param('~calculate_K', False) # if true, provide A,B,Q,R to solve ARE and get K, else provide K
        self.matrices_path = rospy.get_param('~matrices_path', '/matrices')# path to matrices A,B,Q,R,K
        pass

    def init_system(self):
        #i'll read into this, prob not necessary
        pass

    def init_controllers(self):
        
        if self.calculate_K:
            self.calculate_K()
        else:
            self.K = np.atleast_2d(np.genfromtxt(self.matrices_path + '/K.csv', delimiter=','))

    def calculate_K(self):
        self.A = np.atleast_2d(np.genfromtxt(self.matrices_path + '/Ac.csv', delimiter=','))
        self.B = np.atleast_2d(np.genfromtxt(self.matrices_path + '/Bc.csv', delimiter=','))
        # i don't care about output
        C = np.zeros((1,8))
        D = np.zeros((1,2))

        self.Q = np.atleast_2d(np.genfromtxt(self.matrices_path + '/Q.csv', delimiter=','))
        self.R = np.atleast_2d(np.genfromtxt(self.matrices_path + '/R.csv', delimiter=','))

        sys_c = control.ss(self.A,self.B,C,D)
        sys_d = control.c2d(sys_c, self.Ts)

        self.Ad = sys_d.A
        self.Bd = sys_d.B

        # here i'd add the normalization (Tx, Tu) but i'll leave it out for now
        # if normalize: ...

        self.K, P, E = control.dlqr(self.Ad, self.Bd, self.Q, self.R)

        # maybe add support to write K to file, but for now i'll just print it
        print("Calculated K:")
        print(self.K)

    def init_variables(self):
        # STATE
        # Actuator
        self.x
        self.y
        self.d_x
        self.d_y
        # maybe add previous state as buffer variable
        
        # Pendulum
        self.phi
        self.theta
        self.d_phi
        self.d_theta

        # processing times?

        # mode
        self.vicon # set to true if phi & d_phi is calculated from vicon, else they come from the sensor
        # get vicon angle one way or the other and if self.vicon, set the state according to vicon
        # for validation i'll want the vicon stream regardless. (does something break if vicon is not on?)
        pass

    def init_publishers(self):
                                                        # get messages in order
        self.pub_phi = rospy.Publisher(self.CHANGE_THIS, ScalarStamped, queue_size=10)
        self.pub_d_phi = rospy.Publisher(self.CHANGE_THIS, ScalarStamped, queue_size=10)

    def publish_pendulum_states(self):
        # phi prob not here but dphi
        # first try finite difference, but prob a Kalman filter is necessary
        pass

    def get_state(self):
        pass

    def callback_maxon(self, msg):
        pass

    def callback_sensor(self, msg):
        pass

    def state_feedback_step(self):
        pass
    
    def publish_control_variables(self):
        pass

    def run(self):
        pass
