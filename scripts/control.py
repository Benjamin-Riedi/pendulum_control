import control
import rospy
import numpy as np

from control_utils.msg import ScalarStamped
from gelsight_ros.msg import Angles2dStamped

class InvertedPendulumControlNode:
    def __init__(self):
        rospy.init_node('inv_pend_node', anonymous=True)

    def read_ROS_params(self):
        # is it worth implementing params? these would go in launch file, like test_generator
        self.b_calculate_K = rospy.get_param('~calculate_K', False) # if true, provide A,B,Q,R to solve ARE and get K, else provide K
        self.matrices_path = rospy.get_param('~matrices_path', '/matrices') # path to matrices A,B,Q,R,K
        
        # topics
        self.gelsight_angles_topic = rospy.get_param('/topics/gelsight/angles', '/gelsight/angles')
        self.gelsight_anglesD_topic = rospy.get_param('/topics/gelsight/anglesD', '/gelsight/anglesD')
        self.vicon_angles_topic = rospy.get_param('/topics/vicon/angles', '/vicon/angles')
        self.vicon_anglesD_topic = rospy.get_param('/topics/vicon/anglesD', '/vicon/anglesD')

        self.vicon_state_topic = rospy.get_param('/topics/vicon/state', '/vicon/benjamin_v2/Root')
        self.vicon_world_topic = rospy.get_param('/topics/vicon/world', '/vicon/benjamin_v2/World')

        self.bottom_position_topic = rospy.get_param('/topics/position/bottom', '/bottom/x')
        self.top_position_topic = rospy.get_param('/topics/position/top', '/top/x')
        self.bottom_velocity_topic = rospy.get_param('/topics/velocity/bottom', '/bottom/xD')
        self.top_velocity_topic = rospy.get_param('/topics/velocity/top', '/top/xD')

        self.command_bottom_topic = rospy.get_param('/topics/command/bottom', '/bottom/v_sp')
        self.command_top_topic = rospy.get_param('/topics/command/top', '/top/v_sp')
        

    def init_system(self):
        #i'll read into this, prob not necessary
        pass

    def init_controllers(self):
        
        if self.b_calculate_K:
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
        self.x = 0.0
        self.y = 0.0
        self.xD = 0.0
        self.yD = 0.0
        # maybe add previous state as buffer variable
        
        # Pendulum
        self.phi = 0.0
        self.theta = 0.0
        self.phiD = 0.0
        self.thetaD = 0.0

        # processing times?

        # mode
        self.vicon = False # set to true if phi & d_phi is calculated from vicon, else they come from the sensor
        # get vicon angle one way or the other and if self.vicon, set the state according to vicon
        # for validation i'll want the vicon stream regardless. (does something break if vicon is not on?)
        pass

    def init_publishers(self):
        self.pub_vicon_anglesD = rospy.Publisher(self.vicon_anglesD_topic, Angles2dStamped, queue_size=10)
        self.pub_gelsight_anglesD = rospy.Publisher(self.gelsight_anglesD_topic, Angles2dStamped, queue_size=10)
        self.pub_command_bottom = rospy.Publisher(self.command_bottom_topic, ScalarStamped, queue_size=10)
        self.pub_command_top = rospy.Publisher(self.command_top_topic, ScalarStamped, queue_size=10)

        # maybe add publishers for state variables, for debugging/validation
        # maybe add some errors/additional metrics

    def publish_pendulum_states(self):
        # phi prob not here but dphi
        # first try finite difference, but prob a Kalman filter is necessary
        pass

    def get_state(self):
        pass

    def callback_position_bottom(self, msg):
        # this just updates the state variables
        # with this setup i'll need a callback for each variable, but i can just have them all call set_state() to update the state vector
        # so maybe a synchronizer wouldn't be a terrible idea
        pass

    def callback_sensor(self, msg):
        # here i need the state feedback step
        pass

    def state_feedback_step(self):
        pass
    
    def publish_control_variables(self):
        pass

    def run(self):
        rospy.Subscriber(self.bottom_position_topic, ScalarStamped, self.callback_position_bottom)
        # problem discovered: each variable has separate topic but pos and vel are publishes at the same time.
        # do i just ignore this and have getState() just read latest values? what goes into callbacks?
