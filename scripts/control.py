import control
import rospy
import numpy as np

from control_utils.msg import ScalarStamped, VectorStamped
from gelsight_ros.msg import Angles2dStamped
from state_feedback import StateFeedback

class InvertedPendulumControlNode:
    def __init__(self):
        rospy.init_node('inv_pend_node', anonymous=True)

        self.read_ROS_params()
        # self.init_controllers()
        self.init_publishers()
        self.init_variables()

    def read_ROS_params(self):
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

        self.command_bottom_topic = rospy.get_param('/topics/control/command/bottom', '/bottom/v_sp')
        self.command_top_topic = rospy.get_param('/topics/control/command/top', '/top/v_sp')

        self.bottom_state_topic = rospy.get_param('/topics/control/state/bottom', '/bottom/state')
        self.top_state_topic = rospy.get_param('/topics/control/state/top', '/top/state')

        self.bottom_motor_state_topic = rospy.get_param('/topics/Maxon_Motor_bottom/state', '/Maxon_Motor_bottom/state')
        self.top_motor_state_topic = rospy.get_param('/topics/Maxon_Motor_top/state', '/Maxon_Motor_top/state')

    # def init_controllers(self):

    #     # matrices
    #     self.A = np.atleast_2d(
    #         np.genfromtxt(self.matrices_path + "/Ac.csv", delimiter=",")
    #     )
    #     self.B_b = np.atleast_2d(
    #         np.genfromtxt(self.matrices_path + "/Bc.csv", delimiter=",")
    #     )
    #     self.B_t = self.B_b.copy()
    #     self.B_t[0, -1] *= -1

    #     self.Q = np.atleast_2d(
    #         np.genfromtxt(self.matrices_path + "/Q.csv", delimiter=",")
    #     )
    #     self.R = np.atleast_2d(
    #         np.genfromtxt(self.matrices_path + "/R.csv", delimiter=",")
    #     )
        
    #     if self.b_calculate_K:
    #         self.K_b = self.calculate_K(self.B_b)
    #         self.K_t = self.calculate_K(self.B_t)
    #     else:
    #         #adjust for 2 systems
    #         self.K = np.atleast_2d(np.genfromtxt(self.matrices_path + '/K.csv', delimiter=','))

    #     self.controller_bottom = StateFeedback(self.K_b)
    #     self.controller_top = StateFeedback(self.K_t)

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

    def init_publishers(self):
        self.pub_vicon_anglesD = rospy.Publisher(self.vicon_anglesD_topic, Angles2dStamped, queue_size=10)
        self.pub_gelsight_anglesD = rospy.Publisher(self.gelsight_anglesD_topic, Angles2dStamped, queue_size=10)
        self.pub_command_bottom = rospy.Publisher(self.command_bottom_topic, ScalarStamped, queue_size=10)
        self.pub_command_top = rospy.Publisher(self.command_top_topic, ScalarStamped, queue_size=10)
        self.pub_bottom_state = rospy.Publisher(self.bottom_state_topic, VectorStamped, queue_size=10)
        self.pub_top_state = rospy.Publisher(self.top_state_topic, VectorStamped, queue_size=10)

        self.bottom_state_msg = VectorStamped()
        self.top_state_msg = VectorStamped()

        # maybe add publishers for state variables, for debugging/validation
        # maybe add some errors/additional metrics

    def publish_pendulum_states(self):
        # phi prob not here but dphi
        # first try finite difference, but prob a Kalman filter is necessary
        pass

    def get_state(self):
        bottom = np.array([self.x, self.xD, self.phi, self.phiD]).reshape(-1,1)
        top = np.array([self.y, self.yD, self.theta, self.thetaD]).reshape(-1,1)
        return bottom, top
    
    def callback_sensor(self, msg):
        # here i need the state feedback step
        # u_bottom = self.controller_bottom.step(self.get_state())
        # u_top = self.controller_top.step(self.get_state())

        self.bottom_state_msg.vector, self.top_state_msg.vector = self.get_state()
        self.pub_bottom_state.publish(self.bottom_state_msg)
        v_sp_bottom = rospy.wait_for_message(self.command_bottom_topic, ScalarStamped, timeout=0.5)
        self.pub_top_state.publish(self.top_state_msg)
        v_sp_top = rospy.wait_for_message(self.command_top_topic, ScalarStamped, timeout=0.5)

        self.pub_command_bottom.publish(v_sp_bottom)
        self.pub_command_top.publish(v_sp_top)

    def callback_bottom(self, msg):
        # this just updates the state variables
        # with this setup i'll need a callback for each variable, but i can just have them all call set_state() to update the state vector
        # so maybe a synchronizer wouldn't be a terrible idea
        pass

    def callback_top(self, msg):
        pass

    def run(self):
        rospy.Subscriber(self.bottom_state_topic, ScalarStamped, self.callback_bottom)
        rospy.Subscriber(self.top_state_topic, ScalarStamped, self.callback_top)
        rospy.Subscriber(self.vicon_angles_topic, Angles2dStamped, self.callback_sensor) # don't forget about self.vicon distinction
        rospy.Subscriber(self.gelsight_angles_topic, Angles2dStamped, self.callback_sensor)
        # problem discovered: each variable has separate topic but pos and vel are publishes at the same time.
        # do i just ignore this and have getState() just read latest values? what goes into callbacks?
