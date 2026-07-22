import rospy
import numpy as np

from pendulum_control.common import * # this imports the common messages
from pendulum_control import pubArray, subArray
from pendulum_control import PendulumModel, LQGController, Integrator

class LQGNode:
    def __init__(self):
        rospy.init_node(name='lqg', anonymous=True)
        self.read_params()
        self.init_variables()
        self.init_topics()
        self.init_publishers()

        self.pendulum = PendulumModel(m_p=0.01, m_d=0.065, l=0.25, b_bottom=self.b_bottom, Ts=0.01)
        self.integrator = Integrator(self.Ts)
        self.lqg = LQGController(self.pendulum.A, 
                            self.pendulum.B, 
                            self.pendulum.C, 
                            self.pendulum.Tx, 
                            self.pendulum.Ty, 
                            self.pendulum.Tu, 
                            self.pendulum.dynamic_step, self.b_calculate_gains)

    def read_params(self):
        """Read parameters from launch file"""
        self.b_calculate_gains = rospy.get_param("~calculate_gains", True)  # if true, provide A,B,Q,R to solve ARE and get K, else provide K
        self.b_bottom = rospy.get_param('is_bottom')
        self.Ts = rospy.get_param('/Ts')

    def init_topics(self):
        self.v_topic = 'v_sp'
        self.u_topic = 'u'
        self.output_topic = 'y'
        self.state_topic = 'state'
        self.set_state_topic = 'set_state'

    def init_publishers(self):
        self.v_pub = rospy.Publisher(self.v_topic, ScalarStamped, queue_size=1)
        self.u_pub = rospy.Publisher(self.u_topic, ScalarStamped, queue_size=1)
        self.state_pub = rospy.Publisher(self.state_topic, ArrayStamped, queue_size=1)

        self.v_sp_msg = ScalarStamped()
        self.u_msg = ScalarStamped()
        # add state estimation error

    def init_variables(self):
        self.time = rospy.Time.now()

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
        self.x = self.lqg.a_posteriori_estimate()
        self.u = self.lqg.lqr.step(self.x)

        self.u_msg.scalar = self.u.item() # convert 1x1 array to float
        self.u_msg.header.stamp = self.time
        self.u_pub.publish(self.u_msg)

        pubArray(self.state_pub, self.x, self.time)

        self.v_sp_msg.scalar = self.integrator.integrate(self.u)
        self.v_pub.publish(self.v_sp_msg)

        self.x = self.lqg.a_priori_estimate(self.x, self.u)
    
    def run(self):
        rospy.Subscriber(self.output_topic, ArrayStamped, self.callback)
        rospy.Subscriber(self.set_state_topic, ArrayStamped, self.set_state_callback)
        rospy.spin()

if __name__ == "__main__":
    node = LQGNode()
    node.run()