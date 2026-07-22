import rospy
import numpy as np

from pendulum_control.common import * # this imports the common messages
from pendulum_control import pubArray, subArray
from pendulum_control.model import PendulumModel
from pendulum_control.lqr import Integrator, LQRController
# from pendulum_control.srv import SetGains, SetGainsResponse

class StateFeedbackNode:
    def __init__(self):
        rospy.init_node(name='state_feedback')
        self.read_params()
        self.init_variables()
        self.init_topics()
        self.init_publishers()

        pendulum = PendulumModel(m_p=0.01, m_d=0.065, l=0.25, b_bottom=self.b_bottom, Ts=self.Ts)
        self.lqr = LQRController(pendulum.A, pendulum.B, pendulum.Tx, pendulum.Tu, calculate_gains=self.b_calculate_K)
        self.integrator = Integrator()

    def read_params(self):
        self.b_calculate_K = rospy.get_param("~calculate_gains", True)
        self.b_bottom = rospy.get_param("is_bottom")
        self.Ts = rospy.get_param('/Ts')

    def init_topics(self):
        self.state_topic = 'state'
        self.v_topic = 'v_sp'
        self.u_topic = 'u'
        
    def init_publishers(self):
        self.v_pub = rospy.Publisher(self.v_topic, ScalarStamped, queue_size=1)
        self.u_pub = rospy.Publisher(self.u_topic, ScalarStamped, queue_size=1)
        self.v_sp_msg = ScalarStamped()
        self.u_msg = ScalarStamped()

    def init_variables(self):
        self.u = 0.0
        self.x = np.zeros((4, 1))
        self.time = rospy.Time.now()
    
    def callback(self, msg):
        time_delta = (msg.header.stamp - self.time).to_sec()
        # print(f"time between callbacks: {time_delta}")
        self.time = msg.header.stamp

        self.x = subArray(msg)
        self.u = self.lqr.step(self.x)

        self.v_sp_msg.scalar = self.integrator.integrate(self.u, self.Ts)
        self.v_pub.publish(self.v_sp_msg)
        
        self.u_msg.scalar = self.u
        self.u_pub.publish(self.u_msg)
        callback_duration = (rospy.Time.now() - self.time).to_sec()
        # print(f"Callback duration: {callback_duration}")

    def run(self):
        rospy.Subscriber(self.state_topic, ArrayStamped, self.callback)
        rospy.Service('controller/set_gains', SetGains, self.lqr.update_gains)
        rospy.spin()

if __name__ == "__main__":
    node = StateFeedbackNode()
    node.run()