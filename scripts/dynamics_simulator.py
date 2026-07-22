import rospy
import numpy as np

from pendulum_control.common import * # this imports the common messages
from pendulum_control import pubArray, subArray, finite_difference
from pendulum_control.model import PendulumModel
from std_srvs.srv import Trigger, TriggerResponse

class DynamicsSimulatorNode:
    def __init__(self):
        rospy.init_node('dynamics_simulator', anonymous=True)
        self.read_params()
        self.init_topics()
        self.init_publishers()
        self.count = 0

        self.pendulum = PendulumModel(m_p=0.01, m_d=0.065, l=0.25, b_bottom=self.b_bottom, Ts=self.Ts)
        self.set_gains_proxy = rospy.ServiceProxy('controller/set_gains', SetGains)
        self.set_state_proxy = rospy.ServiceProxy("state/set", Trigger)

        self.Qs = [
            [1000, 1, 1, 1]
        ]
        self.Rs = [
            [50]
        ]

    def read_params(self):
        """Read parameters from parameter server"""
        self.Ts = rospy.get_param('/Ts', 0.01)
        self.b_bottom = rospy.get_param("is_bottom")
    
    def init_publishers(self):
        self.output_pub = rospy.Publisher(self.output_topic, ArrayStamped, queue_size=1)
        self.state_pub = rospy.Publisher(self.state_topic, ArrayStamped, queue_size=1)
    
    def init_topics(self):
        self.output_topic = 'y'
        self.state_topic = 'state'
        self.u_topic = 'u'
        self.set_state_topic = 'set_state'
        self.motor_state_topic = 'Maxon_Motor/state'
        self.v_sp_topic = 'v_sp'

    def set_state_callback(self, msg):
        """For testing purposes, set the state estimate x to a specific value."""
        self.x = subArray(msg)
        self.start = rospy.Time.now()
        # self.y = np.asarray(self.Cd @ self.x)  # update measurement based on new state

        pubArray(self.state_pub, self.x, rospy.Time.now())
        # pubArray(self.output_pub, self.y, rospy.Time.now())
    
    def motor_state_callback(self, msg):
        """Callback function to receive motor state"""
        motor_state = msg
        self.x[0] = motor_state.vector[0]  # position
        self.x[2] = motor_state.vector[1]  # velocity
    
    def v_sp_callback(self, msg):
        """Callback function to receive velocity setpoint"""
        self.x[2] = msg.scalar

    def input_callback(self, msg):
        """Callback function to receive control input"""
        self.u = np.array(msg.scalar).reshape(1,1)
        self.x = self.pendulum.dynamic_step(self.x, self.u)

        # self.output = self.dynamics_step()
        # print("Output:", self.output)
        # wait to simulate sensor delay
        rospy.sleep(self.Ts)
        if (rospy.Time.now() - self.start).to_sec() < 4:
        # pubArray(self.output_pub, self.output, rospy.Time.now())
            pubArray(self.state_pub, self.x, rospy.Time.now())
            return
        
        if self.count >= len(self.Qs):
            rospy.signal_shutdown("Simulation complete. Shutting down.")
        self.set_gains_proxy(self.Qs[self.count], self.Rs[self.count])
        self.count += 1
        self.set_state_proxy()

    # def dynamics_step(self):
    #     """Perform one step of the dynamics simulation"""
    #     self.x = np.asarray(self.Ad @ self.x + self.Bd @ self.u)
    #     y = self.Cd @ self.x
    #     return np.asarray(y)
        

    def run(self):
        """Main loop to simulate dynamics"""
        rospy.Subscriber(self.u_topic, ScalarStamped, self.input_callback)
        rospy.Subscriber(self.motor_state_topic, ArrayStamped, self.motor_state_callback)
        rospy.Subscriber(self.set_state_topic, ArrayStamped, self.set_state_callback)
        rospy.Subscriber(self.v_sp_topic, ScalarStamped, self.v_sp_callback)
        rospy.spin()

if __name__ == '__main__':
    node = DynamicsSimulatorNode()
    node.run()