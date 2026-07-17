# import control
import rospy
import numpy as np

from threading import Event, Thread
from std_srvs.srv import Trigger, TriggerResponse
from pendulum_control.common import * # this imports the common messages
from pendulum_control import pubArray, subArray

class InvertedPendulumControlNode:
    def __init__(self):
        rospy.init_node('inv_pend_node', anonymous=True)

        self.read_ROS_params()
        self.init_topics()
        self.init_publishers()
        self.init_variables()

        self.calib_srv = rospy.Service(
            'controller/start',
            Trigger,
            self.service_callback
        )

    def read_ROS_params(self):
        """Load parameters from ROS parameter server"""
        self.b_kalman_filter = rospy.get_param("~kalman_filter", True)
        self.Ts = rospy.get_param("/Ts", 0.01)  # sampling time
        # self.gelsight_angles_topic = rospy.get_param('/topics/gelsight/angles', '/gelsight/angles')
        # self.gelsight_anglesD_topic = rospy.get_param('/topics/gelsight/anglesD', '/gelsight/anglesD')
        # self.vicon_angles_topic = rospy.get_param('/topics/vicon/angles', '/vicon/angles')
        # self.vicon_anglesD_topic = rospy.get_param('/topics/vicon/anglesD', '/vicon/anglesD')

        # self.vicon_state_topic = rospy.get_param('/topics/vicon/state', '/vicon/benjamin_v2/Root')
        # self.vicon_world_topic = rospy.get_param('/topics/vicon/world', '/vicon/benjamin_v2/World')

    def init_variables(self):
        # Actuator
        self.x = 0.0
        self.xD = 0.0
        # maybe add previous state as buffer variable
        
        # Pendulum
        self.phi = 0.0
        self.phiD = 0.0

        # callback time
        self.time = rospy.Time.now()

        self.activate = Event()

        # mode
        self.vicon = True # set to true if phi & d_phi is calculated from vicon, else they come from the sensor
        # get vicon angle one way or the other and if self.vicon, set the state according to vicon
        # for validation i'll want the vicon stream regardless. (does something break if vicon is not on?)

    def init_topics(self):
        self.gelsight_angle_topic = 'gelsight/phi'
        self.gelsight_angle_vel_topic = 'gelsight/dphi'
        self.vicon_angle_topic = 'vicon/phi'
        self.vicon_angle_vel_topic = 'vicon/dphi'

        self.u_topic = 'u'
        self.output_topic = 'y'
        self.state_topic = 'state'
        self.set_state_topic = 'set_state'

        self.motor_state = 'Maxon_Motor/state'

    def init_publishers(self):
        """Initialize ROS publishers, initialize message variables"""
        # self.pub_vicon_anglesD = rospy.Publisher(self.vicon_anglesD_topic, Angles2dStamped, queue_size=10)
        # self.pub_gelsight_anglesD = rospy.Publisher(self.gelsight_anglesD_topic, Angles2dStamped, queue_size=10)

        self.pub_state = rospy.Publisher(self.state_topic, ArrayStamped, queue_size=10)

        self.state_msg = ArrayStamped()

        self.pub_output = rospy.Publisher(self.output_topic, ArrayStamped, queue_size=10)

        self.output_msg = ArrayStamped()

        # maybe add publishers for state variables, for debugging/validation
        # maybe add some errors/additional metrics

    def get_state(self):
        """Concatenate state variables into one vector per subsystem for publishing"""
        state = np.array([self.x, self.phi, self.xD, self.phiD]).reshape(-1,1)
        return state
    
    def get_output(self):
        """Concatenate measurement variables into one vector per subsystem for publishing"""
        output = np.array([self.x, self.phi, self.xD]).reshape(-1,1)
        return output
    
    def finite_difference(self, angle, dt):
        """Calculate angular velocities using finite difference method"""
        return (self.phi - angle) / self.Ts
    
    def callback_sensor(self, msg):
        """Callback function for sensor data (angles)
        Publishes state or measurement depending on whether Kalman filter is used."""
        old_angle = self.phi
        self.phi = msg.scalar
        dt = (msg.header.stamp - self.time).to_sec()
        self.time = msg.header.stamp

        if self.b_kalman_filter:
            output = self.get_output()
            pubArray(self.pub_output, output, self.time)

        else:
            self.phiD = self.finite_difference(old_angle, dt)

            state = self.get_state()
            pubArray(self.pub_state, state, self.time)

    def motor_callback(self, msg):
        """Callback function for motor state data"""
        self.x = msg.vector[0]
        self.xD = msg.vector[1]

    def service_callback(self, req):
        self.activate.set()
        return TriggerResponse(success=True, message="Activated Controller.")

    def run(self):
        """Subscribe to relevant topics and start the ROS node"""
        self.activate.wait()
        rospy.sleep(5.0)
        rospy.Subscriber(self.motor_state, ArrayStamped, self.motor_callback)
        if self.vicon:
            rospy.Subscriber(self.vicon_angle_topic, ScalarStamped, self.callback_sensor)
        else:
            rospy.Subscriber(self.gelsight_angle_topic, ScalarStamped, self.callback_sensor)
        rospy.spin()

if __name__ == "__main__":
    node = InvertedPendulumControlNode()
    Thread(target=node.run, daemon=True).start()
    rospy.spin()