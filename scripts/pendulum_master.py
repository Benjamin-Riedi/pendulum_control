# import control
import rospy
import numpy as np

from gelsight_ros.msg import Angles2dStamped
from pendulum_control.common import * # this imports the common messages
from pendulum_control import pubArray, subArray

class InvertedPendulumControlNode:
    def __init__(self):
        rospy.init_node('inv_pend_node', anonymous=True)

        self.read_ROS_params()
        self.init_publishers()
        self.init_variables()

    def read_ROS_params(self):
        """Load parameters from ROS parameter server"""
        self.gelsight_angles_topic = rospy.get_param('/topics/gelsight/angles', '/gelsight/angles')
        self.gelsight_anglesD_topic = rospy.get_param('/topics/gelsight/anglesD', '/gelsight/anglesD')
        self.vicon_angles_topic = rospy.get_param('/topics/vicon/angles', '/vicon/angles')
        self.vicon_anglesD_topic = rospy.get_param('/topics/vicon/anglesD', '/vicon/anglesD')

        self.vicon_state_topic = rospy.get_param('/topics/vicon/state', '/vicon/benjamin_v2/Root')
        self.vicon_world_topic = rospy.get_param('/topics/vicon/world', '/vicon/benjamin_v2/World')

        self.position_bottom_topic = rospy.get_param('/topics/position/bottom', '/bottom/x')
        self.position_top_topic = rospy.get_param('/topics/position/top', '/top/x')
        self.velocity_bottom_topic = rospy.get_param('/topics/velocity/bottom', '/bottom/xD')
        self.velocity_top_topic = rospy.get_param('/topics/velocity/top', '/top/xD')

        self.command_bottom_topic = rospy.get_param('/topics/control/command/bottom', '/bottom/v_sp')
        self.command_top_topic = rospy.get_param('/topics/control/command/top', '/top/v_sp')

        self.state_bottom_topic = rospy.get_param('/topics/control/state/bottom', '/bottom/state')
        self.state_top_topic = rospy.get_param('/topics/control/state/top', '/top/state')

        self.measurement_bottom_topic = rospy.get_param('/topics/control/measurement/bottom', '/bottom/y')
        self.measurement_top_topic = rospy.get_param('/topics/control/measurement/top', '/top/y')

        self.motor_state_bottom_topic = rospy.get_param('/topics/Maxon_Motor_bottom/state', '/Maxon_Motor_bottom/state')
        self.motor_state_top_topic = rospy.get_param('/topics/Maxon_Motor_top/state', '/Maxon_Motor_top/state')

    def init_variables(self):
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

        # callback time
        self.time = rospy.Time.now()

        # mode
        self.vicon = False # set to true if phi & d_phi is calculated from vicon, else they come from the sensor
        # get vicon angle one way or the other and if self.vicon, set the state according to vicon
        # for validation i'll want the vicon stream regardless. (does something break if vicon is not on?)

    def init_publishers(self):
        """Initialize ROS publishers, initialize message variables"""
        self.pub_vicon_anglesD = rospy.Publisher(self.vicon_anglesD_topic, Angles2dStamped, queue_size=10)
        self.pub_gelsight_anglesD = rospy.Publisher(self.gelsight_anglesD_topic, Angles2dStamped, queue_size=10)
        self.pub_command_bottom = rospy.Publisher(self.command_bottom_topic, ScalarStamped, queue_size=10)
        self.pub_command_top = rospy.Publisher(self.command_top_topic, ScalarStamped, queue_size=10)

        self.pub_bottom_state = rospy.Publisher(self.state_bottom_topic, ArrayStamped, queue_size=10)
        self.pub_top_state = rospy.Publisher(self.state_top_topic, ArrayStamped, queue_size=10)

        self.bottom_state_msg = ArrayStamped()
        self.top_state_msg = ArrayStamped()

        self.pub_bottom_measurement = rospy.Publisher(self.measurement_bottom_topic, ArrayStamped, queue_size=10)
        self.pub_top_measurement = rospy.Publisher(self.measurement_top_topic, ArrayStamped, queue_size=10)

        self.bottom_measurement_msg = ArrayStamped()
        self.top_measurement_msg = ArrayStamped()

        # maybe add publishers for state variables, for debugging/validation
        # maybe add some errors/additional metrics

    def get_state(self):
        """Concatenate state variables into one vector per subsystem for publishing"""
        bottom = np.array([self.x, self.xD, self.phi, self.phiD]).reshape(-1,1)
        top = np.array([self.y, self.yD, self.theta, self.thetaD]).reshape(-1,1)
        return bottom, top
    
    def get_measurement(self):
        """Concatenate measurement variables into one vector per subsystem for publishing"""
        bottom = np.array([self.x, self.xD, self.phi]).reshape(-1,1)
        top = np.array([self.y, self.yD, self.theta]).reshape(-1,1)
        return bottom, top
    
    def finite_difference(self, angles, dt):
        """Calculate angular velocities using finite difference method"""
        return (np.array([self.phi, self.theta]) - angles) / dt
    
    def callback_sensor(self, msg):
        """Callback function for sensor data (angles)
        Publishes state or measurement depending on whether Kalman filter is used."""
        old_angles = np.array([self.phi, self.theta])
        self.phi, self.theta = msg.vector
        dt = (msg.header.stamp - self.time).to_sec()
        self.time = msg.header.stamp

        if self.b_kalman_filter:
            bottom, top = self.get_measurement()
            pubArray(self.pub_bottom_measurement, bottom, self.time)
            pubArray(self.pub_top_measurement, top, self.time)

        else:
            self.phiD, self.thetaD = self.finite_difference(old_angles, dt)

            bottom, top = self.get_state()
            pubArray(self.pub_bottom_state, bottom, self.time)
            pubArray(self.pub_top_state, top, self.time)

    def callback_bottom(self, msg):
        """Callback function for bottom motor state data"""
        self.x = msg.vector[0]
        self.xD = msg.vector[1]
        pass

    def callback_top(self, msg):
        """Callback function for top motor state data"""
        self.y = msg.vector[0]
        self.yD = msg.vector[1]
        pass


    def run(self):
        """Subscribe to relevant topics and start the ROS node"""
        rospy.Subscriber(self.motor_state_bottom_topic, ArrayStamped, self.callback_bottom)
        rospy.Subscriber(self.motor_state_top_topic, ArrayStamped, self.callback_top)
        rospy.Subscriber(self.vicon_angles_topic, ArrayStamped, self.callback_sensor) # don't forget about self.vicon distinction
        rospy.Subscriber(self.gelsight_angles_topic, Angles2dStamped, self.callback_sensor)
        rospy.spin()

if __name__ == "__main__":
    node = InvertedPendulumControlNode()
    node.run()