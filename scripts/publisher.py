import rospy

from ethercat_motor_msgs.msg import MotorCtrlMessage
from pendulum_control.msg import ArrayStamped
from pendulum_control import set_pos, set_vel, set_torque, set_state
from std_srvs.srv import Trigger, TriggerResponse

class Publisher:
    def __init__(self):
        rospy.init_node("command_pub", anonymous=True)
        self.init_topics()
        self.init_publishers()
        self.calib_srv = rospy.Service(
            'state/set',
            Trigger,
            self.service_callback
        )
        rospy.sleep(5)  # wait for publishers to initialize

    def get_params(self):
        # include params to set separate states
        pass

    def init_topics(self):
        self.motor_topic = 'Maxon_Motor/command'
        self.set_state_topic = 'set_state'

    def init_publishers(self):
        # latch=true
        self.motor_top_pub = rospy.Publisher(self.motor_topic, MotorCtrlMessage, queue_size=1)
        self.set_state_pub = rospy.Publisher(self.set_state_topic, ArrayStamped, queue_size=1)

    def service_callback(self, req):
        self.publish()
        return TriggerResponse(success=True, message="Start command received.")

    def publish(self):

        ### SET STATE COMMANDS ###


        state_msg = set_state(x=0.0, phi=0.08, dx=0.0, dphi=0.0)
        self.set_state_pub.publish(state_msg)

        # ### POSITION COMMANDS ###
        
        # top_motor_msg = set_pos(0.0)
        # self.motor_top_pub.publish(top_motor_msg)

        # bottom_motor_msg = set_pos(0.0)
        # self.motor_bottom_pub.publish(bottom_motor_msg)

        # ### VELOCITY COMMANDS ###

        # top_motor_msg = set_vel(0.0)
        # self.motor_top_pub.publish(top_motor_msg)

        # bottom_motor_msg = set_vel(0.0)
        # self.motor_bottom_pub.publish(bottom_motor_msg)

        # ### TORQUE COMMANDS ###

        # top_motor_msg = set_torque(0.0)
        # self.motor_top_pub.publish(top_motor_msg)
        
        # bottom_motor_msg = set_torque(0.0)
        # self.motor_bottom_pub.publish(bottom_motor_msg)

if __name__ == "__main__":
    publisher = Publisher()
    rospy.spin()

