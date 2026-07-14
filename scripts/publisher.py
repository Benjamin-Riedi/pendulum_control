import rospy

from ethercat_motor_msgs.msg import MotorCtrlMessage
from pendulum_control.msg import ArrayStamped
from pendulum_control import set_pos, set_vel, set_torque, set_state

class Publisher:
    def __init__(self):
        rospy.init_node("command_pub", anonymous=True)
        self.get_params()
        self.init_publishers()
        rospy.sleep(0.5)  # wait for publishers to initialize

    def get_params(self):
        self.motor_topic = rospy.get_param("Maxon_Motor/command")
        # include params to set separate states

    def init_topics(self):
        self.set_state_topic = 'set_state'

    def init_publishers(self):
        self.motor_top_pub = rospy.Publisher(self.motor_topic, MotorCtrlMessage, queue_size=1)
        self.set_state_pub = rospy.Publisher(self.set_state_topic, ArrayStamped, queue_size=1)

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
    print('im alive')
    publisher.publish()

