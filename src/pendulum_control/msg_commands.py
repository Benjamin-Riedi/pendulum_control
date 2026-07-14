import rospy

from pendulum_control.msg import ArrayStamped
from ethercat_motor_msgs.msg import MotorCtrlMessage

def set_state(x=0.0, phi=0.0, dx=0.0, dphi=0.0):
    msg = ArrayStamped()
    msg.vector = [x, phi, dx, dphi]
    msg.header.stamp = rospy.Time.now()
    msg.shape = (len(msg.vector), 1)
    return msg

def set_pos(x=0.0):
    msg = MotorCtrlMessage()
    msg.targetPosition = x
    msg.operationMode = 8  # CSP mode
    return msg

def set_vel(v=0.0):
    msg = MotorCtrlMessage()
    msg.targetVelocity = v
    msg.operationMode = 9  # CSV mode
    return msg

def set_torque(t=0.0):
    msg = MotorCtrlMessage()
    msg.targetTorque = t
    msg.operationMode = 10  # CST mode
    return msg