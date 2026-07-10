import numpy as np
from pendulum_control.msg import ArrayStamped

def pubArray(pub, array, timestamp=None):
    msg = ArrayStamped()
    msg.shape = array.shape
    msg.array = array.flatten().tolist()
    if timestamp is not None:
        msg.header.stamp = timestamp
    pub.publish(msg)

def subArray(msg):
    array = np.array(msg.array).reshape(msg.shape)
    return array