import numpy as np
from pendulum_control.msg import ArrayStamped

def pubArray(pub, array, timestamp=None):
    msg = ArrayStamped()
    msg.shape = array.shape
    msg.vector = array.flatten().tolist()
    assert type(array) != 'np.matrix', "array should be a numpy ndarray, not a numpy matrix"
    if timestamp is not None:
        msg.header.stamp = timestamp
    pub.publish(msg)

def subArray(msg):
    array = np.array(msg.vector).reshape(msg.shape)
    return array