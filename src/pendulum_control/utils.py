import numpy as np
import rosbag
import pandas as pd
import os
from pendulum_control.msg import ArrayStamped

def pubArray(pub, array, timestamp=None):
    msg = ArrayStamped()
    msg.shape = array.shape
    msg.vector = np.asarray(array).flatten().tolist()
    assert type(array) != 'np.matrix', "array should be a numpy ndarray, not a numpy matrix"
    if timestamp is not None:
        msg.header.stamp = timestamp
    pub.publish(msg)

def subArray(msg):
    array = np.array(msg.vector).reshape(msg.shape)
    return array

def integrate(v_prev, u, dt):
    """Integrate the input u over time to get the new value of v"""
    return v_prev + u * dt

def integrate_trapezoidal(v_prev, u, u_prev, dt):
    """Integrate the input u over time using trapezoidal rule to get the new value of v"""
    return v_prev + 0.5 * dt * (u + u_prev)

def finite_difference(phi_prev, phi, dt):
    """Calculate the finite difference of phi over time."""
    return (phi - phi_prev) / dt

def bag_to_pd(root, topics):
    """Read specified topics from a rosbag and return a dictionary of pandas DataFrames."""
    bag_file = find_bag(root)
    if bag_file is None:
        print(f"No bag file found in {root}, skipping experiment")
        return None
    bag = rosbag.Bag(bag_file,'r')
    start_time = bag.get_start_time() * 1e9 # convert to ns
    if not topics:
        topics = bag.get_type_and_topic_info().topics.keys()

    row_lists = {topic: [] for topic in topics}

    for topic, msg, t in bag.read_messages(topics=topics):
        dict1 = {}
        dict1['time'] = t.to_nsec() - start_time
        for field in msg.__slots__:
            dict1[field] = getattr(msg, field)
        row_lists[topic].append(dict1)

    data_frames = {topic: pd.DataFrame(row_lists[topic]) for topic in topics}
    return data_frames

def find_bag(exp):
    """Find the rosbag file in an experiment folder."""
    for file in os.listdir(exp):
        if file.endswith(".bag"):
            bag_file = os.path.join(exp, file)
            return bag_file
    print("No bag file found in %s, skipping experiment" % exp)
    return None