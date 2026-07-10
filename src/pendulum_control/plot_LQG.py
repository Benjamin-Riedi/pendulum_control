# plot state variables from topic in rosbag file
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os
# from pendulum_control.msg import ArrayStamped

def find_csv(exp):
    for file in os.listdir(exp):
        if file.endswith(".csv") and file != 'labels.csv':
            csv_file = os.path.join(exp, file)
            return csv_file
    print("No csv file found in %s, skipping experiment" % exp)
    return None

def find_bag(exp):
    for file in os.listdir(exp):
        if file.endswith(".bag"):
            bag_file = os.path.join(exp, file)
            return bag_file
    print("No bag file found in %s, skipping experiment" % exp)
    return None

def main():
    b_all_exps = False

    if b_all_exps:
        exp = '/home/benjamin/tesla_ws/src/pendulum_control/data/exps'
        for subdir in os.listdir(exp):
            subdir_path = os.path.join(exp, subdir)
            bag_file = find_bag(subdir_path)

    else:
        exp = os.getcwd()
        bag_file = find_bag(exp)
    exp = '/home/benjamin/tesla_ws/src/pendulum_control/data/exps'
    topic_name = '/bottom/state'  # replace with your topic name
    bag_file = find_bag(exp)
    if not bag_file:
        return

    bag = rosbag.Bag(bag_file)
    time_stamps = []
    state_data = []

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        time_stamps.append(t.to_sec())
        state_data.append(np.array(msg.vector).reshape(msg.shape))

    bag.close()

    state_data = np.array(state_data)
    time_stamps = np.array(time_stamps)

    plt.figure(figsize=(10, 6))
    print("state_data shape", state_data.shape)
    print(state_data)
    for i in range(state_data.shape[1]):
        plt.plot(time_stamps, state_data[:, i], label=f'State {i+1}')
    
    plt.xlabel('Time (s)')
    plt.ylabel('State Variables')
    plt.title('State Variables over Time')
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    main()