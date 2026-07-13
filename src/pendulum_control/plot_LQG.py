# plot state variables from topic in rosbag file
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os

from utils import bag_to_pd, find_bag

def plot_state(data_frames, root):

    topics = ['/bottom/state']

    for topic in topics:
        if topic not in data_frames:
            print(f"Topic {topic} not found in data frames.")
            pass

        fig, (axx, axd) = plt.subplots(2, 1, sharex=True)

        df = data_frames[topic]
        time = df['time'].to_numpy() * 1e-9 # convert to seconds
        vectors = df['vector']

        # generalize topic names
        x = np.array([row[0] for row in vectors])
        phi = np.rad2deg(np.array([row[1] for row in vectors]))
        dx = np.array([row[2] for row in vectors])
        dphi = np.rad2deg(np.array([row[3] for row in vectors]))

        colors = ['tab:blue', 'tab:red']

        # max values
        factor = 1.1
        mx = np.max(np.abs(x)) * factor
        mdx = np.max(np.abs(dx)) * factor
        mphi = np.max(np.abs(phi)) * factor
        mdphi = np.max(np.abs(dphi)) * factor

        ### POSITION ###

        axx.set_ylabel('Position [m]', color=colors[0])
        axd.set_ylabel('Velocity [m/s]', color=colors[0])
        axd.set_xlabel('time [s]')

        axx.plot(time, x, label=topic.strip('/state') + ' x', color=colors[0])
        axd.plot(time, dx, label=topic.strip('/state') + ' dx', color=colors[0])

        axx.tick_params(axis='y', labelcolor=colors[0])
        axd.tick_params(axis='y', labelcolor=colors[0])

        axx.axhline(linewidth=0.5, color='black', ls='--')
        axd.axhline(linewidth=0.5, color='black', ls='--')

        ### --------- ###

        axp = axx.twinx()  # instantiate a second Axes that shares the same x-axis
        axdp = axd.twinx()

        ### ANGLES ###

        axp.set_ylabel('Angle [deg]', color=colors[1])
        axdp.set_ylabel('Angular Velocity [deg/s]', color=colors[1])

        axp.plot(time, phi, label=topic.strip('/state') + ' phi', color=colors[1])
        axdp.plot(time, dphi, label=topic.strip('/state') + ' dphi', color=colors[1])

        axp.tick_params(axis='y', labelcolor=colors[1])
        axdp.tick_params(axis='y', labelcolor=colors[1])

        ### --------- ###

        # make symmetric to align y=0
        axx.set_ylim(-mx, mx)
        axd.set_ylim(-mdx, mdx)
        axp.set_ylim(-mphi, mphi)
        axdp.set_ylim(-mdphi, mdphi)

        axx.set_title('Position and Angles')
        axd.set_title('First Derivative of Position and Angles')

        axx.legend(loc='upper right')
        axd.legend(loc='upper right')
        axp.legend(loc='lower right')
        axdp.legend(loc='lower right')

        fig.tight_layout()
        fig.set_size_inches(15, 9)
        fig.suptitle(f'Initial State: (x,phi,dx,dphi) = {vectors[0]}', fontsize=16) # maybe add degrees for phi and dphi
        plt.savefig(os.path.join(root, topic.strip('/state') + '_state.png'))
        plt.show()

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
    topic_name = '/bottom/state'  # replace with your topic name
    bag_file = find_bag(exp)
    data_frames = bag_to_pd(exp, [topic_name])
    plot_state(data_frames, exp)
    if not bag_file:
        return

if __name__ == "__main__":
    main()