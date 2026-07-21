# plot state variables from topic in rosbag file
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import os

from utils import bag_to_pd, find_bag

def extract_array(data_frames, topic):
    if topic not in data_frames:
        print(f"Topic {topic} not found in data frames.")
        return [], []
    df = data_frames[topic]
    time = df['time'].to_numpy() * 1e-9 # convert to seconds
    vectors = df['vector']

    return_list = []

    for i in range(len(vectors[0])):
        return_list.append(np.array([row[i] for row in vectors]))

    return return_list, time

def extract_velocity(data_frames, topics):
    for topic in topics:
        if topic not in data_frames:
            print(f"Topic {topic} not found in data frames.")
            continue
        df = data_frames[topic]
        time = df['time'].to_numpy() * 1e-9 # convert to seconds
        actualVelocity = df['actualVelocity'].to_numpy()
        
    return actualVelocity, time

def extract_scalar(data_frames, topic):
    if topic not in data_frames:
        print(f"Topic {topic} not found in data frames.")
        return [], []
    df = data_frames[topic]
    time = df['time'].to_numpy() * 1e-9 # convert to seconds
    scalar = df['scalar'].to_numpy()
        
    return scalar, time

def plot_velocity(data_frames, root, top):
    dir_name = os.path.split(root)[-1]

    if top:
        actualVelocity, time_r = extract_velocity(data_frames, ['/ethercat_master/Maxon_Motor_top/reading'])
        v_sp, time_sp = extract_scalar(data_frames, '/top/v_sp')
    else:
        actualVelocity, time_r = extract_velocity(data_frames, ['/ethercat_master/Maxon_Motor_bottom/reading'])
        v_sp, time_sp = extract_scalar(data_frames, '/bottom/v_sp')

    v_sp *= 1200 # convert to rpm
    mask = (time_r >= time_sp[0]) & (time_r <= time_sp[-1])

    v_sp_interp = np.interp(time_r[mask], time_sp, v_sp)
    error = actualVelocity[mask] - v_sp_interp

    # ax_err.plot(time_r[mask], error)
    # v_sp_interp = np.interp(time_r, time_sp, v_sp)

    # error = actualVelocity - v_sp_interp

    # this calculates the delay between the setpoint and the actual velocity using cross-correlation
    # i didn't write this and have no idea if or how it works
    corr = np.correlate(
    actualVelocity - np.mean(actualVelocity),
    v_sp_interp - np.mean(v_sp_interp),
    mode='full')

    lag = corr.argmax() - (len(actualVelocity) - 1)
    delay = lag * np.mean(np.diff(time_r))

    fig, (axv, axe) = plt.subplots(2, 1, sharex=True)
    axv.plot(time_r, actualVelocity, label='Actual Velocity', color='tab:blue')
    axv.plot(time_sp, v_sp, label='Velocity Setpoint', color='tab:red', linestyle='--')
    axv.set_xlabel('Time [s]')
    axv.set_ylabel('Velocity [rpm]')
    axv.legend()
    axv.set_title(f'Velocities, {dir_name}')

    axe.plot(time_r[mask], error, label='Velocity Error', color='tab:green')
    axe.set_xlabel('Time [s]')
    axe.set_ylabel('Error [rpm]')
    axe.set_title(f'Velocity Error. Delay is {delay:.2f} s')
    fig.tight_layout()
    fig.set_size_inches(15, 9)
    if top:
        plt.savefig(os.path.join(root, 'velocity_tracking_top.png'))
    else:
        plt.savefig(os.path.join(root, 'velocity_tracking_bottom.png'))
    plt.show()

def plot_integration(data_frames, root, topics):
    fig, ((axt, axe1), (axb, axe2)) = plt.subplots(2, 2, sharex='col')
    for topic, ax in zip(topics, [(axt, axe1), (axb, axe2)]):
        v_sp_u, time_u = extract_scalar(data_frames, topic[0])
        v_sp_v, time_v = extract_scalar(data_frames, topic[1])
        error = v_sp_u - v_sp_v
        ax[0].plot(time_u, v_sp_u, label=topic[0].replace('v_sp', '').strip('/') + ' v_sp_u', color='tab:blue')
        ax[0].plot(time_v, v_sp_v, label=topic[1].replace('v_sp', '').strip('/') + ' v_sp_v', color='tab:red')
        ax[1].plot(time_u, error, label='Integration Difference', color='tab:green')
        ax[1].set_ylabel('Difference [m/s]')
        ax[0].set_xlabel('Time [s]')
        ax[0].set_ylabel('Setpoint [m/s]')
        ax[0].set_title(topic)
        ax[0].legend()
    
    fig.tight_layout()
    fig.set_size_inches(15, 9)
    plt.show()


def plot_input(data_frames, root, topics):
    dir_name = os.path.split(root)[-1]

    fig, (axt, axb) = plt.subplots(2, 1, sharex=True)
    for topic, ax in zip(topics,[axt, axb]):
        u, time = extract_scalar(data_frames, topic)

        ax.plot(time, u, label=topic.replace('u', '').strip('/') + ' u', color='tab:blue')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Input [m/s^2]')
        ax.set_title(f'{topic}, {dir_name}')
        ax.legend()
    fig.tight_layout()
    fig.set_size_inches(15, 9)
    plt.savefig(os.path.join(root, 'u.png'))
    plt.show()

def plot_angles(data_frames, root, topics):
    dir_name = os.path.split(root)[-1]
    fig, (axt, axb) = plt.subplots(2, 1, sharex=True)
    for topic, ax in zip(topics,[axt, axb]):
        angle, time = extract_scalar(data_frames, topic)
        angle = np.rad2deg(angle)

        ax.plot(time, angle, label=topic.replace('angle', '').strip('/') + ' angle', color='tab:blue')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Angle [deg]')
        ax.set_title(f'{topic}, {dir_name}')
        ax.legend()
    fig.tight_layout()
    fig.set_size_inches(15, 9)
    plt.savefig(os.path.join(root, 'angles.png'))
    plt.show()

def plot_output(data_frames, root, topics):
    dir_name = os.path.split(root)[-1]

    for topic in topics:

        fig, (axx, axd) = plt.subplots(2, 1, sharex=True)

        (x, phi, dx), time = extract_array(data_frames, topic)
        np.rad2deg(phi,out=phi)

        colors = ['tab:blue', 'tab:red']

        # max values
        factor = 1.1
        mx = np.max(np.abs(x)) * factor
        mdx = np.max(np.abs(dx)) * factor
        mphi = np.max(np.abs(phi)) * factor

        ### POSITION ###

        axx.set_ylabel('Position [m]', color=colors[0])
        axd.set_ylabel('Velocity [m/s]', color=colors[0])
        axd.set_xlabel('time [s]')

        axx.plot(time, x, label=topic.replace('state', '').strip('/') + ' x', color=colors[0])
        axd.plot(time, dx, label=topic.replace('state', '').strip('/') + ' dx', color=colors[0])

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

        axp.plot(time, phi, label=topic.replace('state', '').strip('/') + ' phi', color=colors[1])

        axp.tick_params(axis='y', labelcolor=colors[1])
        axdp.tick_params(axis='y', labelcolor=colors[1])

        ### --------- ###

        # make symmetric to align y=0
        axx.set_ylim(-mx, mx)
        axd.set_ylim(-mdx, mdx)
        axp.set_ylim(-mphi, mphi)

        axx.set_title('Position and Angles')
        axd.set_title('First Derivative of Position and Angles')

        axx.legend(loc='upper right')
        axd.legend(loc='upper right')
        axp.legend(loc='lower right')
        axdp.legend(loc='lower right')

        fig.tight_layout()
        fig.set_size_inches(15, 9)
        fig.suptitle(f'Initial State ({dir_name}): (x,phi,dx,dphi) = ({x[0]:.2f}m, {phi[0]:.2f}°, {dx[0]:.2f}m/s', fontsize=16) # maybe add degrees for phi and dphi
        plt.savefig(os.path.join(root, topic.replace('y', '').strip('/') + '_output.png'))
        plt.show()

def plot_state(data_frames, root, topics):
    dir_name = os.path.split(root)[-1]

    for topic in topics:

        fig, (axx, axd) = plt.subplots(2, 1, sharex=True)

        (x, phi, dx, dphi), time = extract_array(data_frames, topic)
        np.rad2deg(phi,out=phi)
        np.rad2deg(dphi,out=dphi)

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

        axx.plot(time, x, label=topic.replace('state', '').strip('/') + ' x', color=colors[0])
        axd.plot(time, dx, label=topic.replace('state', '').strip('/') + ' dx', color=colors[0])

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

        axp.plot(time, phi, label=topic.replace('state', '').strip('/') + ' phi', color=colors[1])
        axdp.plot(time, dphi, label=topic.replace('state', '').strip('/') + ' dphi', color=colors[1])

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
        fig.suptitle(f'Initial State ({dir_name}): (x,phi,dx,dphi) = ({x[0]:.2f}m, {phi[0]:.2f}°, {dx[0]:.2f}m/s, {dphi[0]:.2f}°/s)', fontsize=16) # maybe add degrees for phi and dphi
        plt.savefig(os.path.join(root, topic.replace('state', '').strip('/') + '_state.png'))
        plt.show()

def main():
    b_all_exps = False

    if b_all_exps:
        exp = '/home/disler/tesla_ws/src/pendulum_control/data/exps'
        for subdir in os.listdir(exp):
            subdir_path = os.path.join(exp, subdir)
            bag_file = find_bag(subdir_path)

    else:
        exp = os.getcwd()
        bag_file = find_bag(exp)

    if not bag_file:
        return
    # topic_names = ['/top/y', '/bottom/y', '/top/v_sp', '/bottom/v_sp', '/ethercat_master/Maxon_Motor_top/reading', '/ethercat_master/Maxon_Motor_bottom/reading']  # replace with your topic name
    topic_names = ['/top/state', '/bottom/state', '/top/u', '/bottom/u', '/top/vicon/phi', '/bottom/vicon/phi', '/top/v_sp', '/bottom/v_sp', '/ethercat_master/Maxon_Motor_top/reading', '/ethercat_master/Maxon_Motor_bottom/reading']  # replace with your topic name
    vsp_topics = [('/top/v_sp', '/top/v_sp_alt'), ('/bottom/v_sp', '/bottom/v_sp_alt')]
    data_frames = bag_to_pd(exp, topic_names)
    # plot_integration(data_frames, exp, vsp_topics)
    plot_angles(data_frames, exp, topic_names[4:6])
    plot_input(data_frames, exp, topic_names[2:4])
    plot_state(data_frames, exp, topic_names[0:2])

    # plot_output(data_frames, exp, topic_names[0:2])
    plot_velocity(data_frames, exp, top=True)
    plot_velocity(data_frames, exp, top=False)

if __name__ == "__main__":
    main()