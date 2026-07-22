import numpy as np
from pendulum_on_cart import Plant
from controllers import ProportionalController as velocity_controller
from controllers import LQR
from plotting import Visualization 

def plant_simulation(plant):
    dt = plant.dt
    N = plant.N
    T = N*dt
    t = np.linspace(0, T, N+1)

    z_x = np.zeros([4, N+1])
    z_y = np.zeros([4, N+1])
    F_x = np.zeros([N,])
    F_y = np.zeros([N,])
    u_x = np.zeros([N,])
    u_y = np.zeros([N,])

    z_x[:,0] = plant.z0_x
    z_y[:,0] = plant.z0_y

    Cv_x = velocity_controller(dt)
    Cv_y = velocity_controller(dt)

    LQR_x = LQR(plant, 'x')
    LQR_y = LQR(plant, 'y')


    '''
    a = 0.01
    omega = 1
    v_ref_y = a*np.sin(t*omega)
    v_ref_x = np.zeros((N+1,))
    '''

    prev_err_x = 0
    prev_err_y = 0
    prev_u_x = 0
    prev_u_y = 0

    z_x_ref = np.array([0, 0, 0, 0])
    z_y_ref = np.array([0, 0, 0, 0])

    v_x_ref = 0
    v_y_ref = 0

    for k in range (0, N):
        e_x = z_x_ref - z_x[:,k]
        e_y = z_y_ref - z_y[:,k]

        u_x = LQR_x.compute(e_x)
        u_y = LQR_y.compute(e_y)

        v_x_ref += u_x*dt
        v_y_ref += u_y*dt

        e_v_x = v_x_ref - z_x[2,k]
        e_v_y = v_y_ref - z_y[2,k]


        F_x[k] = Cv_x.compute(e_v_x, prev_err_x, prev_u_x)
        F_y[k] = Cv_y.compute(e_v_y, prev_err_y, prev_u_y)
        z_x[:,k+1], z_y[:,k+1] = plant.update(F_x[k], F_y[k], k)
        prev_err_x = e_v_x
        prev_err_y = e_v_y
        prev_u_x = F_x[k]
        prev_u_y = F_y[k]

    x = z_x[0,:]
    phi = z_x[1,:]
    x_dot = z_x[2,:]
    phi_dot = z_x[3,:]

    y = z_y[0,:]
    alpha = z_y[1,:]
    y_dot = z_y[2,:]
    alpha_dot = z_y[3,:]

    vis = Visualization()
    
    vis.plot_multiple_signals(t, [phi,alpha], [r'$phi$: pendulum rotation around y', r'$\alpha$: pendulum rotation around x'],
                            title="Pendulum Rotations",
                            xlabel="time [s]",
                            ylabel="rotation [rad]")

    vis.plot_multiple_signals(t, [x,y], [r'$x$: cart translation along x', r'$\alpha$: cart translation along y'],
                            title="Cart Translations",
                            xlabel="time [s]",
                            ylabel="position [m]")
    vis.plot_multiple_signals(t, [phi_dot,alpha_dot], [r'$phi$: pendulum rotation around y', r'$\alpha$: pendulum rotation around x'],
                            title="Pendulum Rotations",
                            xlabel="time [s]",
                            ylabel="rotation [rad]")
    vis.plot_multiple_signals(t, [x_dot,y_dot], [r'$\dot{x}$: speed along x', r'$\dot{y}$: speed along y'],
                            title="Cart Translations",
                            xlabel="time [s]",
                            ylabel="position [m]")

    vis.animate_pendulum_cart(x, y, phi, alpha,
                          L=L, m=m, M=M, dt=dt,
                          step=20, interval=20)


def simulation_decoupled(plant):
    dt = plant.dt
    N = plant.N
    T = N*dt
    t = np.linspace(0, T, N+1)

    z_x = np.zeros([4, N+1])
    z_y = np.zeros([4, N+1])

    # Pre - Allocation of LQR Control Actions
    u_x = np.zeros([N,])
    u_y = np.zeros([N,])

    # Inizialitation of Cart Velocity Controllers
    Cv_x = velocity_controller(dt)
    Cv_y = velocity_controller(dt)

    # Initialization of LQR controllers
    LQR_x = LQR(plant, 'x')
    LQR_y = LQR(plant, 'y')

    # State initialization 
    z_x[:,0] = plant.z0_x
    z_y[:,0] = plant.z0_y

    prev_err_x = 0
    prev_err_y = 0
    prev_u_x = 0
    prev_u_y = 0

    # Reference State
    z_x_ref = np.array([0, 0, 0, 0])
    z_y_ref = np.array([0, 0, 0, 0])

    # Inizialization of Velocity References
    v_x_ref = 0
    v_y_ref = 0

    for k in range (0, N):

        e_x = z_x_ref - z_x[:,k]
        e_y = z_y_ref - z_y[:,k]

        u_x = LQR_x.compute(e_x)
        u_y = LQR_y.compute(e_y)

        v_x_ref += u_x*dt
        v_y_ref += u_y*dt

        e_v_x = v_x_ref - z_x[2,k]
        e_v_y = v_y_ref - z_y[2,k]

        F_x = Cv_x.compute(e_v_x, prev_err_x, prev_u_x)
        F_y = Cv_y.compute(e_v_y, prev_err_y, prev_u_y)
        z_x[:,k+1], z_y[:,k+1] = plant.update(F_x, F_y, k)
        prev_err_x = e_v_x
        prev_err_y = e_v_y
        prev_u_x = F_x
        prev_u_y = F_y

    x = z_x[0,:]
    phi = z_x[1,:]
    x_dot = z_x[2,:]
    phi_dot = z_x[3,:]

    y = z_y[0,:]
    alpha = z_y[1,:]
    y_dot = z_y[2,:]
    alpha_dot = z_y[3,:]

    vis = Visualization()
    
    vis.plot_multiple_signals(t, [np.rad2deg(phi),np.rad2deg(alpha)], [r'$\phi$: pendulum rotation around y', r'$\alpha$: pendulum rotation around x'],
                            title="Pendulum Rotations",
                            xlabel="time [s]",
                            ylabel="rotation [°]")

    vis.plot_multiple_signals(t, [x,y], [r'$x$: cart translation along x', r'$\alpha$: cart translation along y'],
                            title="Cart Translations",
                            xlabel="time [s]",
                            ylabel="position [m]")
    vis.plot_multiple_signals(t, [np.rad2deg(phi_dot),np.rad2deg(alpha_dot)], [r'$\dot{\phi}$: angular speed around y', r'$\dot{\omega}$: angular speed around x'],
                            title="Pendulum Angular Speed",
                            xlabel="time [s]",
                            ylabel="[°/s]")

    vis.plot_multiple_signals(t, [x_dot,y_dot], [r'$\dot{x}$: speed along x', r'$\dot{y}$: speed along y'],
                            title="Cart Speeds",
                            xlabel="time [s]",
                            ylabel="speed [m/s]")

    vis.animate_pendulum_cart(x, y, phi, alpha,
                          L=L, m=m, M=M, dt=dt,
                          step=20, interval=20)
    

     



if __name__ == "__main__":
    z_0_x = np.array([0,np.deg2rad(10),0,0])
    z_0_y = np.array([0,np.deg2rad(10),0,0])
    dt = 0.001
    N = int(4/dt)
    M = 0.074
    m = 0.253
    L = 0.14
    d = 0.0
    


    plant = Plant(z_0_x, z_0_y, N, dt, m, M, L, d)
    simulation_decoupled(plant)

    

