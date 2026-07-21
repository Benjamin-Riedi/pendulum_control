import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class Visualization:

    def __init__(self):
        pass

    def plot_multiple_signals(self, x, ys, labels, title="Signals", 
                              xlabel="time [s]", ylabel="value"):
    
        if len(ys) != len(labels):
            raise ValueError("ys and labels must have the same length")

        fig, ax = plt.subplots(figsize=(7, 4))#, layout='constrained')

        for y, name in zip(ys, labels):
            ax.plot(x, y, label=name)

        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)

        ax.legend()
        plt.grid(True)
        plt.show()

    def plot_3d_trajectory(self, px, py, pz,
                           title="End-effector trajectory",
                           xlabel="x [m]", ylabel="y [m]", zlabel="z [m]"):

        fig = plt.figure(figsize=(6, 5))
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(px, py, pz)

        ax.scatter(px[0], py[0], pz[0], marker='o')
        ax.scatter(px[-1], py[-1], pz[-1], marker='^')

        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_zlabel(zlabel)
        ax.set_title(title)

        ax.view_init(elev=5, azim=70)

        plt.show()
    
    def animate_3d_trajectory(self, px, py, pz,
                              interval=20,
                              title="End-effector trajectory (animated)",
                              xlabel="x [m]", ylabel="y [m]", zlabel="z [m]"):

        px = np.asarray(px)
        py = np.asarray(py)
        pz = np.asarray(pz)

        fig = plt.figure(figsize=(6, 5))
        ax = fig.add_subplot(111, projection='3d')

        xmin, xmax = px.min(), px.max()
        ymin, ymax = py.min(), py.max()
        zmin, zmax = pz.min(), pz.max()

        pad_x = 0.1 * (xmax - xmin if xmax > xmin else 1.0)
        pad_y = 0.1 * (ymax - ymin if ymax > ymin else 1.0)
        pad_z = 0.1 * (zmax - zmin if zmax > zmin else 1.0)

        ax.set_xlim(xmin - pad_x, xmax + pad_x)
        ax.set_ylim(ymin - pad_y, ymax + pad_y)
        ax.set_zlim(zmin - pad_z, zmax + pad_z)

        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_zlabel(zlabel)
        ax.set_title(title)

        line, = ax.plot([], [], [], lw=2)
        point = ax.scatter([], [], [], marker='o')

        def update(frame):
            xdata = px[:frame]
            ydata = py[:frame]
            zdata = pz[:frame]

            line.set_data(xdata, ydata)
            line.set_3d_properties(zdata)

            point._offsets3d = (np.array([px[frame-1]]),
                                np.array([py[frame-1]]),
                                np.array([pz[frame-1]]))
            return line, point

        anim = FuncAnimation(fig,
                             update,
                             frames=len(px),
                             interval=interval,
                             blit=False)

        plt.show()

    def animate_pendulum_cart(self,
                               x_traj, y_traj,
                               phi_traj, alpha_traj,
                               L=0.40, m=0.50, M=1.0,
                               dt=0.001,
                               step=10,
                               interval=20,
                               title="Pendulum on cart — 3D schematic"):
        """
        Minimal schematic animation of the 3D pendulum-on-cart system.

        Parameters
        ----------
        x_traj, y_traj       : cart positions along X and Y [m]
        phi_traj, alpha_traj : pendulum angles [rad]
                               phi   = rotation around Y-axis (XZ plane)
                               alpha = rotation around X-axis (YZ plane)
        L                    : pendulum length [m]
        m, M                 : pendulum mass, cart mass [kg]
        dt                   : simulation time step [s]
        step                 : take every `step`-th sample (controls playback speed)
        interval             : animation frame interval [ms]
        title                : figure title
        """
        import numpy as np
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        from matplotlib.animation import FuncAnimation
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        # ── Subsample trajectories ────────────────────────────────────────────
        xs    = np.asarray(x_traj)[::step]
        ys    = np.asarray(y_traj)[::step]
        phis  = np.asarray(phi_traj)[::step]
        alphs = np.asarray(alpha_traj)[::step]
        N     = len(xs)

        # ── Cart geometry (half-sizes) ────────────────────────────────────────
        cw, cd, ch = 0.18, 0.10, 0.05   # half-width X, half-depth Y, height Z
        pivot_z    = ch                  # pivot is on top of cart

        # ── Figure setup ─────────────────────────────────────────────────────
        fig = plt.figure(figsize=(8, 6))
        ax  = fig.add_subplot(111, projection='3d')
        ax.set_title(title, fontsize=11)

        # fixed axis limits (generous around typical motion)
        lim = max(L * 1.5, np.max(np.abs(xs)) + 0.5, np.max(np.abs(ys)) + 0.5)
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-0.05, L * 1.4)
        ax.set_xlabel("X [m]", labelpad=4)
        ax.set_ylabel("Y [m]", labelpad=4)
        ax.set_zlabel("Z [m]", labelpad=4)
        # ax.set_box_aspect([1, 1, 0.8])
        ax.view_init(elev=18, azim=-55)

        # ── Ground grid ───────────────────────────────────────────────────────
        gv = np.linspace(-lim, lim, 9)
        for gx in gv:
            ax.plot([gx, gx], [-lim, lim], [0, 0],
                    color='gray', lw=0.4, alpha=0.25)
        for gy in gv:
            ax.plot([-lim, lim], [gy, gy], [0, 0],
                    color='gray', lw=0.4, alpha=0.25)

        # ── World reference frame arrows ──────────────────────────────────────
        arrow_len = lim * 0.35
        ax.quiver(0, 0, 0, arrow_len, 0, 0,
                  color='#E24B4A', linewidth=1.2, arrow_length_ratio=0.18)
        ax.quiver(0, 0, 0, 0, arrow_len, 0,
                  color='#1D9E75', linewidth=1.2, arrow_length_ratio=0.18)
        ax.quiver(0, 0, 0, 0, 0, arrow_len,
                  color='#378ADD', linewidth=1.2, arrow_length_ratio=0.18)
        ax.text(arrow_len * 1.12, 0, 0,      'X', color='#E24B4A', fontsize=9)
        ax.text(0, arrow_len * 1.12, 0,      'Y', color='#1D9E75', fontsize=9)
        ax.text(0, 0,      arrow_len * 1.12, 'Z', color='#378ADD', fontsize=9)

        # ── Parameter annotation ──────────────────────────────────────────────
        ax.text2D(0.02, 0.97,
                  f"L={L:.2f} m   m={m:.2f} kg   M={M:.2f} kg",
                  transform=ax.transAxes, fontsize=8,
                  color='gray', va='top')

        # ── Mutable artists ───────────────────────────────────────────────────
        def _cart_verts(cx, cy):
            """Return 6 face vertex arrays for a box centred at (cx, cy, 0)."""
            x0, x1 = cx - cw, cx + cw
            y0, y1 = cy - cd, cy + cd
            z0, z1 = 0.0,     ch
            verts = [
                [[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0]],  # bottom
                [[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]],  # top
                [[x0,y0,z0],[x1,y0,z0],[x1,y0,z1],[x0,y0,z1]],  # front
                [[x0,y1,z0],[x1,y1,z0],[x1,y1,z1],[x0,y1,z1]],  # back
                [[x0,y0,z0],[x0,y1,z0],[x0,y1,z1],[x0,y0,z1]],  # left
                [[x1,y0,z0],[x1,y1,z0],[x1,y1,z1],[x1,y0,z1]],  # right
            ]
            return verts

        cart_poly = Poly3DCollection(
            _cart_verts(0, 0),
            alpha=0.25, facecolor='steelblue', edgecolor='#444444', linewidth=0.4
        )
        ax.add_collection3d(cart_poly)

        rod_line, = ax.plot([], [], [], '-', color='#555555', lw=2.2)
        bob_dot,  = ax.plot([], [], [], 'o', color='#BA7517',
                             markersize=8, zorder=5)
        pivot_dot, = ax.plot([], [], [], 'o', color='#444444',
                              markersize=4, zorder=5)

        # Dashed projection lines (bob → cart plane)
        proj_vert, = ax.plot([], [], [], '--', color='gray',
                              lw=0.7, alpha=0.45)
        proj_horiz, = ax.plot([], [], [], '--', color='gray',
                               lw=0.7, alpha=0.35)

        # Angle arc approximations (polylines on unit sphere surface)
        arc_phi,   = ax.plot([], [], [], '-', color='#E24B4A', lw=1.2, alpha=0.7)
        arc_alpha, = ax.plot([], [], [], '-', color='#1D9E75', lw=1.2, alpha=0.7)

        # φ and α text labels
        phi_txt   = ax.text(0, 0, 0, '', color='#E24B4A', fontsize=9)
        alpha_txt = ax.text(0, 0, 0, '', color='#1D9E75', fontsize=9)

        # Quantity readout (bottom-left of axes)
        readout = ax.text2D(0.02, 0.04, '', transform=ax.transAxes,
                            fontsize=8, color='#555555', family='monospace')

        # ── Init ──────────────────────────────────────────────────────────────
        def init():
            rod_line.set_data([], [])
            rod_line.set_3d_properties([])
            bob_dot.set_data([], [])
            bob_dot.set_3d_properties([])
            return (cart_poly, rod_line, bob_dot, pivot_dot,
                    proj_vert, proj_horiz, arc_phi, arc_alpha,
                    phi_txt, alpha_txt, readout)

        # ── Update ────────────────────────────────────────────────────────────
        def update(k):
            cx, cy   = xs[k],   ys[k]
            phi, alp = phis[k], alphs[k]

            phi = np.arctan2(np.sin(phi), np.cos(phi))
            alp = np.arctan2(np.sin(alp), np.cos(alp))

            # Cart
            cart_poly.set_verts(_cart_verts(cx, cy))

            # Pivot and bob positions
            px, py, pz = cx, cy, pivot_z
            bx = px + L * np.sin(phi)
            by = py - L * np.sin(alp)
            bz = pz + L * np.cos(phi) * np.cos(alp)

            # Rod
            rod_line.set_data([px, bx], [py, by])
            rod_line.set_3d_properties([pz, bz])

            # Bob & pivot
            bob_dot.set_data([bx], [by])
            bob_dot.set_3d_properties([bz])
            pivot_dot.set_data([px], [py])
            pivot_dot.set_3d_properties([pz])

            # Projection lines
            proj_vert.set_data([bx, bx], [by, by])
            proj_vert.set_3d_properties([bz, pz])
            proj_horiz.set_data([bx, px], [by, py])
            proj_horiz.set_3d_properties([pz, pz])

            # φ arc — rotation around Y, in XZ plane
            #   t=0 → Z+ (vertical up),  t=φ → matches bob: bx = px + L*sin(φ)
            arc_r  = L * 0.30
            n_arc  = 20
            if abs(phi) > 0.01:
                t_arc  = np.linspace(0, phi, n_arc)
                ax_arc = px + arc_r * np.sin(t_arc)   # X grows with φ
                ay_arc = np.full(n_arc, py)            # Y fixed
                az_arc = pz + arc_r * np.cos(t_arc)   # Z: arc_r at t=0, shrinks as φ grows
                arc_phi.set_data(ax_arc, ay_arc)
                arc_phi.set_3d_properties(az_arc)
                mid = n_arc // 2
                # phi_txt.set_position_3d((ax_arc[mid], ay_arc[mid],
                #                          az_arc[mid] + 0.04))
                phi_txt.set_text('φ')
            else:
                arc_phi.set_data([], [])
                arc_phi.set_3d_properties([])
                phi_txt.set_text('')

            # α arc — rotation around X, in YZ plane
            #   t=0 → Z+ (vertical up),  t=α → matches bob: by = py - L*sin(α)
            if abs(alp) > 0.01:
                t_arc   = np.linspace(0, alp, n_arc)
                ax_arc2 = np.full(n_arc, px)              # X fixed
                ay_arc2 = py - arc_r * np.sin(t_arc)     # Y: consistent with by = py - L*sin(α)
                az_arc2 = pz + arc_r * np.cos(t_arc)     # Z: arc_r at t=0
                arc_alpha.set_data(ax_arc2, ay_arc2)
                arc_alpha.set_3d_properties(az_arc2)
                mid = n_arc // 2
                # alpha_txt.set_position_3d((ax_arc2[mid], ay_arc2[mid] - 0.04,
                #                             az_arc2[mid]))
                alpha_txt.set_text('α')
            else:
                arc_alpha.set_data([], [])
                arc_alpha.set_3d_properties([])
                alpha_txt.set_text('')

            # Readout
            tk = k * step * dt
            readout.set_text(
                f"t={tk:.2f}s   x={cx:.3f}m  y={cy:.3f}m"
                f"   φ={phi:.3f}rad  α={alp:.3f}rad"
            )

            return (cart_poly, rod_line, bob_dot, pivot_dot,
                    proj_vert, proj_horiz, arc_phi, arc_alpha,
                    phi_txt, alpha_txt, readout)

        anim = FuncAnimation(fig, update, frames=N,
                             init_func=init, interval=interval,
                             blit=False)
        plt.tight_layout()
        plt.show()
        return anim