"""
Interactive 3D visualiser for a serial manipulator.

Left column  — joint-space sliders (FK):  moving these runs FK and syncs Cartesian sliders.
Right column — Cartesian sliders (IK):    X/Y/Z position + ZYZ intrinsic Euler angles.
                                           Moving these runs full-pose IK and syncs joint sliders.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D

from robot.robot import Robot
from robot.ik import inverse_kinematics


_FRAME_COLORS = ["r", "g", "b"]


# ---------------------------------------------------------------------------
# ZYZ Euler helpers
# ---------------------------------------------------------------------------

def _rot_to_zyz(R: np.ndarray) -> tuple[float, float, float]:
    """Extract ZYZ intrinsic Euler angles (alpha, beta, gamma) from R.

    Convention: R = Rz(alpha) @ Ry(beta) @ Rz(gamma)
    Ranges: alpha in [-pi, pi], beta in [0, pi], gamma in [-pi, pi]
    """
    sb = np.sqrt(R[2, 0] ** 2 + R[2, 1] ** 2)
    if sb < 1e-8:
        # Gimbal lock: beta = 0 or pi
        beta = 0.0 if R[2, 2] > 0 else np.pi
        alpha = 0.0
        gamma = np.arctan2(-R[1, 0], R[0, 0]) if R[2, 2] > 0 else np.arctan2(R[1, 0], -R[0, 0])
    else:
        beta  = np.arctan2(sb, R[2, 2])
        alpha = np.arctan2(R[1, 2], R[0, 2])
        gamma = np.arctan2(R[2, 1], -R[2, 0])
    return float(alpha), float(beta), float(gamma)


def _zyz_to_rot(alpha: float, beta: float, gamma: float) -> np.ndarray:
    """ZYZ intrinsic Euler angles to 3x3 rotation matrix."""
    ca, sa = np.cos(alpha), np.sin(alpha)
    cb, sb = np.cos(beta),  np.sin(beta)
    cg, sg = np.cos(gamma), np.sin(gamma)
    Rza = np.array([[ca, -sa, 0], [sa,  ca, 0], [0, 0, 1]])
    Ryb = np.array([[cb,  0, sb], [ 0,   1, 0], [-sb, 0, cb]])
    Rzg = np.array([[cg, -sg, 0], [sg,  cg, 0], [0, 0, 1]])
    return Rza @ Ryb @ Rzg


# ---------------------------------------------------------------------------
# Visualiser
# ---------------------------------------------------------------------------

class RobotVisualiser:
    AXIS_LEN = 0.12

    def __init__(self, robot: Robot):
        self.robot = robot
        self._updating = False
        self._build_gui()

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------

    def _build_gui(self):
        n = self.robot.n_dof

        # ---- Physical layout constants (inches) -------------------------
        ROW_H_IN     = 0.30   # height of each slider bar
        ROW_GAP_IN   = 0.13   # gap between consecutive sliders
        GROUP_GAP_IN = 0.30   # extra gap between position / orientation groups
        HEADER_IN    = 0.38   # space for column header label
        BUTTON_IN    = 0.45   # space for reset button at bottom
        VIEW_H_IN    = 5.5    # height reserved for 3D view + info panel
        FIG_W_IN     = 13.0
        # -----------------------------------------------------------------

        N_RIGHT  = 6                   # 3 position + 3 orientation sliders
        n_rows   = max(n, N_RIGHT)

        slider_area_in = (
            n_rows * (ROW_H_IN + ROW_GAP_IN)
            + GROUP_GAP_IN             # extra gap in right column
            + HEADER_IN
            + BUTTON_IN
        )
        fig_h = VIEW_H_IN + slider_area_in

        self.fig = plt.figure(figsize=(FIG_W_IN, fig_h))
        self.fig.canvas.manager.set_window_title(self.robot.name)

        # Convert physical measurements to normalised figure fractions
        F          = fig_h             # shorthand
        row_h      = ROW_H_IN    / F
        row_gap    = ROW_GAP_IN  / F
        group_gap  = GROUP_GAP_IN / F
        header_h   = HEADER_IN   / F
        button_h   = BUTTON_IN   / F
        slider_frac = slider_area_in / F   # fraction of figure used by slider area

        top_bot = slider_frac + 0.03   # bottom edge of the 3D / info area

        self.ax3d: Axes3D = self.fig.add_axes(
            [0.03, top_bot, 0.52, 1.0 - top_bot - 0.04],
            projection="3d",
        )
        self.ax_info = self.fig.add_axes(
            [0.60, top_bot, 0.37, 1.0 - top_bot - 0.04]
        )
        self.ax_info.axis("off")
        self._info_text = self.ax_info.text(
            0.02, 0.98, "", transform=self.ax_info.transAxes,
            va="top", ha="left", fontsize=9, family="monospace",
        )

        self._joint_sliders: list[Slider] = []
        self._cart_sliders:  list[Slider] = []   # [x, y, z, alpha, beta, gamma]

        # col_top: normalised y of the top of the slider area
        col_top = slider_frac - 0.01
        self._build_joint_sliders(col_top, row_h, row_gap, header_h)
        self._build_cart_sliders(col_top, row_h, row_gap, group_gap, header_h)
        self._build_reset_button(button_h)

        self._draw()

    def _build_joint_sliders(self, col_top, row_h, row_gap, header_h):
        left, width = 0.07, 0.37
        self.fig.text(left + width / 2, col_top - header_h * 0.4,
                      "Joint angles (FK)", ha="center", fontsize=9,
                      fontweight="bold", color="steelblue")

        for i, joint in enumerate(self.robot.joints):
            bot = col_top - header_h - (i + 1) * (row_h + row_gap) + row_gap
            ax_s = self.fig.add_axes([left, bot, width, row_h])
            label = joint.name if joint.name else f"q{i + 1}"
            unit = "m" if joint.joint_type == "prismatic" else "rad"
            lo, hi = joint.limits
            s = Slider(ax_s, label, lo, hi,
                       valinit=self.robot.q[i],
                       valfmt=f"%.3f {unit}")
            s.on_changed(self._on_joint_slider)
            self._joint_sliders.append(s)

    def _build_cart_sliders(self, col_top, row_h, row_gap, group_gap, header_h):
        left, width = 0.57, 0.37
        reach = self._max_reach()
        T0 = self.robot.end_effector_pose()
        pos0 = T0[:3, 3]
        a0, b0, g0 = _rot_to_zyz(T0[:3, :3])

        self.fig.text(left + width / 2, col_top - header_h * 0.4,
                      "Cartesian (IK)", ha="center", fontsize=9,
                      fontweight="bold", color="darkorange")

        pos_specs = [
            ("X (m)", -reach, reach, pos0[0]),
            ("Y (m)", -reach, reach, pos0[1]),
            ("Z (m)", -reach, reach, pos0[2]),
        ]
        ori_specs = [
            ("\u03b1  ZYZ (rad)", -np.pi, np.pi, a0),
            ("\u03b2  ZYZ (rad)",  0.0,   np.pi, b0),
            ("\u03b3  ZYZ (rad)", -np.pi, np.pi, g0),
        ]

        # Position sliders (rows 0-2)
        for i, (label, lo, hi, v0) in enumerate(pos_specs):
            bot = col_top - header_h - (i + 1) * (row_h + row_gap) + row_gap
            ax_s = self.fig.add_axes([left, bot, width, row_h])
            s = Slider(ax_s, label, lo, hi, valinit=v0, valfmt="%.4f")
            s.on_changed(self._on_cart_slider)
            self._cart_sliders.append(s)

        # Orientation group separator label
        ori_top = col_top - header_h - 3 * (row_h + row_gap) - group_gap
        self.fig.text(left + width / 2, ori_top + row_h * 0.8,
                      "Orientation \u2014 ZYZ intrinsic", ha="center",
                      fontsize=8, color="dimgray", style="italic")

        # Orientation sliders (rows 3-5, shifted down by group_gap)
        for i, (label, lo, hi, v0) in enumerate(ori_specs):
            bot = ori_top - (i + 1) * (row_h + row_gap) + row_gap
            ax_s = self.fig.add_axes([left, bot, width, row_h])
            s = Slider(ax_s, label, lo, hi, valinit=v0, valfmt="%.4f rad")
            s.on_changed(self._on_cart_slider)
            self._cart_sliders.append(s)

    def _build_reset_button(self, button_h):
        ax_btn = self.fig.add_axes([0.07, button_h * 0.15, 0.10, button_h * 0.55])
        self.btn_reset = Button(ax_btn, "Reset")
        self.btn_reset.on_clicked(self._on_reset)

    def _max_reach(self) -> float:
        return max(0.5, sum(abs(j.a) + abs(j.d) for j in self.robot.joints))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _ee_cartesian(self) -> tuple[np.ndarray, float, float, float]:
        """Return (pos, alpha, beta, gamma) for the current EE pose."""
        T = self.robot.end_effector_pose()
        return T[:3, 3], *_rot_to_zyz(T[:3, :3])

    def _sync_cart_sliders(self):
        pos, a, b, g = self._ee_cartesian()
        vals = [*pos, a, b, g]
        for s, v in zip(self._cart_sliders, vals):
            s.set_val(np.clip(v, s.valmin, s.valmax))

    def _sync_joint_sliders(self, q: np.ndarray):
        for s, v in zip(self._joint_sliders, q):
            s.set_val(v)

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_joint_slider(self, _val):
        if self._updating:
            return
        self.robot.q = np.array([s.val for s in self._joint_sliders])
        self._updating = True
        self._sync_cart_sliders()
        self._updating = False
        self._draw()

    def _on_cart_slider(self, _val):
        if self._updating:
            return
        vals = [s.val for s in self._cart_sliders]
        target_pos = np.array(vals[:3])
        target_rot = _zyz_to_rot(*vals[3:])
        T_target = np.eye(4)
        T_target[:3, :3] = target_rot
        T_target[:3, 3]  = target_pos

        q_sol = self._solve_ik(T_target)
        self.robot.q = q_sol
        self._updating = True
        self._sync_joint_sliders(q_sol)
        self._updating = False
        self._draw()

    def _solve_ik(self, T_target: np.ndarray) -> np.ndarray:
        """Use analytical IK if available, otherwise fall back to numerical."""
        if hasattr(self.robot, "analytical_ik"):
            limits = [j.limits for j in self.robot.joints]
            q_sol = self.robot.analytical_ik(T_target, self.robot.q, limits)
            if q_sol is not None:
                return q_sol
        # Numerical fallback
        q_sol, _ = inverse_kinematics(
            self.robot, T_target[:3, 3], target_rot=T_target[:3, :3]
        )
        return q_sol

    def _on_reset(self, _event):
        self.robot.q = np.zeros(self.robot.n_dof)
        self._updating = True
        for s in self._joint_sliders:
            s.set_val(0.0)
        self._sync_cart_sliders()
        self._updating = False
        self._draw()

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------

    def _draw(self):
        self.ax3d.cla()
        transforms = self.robot.transforms()
        positions = np.array([T[:3, 3] for T in transforms])
        self._draw_links(positions)
        self._draw_joints(positions)
        self._draw_frames(transforms)
        self._style_axes(positions)
        self._update_info()
        self.fig.canvas.draw_idle()

    def _draw_links(self, positions: np.ndarray):
        for i in range(len(positions) - 1):
            self.ax3d.plot(
                [positions[i, 0], positions[i + 1, 0]],
                [positions[i, 1], positions[i + 1, 1]],
                [positions[i, 2], positions[i + 1, 2]],
                "o-", color="steelblue", linewidth=4,
                markersize=8, markerfacecolor="white",
                markeredgecolor="steelblue", markeredgewidth=2,
                solid_capstyle="round", zorder=3,
            )

    def _draw_joints(self, positions: np.ndarray):
        self.ax3d.scatter(*positions[0],  s=120, c="black",  zorder=5, depthshade=False)
        self.ax3d.scatter(*positions[-1], s=100, c="orange", zorder=5, depthshade=False)

    def _draw_frames(self, transforms: list[np.ndarray]):
        L = self.AXIS_LEN
        for T in transforms:
            origin = T[:3, 3]
            for j, color in enumerate(_FRAME_COLORS):
                self.ax3d.quiver(*origin, *(T[:3, j] * L),
                                 color=color, linewidth=1.2,
                                 arrow_length_ratio=0.25, alpha=0.8)

    def _style_axes(self, positions: np.ndarray):
        extent = np.max(np.linalg.norm(positions, axis=1)) * 1.3 + 0.1
        self.ax3d.set_xlim(-extent, extent)
        self.ax3d.set_ylim(-extent, extent)
        self.ax3d.set_zlim(-0.05, extent * 1.5)
        self.ax3d.set_xlabel("X (m)", fontsize=8)
        self.ax3d.set_ylabel("Y (m)", fontsize=8)
        self.ax3d.set_zlabel("Z (m)", fontsize=8)
        self.ax3d.set_title(self.robot.name, fontsize=10)
        self.ax3d.tick_params(labelsize=7)

    def _update_info(self):
        T = self.robot.end_effector_pose()
        pos = T[:3, 3]
        a, b, g = _rot_to_zyz(T[:3, :3])
        q = self.robot.q
        lines = [
            "End-effector position:",
            f"  x = {pos[0]:+.4f} m",
            f"  y = {pos[1]:+.4f} m",
            f"  z = {pos[2]:+.4f} m",
            "",
            "Orientation (ZYZ):",
            f"  \u03b1 = {a:+.4f} rad",
            f"  \u03b2 = {b:+.4f} rad",
            f"  \u03b3 = {g:+.4f} rad",
            "",
            "Joint angles:",
        ]
        for joint, qi in zip(self.robot.joints, q):
            unit = "m" if joint.joint_type == "prismatic" else "rad"
            name = joint.name if joint.name else "q"
            lines.append(f"  {name}: {qi:+.4f} {unit}")
        self._info_text.set_text("\n".join(lines))

    def show(self):
        plt.show()
