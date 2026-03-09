"""
Interactive 3D visualiser for a serial manipulator using PyQtGraph.

Left column  — joint-space sliders (FK):  moving these runs FK and syncs Cartesian sliders.
Right column — Cartesian sliders (IK):    X/Y/Z position + ZYZ intrinsic Euler angles.
                                           Moving these runs full-pose IK and syncs joint sliders.
"""

import sys
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets


# ---------------------------------------------------------------------------
# STL loader
# ---------------------------------------------------------------------------

def _load_stl(path: str) -> tuple[np.ndarray, np.ndarray]:
    """Load a binary STL file.

    Returns
    -------
    vertices : float32 array, shape (3*N, 3)  — in metres (converted from mm)
    faces    : uint32  array, shape (N, 3)
    """
    with open(path, 'rb') as f:
        data = f.read()
    n = int(np.frombuffer(data, dtype=np.uint32, count=1, offset=80)[0])
    dtype = np.dtype([
        ('normal', np.float32, (3,)),
        ('v0',     np.float32, (3,)),
        ('v1',     np.float32, (3,)),
        ('v2',     np.float32, (3,)),
        ('attr',   np.uint16),
    ])
    tris = np.frombuffer(data, dtype=dtype, count=n, offset=84)
    verts = np.empty((n * 3, 3), dtype=np.float32)
    verts[0::3] = tris['v0']
    verts[1::3] = tris['v1']
    verts[2::3] = tris['v2']
    verts *= 1e-3                                          # mm → m
    faces = np.arange(n * 3, dtype=np.uint32).reshape(-1, 3)
    return verts, faces

from robot.robot import Robot
from robot.ik import inverse_kinematics
from robot.kinematics import forward_kinematics


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
# Float slider widget
# ---------------------------------------------------------------------------

class LabeledSlider(QtWidgets.QWidget):
    """A labeled float slider with an editable value field.

    Parameters
    ----------
    scale : float
        Multiply the internal value by this factor for display/input.
        E.g. ``180/pi`` to show degrees when the internal unit is radians.
    """

    valueChanged = QtCore.Signal(float)
    STEPS = 1000

    def __init__(self, label: str, lo: float, hi: float, v0: float,
                 fmt: str = "{:.3f}", scale: float = 1.0, parent=None):
        super().__init__(parent)
        self._lo    = lo
        self._hi    = hi
        self._fmt   = fmt
        self._scale = scale

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(2, 1, 2, 1)
        layout.setSpacing(4)

        lbl = QtWidgets.QLabel(label)
        lbl.setMinimumWidth(90)
        lbl.setMaximumWidth(140)

        self._slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self._slider.setRange(0, self.STEPS)

        self._val_edit = QtWidgets.QLineEdit()
        self._val_edit.setMinimumWidth(70)
        self._val_edit.setMaximumWidth(70)
        self._val_edit.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)

        layout.addWidget(lbl)
        layout.addWidget(self._slider, stretch=1)
        layout.addWidget(self._val_edit)

        self._slider.valueChanged.connect(self._on_int_changed)
        self._val_edit.editingFinished.connect(self._on_edit_finished)
        self.set_value(v0)

    def _on_int_changed(self, int_val: int):
        v = self._int_to_float(int_val)
        if not self._val_edit.hasFocus():
            self._val_edit.setText(self._fmt.format(v * self._scale))
        self.valueChanged.emit(v)

    def _on_edit_finished(self):
        try:
            v_display = float(self._val_edit.text())
            v_internal = v_display / self._scale
            v_internal = float(np.clip(v_internal, self._lo, self._hi))
            self._slider.blockSignals(True)
            self._slider.setValue(self._float_to_int(v_internal))
            self._slider.blockSignals(False)
            self._val_edit.setText(self._fmt.format(v_internal * self._scale))
            self.valueChanged.emit(v_internal)
        except ValueError:
            # Revert to current slider value
            self._val_edit.setText(self._fmt.format(self.value * self._scale))

    def _int_to_float(self, int_val: int) -> float:
        return self._lo + (int_val / self.STEPS) * (self._hi - self._lo)

    def _float_to_int(self, v: float) -> int:
        return round(np.clip((v - self._lo) / (self._hi - self._lo), 0.0, 1.0) * self.STEPS)

    @property
    def value(self) -> float:
        return self._int_to_float(self._slider.value())

    def set_value(self, v: float):
        """Set value (internal units) without emitting valueChanged."""
        self._slider.blockSignals(True)
        self._slider.setValue(self._float_to_int(v))
        self._slider.blockSignals(False)
        if not self._val_edit.hasFocus():
            self._val_edit.setText(self._fmt.format(np.clip(v, self._lo, self._hi) * self._scale))


# ---------------------------------------------------------------------------
# Visualiser
# ---------------------------------------------------------------------------

# Axis colours: X=red, Y=green, Z=blue
_AXIS_COLORS = [
    (1.0, 0.0, 0.0, 1.0),
    (0.0, 0.8, 0.0, 1.0),
    (0.0, 0.4, 1.0, 1.0),
]


class LabelledGLViewWidget(gl.GLViewWidget):
    """GLViewWidget that overlays 3D-anchored text labels via paintEvent."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._overlay_labels: list[tuple[np.ndarray, str]] = []
        self._ortho = False

    def projectionMatrix(self, region, viewport):
        if not self._ortho:
            return super().projectionMatrix(region, viewport)
        from math import tan, radians
        x0, y0, w, h = viewport
        dist    = self.opts['distance']
        fov     = self.opts['fov']
        half_h  = dist * tan(0.5 * radians(fov))
        half_w  = half_h * w / h
        left    = half_w * ((region[0] - x0) * (2.0 / w) - 1)
        right   = half_w * ((region[0] + region[2] - x0) * (2.0 / w) - 1)
        bottom  = half_h * ((region[1] - y0) * (2.0 / h) - 1)
        top     = half_h * ((region[1] + region[3] - y0) * (2.0 / h) - 1)
        near    = -dist * 1000.0
        far     =  dist * 1000.0
        tr = QtGui.QMatrix4x4()
        tr.ortho(left, right, bottom, top, near, far)
        return tr

    def set_labels(self, labels: list[tuple[np.ndarray, str]]):
        self._overlay_labels = labels
        self.update()

    def _project(self, pos: np.ndarray) -> QtCore.QPointF | None:
        w, h = self.width(), self.height()
        if w == 0 or h == 0:
            return None
        region   = (0, 0, w, h)
        viewport = (0, 0, w, h)
        proj = self.projectionMatrix(region, viewport)
        mv   = self.viewMatrix()
        mvp  = proj * mv
        v = QtGui.QVector4D(float(pos[0]), float(pos[1]), float(pos[2]), 1.0)
        clip = mvp.map(v)
        if abs(clip.w()) < 1e-6:
            return None
        xn = clip.x() / clip.w()
        yn = clip.y() / clip.w()
        sx = (xn + 1.0) / 2.0 * w
        sy = (1.0 - yn) / 2.0 * h
        return QtCore.QPointF(sx, sy)

    def paintEvent(self, event):
        super().paintEvent(event)
        if not self._overlay_labels:
            return
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.TextAntialiasing)
        painter.setFont(QtGui.QFont("Helvetica", 11, QtGui.QFont.Weight.Bold))
        for pos3d, text in self._overlay_labels:
            pt = self._project(pos3d)
            if pt is None:
                continue
            # draw dark outline for contrast
            painter.setPen(QtGui.QColor(0, 0, 0, 200))
            for dx, dy in ((-1, -1), (1, -1), (-1, 1), (1, 1)):
                painter.drawText(pt + QtCore.QPointF(dx, dy), text)
            painter.setPen(QtGui.QColor(255, 220, 0, 255))
            painter.drawText(pt, text)
        painter.end()


class RobotVisualiser:
    AXIS_LEN = 0.12
    # Light silver-grey to mimic the Meca500's appearance
    _MESH_COLOR = (0.80, 0.80, 0.84, 1.0)

    def __init__(self, robot: Robot, stl_map: list[tuple[str, int]] | None = None):
        """
        Parameters
        ----------
        robot   : Robot instance
        stl_map : list of (path, frame_index) pairs.
                  Each STL file is attached to the DH frame at *frame_index*
                  (0 = world/base, 1 = after joint 1, …, n = end-effector).
                  When provided the skeleton lines/dots are hidden.
        """
        self.robot = robot
        self._stl_map = stl_map or []
        self._mesh_items: list[tuple[gl.GLMeshItem, int, np.ndarray]] = []
        self._updating = False
        self._app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
        self._build_gui()

    # ------------------------------------------------------------------
    # GUI construction
    # ------------------------------------------------------------------

    def _build_gui(self):
        self._win = QtWidgets.QMainWindow()
        self._win.setWindowTitle(self.robot.name)
        self._win.resize(1300, 750)

        central = QtWidgets.QWidget()
        self._win.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setSpacing(8)

        # ---- Left: 3D view + info panel ----
        left_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)

        self._view = LabelledGLViewWidget()
        self._view.setMinimumSize(600, 450)
        self._view.setCameraPosition(distance=self._max_reach() * 3.0, elevation=30, azimuth=45)

        grid = gl.GLGridItem()
        grid.scale(0.5, 0.5, 0.5)
        self._view.addItem(grid)

        self._info_label = QtWidgets.QLabel()
        self._info_label.setFont(QtGui.QFont("Monospace", 9))
        self._info_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignTop | QtCore.Qt.AlignmentFlag.AlignLeft)
        self._info_label.setMinimumHeight(140)

        left_layout.addWidget(self._view, stretch=3)
        left_layout.addWidget(self._info_label, stretch=1)

        # ---- Right: slider panels stacked vertically ----
        right_widget = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self._proj_btn = QtWidgets.QPushButton("Switch to Orthographic")
        self._proj_btn.setCheckable(True)
        self._proj_btn.toggled.connect(self._on_proj_toggle)
        right_layout.addWidget(self._proj_btn)
        right_layout.addWidget(self._build_view_buttons())
        right_layout.addWidget(self._build_joint_panel())
        right_layout.addWidget(self._build_cart_panel())

        main_layout.addWidget(left_widget, stretch=2)
        main_layout.addWidget(right_widget, stretch=1)

        # 3D item handles (created lazily on first _draw)
        self._link_item:  gl.GLLinePlotItem    | None = None
        self._joint_item: gl.GLScatterPlotItem | None = None
        self._frame_items: list[gl.GLLinePlotItem] = []

        # Load STL meshes — compute zero-config correction so world-space
        # STL geometry maps correctly to each DH frame.
        if self._stl_map:
            T_zero = forward_kinematics(self.robot.joints, np.zeros(self.robot.n_dof))
        for path, frame_idx in self._stl_map:
            try:
                verts, faces = _load_stl(path)
                md   = gl.MeshData(vertexes=verts, faces=faces)
                item = gl.GLMeshItem(meshdata=md, smooth=False,
                                     color=self._MESH_COLOR,
                                     shader='shaded',
                                     drawFaces=True, drawEdges=False,
                                     glOptions='opaque')
                self._view.addItem(item)
                # T_correction maps STL world-space coords into DH frame coords
                T_correction = np.linalg.inv(T_zero[frame_idx])
                self._mesh_items.append((item, frame_idx, T_correction))
            except Exception as exc:
                print(f"Warning: could not load {path}: {exc}")

        self._draw()

    def _build_joint_panel(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Joint Angles (FK)")
        layout = QtWidgets.QVBoxLayout(box)
        self._joint_sliders: list[LabeledSlider] = []

        for i, joint in enumerate(self.robot.joints):
            label = joint.name if joint.name else f"q{i + 1}"
            lo, hi = joint.limits
            if joint.joint_type == "prismatic":
                s = LabeledSlider(f"{label} (m)", lo, hi, self.robot.q[i],
                                  fmt="{:.4f}", scale=1.0)
            else:
                s = LabeledSlider(f"{label} (deg)", lo, hi, self.robot.q[i],
                                  fmt="{:.2f}", scale=180.0 / np.pi)
            s.valueChanged.connect(self._on_joint_slider)
            layout.addWidget(s)
            self._joint_sliders.append(s)

        layout.addStretch()

        reset_btn = QtWidgets.QPushButton("Reset")
        reset_btn.clicked.connect(self._on_reset)
        layout.addWidget(reset_btn)
        return box

    def _build_view_buttons(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        layout.addWidget(QtWidgets.QLabel("View:"))
        # (label, elevation, azimuth)
        for label, elev, azim in [("X", 0, 90), ("Y", 0, 0), ("Z", 90, 0)]:
            btn = QtWidgets.QPushButton(label)
            btn.setFixedWidth(36)
            btn.clicked.connect(lambda _, e=elev, a=azim: self._view.setCameraPosition(elevation=e, azimuth=a))
            layout.addWidget(btn)
        layout.addStretch()
        return widget

    def _build_cart_panel(self) -> QtWidgets.QGroupBox:
        reach = self._max_reach() * 2
        T0    = self.robot.end_effector_pose()
        pos0  = T0[:3, 3]
        a0, b0, g0 = _rot_to_zyz(T0[:3, :3])

        box = QtWidgets.QGroupBox("Cartesian (IK)")
        box_layout = QtWidgets.QVBoxLayout(box)
        self._cart_sliders: list[LabeledSlider] = []

        # Position sub-group
        pos_box = QtWidgets.QGroupBox("Position")
        pos_layout = QtWidgets.QVBoxLayout(pos_box)
        for label, lo, hi, v0 in [
            ("X (m)", -reach, reach, pos0[0]),
            ("Y (m)", -reach, reach, pos0[1]),
            ("Z (m)", -reach, reach, pos0[2]),
        ]:
            s = LabeledSlider(label, lo, hi, v0, fmt="{:.4f}")
            s.valueChanged.connect(self._on_cart_slider)
            pos_layout.addWidget(s)
            self._cart_sliders.append(s)

        # Orientation sub-group
        ori_box = QtWidgets.QGroupBox("Orientation — ZYZ intrinsic")
        ori_layout = QtWidgets.QVBoxLayout(ori_box)
        for label, lo, hi, v0 in [
            ("\u03b1 ZYZ (deg)", -np.pi, np.pi, a0),
            ("\u03b2 ZYZ (deg)",  0.0,   np.pi, b0),
            ("\u03b3 ZYZ (deg)", -np.pi, np.pi, g0),
        ]:
            s = LabeledSlider(label, lo, hi, v0, fmt="{:.2f}", scale=180.0 / np.pi)
            s.valueChanged.connect(self._on_cart_slider)
            ori_layout.addWidget(s)
            self._cart_sliders.append(s)

        box_layout.addWidget(pos_box)
        box_layout.addWidget(ori_box)
        box_layout.addStretch()
        return box

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _max_reach(self) -> float:
        return max(0.5, sum(abs(j.a) + abs(j.d) for j in self.robot.joints))

    def _ee_cartesian(self) -> tuple[np.ndarray, float, float, float]:
        T = self.robot.end_effector_pose()
        return T[:3, 3], *_rot_to_zyz(T[:3, :3])

    def _sync_cart_sliders(self):
        pos, a, b, g = self._ee_cartesian()
        for s, v in zip(self._cart_sliders, [*pos, a, b, g]):
            s.set_value(v)

    def _sync_joint_sliders(self, q: np.ndarray):
        for s, v in zip(self._joint_sliders, q):
            s.set_value(v)

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_joint_slider(self, _val=None):
        if self._updating:
            return
        self.robot.q = np.array([s.value for s in self._joint_sliders])
        self._updating = True
        self._sync_cart_sliders()
        self._updating = False
        self._draw()

    def _on_cart_slider(self, _val=None):
        if self._updating:
            return
        vals = [s.value for s in self._cart_sliders]
        T_target = np.eye(4)
        T_target[:3, :3] = _zyz_to_rot(*vals[3:])
        T_target[:3, 3]  = vals[:3]
        q_sol = self._solve_ik(T_target)
        self.robot.q = q_sol
        self._updating = True
        self._sync_joint_sliders(q_sol)
        self._updating = False
        self._draw()

    def _solve_ik(self, T_target: np.ndarray) -> np.ndarray:
        if hasattr(self.robot, "analytical_ik"):
            limits = [j.limits for j in self.robot.joints]
            q_sol = self.robot.analytical_ik(T_target, self.robot.q, limits)
            if q_sol is not None:
                return q_sol
        q_sol, _ = inverse_kinematics(self.robot, T_target[:3, 3], target_rot=T_target[:3, :3])
        return q_sol

    def _on_proj_toggle(self, checked: bool):
        self._view._ortho = checked
        self._proj_btn.setText("Switch to Perspective" if checked else "Switch to Orthographic")
        self._view.update()

    def _on_reset(self):
        self.robot.q = np.zeros(self.robot.n_dof)
        self._updating = True
        for s in self._joint_sliders:
            s.set_value(0.0)
        self._sync_cart_sliders()
        self._updating = False
        self._draw()

    # ------------------------------------------------------------------
    # 3D drawing
    # ------------------------------------------------------------------

    def _draw(self):
        transforms = self.robot.transforms()
        positions  = np.array([T[:3, 3] for T in transforms], dtype=np.float32)
        if not self._mesh_items:
            self._draw_links(positions)
            self._draw_joints(positions)
        self._draw_frames(transforms)
        self._draw_labels(positions)
        self._draw_meshes(transforms)
        self._update_info()

    def _draw_links(self, positions: np.ndarray):
        color = (0.27, 0.51, 0.71, 1.0)  # steelblue
        if self._link_item is None:
            self._link_item = gl.GLLinePlotItem(
                pos=positions, color=color, width=3, antialias=True, mode="line_strip"
            )
            self._view.addItem(self._link_item)
        else:
            self._link_item.setData(pos=positions, color=color, width=3)

    def _draw_joints(self, positions: np.ndarray):
        n = len(positions)
        colors = np.tile([0.27, 0.51, 0.71, 1.0], (n, 1)).astype(np.float32)
        colors[0]  = [0.0, 0.0, 0.0, 1.0]   # base: black
        colors[-1] = [1.0, 0.65, 0.0, 1.0]  # EE: orange
        sizes = np.full(n, 10.0)
        sizes[0] = 14.0

        if self._joint_item is None:
            self._joint_item = gl.GLScatterPlotItem(
                pos=positions, color=colors, size=sizes, pxMode=True
            )
            self._view.addItem(self._joint_item)
        else:
            self._joint_item.setData(pos=positions, color=colors, size=sizes)

    def _draw_frames(self, transforms: list[np.ndarray]):
        L = self.AXIS_LEN
        needed = len(transforms) * 3  # 3 axes per frame

        # Add missing items
        while len(self._frame_items) < needed:
            axis_idx = len(self._frame_items) % 3
            item = gl.GLLinePlotItem(
                pos=np.zeros((2, 3), dtype=np.float32),
                color=_AXIS_COLORS[axis_idx],
                width=1.5,
                antialias=True,
            )
            self._view.addItem(item)
            self._frame_items.append(item)

        # Remove excess items (DOF changed)
        while len(self._frame_items) > needed:
            self._view.removeItem(self._frame_items.pop())

        idx = 0
        for T in transforms:
            origin = T[:3, 3].astype(np.float32)
            for j in range(3):
                end = (origin + T[:3, j] * L).astype(np.float32)
                self._frame_items[idx].setData(pos=np.array([origin, end], dtype=np.float32))
                idx += 1

    def _draw_labels(self, positions: np.ndarray):
        labels = []
        for i in range(len(positions) - 1):
            mid = (positions[i] + positions[i + 1]) / 2
            name = self.robot.joints[i].name if self.robot.joints[i].name else f"L{i + 1}"
            labels.append((mid, name))
        self._view.set_labels(labels)

    def _draw_meshes(self, transforms: list[np.ndarray]):
        for item, frame_idx, T_correction in self._mesh_items:
            T = transforms[frame_idx] @ T_correction
            tr = pg.Transform3D(
                float(T[0,0]), float(T[0,1]), float(T[0,2]), float(T[0,3]),
                float(T[1,0]), float(T[1,1]), float(T[1,2]), float(T[1,3]),
                float(T[2,0]), float(T[2,1]), float(T[2,2]), float(T[2,3]),
                float(T[3,0]), float(T[3,1]), float(T[3,2]), float(T[3,3]),
            )
            item.setTransform(tr)

    def _update_info(self):
        T   = self.robot.end_effector_pose()
        pos = T[:3, 3]
        a, b, g = _rot_to_zyz(T[:3, :3])
        q = self.robot.q
        lines = [
            "<b>End-effector position:</b>",
            f"  x = {pos[0]:+.4f} m",
            f"  y = {pos[1]:+.4f} m",
            f"  z = {pos[2]:+.4f} m",
            "",
            "<b>Orientation (ZYZ):</b>",
            f"  \u03b1 = {a:+.4f} rad",
            f"  \u03b2 = {b:+.4f} rad",
            f"  \u03b3 = {g:+.4f} rad",
            "",
            "<b>Joint angles:</b>",
        ]
        for joint, qi in zip(self.robot.joints, q):
            name = joint.name if joint.name else "q"
            if joint.joint_type == "prismatic":
                lines.append(f"  {name}: {qi:+.4f} m")
            else:
                lines.append(f"  {name}: {np.degrees(qi):+.2f} deg")
        self._info_label.setText("<br>".join(lines))

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------

    def show(self):
        self._win.show()
        self._app.exec()
