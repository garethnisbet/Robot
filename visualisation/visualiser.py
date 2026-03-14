"""
Interactive 3D visualiser for a serial manipulator using NiceGUI.

Left  — 3D scene with STL meshes and frame axes.
Right — Joint sliders (FK) or Cartesian sliders (IK), toggled by radio.
Bottom — end-effector readout + toolbar.
"""

import os
import numpy as np
from nicegui import app, ui

from robot.robot import Robot
from robot.ik import inverse_kinematics


# ---------------------------------------------------------------------------
# RPY helpers  (extrinsic XYZ: R = Rz·Ry·Rx)
# ---------------------------------------------------------------------------

def _rot_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy < 1e-8:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0.0
    else:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    return float(rx), float(ry), float(rz)


def _rpy_to_rot(rx: float, ry: float, rz: float) -> np.ndarray:
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# ---------------------------------------------------------------------------
# Visualiser
# ---------------------------------------------------------------------------

class RobotVisualiser:
    AXIS_LEN = 0.12
    _MESH_COLOR = '#b8bcc8'

    def __init__(self, robot: Robot, stl_map: list | None = None):
        self.robot = robot
        self._stl_map = stl_map or []
        self._stl_objects: list[tuple] = []
        self._frame_groups: list = []
        self._joint_sliders: list = []
        self._joint_labels: list = []
        self._cart_sliders: list = []
        self._cart_labels: list = []
        self._updating = False
        self._show_frames = True
        self._ortho = False

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------

    def show(self):
        if self._stl_map:
            stl_dir = os.path.dirname(self._stl_map[0][0])
            app.add_static_files('/stl', stl_dir)

        @ui.page('/')
        def _page():
            self._build_ui()

        ui.run(title=self.robot.name, reload=False, port=3000)

    # ------------------------------------------------------------------
    # GUI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        # Clear stale element lists from any previous page load
        self._stl_objects.clear()
        self._frame_groups.clear()
        self._joint_sliders.clear()
        self._joint_labels.clear()
        self._cart_sliders.clear()
        self._cart_labels.clear()

        ui.add_head_html('''<style>
            body { margin:0; background:#e8e8e8; }
            .nicegui-content { padding:0 !important; }
        </style>
        <script type="module">
            import SceneLib from "nicegui-scene";
            window.__THREE = SceneLib.THREE;
            window.__OrbitControls = SceneLib.OrbitControls;
        </script>''')

        # ── Title bar ──────────────────────────────────────────────────
        with ui.row().classes('w-full items-center').style(
            'background:#ffffff; padding:6px 14px; border-bottom:1px solid #ccc;'
            'box-shadow:0 1px 4px rgba(0,0,0,0.15);'
        ):
            ui.label(f'Robot: {self.robot.name}').style(
                'font-size:15px; font-weight:600; color:#333; font-family:sans-serif;'
            )

        # ── Main area: scene + controls ────────────────────────────────
        with ui.row().classes('w-full no-wrap').style(
            'padding:0; gap:0; align-items:flex-start;'
        ):
            # 3D scene column — 4:3 aspect ratio, height-constrained to viewport
            with ui.column().classes('gap-0').style(
                'height:calc(100vh - 72px); aspect-ratio:4/3; flex-shrink:0;'
                'background:#888; position:relative;'
            ):
                self._scene = ui.scene(
                    width=800, height=600,
                    grid=(1, 10),
                    background_color='#7a7a7a',
                ).style('width:100%; height:100%;')
                with self._scene:
                    self._setup_scene()

            # Controls column
            with ui.column().classes('gap-0').style(
                'width:260px; min-width:260px; background:#ffffff;'
                'border-left:1px solid #ddd; padding:12px; overflow-y:auto;'
            ):
                self._build_controls()

        # ── Bottom bar ─────────────────────────────────────────────────
        self._build_bottom_bar()

        # Initial render
        self._update_scene()
        dist = self._max_reach() * 1.5
        cz = 0.15
        self._scene.move_camera(
            x=0, y=-dist, z=dist * 0.6,
            look_at_x=0, look_at_y=0, look_at_z=cz,
            up_x=0, up_y=0, up_z=1, duration=0,
        )
        ui.timer(0.2, self._init_viewcube, once=True)
        ui.timer(0.5, self._apply_metallic_materials, once=True)

    def _setup_scene(self):
        L = self._max_reach() * 0.25
        self._scene.line([0, 0, 0], [L, 0, 0]).material('#ff4444')
        self._scene.line([0, 0, 0], [0, L, 0]).material('#44cc44')
        self._scene.line([0, 0, 0], [0, 0, L]).material('#4488ff')
        off = L * 1.15
        self._scene.text3d('X', [off, 0, 0]).material('#ff4444').scale(0.04)
        self._scene.text3d('Y', [0, off, 0]).material('#44cc44').scale(0.04)
        self._scene.text3d('Z', [0, 0, off]).material('#4488ff').scale(0.04)

        for entry in self._stl_map:
            path, frame_idx = entry[0], entry[1]
            T_corr = entry[2] if len(entry) > 2 else np.eye(4)
            fname = os.path.basename(path)
            try:
                stl = self._scene.stl(f'/stl/{fname}')
                stl.scale(0.001)
                stl.material(self._MESH_COLOR)
                self._stl_objects.append((stl, frame_idx, T_corr))
            except Exception as exc:
                print(f'Warning: could not load {path}: {exc}')

        for _ in range(len(self.robot.transforms())):
            with self._scene.group() as g:
                self._scene.line([0, 0, 0], [self.AXIS_LEN, 0, 0]).material('#ff4444')
                self._scene.line([0, 0, 0], [0, self.AXIS_LEN, 0]).material('#44cc44')
                self._scene.line([0, 0, 0], [0, 0, self.AXIS_LEN]).material('#4488ff')
            self._frame_groups.append(g)

    def _build_controls(self):
        # Header row: label + RESET
        with ui.row().classes('w-full items-center justify-between').style('margin-bottom:8px;'):
            ui.label('Controls').style('font-size:14px; font-weight:700; color:#333;')
            ui.button('RESET', on_click=self._on_reset).props('flat dense').style(
                'color:#cc3333; font-size:11px; font-weight:700;'
            )

        # Mode toggle
        self._mode = ui.radio(
            ['Joint', 'Cart'], value='Cart',
            on_change=self._on_mode_change,
        ).props('inline dense').style('color:#555; font-size:13px; margin-bottom:6px;')

        # View controls
        with ui.row().classes('w-full items-center').style('gap:4px; margin-bottom:4px;'):
            self._ortho_btn = ui.button('PERSP', on_click=self._toggle_ortho).props(
                'flat dense'
            ).style('color:#555; font-size:11px;')

        ui.separator().style('margin:4px 0 8px 0;')

        # Joint sliders (hidden by default — Cart is default)
        with ui.column().classes('w-full gap-1') as self._joint_panel:
            self._build_joint_sliders()
        self._joint_panel.set_visibility(False)

        # Cartesian sliders (visible by default)
        with ui.column().classes('w-full gap-1') as self._cart_panel:
            self._build_cart_sliders()

        # End Effector readout inside controls panel
        ui.separator().style('margin:8px 0 4px 0;')
        self._ee_label = ui.label('').style(
            'font-size:10px; color:#888; font-family:monospace; white-space:pre;'
        )

    def _build_joint_sliders(self):
        for i, joint in enumerate(self.robot.joints):
            name = joint.name or f'q{i + 1}'
            lo = float(np.degrees(joint.limits[0]))
            hi = float(np.degrees(joint.limits[1]))
            v0 = float(np.degrees(self.robot.q[i]))
            with ui.row().classes('w-full items-center justify-between no-wrap').style('gap:4px;'):
                ui.label(name).style('color:#555; font-size:11px; min-width:56px;')
                lbl = ui.label(f'{v0:.1f}°').style(
                    'color:#333; font-size:11px; min-width:44px; text-align:right;'
                )
                self._joint_labels.append(lbl)
            s = ui.slider(min=lo, max=hi, value=v0, step=0.5,
                          on_change=self._on_joint_slider,
                          ).classes('w-full').props('dense color=blue-5')
            self._joint_sliders.append(s)

    def _build_cart_sliders(self):
        reach_mm = self._max_reach() * 1000 * 2
        T0 = self.robot.end_effector_pose()
        pos_mm = T0[:3, 3] * 1000
        rx0, ry0, rz0 = _rot_to_rpy(T0[:3, :3])

        defs = [
            ('X (mm)', -reach_mm, reach_mm, float(pos_mm[0])),
            ('Y (mm)', -reach_mm, reach_mm, float(pos_mm[1])),
            ('Z (mm)',      0,    reach_mm, float(pos_mm[2])),
            ('Rx (°)',   -180,       180,   float(np.degrees(rx0))),
            ('Ry (°)',   -180,       180,   float(np.degrees(ry0))),
            ('Rz (°)',   -180,       180,   float(np.degrees(rz0))),
        ]
        for label, lo, hi, v0 in defs:
            is_pos = '(mm)' in label
            step = 1.0 if is_pos else 0.5
            fmt = '{:.0f}' if is_pos else '{:.1f}°'
            with ui.row().classes('w-full items-center justify-between no-wrap').style('gap:4px;'):
                ui.label(label).style('color:#555; font-size:11px; min-width:56px;')
                lbl = ui.label(fmt.format(v0)).style(
                    'color:#333; font-size:11px; min-width:44px; text-align:right;'
                )
                self._cart_labels.append(lbl)
            s = ui.slider(min=lo, max=hi, value=v0, step=step,
                          on_change=self._on_cart_slider,
                          ).classes('w-full').props('dense color=orange-6')
            self._cart_sliders.append(s)

    def _build_bottom_bar(self):
        with ui.row().classes('w-full items-center justify-between').style(
            'background:#f0f0f0; padding:4px 12px; border-top:1px solid #ccc;'
            'height:36px; gap:8px;'
        ):
            self._info_label = ui.label('').style(
                'font-size:11px; color:#555; font-family:monospace;'
            )
            with ui.row().classes('items-center gap-1'):
                ui.button('ABOUT').props('flat dense size=sm').style('color:#888; font-size:10px;')
                ui.separator().props('vertical')
                ui.button('SHOW LABELS', on_click=self._toggle_labels).props(
                    'flat dense size=sm'
                ).style('color:#555; font-size:10px;')
                ui.button('SHOW FRAMES', on_click=self._toggle_frames).props(
                    'flat dense size=sm'
                ).style('color:#555; font-size:10px;')
                ui.button('SCREENSHOT', on_click=lambda: ui.run_javascript(
                    'html2canvas(document.body).then(c => { const a = document.createElement("a"); '
                    'a.href = c.toDataURL(); a.download = "screenshot.png"; a.click(); })'
                )).props('flat dense size=sm').style('color:#555; font-size:10px;')
                ui.button('STOP SIMULATOR', on_click=app.shutdown).props(
                    'flat dense size=sm'
                ).style('color:#cc3333; font-size:10px; font-weight:700;')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _max_reach(self) -> float:
        return max(0.5, sum(abs(j.a) + abs(j.d) for j in self.robot.joints))

    def _ee_rpy_mm(self):
        T = self.robot.end_effector_pose()
        pos_mm = T[:3, 3] * 1000
        rx, ry, rz = _rot_to_rpy(T[:3, :3])
        return pos_mm, rx, ry, rz

    # ------------------------------------------------------------------
    # Scene update
    # ------------------------------------------------------------------

    def _update_scene(self):
        transforms = self.robot.transforms()
        for stl, fi, T_corr in self._stl_objects:
            T = transforms[fi] @ T_corr
            stl.move(float(T[0, 3]), float(T[1, 3]), float(T[2, 3]))
            stl.rotate_R(T[:3, :3].tolist())
        for g, T in zip(self._frame_groups, transforms):
            g.move(float(T[0, 3]), float(T[1, 3]), float(T[2, 3]))
            g.rotate_R(T[:3, :3].tolist())
        self._update_readouts()

    def _update_readouts(self):
        pos_mm, rx, ry, rz = self._ee_rpy_mm()
        text = (
            f'x: {pos_mm[0]:+7.1f}  Y: {pos_mm[1]:+7.1f}  Z: {pos_mm[2]:+7.1f} mm   '
            f'Rx: {np.degrees(rx):+6.1f}°  Ry: {np.degrees(ry):+6.1f}°  Rz: {np.degrees(rz):+6.1f}°'
        )
        self._info_label.set_text(text)
        # Compact version in controls panel
        self._ee_label.set_text(
            f'x={pos_mm[0]:+.1f} y={pos_mm[1]:+.1f} z={pos_mm[2]:+.1f}\n'
            f'Rx={np.degrees(rx):+.1f}° Ry={np.degrees(ry):+.1f}° Rz={np.degrees(rz):+.1f}°'
        )

    # ------------------------------------------------------------------
    # Slider sync (server→UI only, does NOT trigger on_change)
    # ------------------------------------------------------------------

    def _sync_cart_sliders(self):
        pos_mm, rx, ry, rz = self._ee_rpy_mm()
        vals = [pos_mm[0], pos_mm[1], pos_mm[2],
                np.degrees(rx), np.degrees(ry), np.degrees(rz)]
        fmts = ['{:.0f}', '{:.0f}', '{:.0f}', '{:.1f}°', '{:.1f}°', '{:.1f}°']
        for s, lbl, v, fmt in zip(self._cart_sliders, self._cart_labels, vals, fmts):
            s.value = float(v)
            lbl.set_text(fmt.format(v))

    def _sync_joint_sliders(self, q: np.ndarray):
        for s, lbl, v in zip(self._joint_sliders, self._joint_labels, q):
            deg = np.degrees(v)
            s.value = float(deg)
            lbl.set_text(f'{deg:.1f}°')

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_mode_change(self, _e=None):
        is_joint = self._mode.value == 'Joint'
        self._joint_panel.set_visibility(is_joint)
        self._cart_panel.set_visibility(not is_joint)

    def _on_joint_slider(self, _e=None):
        if self._updating:
            return
        self._updating = True
        try:
            q = np.array([np.radians(s.value) for s in self._joint_sliders])
            self.robot.q = q
            for s, lbl in zip(self._joint_sliders, self._joint_labels):
                lbl.set_text(f'{s.value:.1f}°')
            self._sync_cart_sliders()
            self._update_scene()
        except Exception as exc:
            print(f'[FK] ERROR: {exc}', flush=True)
        finally:
            self._updating = False

    def _on_cart_slider(self, _e=None):
        if self._updating:
            return
        self._updating = True
        try:
            vals = [s.value for s in self._cart_sliders]
            fmts = ['{:.0f}', '{:.0f}', '{:.0f}', '{:.1f}°', '{:.1f}°', '{:.1f}°']
            for s, lbl, fmt in zip(self._cart_sliders, self._cart_labels, fmts):
                lbl.set_text(fmt.format(s.value))

            T_target = np.eye(4)
            T_target[:3, :3] = _rpy_to_rot(
                np.radians(vals[3]), np.radians(vals[4]), np.radians(vals[5])
            )
            T_target[:3, 3] = [vals[0] / 1000.0, vals[1] / 1000.0, vals[2] / 1000.0]

            q_sol = self._solve_ik(T_target)
            self.robot.q = q_sol
            self._sync_joint_sliders(q_sol)
            self._update_scene()
        except Exception as exc:
            print(f'[IK] ERROR: {exc}', flush=True)
        finally:
            self._updating = False

    def _solve_ik(self, T_target: np.ndarray) -> np.ndarray:
        if hasattr(self.robot, 'analytical_ik'):
            limits = [j.limits for j in self.robot.joints]
            q_sol = self.robot.analytical_ik(T_target, self.robot.q, limits)
            if q_sol is not None:
                return q_sol
        q_sol, _ = inverse_kinematics(
            self.robot, T_target[:3, 3], target_rot=T_target[:3, :3]
        )
        return q_sol

    def _on_reset(self):
        self._updating = True
        try:
            self.robot.q = np.zeros(self.robot.n_dof)
            for s, lbl in zip(self._joint_sliders, self._joint_labels):
                s.value = 0.0
                lbl.set_text('0.0°')
            self._sync_cart_sliders()
            self._update_scene()
        finally:
            self._updating = False

    async def _toggle_ortho(self):
        self._ortho = not self._ortho
        sid = self._scene.id
        size = self._max_reach() * 3
        if self._ortho:
            js = f"""
            const THREE = window.__THREE;
            const OC = window.__OrbitControls;
            const vue = getElement({sid});
            const old = vue.camera;
            const aspect = vue.$el.clientWidth / vue.$el.clientHeight;
            const cam = new THREE.OrthographicCamera(
                -{size}/2*aspect, {size}/2*aspect, {size}/2, -{size}/2, 0.01, 1000);
            cam.position.copy(old.position);
            cam.up.copy(old.up);
            cam.lookAt(vue.look_at);
            vue.camera = cam;
            vue.cameraType = 'orthographic';
            vue.cameraParams = {{size: {size}, near: 0.01, far: 1000}};
            vue.controls.dispose();
            vue.controls = new OC(cam, vue.renderer.domElement);
            vue.controls.target.copy(vue.look_at);
            vue.controls.update();
            """
            self._ortho_btn.set_text('ORTHO')
        else:
            js = f"""
            const THREE = window.__THREE;
            const OC = window.__OrbitControls;
            const vue = getElement({sid});
            const old = vue.camera;
            const aspect = vue.$el.clientWidth / vue.$el.clientHeight;
            const cam = new THREE.PerspectiveCamera(75, aspect, 0.01, 1000);
            cam.position.copy(old.position);
            cam.up.copy(old.up);
            cam.lookAt(vue.look_at);
            vue.camera = cam;
            vue.cameraType = 'perspective';
            vue.cameraParams = {{fov: 75, near: 0.01, far: 1000}};
            vue.controls.dispose();
            vue.controls = new OC(cam, vue.renderer.domElement);
            vue.controls.target.copy(vue.look_at);
            vue.controls.update();
            """
            self._ortho_btn.set_text('PERSP')
        await ui.run_javascript(js)

    async def _init_viewcube(self):
        sid = self._scene.id
        dist = self._max_reach() * 1.5
        cz = 0.15
        js = f"""
        (function() {{
            if (document.querySelector('.vc-overlay')) return;
            const THREE = window.__THREE;
            const vue = getElement({sid});
            const el = vue.$el;
            const dist = {dist}, cz = {cz};

            const sz = 90;
            const ov = document.createElement('div');
            ov.className = 'vc-overlay';
            ov.style.cssText = 'position:absolute;top:8px;right:8px;width:'+sz+'px;height:'+sz+'px;z-index:20;cursor:pointer;';
            el.style.position = 'relative';
            el.appendChild(ov);

            const canvas = document.createElement('canvas');
            canvas.style.width = sz+'px'; canvas.style.height = sz+'px';
            ov.appendChild(canvas);

            const renderer = new THREE.WebGLRenderer({{canvas, alpha:true, antialias:true}});
            renderer.setSize(sz, sz);
            renderer.setPixelRatio(window.devicePixelRatio);

            const vcScene = new THREE.Scene();
            const vcCam = new THREE.OrthographicCamera(-1.5, 1.5, 1.5, -1.5, 0.1, 20);

            // face: pos, rotation, label, normal color, hover color, snap target [x,y,z,lx,ly,lz,ux,uy,uz]
            const faceData = [
                {{pos:[1,0,0],  ry:Math.PI/2,  rx:0,          label:'+X', bg:'#cc3333', hov:'#ff5555', snap:[dist,0,cz,0,0,cz,0,0,1]}},
                {{pos:[-1,0,0], ry:-Math.PI/2, rx:0,          label:'-X', bg:'#993333', hov:'#cc4444', snap:[-dist,0,cz,0,0,cz,0,0,1]}},
                {{pos:[0,1,0],  ry:0,          rx:-Math.PI/2, label:'+Y', bg:'#33aa33', hov:'#44dd44', snap:[0,dist,cz,0,0,cz,0,0,1]}},
                {{pos:[0,-1,0], ry:0,          rx:Math.PI/2,  label:'-Y', bg:'#227722', hov:'#33bb33', snap:[0,-dist,cz,0,0,cz,0,0,1]}},
                {{pos:[0,0,1],  ry:0,          rx:0,          label:'+Z', bg:'#3366cc', hov:'#4488ff', snap:[0,0,dist,0,0,0,0,1,0]}},
                {{pos:[0,0,-1], ry:Math.PI,    rx:0,          label:'-Z', bg:'#224488', hov:'#3355aa', snap:[0,0,-dist,0,0,0,0,1,0]}},
            ];

            const cube = new THREE.Group();
            vcScene.add(cube);
            const meshes = [];

            function makeTex(label, bg) {{
                const tc = document.createElement('canvas');
                tc.width = 64; tc.height = 64;
                const ctx = tc.getContext('2d');
                ctx.fillStyle = bg;
                ctx.fillRect(0,0,64,64);
                ctx.strokeStyle = 'rgba(255,255,255,0.5)';
                ctx.lineWidth = 2;
                ctx.strokeRect(2,2,60,60);
                ctx.fillStyle = '#fff';
                ctx.font = 'bold 18px sans-serif';
                ctx.textAlign = 'center';
                ctx.textBaseline = 'middle';
                ctx.fillText(label, 32, 32);
                return new THREE.CanvasTexture(tc);
            }}

            faceData.forEach((f, i) => {{
                const mat = new THREE.MeshBasicMaterial({{map: makeTex(f.label, f.bg), transparent:false}});
                const mesh = new THREE.Mesh(new THREE.PlaneGeometry(2,2), mat);
                mesh.position.set(f.pos[0], f.pos[1], f.pos[2]);
                mesh.rotation.y = f.ry;
                mesh.rotation.x = f.rx;
                mesh._faceIdx = i;
                cube.add(mesh);
                meshes.push(mesh);
            }});

            cube.add(new THREE.LineSegments(
                new THREE.EdgesGeometry(new THREE.BoxGeometry(2,2,2)),
                new THREE.LineBasicMaterial({{color:0x222222}})
            ));

            const raycaster = new THREE.Raycaster();
            let hoveredIdx = -1;

            function getMouseNDC(e) {{
                const r = canvas.getBoundingClientRect();
                return new THREE.Vector2(
                    ((e.clientX - r.left) / sz) * 2 - 1,
                    -((e.clientY - r.top)  / sz) * 2 + 1
                );
            }}

            function setHover(idx) {{
                if (idx === hoveredIdx) return;
                // Restore old
                if (hoveredIdx >= 0) {{
                    const f = faceData[hoveredIdx];
                    meshes[hoveredIdx].material.map = makeTex(f.label, f.bg);
                    meshes[hoveredIdx].material.needsUpdate = true;
                }}
                hoveredIdx = idx;
                if (idx >= 0) {{
                    const f = faceData[idx];
                    meshes[idx].material.map = makeTex(f.label, f.hov);
                    meshes[idx].material.needsUpdate = true;
                    ov.style.cursor = 'pointer';
                }} else {{
                    ov.style.cursor = 'default';
                }}
            }}

            ov.addEventListener('mousemove', e => {{
                raycaster.setFromCamera(getMouseNDC(e), vcCam);
                const hits = raycaster.intersectObjects(meshes);
                setHover(hits.length > 0 ? hits[0].object._faceIdx : -1);
            }});
            ov.addEventListener('mouseleave', () => setHover(-1));

            ov.addEventListener('click', e => {{
                raycaster.setFromCamera(getMouseNDC(e), vcCam);
                const hits = raycaster.intersectObjects(meshes);
                if (hits.length > 0) {{
                    const s = faceData[hits[0].object._faceIdx].snap;
                    vue.move_camera(s[0],s[1],s[2], s[3],s[4],s[5], s[6],s[7],s[8], 0.3);
                }}
            }});

            function animate() {{
                requestAnimationFrame(animate);
                const p = vue.camera.position, d = p.length() || 1;
                vcCam.position.set(p.x/d*4, p.y/d*4, p.z/d*4);
                vcCam.up.copy(vue.camera.up);
                vcCam.lookAt(0,0,0);
                renderer.render(vcScene, vcCam);
            }}
            animate();
        }})();
        """
        await ui.run_javascript(js)

    async def _apply_metallic_materials(self):
        sid = self._scene.id
        js = f"""
        (function() {{
            const THREE = window.__THREE;
            const vue = getElement({sid});
            if (!vue || !vue.scene || !vue.renderer) return;

            // Add hemisphere + extra directional lights for metallic shading
            const hemi = new THREE.HemisphereLight(0xffffff, 0x444444, 0.6);
            vue.scene.add(hemi);
            const dir1 = new THREE.DirectionalLight(0xffffff, 0.8);
            dir1.position.set(1, 2, 3);
            vue.scene.add(dir1);
            const dir2 = new THREE.DirectionalLight(0xffffff, 0.4);
            dir2.position.set(-2, -1, 2);
            vue.scene.add(dir2);

            // Build a simple gradient equirectangular texture for env map
            const w = 256, h = 128;
            const envCanvas = document.createElement('canvas');
            envCanvas.width = w; envCanvas.height = h;
            const ctx = envCanvas.getContext('2d');
            const grad = ctx.createLinearGradient(0, 0, 0, h);
            grad.addColorStop(0,   '#c8d8e8');
            grad.addColorStop(0.4, '#e8eef4');
            grad.addColorStop(1,   '#707070');
            ctx.fillStyle = grad;
            ctx.fillRect(0, 0, w, h);
            const envTex = new THREE.CanvasTexture(envCanvas);
            envTex.mapping = THREE.EquirectangularReflectionMapping;

            const pmrem = new THREE.PMREMGenerator(vue.renderer);
            pmrem.compileEquirectangularShader();
            const envMap = pmrem.fromEquirectangular(envTex).texture;
            vue.scene.environment = envMap;
            envTex.dispose();
            pmrem.dispose();

            // Log all meshes and their world bounding boxes for diagnostics
            const box = new THREE.Box3();
            vue.scene.traverse(child => {{
                if (child.isMesh) {{
                    box.setFromObject(child);
                    const sz = new THREE.Vector3(); box.getSize(sz);
                    const ctr = new THREE.Vector3(); box.getCenter(ctr);
                    console.log('MESH', child.uuid.slice(0,6), child.material.type,
                        'size', sz.x.toFixed(3), sz.y.toFixed(3), sz.z.toFixed(3),
                        'center', ctr.x.toFixed(3), ctr.y.toFixed(3), ctr.z.toFixed(3));
                }}
            }});

            // Replace MeshPhongMaterial with MeshStandardMaterial on STL meshes only
            // Skip "scene" key to avoid traversing the entire scene tree
            vue.objects.forEach((obj, key) => {{
                if (key === 'scene') return;
                obj.traverse(child => {{
                    if (child.isMesh && child.material && child.material.type === 'MeshPhongMaterial') {{
                        const old = child.material;
                        child.material = new THREE.MeshStandardMaterial({{
                            color: old.color.clone(),
                            metalness: 0.75,
                            roughness: 0.25,
                            envMap: envMap,
                            envMapIntensity: 1.5,
                            transparent: old.transparent,
                            opacity: old.opacity,
                            side: old.side,
                        }});
                        old.dispose();
                    }}
                }});
            }});
        }})();
        """
        await ui.run_javascript(js)

    def _toggle_frames(self):
        self._show_frames = not self._show_frames
        for g in self._frame_groups:
            g.visible(self._show_frames)

    def _toggle_labels(self):
        pass  # placeholder — no label objects currently in scene
