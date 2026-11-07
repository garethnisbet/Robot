"""Closed-form forward / inverse kinematics for the KLiK3 arm (plain Python, no FreeCAD).

Joint chain, matching klik3_freecad.build() (right-handed rotations, degrees):
    Rz(q1) . T(J2X,0,J2Z) Ry(q2) . T(0,0,L2) Ry(q3) . T(J4X,0,J4Z) Rx(q4) . T(J5X,0,0) Ry(q5) . T(WRIST,0,0) Rx(q6) . T(TOOL,0,0)
TOOL moves the reported point from the J6 origin out along the J6 axis (the flange
mounting face, as RobotVisualisation and the Meca500 report it).
J4, J5 and J6 meet at the J5 centre (a spherical wrist), so position and
orientation split: the arm (q1-q3) places the wrist centre, the wrist (q4-q6)
sets the orientation.

Poses follow the Meca500 convention: flange-centre position (mm) plus the tool
frame (z along the flange normal) as mobile XYZ Euler angles (deg), i.e.
R_tool = Rx(a) Ry(b) Rz(g) and R_tool = R_flange . Ry(90). At zero joints that
reads (J2X + J4X + J5X + WRIST + TOOL, 0, J2Z + L2 + J4Z, 0, 90, 0) - the same angles a
Meca500 shows at zero. J2X shifts J2 along the arm plane's x (in the J1 frame), so
the arm still lies in a plane through J1 and the closed form holds.
"""
import math
from itertools import product


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _t(a):
    return [list(r) for r in zip(*a)]


def _wrap(a):
    """Wrap degrees to (-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0 if (a + 180.0) % 360.0 else 180.0


RY90 = _ry(math.pi / 2)


def euler_to_matrix(a, b, g):
    r = math.radians
    return _mm(_mm(_rx(r(a)), _ry(r(b))), _rz(r(g)))


def matrix_to_euler(R, a_hint=0.0):
    """Mobile XYZ Euler angles (deg) of R. At b = +-90 (gimbal lock) a keeps a_hint."""
    b = math.degrees(math.asin(max(-1.0, min(1.0, R[0][2]))))
    if math.hypot(R[1][2], R[2][2]) > 1e-6:
        a = math.degrees(math.atan2(-R[1][2], R[2][2]))
        g = math.degrees(math.atan2(-R[0][1], R[0][0]))
    else:                               # a and g share one axis: keep a, put the rest in g
        a = a_hint
        Rg = _mm(_t(euler_to_matrix(a, b, 0)), R)
        g = math.degrees(math.atan2(Rg[1][0], Rg[0][0]))
    return a, b, g


class KLiK3Kinematics:
    def __init__(self, J2Z, L2, J4X, J4Z, J5X, WRIST, limits, J2X=0.0, TOOL=0.0):
        self.J2X, self.TOOL = J2X, TOOL
        self.J2Z, self.L2, self.J4X, self.J4Z, self.J5X, self.WRIST = J2Z, L2, J4X, J4Z, J5X, WRIST
        self.limits = [tuple(l) for l in limits]

    # --- forward --------------------------------------------------------------------
    def flange(self, q):
        """(position, rotation matrix) of the flange frame for joint angles q (deg)."""
        r = [math.radians(v) for v in q]
        R = _rz(r[0])
        p = _mv(R, [self.J2X, 0.0, self.J2Z])
        R = _mm(R, _ry(r[1]))
        p = [a + b for a, b in zip(p, _mv(R, [0, 0, self.L2]))]
        R = _mm(R, _ry(r[2]))
        p = [a + b for a, b in zip(p, _mv(R, [self.J4X, 0, self.J4Z]))]
        R = _mm(R, _rx(r[3]))
        p = [a + b for a, b in zip(p, _mv(R, [self.J5X, 0, 0]))]
        R = _mm(R, _ry(r[4]))
        p = [a + b for a, b in zip(p, _mv(R, [self.WRIST, 0, 0]))]
        R = _mm(R, _rx(r[5]))
        p = [a + b for a, b in zip(p, _mv(R, [self.TOOL, 0, 0]))]
        return p, R

    def fk(self, q, a_hint=0.0):
        """Pose (x, y, z, a, b, g) in the Meca500 convention."""
        p, R = self.flange(q)
        return tuple(p) + matrix_to_euler(_mm(R, RY90), a_hint)

    # --- inverse --------------------------------------------------------------------
    def solutions(self, pose):
        """All closed-form joint solutions (deg) for a pose, limits not applied."""
        x, y, z, a, b, g = pose
        R = _mm(euler_to_matrix(a, b, g), _t(RY90))         # flange rotation
        w = self.WRIST + self.TOOL
        wc = [x - w * R[0][0], y - w * R[1][0], z - w * R[2][0]]
        D, H = self.J4X + self.J5X, self.J4Z                 # J3 -> wrist centre in the J3 frame
        Lb, phi = math.hypot(D, H), math.atan2(H, D)
        out = []
        rho = math.hypot(wc[0], wc[1])
        for front in (True, False):
            if rho < 1e-9:
                q1 = 0.0 if front else math.pi
            else:
                q1 = math.atan2(wc[1], wc[0]) if front else math.atan2(-wc[1], -wc[0])
            u = (rho if front else -rho) - self.J2X          # reach from J2 along the arm plane's x
            w = wc[2] - self.J2Z
            # |a + b|: L2 along z, Lb at angle (phi - q3) from the forearm x after Ry(q2+q3)
            c = (u * u + w * w - self.L2 ** 2 - Lb ** 2) / (2 * self.L2 * Lb)
            if abs(c) > 1 + 1e-9:
                continue
            c = max(-1.0, min(1.0, c))
            for s3 in (1, -1):
                # in the plane: u = L2 sin q2 + Lb cos(q2 + q3 - phi)... solved via angle t = q3 - phi
                # vector after Ry(q2): a = (L2 sin q2, L2 cos q2); b = Lb (cos(q23) cos phi.. ) -> use
                # b in the J2 frame = Ry(q3)(D, H) = Lb (cos(phi - q3), sin(phi - q3)) as (x, z)
                # |a + b|^2 = L2^2 + Lb^2 + 2 L2 Lb sin(phi - q3)
                t = math.asin(c) if s3 > 0 else math.pi - math.asin(c)      # t = phi - q3
                q3 = phi - t
                bx, bz = Lb * math.cos(t), Lb * math.sin(t)                 # b before Ry(q2)
                ax, az = 0.0, self.L2
                vx, vz = ax + bx, az + bz                                   # rotate by q2 onto (u, w)
                # Ry(q2): x' = x cos + z sin, z' = -x sin + z cos
                q2 = math.atan2(vz * u - vx * w, vx * u + vz * w)
                R03 = _mm(_mm(_rz(q1), _ry(q2)), _ry(q3))
                Rw = _mm(_t(R03), R)                                        # = Rx(q4) Ry(q5) Rx(q6)
                c5 = max(-1.0, min(1.0, Rw[0][0]))
                s5 = math.hypot(Rw[1][0], Rw[2][0])
                for sg in (1, -1):
                    q5 = math.atan2(sg * s5, c5)
                    if s5 > 1e-6:
                        q4 = math.atan2(sg * Rw[1][0], -sg * Rw[2][0])
                        q6 = math.atan2(sg * Rw[0][1], sg * Rw[0][2])
                    else:                                                   # wrist singular: q4 + q6 fixed
                        q4 = None
                        q6 = math.atan2(Rw[2][1], Rw[1][1])
                    out.append([math.degrees(v) if v is not None else None
                                for v in (q1, q2, q3, q4, q5, q6)])
        return out

    def ik(self, pose, current):
        """Closest in-limit solution to `current` (deg), or None if unreachable."""
        best, best_cost = None, None
        for s in self.solutions(pose):
            if s[3] is None:                                # singular wrist: keep J4, give J6 the rest
                s = list(s)
                s[3] = current[3]
                s[5] = s[5] - s[3] if abs(s[4]) < 90 else s[3] - s[5]    # q5 = 0: q4+q6 fixed; 180: q4-q6
            cand = list(s)
            # J4 / J6 may be shifted by whole turns to sit nearest the current angle
            for j in (3, 5):
                cand[j] = current[j] + _wrap(cand[j] - current[j])
            for j in (0, 1, 2, 4):
                cand[j] = _wrap(cand[j])
            if not all(lo - 1e-6 <= v <= hi + 1e-6 for v, (lo, hi) in zip(cand, self.limits)):
                # try the J4 turn the other way before giving up on this branch
                alt = list(cand)
                for j in (3, 5):
                    lo, hi = self.limits[j]
                    if alt[j] > hi:
                        alt[j] -= 360
                    elif alt[j] < lo:
                        alt[j] += 360
                if not all(lo - 1e-6 <= v <= hi + 1e-6 for v, (lo, hi) in zip(alt, self.limits)):
                    continue
                cand = alt
            cost = sum(w * (c - q) ** 2 for w, c, q in zip((3, 3, 2, 1, 1, 0.5), cand, current))
            if best is None or cost < best_cost:
                best, best_cost = cand, cost
        return best


# --- gravity sag: gear wind-up + output-bearing tilt (FEA report, 2026-10-25) -------------
# The arm's own links bend only ~50 um at full stretch; the strain-wave gears and the actuators'
# output bearings give ~1.4 mm. This model predicts that sag from the link masses (klik3_mass.json,
# written by tools/fc_scripts/mass_props.py) and corrects the commanded pose for it.

# Harmonic Drive torsional stiffness, ratio >= 80 (CSF-ULW brochure): T1, T2 (N.m), K1, K2, K3 (N.m/rad)
GEAR_TORSION = {8: (0.29, 0.75, 910.0, 1000.0, 1200.0), 11: (0.8, 2.0, 2700.0, 3400.0, 4400.0)}
GEAR_SIZE = {'FHA-8C': 8, 'FHA-11C': 11, 'RSA-8A': 8, 'RSF-5B': None}
# output-bearing moment stiffness (N.m/rad, catalogue); support bearings add in parallel
TILT_STIFFNESS = {'FHA-8C': 2.0e4, 'FHA-11C': 4.0e4, 'RSA-8A': 2.76e3, 'RSF-5B': 741.0}
# J1 thin-section support ring: assumed equal to the FHA-8C's own bearing (confirm from the bearing catalogue).
# J2/J3/J5: far-side crossed-roller (IKO CRBT) a span s from the actuator's bearing; the pair resists tilt as a
# radial couple, s^2 * k_a k_s / (k_a + k_s), with an assumed 100 N/um radial stiffness each (spans from the model)
CRB_RADIAL_K = 1.0e8
SUPPORT_SPAN = {2: 0.0628, 3: 0.0618, 5: 0.0605}
SUPPORT_TILT = {1: 2.0e4, **{k: s * s * CRB_RADIAL_K / 2 for k, s in SUPPORT_SPAN.items()}}
JOINT_AXES = [(0, 0, 1), (0, 1, 0), (0, 1, 0), (1, 0, 0), (0, 1, 0), (1, 0, 0)]


def windup(T, size):
    """Gear wind-up (rad) under torque T (N.m): three-slope curve, odd in T."""
    if size is None:
        return 0.0
    T1, T2, K1, K2, K3 = GEAR_TORSION[size]
    a = abs(T)
    th = a / K1 if a <= T1 else (T1 / K1 + (a - T1) / K2 if a <= T2 else T1 / K1 + (T2 - T1) / K2 + (a - T2) / K3)
    return math.copysign(th, T)


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _rotvec_exp(w):
    th = math.sqrt(sum(v * v for v in w))
    if th < 1e-15:
        return [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    k = [v / th for v in w]
    K = [[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]
    K2 = _mm(K, K)
    s, c = math.sin(th), 1 - math.cos(th)
    return [[(i == j) + s * K[i][j] + c * K2[i][j] for j in range(3)] for i in range(3)]


def _rotvec_log(R):
    c = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2))
    th = math.acos(c)
    v = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]]
    if th < 1e-12:
        return [x / 2 for x in v]
    return [x * th / (2 * math.sin(th)) for x in v]


class SagModel:
    """Static sag of the flange under gravity: each joint winds up about its axis (gear) and
    tilts about the other two (output bearing). payload_kg sits payload_offset mm beyond the
    flange face along the flange normal."""

    def __init__(self, kin, mass_json=None, payload_kg=0.5, payload_offset=0.0):
        import json, os
        mass_json = mass_json or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'klik3_mass.json')
        d = json.load(open(mass_json))
        self.kin = kin
        self.links = [(d['links'][str(j)]['kg'], d['links'][str(j)]['com']) for j in range(1, 7)]
        self.acts = d['actuators']
        self.face_x = d['flange_face_x']
        self.bearing = [d.get('bearing_points', {}).get(str(j), [0, 0, 0]) for j in range(1, 7)]   # output-bearing centre, joint frame
        self.payload_kg, self.payload_offset = payload_kg, payload_offset

    def frames(self, q):
        """World origin and rotation of link frames 1..6 and the flange (same chain as flange())."""
        k = self.kin
        r = [math.radians(v) for v in q]
        steps = [(_rz, [0, 0, 0]), (_ry, [k.J2X, 0, k.J2Z]), (_ry, [0, 0, k.L2]), (_rx, [k.J4X, 0, k.J4Z]),
                 (_ry, [k.J5X, 0, 0]), (_rx, [k.WRIST, 0, 0])]
        p, R, out = [0.0, 0.0, 0.0], [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]], []
        for (rot, off), a in zip(steps, r):
            p = [x + y for x, y in zip(p, _mv(R, off))]
            R = _mm(R, rot(a))
            out.append((p, R))
        return out

    def joint_loads(self, q):
        """[(torque about the axis N.m, tilt moment vector N.m, axis, bearing centre)] for J1..J6."""
        F = self.frames(q)
        pts = []
        for (m, c), (p, R) in zip(self.links, F):
            pts.append((m, [a + b for a, b in zip(p, _mv(R, c))]))
        p6, R6 = F[5]
        pts[-1:] = [pts[-1], (self.payload_kg, [a + b for a, b in zip(p6, _mv(R6, [self.face_x + self.payload_offset, 0, 0]))])]
        out = []
        for k, (p, R) in enumerate(F):
            p = [a + b for a, b in zip(p, _mv(R, self.bearing[k]))]     # moments taken at the output bearing
            M = [0.0, 0.0, 0.0]
            for m, c in pts[k:]:
                f = [0, 0, -9.81 * m]
                M = [a + b for a, b in zip(M, _cross([x - y for x, y in zip(c, p)], f))]
            M = [v / 1000.0 for v in M]                                  # N.mm -> N.m
            ax = _mv(R, JOINT_AXES[k])
            T = sum(a * b for a, b in zip(M, ax))
            out.append((T, [m - T * a for m, a in zip(M, ax)], ax, p))
        return out

    def predict(self, q):
        """(flange position error mm, flange rotation-vector error rad) caused by sag at q."""
        F = self.frames(q)
        pf, Rf = F[5]
        pf = [a + b for a, b in zip(pf, _mv(Rf, [getattr(self.kin, 'TOOL', 0.0), 0, 0]))]   # at the reported point
        dp, dw = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        for k, (T, Mt, ax, p) in enumerate(self.joint_loads(q)):
            act = self.acts[k]
            kt = TILT_STIFFNESS[act] + SUPPORT_TILT.get(k + 1, 0.0)
            w = [a * windup(T, GEAR_SIZE[act]) + m / kt for a, m in zip(ax, Mt)]
            dw = [a + b for a, b in zip(dw, w)]
            dp = [a + b for a, b in zip(dp, _cross(w, [x - y for x, y in zip(pf, p)]))]
        return dp, dw

    def compensated_ik(self, pose, current, iters=4):
        """Joint command that lands the sagging flange on `pose` (Meca500 convention).
        Returns (q, residual position mm, residual angle rad) or (None, None, None)."""
        x, y, z, a, b, g = pose
        p_t, R_t = [x, y, z], _mm(euler_to_matrix(a, b, g), _t(RY90))
        p_c, R_c = list(p_t), R_t
        q = None
        for _ in range(iters + 1):
            q = self.kin.ik(tuple(p_c) + matrix_to_euler(_mm(R_c, RY90), a), current)
            if q is None:
                return None, None, None
            p, R = self.kin.flange(q)
            dp, dw = self.predict(q)
            p_a, R_a = [u + v for u, v in zip(p, dp)], _mm(_rotvec_exp(dw), R)
            e_p = [u - v for u, v in zip(p_t, p_a)]
            e_w = _rotvec_log(_mm(R_t, _t(R_a)))
            p_c = [u + v for u, v in zip(p_c, e_p)]
            R_c = _mm(_rotvec_exp(e_w), R_c)
            current = q
        return q, math.sqrt(sum(v * v for v in e_p)), math.sqrt(sum(v * v for v in e_w))
