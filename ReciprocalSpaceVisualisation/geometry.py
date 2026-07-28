#!/usr/bin/env python3
"""Detector geometry and the pixel <-> reciprocal-space transformation.

Everything needed to turn a detector pixel into a scattering vector **Q**, and
back again, from first principles. The detector is described the way it is
actually known on a beamline:

    * **position**   — distance from the sample to the detector (along the beam),
    * **orientation** — tilt of the panel (three rotations), or explicit axes,
    * **pixel size**  — mm per pixel, fast and slow directions independently,
    * **size**        — number of pixels ``(n_slow, n_fast)``,
    * **origin pixel** — the pixel that sits on the beam before tilts (the beam
      centre for an untilted panel).

Frames and conventions
----------------------
**Lab frame** (right-handed, sample at the origin): the incident beam runs along
``+y`` by default — matching the beam frame of the sibling ``DMS`` project's
``ts_quasi.psith2v`` — with ``+x`` horizontal-transverse and ``+z`` vertical.
Nothing below hard-codes this: the beam direction is a parameter, so any
convention works as long as it is used consistently.

**Detector indexing** follows numpy: an image is ``im[slow, fast]`` — ``slow`` is
the row index, ``fast`` the column index. ``Detector.origin`` is the lab position
of the **centre** of pixel ``(0, 0)``; pixel ``(s, f)`` (fractional allowed) sits
at ``origin + f * pixel_fast * fast_axis + s * pixel_slow * slow_axis``.

**Q convention** is ``k = 1/λ`` by default, so ``|Q| = 2 sinθ / λ = 1/d`` — the
same convention as ``ts_quasi`` (``ko = E / 12.398``) and as the reciprocal
lattice this viewer draws (``Q = B·hkl``). Pass ``two_pi=True`` to ``Beam`` for
the ``k = 2π/λ`` convention instead.

Lengths (distance, pixel size) are in **mm**; only ratios matter, so any
consistent unit works. Q comes out in Å⁻¹.

The scattering geometry itself is the textbook one::

    k_in  = k0 * beam_direction              (incident wavevector)
    k_out = k0 * unit(pixel_position)        (elastic: |k_out| = |k_in| = k0)
    Q     = k_out - k_in

so every pixel of a still image maps onto the Ewald sphere, ``|Q + k_in| = k0``.
Rotating the sample by ``φ`` sweeps that surface through the volume; the sample
frame Q is ``Q_sample = R(φ)⁻¹ · Q_lab`` (see :class:`Goniometer`).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Sequence

import numpy as np

# CODATA hc in keV·Å. ts_quasi uses the rounded 12.398 (a 1.6e-6 relative
# difference — far below any geometric error, but we keep the exact value).
HC_KEV_ANGSTROM = 12.398419843320026


# ── small vector helpers ───────────────────────────────────────────────────
def unit(v) -> np.ndarray:
    """Normalise a vector (or the last axis of an array of vectors)."""
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    if np.any(n == 0):
        raise ValueError("cannot normalise a zero-length vector")
    return v / n


def rotation_matrix(axis, angle_deg: float) -> np.ndarray:
    """Right-handed rotation of `angle_deg` about `axis` (Rodrigues, column convention).

    Applies to column vectors as ``R @ v``; for arrays of row vectors use
    :func:`apply_rotation`.
    """
    k = unit(axis)
    a = np.radians(float(angle_deg))
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(a) * K + (1.0 - np.cos(a)) * (K @ K)


def rotation_xyz(rx: float, ry: float, rz: float) -> np.ndarray:
    """Extrinsic rotations about the lab x, y then z axes: ``Rz @ Ry @ Rx``."""
    return (rotation_matrix([0, 0, 1], rz)
            @ rotation_matrix([0, 1, 0], ry)
            @ rotation_matrix([1, 0, 0], rx))


def apply_rotation(R: np.ndarray, v) -> np.ndarray:
    """Apply column-convention `R` to an array of row vectors ``(..., 3)``."""
    return np.asarray(v, dtype=float) @ np.asarray(R, dtype=float).T


# ── beam ───────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Beam:
    """Monochromatic incident beam.

    Parameters
    ----------
    energy_kev : photon energy in keV.
    direction  : propagation direction in the lab frame (need not be normalised).
    two_pi     : use ``k = 2π/λ`` instead of the default ``k = 1/λ``.
    """

    energy_kev: float = 15.0
    direction: tuple = (0.0, 1.0, 0.0)
    two_pi: bool = False

    @property
    def wavelength(self) -> float:
        """Wavelength in Å."""
        return HC_KEV_ANGSTROM / float(self.energy_kev)

    @property
    def k0(self) -> float:
        """Wavevector magnitude in Å⁻¹ (``1/λ``, or ``2π/λ`` if `two_pi`)."""
        k = 1.0 / self.wavelength
        return 2.0 * np.pi * k if self.two_pi else k

    @property
    def s0(self) -> np.ndarray:
        """Unit propagation direction."""
        return unit(self.direction)

    @property
    def k_in(self) -> np.ndarray:
        """Incident wavevector ``k0 * s0`` (Å⁻¹)."""
        return self.k0 * self.s0


# ── detector ───────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Detector:
    """A flat pixel-array detector positioned and oriented in the lab frame.

    Attributes
    ----------
    shape       : ``(n_slow, n_fast)`` — image size in pixels, numpy order.
    pixel_size  : ``(slow, fast)`` pixel pitch in mm.
    origin      : lab position (mm) of the **centre of pixel (0, 0)**.
    fast_axis   : unit lab vector along increasing fast (column) index.
    slow_axis   : unit lab vector along increasing slow (row) index.

    Build one with :meth:`from_beam_centre` unless you already have the axes.
    """

    shape: tuple = (1024, 1024)
    pixel_size: tuple = (0.1, 0.1)
    origin: np.ndarray = field(default_factory=lambda: np.array([-51.15, 200.0, 51.15]))
    fast_axis: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    slow_axis: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -1.0]))

    def __post_init__(self):
        object.__setattr__(self, "shape", (int(self.shape[0]), int(self.shape[1])))
        object.__setattr__(self, "pixel_size",
                           (float(self.pixel_size[0]), float(self.pixel_size[1])))
        object.__setattr__(self, "origin", np.asarray(self.origin, dtype=float).reshape(3))
        object.__setattr__(self, "fast_axis", unit(self.fast_axis))
        object.__setattr__(self, "slow_axis", unit(self.slow_axis))
        if abs(float(self.fast_axis @ self.slow_axis)) > 1e-9:
            raise ValueError("fast_axis and slow_axis must be perpendicular")

    # ── construction ───────────────────────────────────────────────────────
    @classmethod
    def from_beam_centre(
        cls,
        distance: float,
        shape: Sequence[int] = (1024, 1024),
        pixel_size: float | Sequence[float] = 0.1,
        origin_pixel: Sequence[float] | None = None,
        beam: Beam | None = None,
        tilt: Sequence[float] = (0.0, 0.0, 0.0),
        pivot: str = "origin_pixel",
        two_theta: float = 0.0,
        gamma: float = 0.0,
        fast_axis: Sequence[float] | None = None,
        slow_axis: Sequence[float] | None = None,
    ) -> "Detector":
        """Build a detector from beamline-style parameters.

        Parameters
        ----------
        distance     : sample → detector distance in mm, measured along the beam
                       to `origin_pixel` (before any tilt is applied).
        shape        : ``(n_slow, n_fast)`` pixels.
        pixel_size   : mm, scalar or ``(slow, fast)``.
        origin_pixel : ``(slow, fast)`` pixel placed on the beam axis — the beam
                       centre for an untilted panel. Defaults to the image centre,
                       ``((n_slow - 1)/2, (n_fast - 1)/2)``.
        beam         : sets the beam direction; defaults to ``Beam()`` (+y).
        tilt         : ``(rx, ry, rz)`` degrees, extrinsic about the lab axes,
                       applied to the panel after it is placed.
        pivot        : ``"origin_pixel"`` (default) rotates the panel about the
                       point it occupies — a true panel tilt, which leaves the
                       origin pixel on the beam. ``"sample"`` rotates about the
                       lab origin instead, i.e. swings the whole detector arm.
        two_theta    : detector-arm scattering angle in degrees — swings the
                       whole panel about the sample in the **horizontal** plane
                       (about the vertical lab ``+z``). Applied after `tilt`.
        gamma        : detector-arm out-of-plane angle in degrees — swings the
                       panel **vertically** (about the transverse-horizontal lab
                       ``+x``). Together ``(two_theta, gamma)`` position the arm;
                       these assume the module's default frame (beam ``+y``,
                       ``+z`` up), and the sign is a convention.
        fast_axis, slow_axis
                     : untilted panel axes. Default to a panel facing the beam
                       with `fast` = lab x and `slow` = -(the axis completing the
                       right-handed set), i.e. rows running downwards for a
                       vertical panel — the usual image orientation.
        """
        beam = beam or Beam()
        s0 = beam.s0
        psize = (float(pixel_size), float(pixel_size)) if np.isscalar(pixel_size) \
            else (float(pixel_size[0]), float(pixel_size[1]))
        shape = (int(shape[0]), int(shape[1]))
        if origin_pixel is None:
            origin_pixel = ((shape[0] - 1) / 2.0, (shape[1] - 1) / 2.0)
        s_o, f_o = float(origin_pixel[0]), float(origin_pixel[1])

        if fast_axis is None or slow_axis is None:
            # Panel normal anti-parallel to nothing in particular: pick the lab
            # axis least aligned with the beam as "fast", orthogonalise, and make
            # `slow` run so that fast x slow points downstream (along the beam).
            seed = np.eye(3)[int(np.argmin(np.abs(s0)))]
            f = unit(seed - (seed @ s0) * s0) if fast_axis is None else unit(fast_axis)
            s = unit(np.cross(s0, f)) if slow_axis is None else unit(slow_axis)
        else:
            f, s = unit(fast_axis), unit(slow_axis)

        origin = distance * s0 - (f_o * psize[1] * f + s_o * psize[0] * s)

        rx, ry, rz = (float(t) for t in tilt)
        if any((rx, ry, rz)):
            R = rotation_xyz(rx, ry, rz)
            centre = distance * s0 if pivot == "origin_pixel" else np.zeros(3)
            if pivot not in ("origin_pixel", "sample"):
                raise ValueError("pivot must be 'origin_pixel' or 'sample'")
            origin = centre + R @ (origin - centre)
            f, s = R @ f, R @ s

        # Detector-arm swing about the sample (origin): 2θ in the horizontal
        # plane (about vertical +z), γ out of it (about transverse-horizontal
        # +x). Independent of `tilt`, which orients the panel about its own
        # pixel — this points the whole arm.
        tt, gg = float(two_theta), float(gamma)
        if tt or gg:
            R_arm = rotation_matrix([1.0, 0.0, 0.0], gg) @ rotation_matrix([0.0, 0.0, 1.0], tt)
            origin = R_arm @ origin
            f, s = R_arm @ f, R_arm @ s

        return cls(shape=shape, pixel_size=psize, origin=origin,
                   fast_axis=f, slow_axis=s)

    def moved(self, **kwargs) -> "Detector":
        """Copy with fields replaced (``dataclasses.replace``)."""
        return replace(self, **kwargs)

    # ── geometry ───────────────────────────────────────────────────────────
    @property
    def normal(self) -> np.ndarray:
        """Panel normal, ``fast × slow`` (points downstream for the defaults)."""
        return unit(np.cross(self.fast_axis, self.slow_axis))

    def lab_from_pixel(self, slow, fast) -> np.ndarray:
        """Lab position (mm) of fractional pixel ``(slow, fast)``.

        Broadcasts: scalars give ``(3,)``, arrays give ``(..., 3)``.
        """
        s = np.asarray(slow, dtype=float)
        f = np.asarray(fast, dtype=float)
        return (self.origin
                + f[..., None] * (self.pixel_size[1] * self.fast_axis)
                + s[..., None] * (self.pixel_size[0] * self.slow_axis))

    def pixel_centres(self) -> np.ndarray:
        """Lab positions of every pixel centre, ``(n_slow, n_fast, 3)``."""
        s, f = np.indices(self.shape, dtype=float)
        return self.lab_from_pixel(s, f)

    def pixel_from_lab(self, position) -> np.ndarray:
        """Fractional ``(slow, fast)`` of a lab point assumed to lie in the panel."""
        d = np.asarray(position, dtype=float) - self.origin
        return np.stack([(d @ self.slow_axis) / self.pixel_size[0],
                         (d @ self.fast_axis) / self.pixel_size[1]], axis=-1)

    def intersect(self, direction) -> np.ndarray:
        """Where rays from the sample along `direction` hit the panel.

        Returns fractional ``(..., 2)`` ``(slow, fast)`` pixel coordinates; NaN
        where the ray is parallel to the panel or would hit it behind the sample.
        """
        d = np.asarray(direction, dtype=float)
        n = self.normal
        denom = d @ n
        t = np.divide(self.origin @ n, denom,
                      out=np.full(np.shape(denom), np.nan),
                      where=np.abs(denom) > 1e-12)
        t = np.where(t > 0, t, np.nan)
        return self.pixel_from_lab(t[..., None] * d)

    def contains(self, slow, fast, edge: float = 0.5) -> np.ndarray:
        """Mask of pixel coordinates lying on the panel (`edge` = half-pixel margin)."""
        s = np.asarray(slow, dtype=float)
        f = np.asarray(fast, dtype=float)
        with np.errstate(invalid="ignore"):
            return ((s >= -edge) & (s <= self.shape[0] - 1 + edge)
                    & (f >= -edge) & (f <= self.shape[1] - 1 + edge))

    def beam_centre(self, beam: Beam) -> np.ndarray:
        """Fractional ``(slow, fast)`` pixel struck by the direct beam."""
        return self.intersect(beam.s0)

    def solid_angle(self) -> np.ndarray:
        """Per-pixel solid angle (sr), ``(n_slow, n_fast)``.

        ``ΔΩ = A cos α / r²`` with ``α`` the angle between the panel normal and
        the scattered ray — the leading intensity correction when merging pixels
        that view the sample from different distances and obliquities.
        """
        p = self.pixel_centres()
        r = np.linalg.norm(p, axis=-1)
        cos_a = np.abs(p @ self.normal) / r
        area = self.pixel_size[0] * self.pixel_size[1]
        return area * cos_a / r**2

    def two_theta(self, beam: Beam) -> np.ndarray:
        """Scattering angle 2θ (degrees) for every pixel, ``(n_slow, n_fast)``."""
        khat = unit(self.pixel_centres())
        return np.degrees(np.arccos(np.clip(khat @ beam.s0, -1.0, 1.0)))


# ── the transformation ─────────────────────────────────────────────────────
def q_from_pixel(detector: Detector, beam: Beam, slow, fast) -> np.ndarray:
    """Scattering vector ``Q = k_out - k_in`` (Å⁻¹, lab frame) for fractional pixels.

    Elastic scattering: the outgoing wavevector points from the sample to the
    pixel and has the same length as the incident one.
    """
    khat = unit(detector.lab_from_pixel(slow, fast))
    return beam.k0 * khat - beam.k_in


def q_map(detector: Detector, beam: Beam) -> np.ndarray:
    """`Q` for every pixel centre, ``(n_slow, n_fast, 3)`` — the whole image at once."""
    khat = unit(detector.pixel_centres())
    return beam.k0 * khat - beam.k_in


def pixel_from_q(detector: Detector, beam: Beam, q):
    """Inverse map: where does scattering vector `Q` land on the detector?

    Returns ``(pixel, excitation)`` where `pixel` is fractional ``(..., 2)``
    ``(slow, fast)`` (NaN if the ray misses the panel plane) and `excitation` is
    ``|k_in + Q| - k0``: zero when `Q` lies exactly on the Ewald sphere, and the
    signed distance off it otherwise. Pixels are found by projecting the ray
    along ``k_in + Q``, so an off-sphere `Q` still gives the pixel it would
    illuminate.
    """
    q = np.asarray(q, dtype=float)
    k_out = beam.k_in + q
    excitation = np.linalg.norm(k_out, axis=-1) - beam.k0
    return detector.intersect(k_out), excitation


# ── sample rotation ────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Goniometer:
    """Sample rotation: lab ↔ sample(crystal) frame.

    ``axis`` is the rotation axis in the lab frame (default ``+z``, vertical for
    the default beam-along-y frame). ``u_matrix`` is the fixed crystal
    orientation at zero angle, mapping crystal-frame vectors to the lab frame
    (``Q_lab = U · Q_crystal`` at φ = 0); leave it as the identity if the crystal
    frame *is* the sample frame.

    At angle φ the sample has been rotated by ``R(φ)``, so::

        Q_lab    = R(φ) · U · Q_crystal
        Q_crystal = U⁻¹ · R(φ)⁻¹ · Q_lab
    """

    axis: tuple = (0.0, 0.0, 1.0)
    u_matrix: np.ndarray = field(default_factory=lambda: np.eye(3))

    def __post_init__(self):
        object.__setattr__(self, "u_matrix", np.asarray(self.u_matrix, dtype=float).reshape(3, 3))

    def rotation(self, angle_deg: float) -> np.ndarray:
        """Lab-frame rotation matrix at `angle_deg`."""
        return rotation_matrix(self.axis, angle_deg)

    def to_sample(self, q_lab, angle_deg: float) -> np.ndarray:
        """Lab-frame `Q` → crystal frame at rotation `angle_deg`."""
        M = (self.rotation(angle_deg) @ self.u_matrix).T   # inverse of an orthogonal matrix
        return apply_rotation(M, q_lab)

    def to_lab(self, q_sample, angle_deg: float) -> np.ndarray:
        """Crystal-frame `Q` → lab frame at rotation `angle_deg`."""
        return apply_rotation(self.rotation(angle_deg) @ self.u_matrix, q_sample)


def q_map_sample(detector: Detector, beam: Beam, gonio: Goniometer,
                 angle_deg: float) -> np.ndarray:
    """Per-pixel `Q` in the **sample/crystal** frame for one image of a rotation series."""
    return gonio.to_sample(q_map(detector, beam), angle_deg)
