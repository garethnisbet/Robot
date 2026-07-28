"""Geometric identities the pixel <-> Q transformation must satisfy."""

import numpy as np
import pytest

from geometry import (Beam, Detector, Goniometer, HC_KEV_ANGSTROM, pixel_from_q,
                      q_from_pixel, q_map, rotation_matrix, unit)


# ── fixtures: a plain detector and a deliberately awkward one ──────────────
def plain_detector(beam):
    return Detector.from_beam_centre(distance=150.0, shape=(256, 320),
                                     pixel_size=0.15, beam=beam)


def awkward_detector(beam):
    """Non-square pixels, off-centre origin pixel, tilted in all three axes."""
    return Detector.from_beam_centre(distance=95.0, shape=(256, 320),
                                     pixel_size=(0.172, 0.135),
                                     origin_pixel=(103.0, 171.5), beam=beam,
                                     tilt=(7.0, -11.0, 23.0))


@pytest.fixture(params=["plain", "awkward"])
def geom(request):
    beam = Beam(energy_kev=15.0)
    det = plain_detector(beam) if request.param == "plain" else awkward_detector(beam)
    return det, beam


# ── beam ───────────────────────────────────────────────────────────────────
def test_wavelength_and_k0():
    beam = Beam(energy_kev=12.398419843320026)
    assert beam.wavelength == pytest.approx(1.0)
    assert beam.k0 == pytest.approx(1.0)
    assert Beam(energy_kev=15.0).k0 == pytest.approx(15.0 / HC_KEV_ANGSTROM)
    assert Beam(energy_kev=15.0, two_pi=True).k0 == pytest.approx(
        2 * np.pi * 15.0 / HC_KEV_ANGSTROM)
    np.testing.assert_allclose(Beam().k_in, [0, Beam().k0, 0])


# ── detector placement ─────────────────────────────────────────────────────
def test_origin_pixel_defaults_to_image_centre():
    beam = Beam()
    det = Detector.from_beam_centre(distance=100.0, shape=(256, 320), pixel_size=0.1, beam=beam)
    np.testing.assert_allclose(det.beam_centre(beam), [127.5, 159.5], atol=1e-9)


def test_origin_pixel_sits_on_the_beam_at_the_stated_distance():
    beam = Beam()
    det = Detector.from_beam_centre(distance=137.0, shape=(256, 320), pixel_size=(0.2, 0.1),
                                    origin_pixel=(60.0, 200.0), beam=beam)
    p = det.lab_from_pixel(60.0, 200.0)
    np.testing.assert_allclose(p, 137.0 * beam.s0, atol=1e-9)
    np.testing.assert_allclose(det.beam_centre(beam), [60.0, 200.0], atol=1e-9)


def test_pixel_pitch_is_honoured_per_axis():
    det = Detector.from_beam_centre(distance=100.0, shape=(64, 64), pixel_size=(0.2, 0.05))
    step_slow = det.lab_from_pixel(1, 0) - det.lab_from_pixel(0, 0)
    step_fast = det.lab_from_pixel(0, 1) - det.lab_from_pixel(0, 0)
    assert np.linalg.norm(step_slow) == pytest.approx(0.2)
    assert np.linalg.norm(step_fast) == pytest.approx(0.05)
    assert step_slow @ step_fast == pytest.approx(0.0, abs=1e-12)


def test_tilt_about_origin_pixel_keeps_it_on_the_beam():
    beam = Beam()
    det = Detector.from_beam_centre(distance=120.0, shape=(256, 256), pixel_size=0.1,
                                    origin_pixel=(100.0, 30.0), beam=beam,
                                    tilt=(12.0, -5.0, 40.0), pivot="origin_pixel")
    np.testing.assert_allclose(det.beam_centre(beam), [100.0, 30.0], atol=1e-9)
    np.testing.assert_allclose(det.lab_from_pixel(100.0, 30.0), 120.0 * beam.s0, atol=1e-9)


def test_tilt_about_sample_swings_the_panel_but_keeps_the_distance():
    beam = Beam()
    det = Detector.from_beam_centre(distance=120.0, shape=(256, 256), pixel_size=0.1,
                                    beam=beam, tilt=(0.0, 0.0, 30.0), pivot="sample")
    p = det.lab_from_pixel(127.5, 127.5)
    assert np.linalg.norm(p) == pytest.approx(120.0)              # arm length unchanged
    # ...and it now sits 30 deg off the beam, so the direct beam misses that pixel
    assert np.degrees(np.arccos(unit(p) @ beam.s0)) == pytest.approx(30.0)
    assert not np.allclose(det.beam_centre(beam), [127.5, 127.5], atol=1e-6)


def test_non_perpendicular_axes_rejected():
    with pytest.raises(ValueError):
        Detector(fast_axis=[1, 0, 0], slow_axis=[1, 1, 0])


def test_pixel_lab_round_trip(geom):
    det, _ = geom
    rng = np.random.default_rng(0)
    s = rng.uniform(0, det.shape[0] - 1, 200)
    f = rng.uniform(0, det.shape[1] - 1, 200)
    back = det.pixel_from_lab(det.lab_from_pixel(s, f))
    np.testing.assert_allclose(back, np.stack([s, f], axis=-1), atol=1e-9)


def test_intersect_inverts_lab_from_pixel(geom):
    """A ray fired at a pixel comes back to that pixel."""
    det, _ = geom
    rng = np.random.default_rng(1)
    s = rng.uniform(0, det.shape[0] - 1, 200)
    f = rng.uniform(0, det.shape[1] - 1, 200)
    direction = det.lab_from_pixel(s, f)                  # not normalised on purpose
    np.testing.assert_allclose(det.intersect(direction),
                               np.stack([s, f], axis=-1), atol=1e-8)


def test_intersect_rejects_rays_behind_the_sample(geom):
    det, _ = geom
    assert np.all(np.isnan(det.intersect(-det.lab_from_pixel(10.0, 10.0))))


# ── the transformation ─────────────────────────────────────────────────────
def test_beam_centre_maps_to_zero_q(geom):
    det, beam = geom
    s, f = det.beam_centre(beam)
    np.testing.assert_allclose(q_from_pixel(det, beam, s, f), np.zeros(3), atol=1e-12)


def test_every_pixel_lies_on_the_ewald_sphere(geom):
    det, beam = geom
    q = q_map(det, beam)
    np.testing.assert_allclose(np.linalg.norm(q + beam.k_in, axis=-1), beam.k0, rtol=1e-12)


def test_q_magnitude_matches_braggs_law(geom):
    """|Q| = 2 sin(theta) / lambda, with 2theta read off the pixel directions."""
    det, beam = geom
    q = q_map(det, beam)
    two_theta = np.radians(det.two_theta(beam))
    np.testing.assert_allclose(np.linalg.norm(q, axis=-1),
                               2 * np.sin(two_theta / 2) / beam.wavelength,
                               rtol=1e-9, atol=1e-12 * beam.k0)


def test_q_bisects_the_scattering_triangle(geom):
    """Elastic scattering forces Q.(k_in + k_out) = 0."""
    det, beam = geom
    q = q_map(det, beam)
    k_out = q + beam.k_in
    dot = np.einsum("...i,...i->...", q, k_out + beam.k_in)
    assert np.abs(dot).max() < 1e-12 * beam.k0**2


def test_pixel_q_round_trip(geom):
    """pixel -> Q -> pixel is the identity, and lands back on the Ewald sphere."""
    det, beam = geom
    rng = np.random.default_rng(2)
    s = rng.uniform(0, det.shape[0] - 1, 500)
    f = rng.uniform(0, det.shape[1] - 1, 500)
    q = q_from_pixel(det, beam, s, f)
    px, excitation = pixel_from_q(det, beam, q)
    np.testing.assert_allclose(px, np.stack([s, f], axis=-1), atol=1e-8)
    assert np.abs(excitation).max() < 1e-12


def test_q_scales_with_pixel_size_not_with_distance():
    """Doubling both distance and pixel pitch views the same Q — the two are degenerate."""
    beam = Beam()
    a = Detector.from_beam_centre(distance=100.0, shape=(64, 64), pixel_size=0.1, beam=beam)
    b = Detector.from_beam_centre(distance=200.0, shape=(64, 64), pixel_size=0.2, beam=beam)
    np.testing.assert_allclose(q_map(a, beam), q_map(b, beam), atol=1e-12)


def test_two_pi_convention_only_rescales():
    beam1 = Beam(energy_kev=15.0)
    beam2 = Beam(energy_kev=15.0, two_pi=True)
    det = plain_detector(beam1)
    np.testing.assert_allclose(q_map(det, beam2), 2 * np.pi * q_map(det, beam1),
                               rtol=1e-9, atol=1e-12 * beam1.k0)


def test_off_ewald_q_reports_its_excitation_error(geom):
    det, beam = geom
    q = q_from_pixel(det, beam, 30.0, 40.0)
    k_out = q + beam.k_in
    stretched = 1.01 * k_out - beam.k_in                  # push 1% off the sphere
    px, excitation = pixel_from_q(det, beam, stretched)
    np.testing.assert_allclose(px, [30.0, 40.0], atol=1e-8)   # same ray, same pixel
    assert excitation == pytest.approx(0.01 * beam.k0)


# ── solid angle ────────────────────────────────────────────────────────────
def test_solid_angle_matches_the_analytic_rectangle():
    """Summed pixel solid angles equal the closed form for an on-axis rectangle."""
    beam = Beam()
    d, n, p = 100.0, 200, 0.2
    det = Detector.from_beam_centre(distance=d, shape=(n, n), pixel_size=p, beam=beam)
    a = b = n * p / 2.0                                    # half-widths
    analytic = 4 * np.arctan(a * b / (d * np.sqrt(a**2 + b**2 + d**2)))
    assert det.solid_angle().sum() == pytest.approx(analytic, rel=1e-4)


def test_solid_angle_falls_off_towards_the_corners():
    det = Detector.from_beam_centre(distance=60.0, shape=(101, 101), pixel_size=0.2)
    sa = det.solid_angle()
    assert sa[50, 50] == sa.max()
    assert sa[0, 0] < sa[50, 50]


# ── goniometer ─────────────────────────────────────────────────────────────
def test_goniometer_round_trip():
    g = Goniometer(axis=(0, 0, 1), u_matrix=rotation_matrix([1, 2, 3], 37.0))
    rng = np.random.default_rng(3)
    q = rng.normal(size=(50, 3))
    np.testing.assert_allclose(g.to_sample(g.to_lab(q, 23.5), 23.5), q, atol=1e-12)


def test_rotation_preserves_length_and_the_axis():
    g = Goniometer(axis=(0, 0, 1))
    q = np.array([[0.3, -0.1, 0.7]])
    r = g.to_lab(q, 41.0)
    np.testing.assert_allclose(np.linalg.norm(r), np.linalg.norm(q))
    assert r[0, 2] == pytest.approx(q[0, 2])               # component along the axis is fixed
    np.testing.assert_allclose(g.to_lab(q, 360.0), q, atol=1e-12)


def test_identity_goniometer_is_a_no_op():
    g = Goniometer()
    q = np.array([[0.1, 0.2, 0.3]])
    np.testing.assert_allclose(g.to_sample(q, 0.0), q, atol=1e-15)
