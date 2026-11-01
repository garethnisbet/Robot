"""Stage 3 in simulation: brick detection on viewer-rendered images.

Random layouts of studs-up bricks in ten LEGO colours, seen from several angles at
1920x1080. The camera pose comes from the base mat (not ground truth), so these
are end-to-end checks. Skipped without ../VRRobotController and Chrome.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "apps"))
from legosort import simserver  # noqa: E402
from legosort.boards import BaseMat  # noqa: E402
from legosort.detect import cluster_colours, detect_bricks  # noqa: E402
from legosort.lego import random_layout, yaw_diff  # noqa: E402
from legosort.tracker import BaseTracker  # noqa: E402

pytestmark = pytest.mark.skipif(not simserver.available(), reason="VRRobotController viewer or Chrome not available")

VIEWS = [(-90, 60, 600), (-20, 45, 650), (60, 70, 550), (-140, 40, 700)]
REGION = (143, 317, -148, 148)


@pytest.fixture(scope="module")
def renders():
    from sim_view import SAFE_POSE, camera_at
    from legosort.sim import Viewer, build_scene, place_bricks
    mat = BaseMat.load()
    out = []
    with simserver.HeadlessViewer() as hv:
        v = Viewer(hv.ws_url, hv.session)
        v.set_joints(SAFE_POSE)
        build_scene(v, mat)
        for seed in (2, 3):
            bricks = random_layout(14, REGION, np.random.default_rng(seed))
            place_bricks(v, bricks)
            for az, el, d in VIEWS:
                cam = camera_at(az, el, d)
                cam.width, cam.height = 1920, 1080
                img, K = v.capture(cam, exposure=0.15)
                out.append((seed, (az, el, d), bricks, img, K))
        v.close()
    return out


def match(dets, bricks, tol=6.0):
    pairs, used = [], set()
    for b in bricks:
        dist = [np.hypot(d.x - b.x, d.y - b.y) if i not in used else np.inf for i, d in enumerate(dets)]
        if dets and min(dist) < tol:
            i = int(np.argmin(dist))
            used.add(i)
            pairs.append((b, dets[i]))
        else:
            pairs.append((b, None))
    return pairs, len(dets) - len(used)


def test_detection_per_view(renders):
    mat = BaseMat.load()
    report = []
    for seed, view, bricks, img, K in renders:
        pose = BaseTracker(mat, K, np.zeros(5)).estimate(img)
        assert pose is not None
        dets = detect_bricks(img, K, np.zeros(5), pose.T_cam_robot, mat=mat)
        pairs, extra = match(dets, bricks)
        missed = [f"{b.colour} {b.nx}x{b.ny}" for b, d in pairs if d is None]
        assert not missed, f"seed {seed} view {view}: missed {missed}"
        assert extra == 0, f"seed {seed} view {view}: {extra} false detections"
        for b, d in pairs:
            long_yaw = b.yaw + (90 if b.ny > b.nx else 0)
            period = 90 if b.nx == b.ny else 180
            where = f"seed {seed} view {view} {b.colour} {b.nx}x{b.ny}"
            assert np.hypot(d.x - b.x, d.y - b.y) < 1.0, where
            assert yaw_diff(d.yaw, long_yaw, period) < 2.0, where
            assert (d.nx, d.ny) == (max(b.nx, b.ny), min(b.nx, b.ny)), where
        report.append(max(np.hypot(d.x - b.x, d.y - b.y) for b, d in pairs))
    print("worst position error per view (mm):", np.round(report, 2))


def test_colour_clusters_match_true_colours(renders):
    """Each view's clusters partition the bricks exactly by true colour."""
    mat = BaseMat.load()
    for seed, view, bricks, img, K in renders:
        pose = BaseTracker(mat, K, np.zeros(5)).estimate(img)
        dets = detect_bricks(img, K, np.zeros(5), pose.T_cam_robot, mat=mat)
        cluster_colours(dets)
        pairs, _ = match(dets, bricks)
        by_cluster, by_colour = {}, {}
        for b, d in pairs:
            by_cluster.setdefault(d.cluster, set()).add(b.colour)
            by_colour.setdefault(b.colour, set()).add(d.cluster)
        mixed = {k: v for k, v in by_cluster.items() if len(v) > 1}
        split = {k: v for k, v in by_colour.items() if len(v) > 1}
        assert not mixed and not split, f"seed {seed} view {view}: mixed {mixed} split {split}"
