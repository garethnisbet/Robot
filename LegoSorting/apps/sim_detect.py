"""Brick detection in the twin: lay out bricks, look with a virtual camera, show what was found.

    python3 apps/sim_detect.py --url ws://127.0.0.1:8099/ws [--seed 2] [--view -20 45 650]
    python3 apps/sim_detect.py                  # headless: just saves the images

With --plates, detections also appear in the viewer as thin translucent plates
just above each brick, coloured by colour cluster. The camera image with its overlay is saved to
data/sim_detect.png, and a 3D view of the twin with the detection plates to
data/sim_detect_3d.png.
"""
import argparse
import json
import sys
import urllib.request
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from legosort import overlay  # noqa: E402
from legosort.boards import BaseMat  # noqa: E402
from legosort.camera import DATA  # noqa: E402
from legosort.detect import cluster_colours, detect_bricks  # noqa: E402
from legosort.geometry import make_T, rot_z  # noqa: E402
from legosort.lego import random_layout  # noqa: E402
from legosort.sim import (SIM_PREFIX, MeshBuilder, Viewer, build_scene,  # noqa: E402
                          place_bricks, to_api)
from legosort.tracker import BaseTracker  # noqa: E402
from sim_view import SAFE_POSE, camera_at  # noqa: E402

PALETTE = [(1, 0, 1), (0, 1, 1), (1, 0.5, 0), (0.3, 0.3, 1), (1, 1, 0), (0, 1, 0.3),
           (1, 0.3, 0.3), (0.6, 0.2, 1), (0.2, 0.8, 0.8), (1, 0.8, 0.5)]


def show_detections(viewer, dets, z=13.0):
    viewer.remove_objects(SIM_PREFIX + "det_")
    for i, d in enumerate(dets):
        mb = MeshBuilder()
        mb.box(make_T(rot_z(d.yaw), [d.x, d.y, z]), (d.length, d.width, 0.6), PALETTE[d.cluster % len(PALETTE)])
        viewer.add_glb(f"{SIM_PREFIX}det_{i:02d}_c{d.cluster}_{d.nx}x{d.ny}", mb.glb(),
                       {"opacity": 0.55, "roughness": 1.0, "metalness": 0.0})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", help="viewer WebSocket URL (default: start a headless viewer)")
    ap.add_argument("--session", help="viewer session id (default: the first one)")
    ap.add_argument("--seed", type=int, default=2)
    ap.add_argument("--plates", action="store_true", help="show detections as plates in the viewer")
    ap.add_argument("--n", type=int, default=14)
    ap.add_argument("--view", type=float, nargs=3, default=(-20, 45, 650), metavar=("AZ", "EL", "DIST"))
    args = ap.parse_args()
    if args.url:
        return run(args)
    from legosort.simserver import HeadlessViewer
    with HeadlessViewer() as hv:
        args.url, args.session = hv.ws_url, hv.session
        run(args)


def run(args):
    session = args.session
    if not session:
        http = args.url.replace("ws://", "http://").rsplit("/ws", 1)[0]
        session = json.loads(urllib.request.urlopen(http + "/sessions").read())[0]["id"]
    v = Viewer(args.url, session)
    mat = BaseMat.load()
    v.set_joints(SAFE_POSE)
    build_scene(v, mat)
    bricks = random_layout(args.n, (143, 317, -148, 148), np.random.default_rng(args.seed))
    place_bricks(v, bricks)

    cam = camera_at(*args.view)
    cam.width, cam.height = 1920, 1080
    img, K = v.capture(cam, exposure=0.15)
    pose = BaseTracker(mat, K, np.zeros(5)).estimate(img)
    dets = detect_bricks(img, K, np.zeros(5), pose.T_cam_robot, mat=mat)
    cluster_colours(dets)
    if args.plates:
        show_detections(v, dets)
    else:
        v.remove_objects(SIM_PREFIX + "det_")

    # point the viewer's own camera at the mat so the result is easy to see
    v.request({"cmd": "setCamera", "position": to_api([560, -260, 420]), "target": to_api([230, 0, 0])}, "camera")
    from legosort.sim import SimCamera
    view3d = SimCamera(eye=(520, -330, 330), target=(215, 10, 0), up=(0, 0, 1), fov_deg=50, width=1600, height=900)
    img3d, _ = v.capture(view3d, exposure=0.15)

    for d in dets:
        pts = cv2.projectPoints(np.column_stack([d.corners, np.full(4, 9.9)]),
                                cv2.Rodrigues(pose.T_cam_robot[:3, :3])[0], pose.T_cam_robot[:3, 3],
                                K, np.zeros(5))[0].reshape(-1, 2)
        c = tuple(int(255 * x) for x in PALETTE[d.cluster % len(PALETTE)][::-1])
        overlay.polyline(img, pts, c, thickness=2)
        cv2.putText(img, f"{d.nx}x{d.ny} c{d.cluster}", (int(pts[:, 0].max()) + 4, int(pts[:, 1].min())),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2, cv2.LINE_AA)
    overlay.text_panel(img, [f"{len(dets)} bricks found ({len(bricks)} placed), "
                             f"{len(set(d.cluster for d in dets))} colour groups"])
    DATA.mkdir(exist_ok=True)
    cv2.imwrite(str(DATA / "sim_detect.png"), img)
    cv2.imwrite(str(DATA / "sim_detect_3d.png"), img3d)
    print(f"{len(dets)} detections, {len(set(d.cluster for d in dets))} colour clusters; image data/sim_detect.png")
    for d in sorted(dets, key=lambda d: d.cluster):
        print(f"  cluster {d.cluster}: {d.nx}x{d.ny} at ({d.x:6.1f}, {d.y:6.1f}) yaw {d.yaw:6.1f}  Lab {np.round(d.lab).astype(int)}")
    v.close()


if __name__ == "__main__":
    main()
