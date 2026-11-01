"""Pour a pile of bricks onto the mat with physics (gravity + contact) and show it in the twin.

    python3 apps/sim_pile.py --url ws://127.0.0.1:8099/ws [--n 30] [--seed 1]
    python3 apps/sim_pile.py                       # headless: just saves the images
    python3 apps/sim_pile.py --remove 5            # then lift the top 5 bricks off one by one

The pile is simulated in MuJoCo (legosort/physics.py), then each brick is sent to
the viewer at its full 3D pose. Images: data/sim_pile.png (overhead camera) and
data/sim_pile_3d.png (oblique view). With --remove, the highest brick is taken away
and the pile settles again each time, as a pick would leave it; the viewer is
updated after each removal (images data/sim_pile_after.png / _3d).
"""
import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from legosort.boards import BaseMat  # noqa: E402
from legosort.camera import DATA  # noqa: E402
from legosort.lego import COLOURS, Brick  # noqa: E402
from legosort.physics import PileSim  # noqa: E402
from legosort.sim import SimCamera, Viewer, build_scene, place_bricks, to_api  # noqa: E402
from sim_view import SAFE_POSE, camera_at  # noqa: E402

SIZES = ((2, 4), (2, 2), (2, 3), (1, 2), (2, 6), (1, 4), (1, 1), (1, 3))


def random_bricks(n, rng, colours=None):
    colours = colours or [c for c in COLOURS if c not in ("white", "tan")]
    return [Brick(*SIZES[rng.integers(len(SIZES))], colours[rng.integers(len(colours))]) for _ in range(n)]


def summary(pile):
    up = sum(b.tilt < 20 for b in pile)
    down = sum(b.tilt > 160 for b in pile)
    side = sum(70 < b.tilt < 110 for b in pile)
    on_mat = sum(_lowest(b) < 0.5 for b in pile)
    return (f"{len(pile)} bricks: {up} studs up, {down} upside down, {side} on their side, "
            f"{len(pile) - up - down - side} tilted; {on_mat} touch the mat, "
            f"top {max(_highest(b) for b in pile):.1f} mm")


def _corners(b):
    L, W, H = b.length, b.width, b.height
    c = np.array([[x, y, z] for x in (-L / 2, L / 2) for y in (-W / 2, W / 2) for z in (0, H)])
    T = b.T
    return c @ T[:3, :3].T + T[:3, 3]


def _lowest(b):
    return _corners(b)[:, 2].min()


def _highest(b):
    return _corners(b)[:, 2].max()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", help="viewer WebSocket URL (default: start a headless viewer)")
    ap.add_argument("--session", help="viewer session id (default: the first one)")
    ap.add_argument("--n", type=int, default=30)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--centre", type=float, nargs=2, default=(230.0, 0.0), metavar=("X", "Y"))
    ap.add_argument("--spread", type=float, default=15.0, help="mm, 1 sigma of where bricks are dropped")
    ap.add_argument("--remove", type=int, default=0, help="then lift off this many top bricks")
    ap.add_argument("--view", type=float, nargs=3, default=(-20, 60, 550), metavar=("AZ", "EL", "DIST"))
    args = ap.parse_args()
    if args.url:
        return run(args)
    from legosort.simserver import HeadlessViewer
    with HeadlessViewer() as hv:
        args.url, args.session = hv.ws_url, hv.session
        run(args)


def run(args):
    rng = np.random.default_rng(args.seed)
    t0 = time.time()
    mat = BaseMat.load()
    sim = PileSim(random_bricks(args.n, rng))
    sim.pour(centre=args.centre, spread=args.spread, rng=rng)
    pile = sim.bricks()
    print(f"poured in {time.time() - t0:.1f} s: {summary(pile)}")

    session = args.session
    if not session:
        http = args.url.replace("ws://", "http://").rsplit("/ws", 1)[0]
        session = json.loads(urllib.request.urlopen(http + "/sessions").read())[0]["id"]
    v = Viewer(args.url, session)
    v.set_joints(SAFE_POSE)
    build_scene(v, mat)
    cx, cy = args.centre
    v.request({"cmd": "setCamera", "position": to_api([cx + 250, cy - 220, 260]), "target": to_api([cx, cy, 0])},
              "camera")
    DATA.mkdir(exist_ok=True)

    def snap(tag):
        place_bricks(v, sim.bricks())
        cam = camera_at(*args.view)
        cam.width, cam.height = 1600, 900
        img, _ = v.capture(cam, exposure=0.15)
        view3d = SimCamera(eye=(cx + 200, cy - 170, 150), target=(cx, cy, 5), up=(0, 0, 1), fov_deg=40,
                           width=1600, height=900)
        img3d, _ = v.capture(view3d, exposure=0.15)
        cv2.imwrite(str(DATA / f"sim_pile{tag}.png"), img)
        cv2.imwrite(str(DATA / f"sim_pile{tag}_3d.png"), img3d)

    snap("")
    print("images data/sim_pile.png, data/sim_pile_3d.png")
    for _ in range(args.remove):
        idx = sim.indices()
        top = idx[int(np.argmax([_highest(b) for b in sim.bricks()]))]
        b = sim.proto[top]
        sim.remove(top)
        t = sim.settle()
        print(f"lifted {b.colour} {b.nx}x{b.ny}; pile settled in {t:.2f} s: {summary(sim.bricks())}")
    if args.remove:
        snap("_after")
        print("images data/sim_pile_after.png, data/sim_pile_after_3d.png")
    v.close()


if __name__ == "__main__":
    main()
