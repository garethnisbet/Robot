"""The robot sorts a physics-simulated pile until every brick is on its colour's spot.

    python3 apps/sim_pick.py --url ws://127.0.0.1:8099/ws     # watch it in your viewer
    python3 apps/sim_pick.py --video                          # headless; data/sim_pick.mp4
    python3 apps/sim_pick.py --n 20 --seed 3 --no-viewer      # physics only, fast; prints the steps

Each step: grip a brick from the side if one can be gripped (outermost first) and drop
it on its colour's spot; otherwise push a low brick towards its spot with the flat side
of the gripper body, in short legs that steer round the others (1x1, 1x3, 1x4 bricks
and 2xN bricks on their side can never be gripped by the bare jaws). A brick counts as
sorted once it is within SORTED mm of its own spot.

Everything is physical (legosort/physics.py): the arm follows its joint path exactly
and pushes whatever it meets; the fingers squeeze with a set force and the brick is
held by friction only, so a poor grasp can slip, and neighbours get disturbed.
Grasp choice (legosort/pick.py) uses the simulator's true brick poses for now, not
the camera.
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
from legosort.physics import PileSim  # noqa: E402
from legosort.pick import HOME, pick_side, pile_top, plan_push, push, side_grasps  # noqa: E402
from legosort.sim import PhysicsView, SimCamera, Viewer, build_scene, to_api  # noqa: E402
from sim_pile import random_bricks, summary  # noqa: E402

# Drop spots, one per colour, on an arc 235 mm from the base (reachable with the side-mounted
# gripper: the TCP is 21 mm off the flange axis), every 15 deg, >= 150 mm from the default
# pile centre (165, 0). That leaves the band between the pile and the spots free for spreading the pile.
TRAVEL_CLEAR = 25.0      # mm of TCP height above the pile's top while travelling (a held brick hangs below)
SPOT_R = 235.0


def _arc(r, a):
    return (round(r * np.cos(np.radians(a)), 1), round(r * np.sin(np.radians(a)), 1))


# nearest the pile first: the first colours (in pour order) get the shortest trips
BINS = [_arc(SPOT_R, a) for a in (-40, 40, -55, 55, -70, 70, -85, 85, -100, 100)]
SPREAD = 40.0            # mm: a push that has to shove others moves a brick this far out from the pile
SPREAD_BAND = (115.0, SPOT_R - 15.0)     # mm from the robot axis: where spreading may leave a brick
SPOT_BAND = (125.0, SPOT_R + 15.0)       # ... and where a push towards a spot may
MAX_SPREADS = 6          # spreading pushes per brick between bricks getting sorted (else they ping-pong)
PUSH_SHORT = 15.0        # mm: a pushed brick is aimed this far short of its spot, not into the heap
SORTED = 30.0            # mm: a brick this close to its colour's spot is done


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", help="viewer WebSocket URL (default: start a headless viewer)")
    ap.add_argument("--session", help="viewer session id (default: the first one)")
    ap.add_argument("--n", type=int, default=20, help="bricks in the pile")
    ap.add_argument("--steps", type=int, default=200, help="give up after this many grips + pushes")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--centre", type=float, nargs=2, default=(165.0, 0.0), metavar=("X", "Y"))
    ap.add_argument("--video", action="store_true", help="record data/sim_pick.mp4 (slow)")
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--no-viewer", action="store_true", help="physics only, as fast as it runs")
    ap.add_argument("--fingers", action="store_true", help="printed stepped fingers on the jaws")
    args = ap.parse_args()
    if args.url or args.no_viewer:
        return run(args)
    from legosort.simserver import HeadlessViewer
    with HeadlessViewer() as hv:
        args.url, args.session = hv.ws_url, hv.session
        run(args)


def run(args):
    rng = np.random.default_rng(args.seed)
    t0 = time.time()
    from legosort.megp25e import MEGP25E
    sim = PileSim(random_bricks(args.n, rng), robot=True, q0=HOME,
                  gripper=MEGP25E(fingers=getattr(args, "fingers", False)))
    sim.pour(centre=args.centre, spread=15.0, rng=rng)
    print(f"poured in {time.time() - t0:.1f} s: {summary(sim.bricks())}")

    v = writer = None
    cx, cy = args.centre
    if not args.no_viewer:
        session = args.session
        if not session:
            http = args.url.replace("ws://", "http://").rsplit("/ws", 1)[0]
            session = json.loads(urllib.request.urlopen(http + "/sessions").read())[0]["id"]
        v = Viewer(args.url, session)
        build_scene(v, BaseMat.load())
        v.set_joints(HOME)
        cx, cy = args.centre
        v.request({"cmd": "setCamera", "position": to_api([cx + 330, cy - 330, 330]), "target": to_api([cx - 40, cy, 40])},
                  "camera")

        writer = None
        if args.video:
            cam = SimCamera(eye=(cx + 170, cy - 280, 190), target=(cx - 15, cy, 25), up=(0, 0, 1), fov_deg=50,
                            width=1280, height=720)
            DATA.mkdir(exist_ok=True)
            path = str(DATA / "sim_pick.mp4")
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (cam.width, cam.height))

            def record(_sim):
                img, _ = v.capture(cam, exposure=0.15)
                writer.write(img)
        view = PhysicsView(v, sim, on_frame=record if writer else None)
        sim.frame_rate = args.fps if writer else 30.0
        sim.on_frame = view.update
        if not writer:                       # live viewer: run at roughly real time
            start = (time.monotonic(), sim.time)

            def paced(s):
                view.update()
                lag = (s.time - start[1]) - (time.monotonic() - start[0])
                if lag > 0:
                    time.sleep(lag)
            sim.on_frame = paced

    # No home pose between picks: each cycle starts from the last drop spot and travels at a
    # height set by the pile's current top (plus room for a hanging brick), so the arm
    # comes down as the pile shrinks. Grips come from the side, outermost brick first.
    def travel_z():
        return pile_top(sim, args.centre) + TRAVEL_CLEAR

    def feasible(g, tz):
        out = g.centre - g.approach * 20
        try:
            q = sim.ik([out[0], out[1], max(tz, out[2])], g.R)
            q = sim.ik(out, g.R, q)
            q = sim.ik(g.centre, g.R, q)
            sim.ik([g.centre[0], g.centre[1], max(tz, g.centre[2] + 15)], g.R, q)
        except ValueError:
            return False
        return sim.clearance(q) >= 0.5

    colours = sorted({b.colour for b in sim.proto}, key=[b.colour for b in sim.proto].index)
    if len(colours) > len(BINS):
        raise SystemExit(f"{len(colours)} colours but only {len(BINS)} drop spots")
    spot = {c: np.array(BINS[k]) for k, c in enumerate(colours)}

    def unsorted():
        idx, bricks = sim.indices(), sim.bricks()
        return [k for k, b in enumerate(bricks)
                if np.hypot(*(b.T[:2, 3] - spot[b.colour])) > SORTED], idx, bricks

    t_start = sim.time
    fails = {}                     # sim index -> grips that missed or pushes that did nothing
    spreads = {}                   # sim index -> spreading pushes
    held = missed = pushes = 0
    left = None
    for step in range(args.steps):
        todo, idx_map, bricks = unsorted()
        if left is not None and len(todo) < left:
            spreads.clear()        # progress: the pile has changed, spreading may help again
        left = len(todo)
        if not todo:
            print(f"all {len(bricks)} bricks sorted")
            break
        tz = travel_z()
        names = {k: f"{bricks[k].colour} {sim.proto[int(idx_map[k])].nx}x{sim.proto[int(idx_map[k])].ny}"
                 for k in todo}
        g = next((g for g in side_grasps(bricks, sim.gripper, args.centre, travel_z=tz)
                  if g.index in todo and fails.get(int(idx_map[g.index]), 0) < 3 and feasible(g, tz)), None)
        t0 = sim.time
        if g is not None:
            i = int(idx_map[g.index])
            dest = spot[sim.proto[i].colour]
            how = "" if len(sim.gripper.steps) == 1 else (" (narrow step)" if g.step < 14 else " (wide step)")
            print(f"{step + 1}: grip {names[g.index]}{how}, {g.radius:.0f} mm from the pile centre "
                  f"-> ({dest[0]:.0f}, {dest[1]:.0f}); travelling at {tz:.0f} mm")
            try:
                ok = pick_side(sim, i, g, dest, tz)
            except ValueError as e:
                print(f"  gave up: {e}")
                ok = False
            held += ok
            missed += not ok
            if ok:
                now = sim.bricks()[list(sim.indices()).index(i)]
                print(f"  landed {np.hypot(*(now.T[:2, 3] - dest)):.0f} mm from its spot")
            if not ok:
                fails[i] = fails.get(i, 0) + 1
            print(f"  {sim.time - t0:.1f} s")
            continue
        # nothing to grip: push the outermost low brick that can move towards its spot
        # A clean push (touching nothing else) towards the brick's spot if any brick has one;
        # otherwise spread the pile: push a brick outward from the middle of what is left,
        # shoving its neighbours if need be (never towards a spot).
        mid = np.mean([bricks[k].T[:2, 3] for k in todo], axis=0)
        order = sorted(todo, key=lambda k: -np.hypot(*(bricks[k].T[:2, 3] - mid)))
        plan, kind = None, ""
        for k in order:
            if fails.get(int(idx_map[k]), 0) >= 6:
                continue
            c = bricks[k].T[:2, 3]
            sp = spot[bricks[k].colour]
            goal = sp + (c - sp) / max(np.linalg.norm(c - sp), 1e-6) * PUSH_SHORT
            heap = [j for j, o in enumerate(bricks) if o.colour == bricks[k].colour and j not in todo]
            plan = plan_push(sim, bricks, k, goal, tz, band=SPOT_BAND, ignore=heap)
            if plan is not None:
                kind = " towards its spot"
                break
        if plan is None:
            # spread the pile: of all the low bricks and 12 directions, the push (shoving
            # neighbours if need be, never towards a spot) that leaves a brick most room
            best = None
            for k in order:
                sk = int(idx_map[k])
                if fails.get(sk, 0) >= 6 or spreads.get(sk, 0) >= MAX_SPREADS:
                    continue
                c = bricks[k].T[:2, 3]
                out = c - mid
                goal = c + out / max(np.linalg.norm(out), 1e-6) * SPREAD
                r = plan_push(sim, bricks, k, goal, tz, contact=True, spots=list(spot.values()),
                              band=SPREAD_BAND, turns=range(0, 360, 30), best=True)
                if r is not None and (best is None or r[1] > best[2]):
                    best = (k, r[0], r[1])
            if best is not None:
                k, plan, room = best
                kind = f" to spread the pile ({room:.0f} mm clear)"
        if plan is None:
            print(f"stuck: no brick can be gripped or pushed ({len(todo)} left)")
            break
        i = int(idx_map[k])
        before = bricks[k].T[:2, 3].copy()
        dest = spot[bricks[k].colour]
        print(f"{step + 1}: push {names[k]}{kind}")
        try:
            push(sim, plan)
        except ValueError as e:
            print(f"  gave up: {e}")
        pushes += 1
        if kind.startswith(" to spread"):
            spreads[i] = spreads.get(i, 0) + 1
        after = sim.bricks()[list(sim.indices()).index(i)].T[:2, 3]
        moved = np.linalg.norm(after - before)
        if moved < 5:
            fails[i] = fails.get(i, 0) + 1
        print(f"  moved {moved:.0f} mm, {sim.time - t0:.1f} s")
    todo, _, bricks = unsorted()
    print(f"{len(bricks) - len(todo)} of {len(bricks)} bricks sorted: {held} grips held, {missed} missed, "
          f"{pushes} pushes, {sim.time - t_start:.0f} s of robot time")
    if writer:
        writer.release()
        print("video data/sim_pick.mp4")
    if v:
        v.close()
    return sim, spot


if __name__ == "__main__":
    main()
