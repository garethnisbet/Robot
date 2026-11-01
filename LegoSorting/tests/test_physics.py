"""Physics pile: bricks fall, rest on the mat and on each other, and slump when one is removed."""
import numpy as np

from legosort.lego import Brick
from legosort.physics import PileSim


def lowest(b):
    L, W, H = b.length, b.width, b.height
    c = np.array([[x, y, z] for x in (-L / 2, L / 2) for y in (-W / 2, W / 2) for z in (0, H)])
    T = b.T
    return (c @ T[:3, :3].T + T[:3, 3])[:, 2].min()


def test_single_brick_rests_studs_up():
    sim = PileSim([Brick(2, 4, "red")])
    sim.place(0, np.eye(3), (230, 0, 5.0))
    sim.settle()
    b = sim.bricks()[0]
    assert abs(b.z) < 0.05 and b.tilt < 0.5
    assert abs(b.x - 230) < 0.5 and abs(b.y) < 0.5


def test_stacked_brick_sits_on_studs_or_clutch():
    sim = PileSim([Brick(2, 4, "red"), Brick(2, 4, "blue")])
    sim.place(0, np.eye(3), (230, 0, 0))
    sim.place(1, np.eye(3), (230, 0, 9.6 + 1.7 + 2))     # on top of the studs
    sim.settle()
    top = sim.bricks()[1]
    # hollow underside: studs slot in, so it ends up on the lower brick's top (9.6) not its studs (11.3)
    assert 9.4 < top.z < 9.9, top.z


def test_pour_and_remove():
    rng = np.random.default_rng(0)
    bricks = [Brick(2, 4, "red"), Brick(2, 2, "blue"), Brick(1, 4, "green"), Brick(2, 3, "yellow")] * 3
    sim = PileSim(bricks)
    sim.pour(rng=rng)
    pile = sim.bricks()
    assert len(pile) == len(bricks)
    assert min(lowest(b) for b in pile) > -0.1                 # nothing sinks through the mat
    assert sum(lowest(b) > 1.0 for b in pile) >= 1              # something rests on another brick
    assert sim.data.ncon == 0 or -sim.data.contact.dist.min() < 0.1   # interpenetration < 0.1 mm
    sim.remove(int(sim.indices()[0]))
    sim.settle()
    assert len(sim.bricks()) == len(bricks) - 1
    assert max(sim.speed()) < 5.0


def test_robot_matches_kinematics_and_picks():
    from legosort.geometry import rot_z
    from legosort.pick import HOME, grasps, pick_and_place
    from legosort.robotcal import _kin
    sim = PileSim([Brick(2, 4, "red")], robot=True, q0=HOME)
    q = [20, 30, -10, 5, 60, 30]
    sim.set_joints(q)
    f = _kin().f_kinematics(np.array(q, float))
    R, p = sim.tcp_pose()
    T = sim.gripper.T_tool_grip
    R_tool = np.column_stack([f[5], f[6], f[7]])
    assert np.linalg.norm(p - (f[8] + R_tool @ T[:3, 3])) < 0.1     # viewer vs GNK: ~0.003 rad
    assert np.abs(R - R_tool @ T[:3, :3]).max() < 0.01
    sim.set_joints(HOME)
    sim.place(0, rot_z(30), (200, 0, 0))
    sim.settle()
    from legosort.pick import orient
    g = orient(sim, grasps(sim.bricks(), sim.gripper)[0])
    assert g is not None
    assert pick_and_place(sim, 0, g, (150, -105), travel_z=40.0, log=lambda *a: None)
    sim.settle()
    b = sim.bricks()[0]
    assert np.hypot(b.x - 150, b.y + 105) < 15 and b.z < 1.0


def test_side_grip_from_outside():
    from legosort.geometry import rot_z
    from legosort.pick import HOME, pick_side, side_grasps
    sim = PileSim([Brick(2, 4, "red"), Brick(2, 2, "blue")], robot=True, q0=HOME)
    sim.place(0, rot_z(20), (215, 0, 0))
    sim.place(1, rot_z(0), (185, 0, 0))            # inner brick, next to the pile centre
    sim.settle()
    gs = side_grasps(sim.bricks(), sim.gripper, (185, 0))
    assert gs and gs[0].index == 0                 # outermost first
    g = gs[0]
    assert abs(g.R[2, 1] + 1) < 1e-6               # wrist above: grip y points down
    assert pick_side(sim, 0, g, (150, -105), travel_z=40.0, log=lambda *a: None)
    sim.settle()
    b = sim.bricks()[0]
    assert np.hypot(b.x - 150, b.y + 105) < 15 and b.z < 1.0 and b.tilt < 5


def test_push_one_wide_brick():
    """A 1x1 can't be gripped by the bare jaws: it is pushed, jaws first, and stays near the mat."""
    from legosort.geometry import rot_z
    from legosort.pick import HOME, plan_push, push
    sim = PileSim([Brick(1, 1, "red")], robot=True, q0=HOME)
    sim.place(0, rot_z(10), (170, 20, 0))
    sim.settle()
    plan = plan_push(sim, sim.bricks(), 0, (150, -60), travel_z=40.0)
    assert plan is not None
    push(sim, plan)
    sim.settle()
    b = sim.bricks()[0]
    assert np.hypot(b.x - 150, b.y + 60) < 8 and b.z < 1.0     # one leg, capped at the goal


def test_stepped_fingers_grip_one_and_two_wide():
    """With the printed stepped fingers a 1x1 is held on the narrow step, a 2x4 on the wide one."""
    from legosort.geometry import rot_z
    from legosort.megp25e import MEGP25E
    from legosort.pick import HOME, pick_side, side_grasps
    g = MEGP25E(fingers=True)
    assert abs(g.steps[0][0] - 17.0) < 0.1 and abs(g.steps[1][0] - 12.2) < 0.1
    for brick, step in ((Brick(1, 1, "red"), 12.2), (Brick(2, 4, "blue"), 17.0)):
        sim = PileSim([brick], robot=True, q0=HOME, gripper=g)
        sim.place(0, rot_z(15), (180, 0, 0))
        sim.settle()
        gs = side_grasps(sim.bricks(), sim.gripper, (180, 0), travel_z=40.0)
        assert gs and abs(gs[0].step - step) < 0.1
        assert pick_side(sim, 0, gs[0], (150, -105), travel_z=40.0, log=lambda *a: None)
        sim.settle()
        b = sim.bricks()[0]
        assert np.hypot(b.x - 150, b.y + 105) < 15 and b.z < 1.0
