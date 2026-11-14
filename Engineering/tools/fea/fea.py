"""Per-link static FEA of KLiK3 with CalculiX (C3D10, Al 6061-T6).
Clamp: surface nodes within TOL of the driving actuator (rotor or stator) in this link.
Load:  outboard weight (+payload) as force F and moment M about the next joint centre, spread
       over the surface nodes within TOL of the next actuator (minimum-norm, like a distributing
       coupling); plus gravity on the link itself (density scaled for covers, screws, flex).
Usage: python3 fea.py <j> [<j> ...]"""
import sys, os, json, subprocess, glob
import numpy as np
from scipy.spatial import cKDTree
D = os.environ.get('FEA_D', os.path.dirname(os.path.abspath(__file__)))
E, NU, RHO, SY = 68900.0, 0.33, 2.7e-9, 276.0
TOL = 1.0
CASE = sys.argv[1] if len(sys.argv) > 1 else 'stretch'
INFO = json.load(open(os.path.join(D, 'fea_%s.json' % CASE)))
MID = {(0, 1): 4, (1, 2): 5, (0, 2): 6, (0, 3): 7, (1, 3): 8, (2, 3): 9}

def parse_inp(f):
    nodes, tets, mode = {}, [], None
    for line in open(f):
        if line.startswith('*'):
            u = line.upper().replace(' ', '')
            mode = 'n' if u.startswith('*NODE') else ('e' if u.startswith('*ELEMENT') and 'C3D10' in u else None)
            continue
        if mode == 'n':
            p = line.split(',');  nodes[int(p[0])] = [float(v) for v in p[1:4]]
        elif mode == 'e':
            p = [int(v) for v in line.replace(',', ' ').split()]
            tets.append(p[1:11])
    tets = np.array(tets)
    X = lambda k: np.array([nodes[i] for i in tets[:, k]])
    a, b, c, d = X(0), X(1), X(2), X(3)
    v = np.einsum('ij,ij->i', b - a, np.cross(c - a, d - a)) / 6
    keep = v > 1e-6 * np.median(v)                      # drop zero-volume slivers (no stiffness, singular Jacobian)
    if (~keep).sum(): print('dropped %d degenerate tets' % (~keep).sum())
    return nodes, tets[keep]

def surface(tets):
    corner_faces = [(0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)]
    allf = []
    for f in corner_faces:
        allf.append(tets[:, list(f)])
    F = np.concatenate(allf)
    key = np.sort(F, axis=1)
    _, idx, cnt = np.unique(key, axis=0, return_index=True, return_counts=True)
    bnd = idx[cnt == 1]
    ftype = bnd // len(tets); el = bnd % len(tets)
    tris, snodes = [], set()
    for ft, e in zip(ftype, el):
        a, b, c = corner_faces[ft]; t = tets[e]
        tri = [t[a], t[b], t[c]]
        mids = [t[MID[tuple(sorted(p))]] for p in ((a, b), (b, c), (a, c))]
        tris.append(tri); snodes.update(tri); snodes.update(mids)
    return np.array(tris), np.array(sorted(snodes))

def parse_frd(f):
    out, cur = {}, None
    for line in open(f):
        if line.startswith(' -4'):
            cur = line.split()[1]; out[cur] = {}
        elif cur and line.startswith(' -1'):
            nid = int(line[3:13]); vals = [float(line[13 + 12 * i:25 + 12 * i]) for i in range((len(line.rstrip()) - 13) // 12)]
            out[cur][nid] = vals
        elif line.startswith(' -3'):
            cur = None
    return out

def run(j):
    L = INFO['links'][str(j)]
    nodes, tets = parse_inp(os.path.join(D, 'link%d.inp' % j))
    used = np.unique(tets)
    ids = used; xyz = np.array([nodes[i] for i in ids]); pos = {n: k for k, n in enumerate(ids)}
    tris, snodes = surface(tets)
    sx = np.array([xyz[pos[n]] for n in snodes])
    def near(tag):
        tol = {(4, 'fix'): 2.5}.get((j, tag), TOL)   # forearm: a 2 mm trim ring sits between it and the J4 rotor
        pts = np.load(os.path.join(D, 'link%d_%s_pts.npy' % (j, tag)))
        d, _ = cKDTree(pts).query(sx, distance_upper_bound=tol + 1)
        return snodes[d < tol]
    if j == 0:
        fix = snodes[sx[:, 2] < xyz[:, 2].min() + 1e-6]
    else:
        fix = near('fix')
    load = near('load')
    assert len(fix) > 20 and len(load) > 20, (len(fix), len(load))
    ref = np.array(L['ref_local']); F = np.array(L['F_local_N']); M = np.array(L['M_local_Nmm'])
    r = np.array([xyz[pos[n]] for n in load]); c = r.mean(0); q = r - c
    J = (q * q).sum() * np.eye(3) - q.T @ q
    Mc = M + np.cross(ref - c, F)                        # moment about the load-node centroid
    om = np.linalg.solve(J, Mc)
    f = F / len(load) + np.cross(om, q)
    assert np.allclose(f.sum(0), F, atol=1e-6) and np.allclose(np.cross(q, f).sum(0), Mc, atol=1e-3)
    g = np.array(L['gravity_local']); gm = np.linalg.norm(g)
    name = os.path.join(D, 'run%d_%s' % (j, CASE))
    with open(name + '.inp', 'w') as o:
        o.write('*NODE\n'); o.writelines('%d,%.6f,%.6f,%.6f\n' % (n, *nodes[n]) for n in ids)
        o.write('*ELEMENT,TYPE=C3D10,ELSET=EALL\n'); o.writelines(','.join(map(str, [k + 1] + list(t))) + '\n' for k, t in enumerate(tets))
        o.write('*NSET,NSET=FIX\n'); o.writelines('%d,\n' % n for n in fix)
        o.write('*NSET,NSET=LOAD\n'); o.writelines('%d,\n' % n for n in load)
        o.write('*MATERIAL,NAME=AL6061\n*ELASTIC\n%g,%g\n*DENSITY\n%g\n' % (E, NU, RHO * L['density_factor']))
        o.write('*SOLID SECTION,ELSET=EALL,MATERIAL=AL6061\n*STEP\n*STATIC\n*BOUNDARY\nFIX,1,3\n*CLOAD\n')
        for n, fi in zip(load, f):
            for k in range(3): o.write('%d,%d,%.8e\n' % (n, k + 1, fi[k]))
        o.write('*DLOAD\nEALL,GRAV,%g,%g,%g,%g\n' % (gm, *(g / gm)))
        o.write('*NODE FILE\nU\n*EL FILE\nS\n*END STEP\n')
    env = dict(os.environ, OMP_NUM_THREADS='8', CCX_NPROC_EQUATION_SOLVER='8')
    mnt = glob.glob('/tmp/.mount_FreeCA*/usr')[0]; env['LD_LIBRARY_PATH'] = mnt + '/lib'
    p = subprocess.run([mnt + '/bin/ccx', '-i', os.path.basename(name)], env=env, capture_output=True, text=True, cwd=D)
    if 'Job finished' not in p.stdout:
        print(p.stdout[-2000:], p.stderr[-2000:]); raise SystemExit('ccx failed %d' % j)
    res = parse_frd(name + '.frd')
    U = np.array([res['DISP'][n][:3] for n in ids]); S = np.array([res['STRESS'][n][:6] for n in ids])
    sxx, syy, szz, sxy, syz, szx = S.T
    vm = np.sqrt(0.5 * ((sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2) + 3 * (sxy ** 2 + syz ** 2 + szx ** 2))
    # stresses away from the clamp / load patches (point-load and clamp-edge singularities)
    bc = np.array([xyz[pos[n]] for n in np.concatenate([fix, load])])
    dbc, _ = cKDTree(bc).query(xyz)
    far = dbc > 3.0
    # rigid-body fit of the load patch: u = t + th x (r - ref)
    rr = r - ref; ul = np.array([U[pos[n]] for n in load])
    A = np.zeros((3 * len(rr), 6)); b = ul.reshape(-1)
    for i, v in enumerate(rr):
        A[3 * i:3 * i + 3, :3] = np.eye(3)
        A[3 * i:3 * i + 3, 3:] = -np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    sol = np.linalg.lstsq(A, b, rcond=None)[0]; t, th = sol[:3], sol[3:]
    tip = np.array(L['tip_local']); dtip = t + np.cross(th, tip - ref)
    k_all, k_far = int(vm.argmax()), int(np.where(far)[0][vm[far].argmax()])
    out = dict(case=CASE, j=j, nodes=len(ids), tets=len(tets), fix_nodes=len(fix), load_nodes=len(load),
               vm_max=float(vm.max()), vm_max_at=xyz[k_all].tolist(), vm_far=float(vm[far].max()), vm_far_at=xyz[k_far].tolist(),
               vm_p99=float(np.percentile(vm, 99.9)), umax=float(np.linalg.norm(U, axis=1).max()),
               ref_t_um=(t * 1000).tolist(), ref_rot_urad=(th * 1e6).tolist(), tip_local_um=(dtip * 1000).tolist(),
               F=F.tolist(), M_Nm=(M / 1000).tolist(), mass_kg=L['struct_kg'])
    np.savez_compressed(name + '_res.npz', xyz=xyz, U=U, vm=vm, tris=np.vectorize(pos.get)(tris), fix=np.vectorize(pos.get)(fix), load=np.vectorize(pos.get)(load))
    json.dump(out, open(name + '_res.json', 'w'), indent=1)
    print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in out.items() if not isinstance(v, list)}),
          'tip um', np.round(dtip * 1000, 2), 'rot urad', np.round(th * 1e6, 2), flush=True)

for a in sys.argv[2:]: run(int(a))
