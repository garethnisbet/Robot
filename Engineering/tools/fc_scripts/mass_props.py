"""Mass and centre of mass of each KLiK3 link in its own joint frame (links 1-6; link 0 is
fixed), for the gravity / gear-sag model in klik3_ik.py. Actuators use catalogue masses
(rotors are inside them), screws/dowels/bearings steel, the rest aluminium.
Writes klik3_mass.json next to klik3_freecad.py."""
import FreeCAD as App, re, json, os
import actuator_fit as af, klik3_freecad as k3
d = App.getDocument('KLiK3'); sh = d.getObject('Joints')
old = [sh.get('B%d' % i) for i in range(2, 8)]
for i in range(2, 8): sh.set('B%d' % i, '0')
d.recompute()
ACT = {'J%d_actuator' % j: k3.ACTUATORS[j - 1][0] for j in range(1, 7)}
STEEL = re.compile('screw|bolt|dowel|support', re.I)
def link_of(o):
    g = o.getParentGeoFeatureGroup()
    while g is not None and not re.match(r'Link\d$|KLiK3$', g.Name): g = g.getParentGeoFeatureGroup()
    return None if g is None else (0 if g.Name == 'KLiK3' else int(g.Name[4:]))
acc = {j: [0.0, App.Vector()] for j in range(1, 7)}
for o in d.Objects:
    if not o.TypeId.startswith('Part::') or o.Shape.isNull() or o.Name.startswith('Meca'): continue
    l = link_of(o)
    if not l: continue
    if o.Name.endswith('_output'): continue
    s = o.Shape.copy(); s.Placement = d.getObject('Link%d' % l).getGlobalPlacement().inverse().multiply(o.getGlobalPlacement())
    m = af.MASS[ACT[o.Name]] if o.Name in ACT else s.Volume * (7.85 if STEEL.search(o.Name) else 2.7) * 1e-6
    sols = s.Solids                                       # Shape.CenterOfMass of a multi-solid compound is only its first solid's
    c = sum((so.CenterOfMass * so.Volume for so in sols), App.Vector()) * (1.0 / sum(so.Volume for so in sols)) if sols else s.BoundBox.Center
    acc[l][0] += m; acc[l][1] = acc[l][1] + c * m
fl = d.getObject('Flange')
out = {'links': {j: {'kg': m, 'com': list(c * (1.0 / m))} for j, (m, c) in acc.items()},
       'flange_face_x': fl.Shape.BoundBox.XMax,
       'actuators': [a[0] for a in k3.ACTUATORS],
       # each actuator's output disc centre (its output bearing) in that joint's link frame
       'bearing_points': {k: list(d.getObject('Link%d' % k).getGlobalPlacement().inverse().multVec(
           (lambda s: (setattr(s, 'Placement', d.getObject('J%d_output' % k).getGlobalPlacement()), s)[1])(d.getObject('J%d_output' % k).Shape.copy()).Solids[0].CenterOfMass))
           for k in range(1, 7)}}
json.dump(out, open(os.path.join(os.path.dirname(k3.__file__), 'klik3_mass.json'), 'w'), indent=1)
for i, v in enumerate(old, 2): sh.set('B%d' % i, str(v))
d.recompute()
for j, v in out['links'].items(): print(j, round(v['kg'], 3), [round(x, 1) for x in v['com']])
print('flange face x', out['flange_face_x']); print('bearing points', {k: [round(x, 1) for x in v] for k, v in out['bearing_points'].items()})
