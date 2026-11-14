import Part, numpy as np, glob, os
P=globals().get('P', __import__('os').environ.get('FEA_D', '.'))
for f in sorted(glob.glob(P+'/link*_*.brep')):
    s=Part.Shape(); s.importBrep(f)
    pts=[]
    for face in s.Faces:
        v,t=face.tessellate(0.02)
        v=np.array([[p.x,p.y,p.z] for p in v])
        for a,b,c in t:
            A,B,C=v[a],v[b],v[c]
            L=max(np.linalg.norm(B-A),np.linalg.norm(C-B),np.linalg.norm(A-C))
            n=max(1,int(np.ceil(L/0.25)))
            for i in range(n+1):
                for k in range(n+1-i):
                    u=i/n; w=k/n; pts.append(A+(B-A)*u+(C-A)*w)
    pts=np.array(pts); np.save(f.replace('.brep','_pts.npy'),pts); print(os.path.basename(f), len(pts))
