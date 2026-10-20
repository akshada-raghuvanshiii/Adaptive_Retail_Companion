import meshio, numpy as np, sys
m=meshio.read("raw/sim_0/region1.vtu"); P=m.points
S=np.asarray(m.point_data["cauchy stress"]); U=np.asarray(m.point_data["displacement"])
T=m.cells_dict["tetra10"][:,:4]
X=P[T]  # (n,4,3)
def locate(pts):
    # barycentric for all tets, vectorized per point batch
    v0=X[:,0]; Mi=np.linalg.inv(np.stack([X[:,1]-v0,X[:,2]-v0,X[:,3]-v0],axis=2))
    out=[]
    for p in pts:
        l=np.einsum('nij,nj->ni',Mi,p-v0); l0=1-l.sum(1)
        ok=np.where((l>=-1e-9).all(1)&(l0>=-1e-9))[0]
        if len(ok)==0: out.append(None); continue
        k=ok[0]; w=np.r_[l0[k],l[k]]; out.append((k,w))
    return out
def section(y0, xr, zr, n=(30,120)):
    xs=np.linspace(*xr,n[0]); zs=np.linspace(*zr,n[1]); dA=(xs[1]-xs[0])*(zs[1]-zs[0])
    pts=np.array([[x,y0,z] for x in xs for z in zs])
    F=np.zeros(3); Mz=0; A=0
    for p,r in zip(pts,locate(pts)):
        if r is None: continue
        k,w=r; s=(w[:,None]*S[T[k]]).sum(0)
        # order xx yy zz xy yz xz -> traction on +y face: (xy, yy, yz)
        t=np.array([s[3],s[1],s[4]]); F+=t*dA; A+=dA
        Mz+= (p[0]*t[1])*dA
    return F,A,Mz
if __name__=="__main__":
    for y0 in (0.020,0.030,0.040,0.050,0.060,0.070):
        F,A,Mz=section(y0,(-0.0250,-0.0205),(-0.0255,0.0255))
        print("thin y=%.0fmm  A=%.1f mm2 F=%s N"%(y0*1000,A*1e6,np.round(F,2)))
    for y0 in (-0.060,-0.040,-0.020,0.0):
        F,A,Mz=section(y0,(-0.0025,0.0135),(-0.0305,0.0305))
        print("thick y=%.0fmm A=%.1f mm2 F=%s N"%(y0*1000,A*1e6,np.round(F,2)))
