import numpy as np, sys, json, time
from skfem import *
from skfem.models.elasticity import linear_elasticity, lame_parameters
from skfem.helpers import dot
E=float(sys.argv[2]) if len(sys.argv)>2 else 70e3   # MPa (N/mm^2), mm units
nu=0.33
case=sys.argv[1]
d=np.load("mesh.npz"); m=MeshTet(d["p"].T.copy(), d["t"].T.copy())
e=ElementVector(ElementTetP2()); ib=Basis(m,e,intorder=4)
lam,mu=lame_parameters(E,nu)
t0=time.time(); K=asm(linear_elasticity(lam,mu),ib)
YC,ZC,XM=44.36,0.0,-22.81
fac=m.facets; fm=m.p[:,fac].mean(axis=0) if False else m.p[:,m.facets].mean(axis=1)
bnd=m.boundary_facets()
fp=m.p[:,m.facets[:,bnd]]      # 3 x 3 x nb
cen=fp.mean(axis=1)
base=bnd[np.abs(cen[1]-11.8645)<0.01]
rr=np.hypot(cen[1]-YC,cen[2]-ZC)
hole=bnd[(np.abs(rr-9.0)<0.25)&(cen[0]>-24.6)&(cen[0]<-21.0)]
face_m=bnd[(np.abs(cen[0]-(-21.06))<0.01)&(rr>9.0)&(rr<22.5)]
print("facets base",len(base),"hole",len(hole),"motorface",len(face_m))
def fbasis(f): return FacetBasis(m,e,facets=f,intorder=4)
f=np.zeros(K.shape[0])
def add_uniform(facets,vec):
    fb=fbasis(facets); area=asm(Functional(lambda w: 1.0+0*w.x[0]),fb)
    tr=np.array(vec)/area
    @LinearForm
    def L(v,w): return tr[0]*v.value[0]+tr[1]*v.value[1]+tr[2]*v.value[2]
    return asm(L,fb)
def add_moment_z(facets,Mz):
    # traction t_x = -k (y-yc) on face; moment about z through hole centre = k * int (y-yc)^2 dA
    fb=fbasis(facets); Iy=asm(Functional(lambda w:(w.x[1]-YC)**2),fb); k=Mz/Iy
    @LinearForm
    def L(v,w): return -k*(w.x[1]-YC)*v.value[0]
    return asm(L,fb)
info={}
if case=="simscale":
    f=add_uniform(hole,[10.0,0,10.0])
elif case=="design":
    Fv=float(sys.argv[3]); ecc=float(sys.argv[4])
    # wheel reaction: towards the root (-y, part frame); offset ecc on the side away from the motor (-x)
    f=add_uniform(hole,[0,-Fv,0]) + add_moment_z(face_m, Fv*ecc)
    info.update(Fv=Fv,e=ecc)
D=ib.get_dofs(base).all()
u=solve(*condense(K,f,D=D))
print("solve s %.1f"%(time.time()-t0))
# stresses at quadrature points
from skfem.helpers import sym_grad, trace, eye
def stress_at(basis,u):
    U=basis.interpolate(u); G=U.grad  # 3x3xnelxnqp
    eps=0.5*(G+np.transpose(G,(1,0,2,3)))
    tr=eps[0,0]+eps[1,1]+eps[2,2]
    s=2*mu*eps; 
    for i in range(3): s[i,i]+=lam*tr
    vm=np.sqrt(0.5*((s[0,0]-s[1,1])**2+(s[1,1]-s[2,2])**2+(s[2,2]-s[0,0])**2)+3*(s[0,1]**2+s[1,2]**2+s[0,2]**2))
    return vm
vm=stress_at(ib,u); X=ib.global_coordinates().value  # 3 x nel x nqp
disp=u[ib.nodal_dofs].T if False else None
ux=u[ib.nodal_dofs[0]]; uy=u[ib.nodal_dofs[1]]; uz=u[ib.nodal_dofs[2]]
umag=np.sqrt(ux**2+uy**2+uz**2)
y=X[1]; away=y>12.9   # exclude the layer of elements touching the fixed base (support singularity)
pk=vm[away].max(); i=np.unravel_index(np.argmax(np.where(away,vm,0)),vm.shape)
loc=X[:,i[0],i[1]]
root=(y>19.4)&(y<21.0)
info.update(case=case,E=E,dofs=int(K.shape[0]),vm_max_all=float(vm.max()),vm_peak=float(pk),peak_loc=loc.round(2).tolist(),
            vm_root_p95=float(np.percentile(vm[root],95)),umax=float(umag.max()),ux_max=float(np.abs(ux).max()))
print(json.dumps(info))
json.dump(info,open(f"res_{case}.json","w"))
np.savez(f"field_{case}.npz",X=X,vm=vm,u=np.vstack([ux,uy,uz]),pn=m.p)
