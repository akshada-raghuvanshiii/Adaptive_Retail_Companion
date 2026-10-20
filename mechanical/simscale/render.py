import meshio, numpy as np, collections
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
m=meshio.read("raw/sim_0/region1.vtu"); P=m.points*1000
vm=np.asarray(m.point_data["von Mises stress"]).ravel()/1e6
U=np.asarray(m.point_data["displacement"])*1000
T=m.cells_dict["tetra10"][:,:4]
faces=collections.Counter()
F=[]
for t in T:
    for f in ((0,1,2),(0,1,3),(0,2,3),(1,2,3)):
        key=tuple(sorted(t[list(f)])); faces[key]+=1
surf=np.array([k for k,c in faces.items() if c==1])
np.save("surf.npy",surf)
def view(ax,elev,azim,field,vmax,title,scale=0):
    Q=P+scale*U
    tri=Q[surf]; val=field[surf].mean(1)
    pc=Poly3DCollection(tri,cmap="turbo",edgecolor="none"); pc.set_array(np.clip(val,0,vmax)); pc.set_clim(0,vmax)
    ax.add_collection3d(pc)
    for i,a in enumerate("xyz"):
        getattr(ax,f"set_{a}lim")(P[:,i].min(),P[:,i].max())
    ax.set_box_aspect(np.ptp(P,0)); ax.view_init(elev,azim); ax.set_title(title,fontsize=9)
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.set_zlabel("z (mm)")
    return pc
fig=plt.figure(figsize=(12,8))
for i,(e,a) in enumerate([(25,-60),(25,120),(90,-90),(0,0)]):
    ax=fig.add_subplot(2,2,i+1,projection="3d")
    pc=view(ax,e,a,vm,3.0,f"von Mises (MPa, clipped at 3) view {i+1}")
fig.colorbar(pc,ax=fig.axes,shrink=0.5)
fig.savefig("views_vm.png",dpi=110)
