import numpy as np
from skfem import MeshTet
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import proj3d
d=np.load("mesh_h125.npz"); m=MeshTet(d["p"].T.copy(), d["t"].T.copy())
f=np.load("field_design_h125.npz"); vm=f["vm"]; X=f["X"]; u=f["u"][:, :m.p.shape[1]]
b=m.boundary_facets(); el=m.f2t[0,b]
val=vm.max(axis=1)[el]
P=m.p.T; scale=0.0
Q=(P+scale*u.T)[:,[0,2,1]]           # y up
tri=Q[m.facets[:,b].T]
fig=plt.figure(figsize=(8,6.4)); ax=fig.add_axes([0,0,0.8,1],projection="3d")
pc=Poly3DCollection(tri,cmap="turbo",edgecolor="none"); pc.set_array(np.clip(val,0,250)); pc.set_clim(0,250); ax.add_collection3d(pc)
q=P[:,[0,2,1]]
for j,a in enumerate("xyz"): getattr(ax,f"set_{a}lim")(q[:,j].min()-25,q[:,j].max()+5)
ax.set_box_aspect((np.ptp(q[:,0])+30,np.ptp(q[:,1])+30,np.ptp(q[:,2])+30)); ax.view_init(18,-125); ax.set_axis_off()
fig.canvas.draw()
def ann(pt,txt,xy,c):
    x2,y2,_=proj3d.proj_transform(pt[0],pt[2],pt[1],ax.get_proj())
    ax.annotate(txt,xy=(x2,y2),xycoords="data",xytext=xy,textcoords="figure fraction",fontsize=9,color=c,arrowprops=dict(arrowstyle="->",color=c,lw=1.1))
ann([-24.4,44.3,-12.4],"246 MPa: net section between the\nshaft hole and a motor-screw hole","black",(0.02,0.86)) if False else None
ann(np.array([-24.4,44.3,-12.4])+scale*0,"246 MPa: net section between the\nshaft hole and a motor-screw hole",(0.02,0.86),"black")
ann([-24.5,18.7,24.8],"139 MPa: root fillet",(0.02,0.10),"#b91c1c")
cax=fig.add_axes([0.82,0.22,0.025,0.55]); cb=fig.colorbar(pc,cax=cax); cb.set_label("von Mises stress (MPa)")
fig.text(0.02,0.02,"Load: 181 N wheel load at 71.7 mm lateral offset; base fixed; aluminium (E = 70 GPa).",fontsize=8)
fig.savefig("/home/claude/work/report2/gen/fig_fea_design.png",dpi=220,bbox_inches="tight")
