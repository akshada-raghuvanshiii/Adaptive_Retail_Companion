import json, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
tris=np.load("../step/tris.npy",allow_pickle=True).item()
pl=json.load(open("placements.json"))
key={"extrude":"extrude","flange":"flange","new sleeve":"new_sleeve","springs":"springs"}
cols={"extrude":"#7aa6d6","flange":"#e3a35b","new_sleeve":"#9fc79a","springs":"#d46a6a"}
parts=[]
for p in pl:
    k=key.get(p["name"]);
    if not k: continue
    R=np.array(p["R"]); t=np.array(p["t"])
    T=tris[k]@R.T+t
    parts.append((k,T))
allp=np.concatenate([T.reshape(-1,3) for _,T in parts]); c=allp.mean(0)
for k,T in parts:
    q=T.reshape(-1,3)-c; print(k,"bbox",q.min(0).round(1),q.max(0).round(1))
np.save("parts_asm.npy",np.array([(k,T-c) for k,T in parts],dtype=object),allow_pickle=True)
fig=plt.figure(figsize=(12,9))
for i,(e,a) in enumerate([(20,-60),(20,30),(0,0),(0,-90)]):
    ax=fig.add_subplot(2,2,i+1,projection="3d")
    for k,T in parts:
        ax.add_collection3d(Poly3DCollection(T-c,facecolor=cols[k],edgecolor=(0,0,0,0.06),linewidth=0.2))
    q=allp-c
    for j,a_ in enumerate("xyz"): getattr(ax,f"set_{a_}lim")(q[:,j].min(),q[:,j].max())
    ax.set_box_aspect(np.ptp(q,0)); ax.view_init(e,a); ax.set_xlabel("x");ax.set_ylabel("y");ax.set_zlabel("z"); ax.set_title(f"elev {e} azim {a}",fontsize=8)
fig.savefig("assembly.png",dpi=90)
