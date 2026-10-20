import gmsh, json, numpy as np, itertools
D="/home/claude/work/kart/raw/kart/"
L=[l for l in json.load(open("leaves.json")) if "^" not in l["name"] and l["name"]!="Part2"]
# one drive module (x ~ 958) plus its motor and wheel
sel=[l for l in L if (np.array(l["t"])[0]>800 and np.array(l["t"])[2]<1200) or l["name"] in ("motor","motor wheels") and np.array(l["t"])[0]>800]
gmsh.initialize(); gmsh.option.setNumber("General.Terminal",0)
ent={}
for l in sel:
    v=gmsh.model.occ.importShapes(D+l["name"]+".STEP"); R=np.array(l["R"]); t=np.array(l["t"])
    gmsh.model.occ.affineTransform(v,np.hstack([R,t[:,None]]).ravel().tolist()); ent.setdefault(l["name"],[]).extend(v)
gmsh.model.occ.synchronize()
names=list(ent)
for a,b in itertools.combinations(names,2):
    out,_=gmsh.model.occ.intersect(ent[a],ent[b],removeObject=False,removeTool=False); gmsh.model.occ.synchronize()
    vol=sum(gmsh.model.occ.getMass(d,t) for d,t in out if d==3)
    if vol>0.01: print("%-24s x %-24s overlap %.2f mm3"%(a,b,vol))
    if out: gmsh.model.occ.remove(out,recursive=True); gmsh.model.occ.synchronize()
print("checked",names)
gmsh.finalize()
