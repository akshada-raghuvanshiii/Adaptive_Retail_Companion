import gmsh, json, numpy as np, time, os
D="/home/claude/work/kart/raw/kart/"
leaves=[l for l in json.load(open("leaves.json")) if "^" not in l["name"]]
gmsh.initialize(); gmsh.option.setNumber("General.Terminal",0)
info={}; meshes={}
for name in sorted(set(l["name"] for l in leaves)):
    t0=time.time(); gmsh.clear(); gmsh.model.occ.importShapes(D+name+".STEP"); gmsh.model.occ.synchronize()
    vols=gmsh.model.getEntities(3)
    V=sum(gmsh.model.occ.getMass(d,t) for d,t in vols)
    com=sum(np.array(gmsh.model.occ.getCenterOfMass(d,t))*gmsh.model.occ.getMass(d,t) for d,t in vols)/V
    gmsh.option.setNumber("Mesh.MeshSizeMax",8 if name=="Part2" else 3); gmsh.option.setNumber("Mesh.MeshSizeMin",1 if name!="Part2" else 3)
    try:
        gmsh.model.mesh.generate(2)
        tags,c,_=gmsh.model.mesh.getNodes(); Pn=c.reshape(-1,3); idx={t:i for i,t in enumerate(tags)}
        _,_,en=gmsh.model.mesh.getElements(2); T=np.array([idx[n] for n in np.concatenate(en)]).reshape(-1,3)
        meshes[name]=Pn[T]
    except Exception as e: print("mesh fail",name,e)
    info[name]=dict(V=V,com=com.tolist(),nvol=len(vols))
    print(name,"vols",len(vols),"V %.0f mm3"%V,"t %.1fs"%(time.time()-t0),flush=True)
gmsh.finalize()
json.dump(info,open("info.json","w"),indent=1); np.save("meshes.npy",meshes,allow_pickle=True)
