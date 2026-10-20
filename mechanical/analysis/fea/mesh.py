import gmsh, numpy as np, sys
h=float(sys.argv[1]) if len(sys.argv)>1 else 1.6
gmsh.initialize(); gmsh.option.setNumber("General.Terminal",0)
gmsh.model.occ.importShapes("/home/claude/work/kart/raw/kart/flange.STEP"); gmsh.model.occ.synchronize()
gmsh.option.setNumber("Mesh.MeshSizeMax",h); gmsh.option.setNumber("Mesh.MeshSizeMin",h*0.4)
gmsh.option.setNumber("Mesh.MeshSizeFromCurvature",12)
gmsh.model.mesh.generate(3)
tags,c,_=gmsh.model.mesh.getNodes(); P=c.reshape(-1,3); idx=np.zeros(int(tags.max())+1,int); idx[tags]=np.arange(len(tags))
et,_,en=gmsh.model.mesh.getElements(3); T=idx[np.array(en[0],dtype=int)].reshape(-1,4)
np.savez("mesh.npz",p=P,t=T)
print("nodes",len(P),"tets",len(T))
gmsh.finalize()
