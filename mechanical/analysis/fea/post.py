import numpy as np, json, sys
f=np.load(sys.argv[1]); X=f["X"]; vm=f["vm"]; u=f["u"]; pn=f["pn"]
y=X[1]; z=X[2]
reg=lambda m: (vm[m].max(), X[:,*np.unravel_index(np.argmax(np.where(m,vm,-1)),vm.shape)].round(1))
root=(y>12.9)&(y<26); holes=(y>30)&(y<60)
r1=reg(root); r2=reg(holes)
print("root zone max %.1f MPa at"%r1[0],r1[1],"| p99 %.1f"%np.percentile(vm[root],99))
print("hole zone max %.1f MPa at"%r2[0],r2[1],"| p99 %.1f"%np.percentile(vm[holes],99))
# tilt: ux of P2 nodal dofs -> use mesh vertices only (first len(pn) entries correspond? not guaranteed) ; use quadrature-free approach:
ux=u[0][:pn.shape[1]]
sel=(np.abs(pn[2])>18)&(np.abs(pn[2])<24)&(np.abs(pn[0]+22.81)<2)
yy=pn[1][sel]; uu=ux[sel]
for y0 in (19.4,30,44.36,60):
    k=np.abs(yy-y0)<2; 
print("tip |u| max %.3f mm"%np.abs(u).max())
o=np.argsort(yy); yy=yy[o]; uu=uu[o]
a=np.polyfit(yy[(yy>40)&(yy<49)],uu[(yy>40)&(yy<49)],1)[0]
print("slope dux/dy at shaft %.5f rad = %.2f deg"%(a,np.degrees(a)))
print("ux at shaft centre ~ %.3f mm"%np.interp(44.36,yy,uu))
