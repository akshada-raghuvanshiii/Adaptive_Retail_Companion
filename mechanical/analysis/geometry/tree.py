import re, numpy as np, json, os
D="/home/claude/work/kart/raw/kart/"
def parse(fn):
    s=open(fn,errors="ignore").read()
    E={int(m.group(1)):m.group(2) for m in re.finditer(r"#(\d+)\s*=\s*(.*?);\s*$",s,re.M|re.S)}
    refs=lambda i:[int(x) for x in re.findall(r"#(\d+)",E[i])]
    def pt(i): return np.array([float(x) for x in re.search(r"\(\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*\)\s*\)",E[i]).groups()])
    def ax(i):
        r=refs(i); o=pt(r[0]); z=pt(r[1]); x=pt(r[2]); z/=np.linalg.norm(z); x-=x.dot(z)*z; x/=np.linalg.norm(x)
        return o,np.column_stack([x,np.cross(z,x),z])
    def pname(pd):
        prod=refs(refs(pd)[0])[0]; return re.search(r"'([^']*)'",E[prod]).group(1)
    out=[]
    for i,t in E.items():
        if t.startswith("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION"):
            rel,pds=refs(i); nauo=refs(pds)[0]; par_pd,ch_pd=refs(nauo)[0],refs(nauo)[1]
            idt=[r for r in refs(rel) if E[r].startswith("ITEM_DEFINED")][0]; a1,a2=refs(idt)
            o1,R1=ax(a1); o2,R2=ax(a2); R=R1@R2.T
            out.append(dict(parent=pname(par_pd),child=pname(ch_pd),R=R.tolist(),t=(o1-R@o2).tolist(),nauo=re.search(r"'([^']*)'",E[nauo]).group(1)))
    return out
A=parse(D+"modular assem.STEP")+parse(D+"Assem1.STEP")
# expand tree from root
kids={}
for a in A: kids.setdefault(a["parent"],[]).append(a)
leaves=[]
def walk(name,R,t,path):
    if name not in kids:
        leaves.append(dict(name=name,R=R.tolist(),t=t.tolist(),path=path)); return
    for a in kids[name]:
        Rc=np.array(a["R"]); tc=np.array(a["t"])
        walk(a["child"],R@Rc,R@tc+t,path+[a["nauo"]])
walk("modular assem",np.eye(3),np.zeros(3),[])
for l in leaves: print(l["name"],np.round(l["t"],1))
json.dump(leaves,open("leaves.json","w"),indent=1)
