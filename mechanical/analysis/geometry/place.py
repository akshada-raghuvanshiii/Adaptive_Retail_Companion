import json, numpy as np
leaves=[l for l in json.load(open("leaves.json")) if "^" not in l["name"]]
M=np.load("meshes.npy",allow_pickle=True).item(); info=json.load(open("info.json"))
placed=[]
for i,l in enumerate(leaves):
    R=np.array(l["R"]); t=np.array(l["t"]); T=M[l["name"]]@R.T+t
    com=R@np.array(info[l["name"]]["com"])+t
    placed.append(dict(name=l["name"],i=i,T=T,com=com,V=info[l["name"]]["V"],R=R,t=t))
    q=T.reshape(-1,3); print("%-24s"%l["name"],"bbox",q.min(0).round(1),q.max(0).round(1),"com",com.round(1))
np.save("placed.npy",placed,allow_pickle=True)
allq=np.concatenate([p["T"].reshape(-1,3) for p in placed]); print("ALL",allq.min(0).round(1),allq.max(0).round(1),"size",np.ptp(allq,0).round(1))
